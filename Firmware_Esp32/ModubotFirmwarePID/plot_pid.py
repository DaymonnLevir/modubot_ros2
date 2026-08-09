#!/usr/bin/env python3
"""
Plotter em tempo real da telemetria PI do ModubotFirmwarePID.

Mostra setpoint x velocidade medida (por roda) e a saída do controlador,
e aceita comandos digitados no terminal enquanto plota (para ajustar
kp/ki ao vivo até chegar num bom resultado).

O firmware fala em rad/s (não conhece o raio da roda — isso é do ROS).
Este script converte para m/s na TELA usando --radius, então você digita
degraus na unidade que preferir e lê o gráfico em m/s.

Uso:
    pip install pyserial matplotlib
    python plot_pid.py COM5 --radius 0.078            # Windows
    python plot_pid.py /dev/ttyUSB0 --radius 0.078    # Jetson / Linux
    python plot_pid.py COM5 --radius 0.078 --log ensaio1.csv

Comandos (digitar no terminal + Enter):
    W 4 4          degrau de 4 rad/s nas duas rodas
    w 0.3 0.3      degrau em m/s (minúsculo; convertido aqui usando --radius)
    K 6 30 0       ganhos kp ki kd (DAC por rad/s)
    F 5.9 7        feedforward kff dac_min
    M 0 / M 1      malha aberta / fechada
    V 0.2 0.2      fração de DAC (só vale em malha aberta)
    S              freio
    G              mostra config do firmware
    q              sai (envia S e P 0 antes)

O script reenvia o último comando de movimento a cada 200 ms (keepalive)
para o watchdog do firmware não frear no meio de um degrau.

Atenção: usa a mesma serial do bridge ROS — não rodar os dois ao mesmo tempo.
"""

import argparse
import collections
import sys
import threading
import time

import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

KEEPALIVE_S = 0.2
BUF_LEN = 1500          # ~60 s a 25 Hz

FIELDS = ["t_ms", "spL", "vL", "uL", "dacL", "spR", "vR", "uR", "dacR"]


class Telemetry:
    def __init__(self):
        self.lock = threading.Lock()
        self.data = {f: collections.deque(maxlen=BUF_LEN) for f in FIELDS}

    def append(self, vals):
        with self.lock:
            for f, v in zip(FIELDS, vals):
                self.data[f].append(v)

    def snapshot(self):
        with self.lock:
            return {f: list(d) for f, d in self.data.items()}


def reader_thread(ser, tel, logf, stop):
    while not stop.is_set():
        try:
            raw = ser.readline()
        except (serial.SerialException, OSError):
            print("!! serial desconectada")
            stop.set()
            return
        line = raw.decode(errors="ignore").strip()
        if not line:
            continue
        if line.startswith("T "):
            parts = line.split()
            if len(parts) == 10:
                try:
                    vals = [float(x) for x in parts[1:]]
                except ValueError:
                    continue
                tel.append(vals)
                if logf:
                    logf.write(",".join(parts[1:]) + "\n")
        elif line.startswith("#") or line.startswith("!"):
            print(line)          # mensagens do firmware
        # linhas 'O' (odometria) são ignoradas aqui


class Commander:
    """Envia comandos e mantém keepalive do último comando de movimento."""

    def __init__(self, ser, radius):
        self.ser = ser
        self.radius = radius
        self.lock = threading.Lock()
        self.last_motion = None   # último V/W para reenvio periódico

    def send(self, cmd):
        cmd = cmd.strip()
        if not cmd:
            return
        # 'w' minúsculo: atalho em m/s -> converte para rad/s (comando 'W')
        if cmd[0] == "w":
            parts = cmd.split()
            if len(parts) == 3:
                try:
                    vl, vr = float(parts[1]), float(parts[2])
                except ValueError:
                    print("!! uso: w <vL_m/s> <vR_m/s>")
                    return
                cmd = f"W {vl / self.radius:.3f} {vr / self.radius:.3f}"
                print(f">> {cmd}")
            else:
                print("!! uso: w <vL_m/s> <vR_m/s>")
                return
        with self.lock:
            self.ser.write((cmd + "\n").encode())
        c = cmd[0].upper()
        if c in ("V", "W"):
            self.last_motion = cmd
        elif c == "S":
            self.last_motion = None

    def keepalive_loop(self, stop):
        while not stop.is_set():
            time.sleep(KEEPALIVE_S)
            cmd = self.last_motion
            if cmd:
                with self.lock:
                    try:
                        self.ser.write((cmd + "\n").encode())
                    except (serial.SerialException, OSError):
                        stop.set()
                        return


def stdin_thread(cmdr, stop):
    print(">> comandos: W rad/s | w m/s | K kp ki kd | F | M | S | G  ('q' sai)")
    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue
        if line.lower() == "q":
            stop.set()
            return
        cmdr.send(line)


def main():
    ap = argparse.ArgumentParser(description="Plot em tempo real do PI ModuBot")
    ap.add_argument("port", help="porta serial (COM5, /dev/ttyUSB0...)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--hz", type=int, default=25, help="taxa da telemetria")
    ap.add_argument("--radius", type=float, default=0.078,
                    help="raio da roda (m) — só para exibir m/s; o firmware "
                         "trabalha em rad/s e não conhece este valor")
    ap.add_argument("--rads", action="store_true",
                    help="plota em rad/s (sem conversão)")
    ap.add_argument("--log", help="arquivo CSV para gravar a telemetria")
    args = ap.parse_args()

    if args.radius <= 0.0:
        ap.error("--radius deve ser maior que zero")

    ser = serial.Serial(args.port, args.baud, timeout=0.5)
    time.sleep(0.5)

    logf = None
    if args.log:
        logf = open(args.log, "w", buffering=1)
        logf.write(",".join(FIELDS) + "\n")

    tel = Telemetry()
    stop = threading.Event()
    cmdr = Commander(ser, args.radius)

    # Conversão só de exibição: o firmware trabalha em rad/s.
    scale = 1.0 if args.rads else args.radius
    unit = "rad/s" if args.rads else "m/s"

    threading.Thread(target=reader_thread, args=(ser, tel, logf, stop), daemon=True).start()
    threading.Thread(target=cmdr.keepalive_loop, args=(stop,), daemon=True).start()
    threading.Thread(target=stdin_thread, args=(cmdr, stop), daemon=True).start()

    cmdr.send(f"P 1 {args.hz}")   # liga telemetria
    cmdr.send("G")

    fig, (axL, axR, axU) = plt.subplots(3, 1, sharex=True, figsize=(10, 8))
    fig.canvas.manager.set_window_title("ModuBot PI")

    lnSpL, = axL.plot([], [], "k--", label="setpoint L")
    lnVL,  = axL.plot([], [], "b-",  label="medido L")
    lnSpR, = axR.plot([], [], "k--", label="setpoint R")
    lnVR,  = axR.plot([], [], "r-",  label="medido R")
    lnUL,  = axU.plot([], [], "b-",  label="u L (pedido)")
    lnUR,  = axU.plot([], [], "r-",  label="u R (pedido)")
    lnDL,  = axU.plot([], [], "b:",  lw=1, label="DAC L (aplicado)")
    lnDR,  = axU.plot([], [], "r:",  lw=1, label="DAC R (aplicado)")
    # Saturacao: u encostando nestas linhas = o hardware nao entrega mais.
    for lim in (255, -255):
        axU.axhline(lim, color="gray", lw=0.8, ls="--", alpha=0.6)

    axL.set_ylabel(f"vel L ({unit})")
    axR.set_ylabel(f"vel R ({unit})")
    axU.set_ylabel("saida PI (DAC)")
    axU.set_xlabel("tempo (s)")
    for ax in (axL, axR, axU):
        ax.grid(True, alpha=0.3)
        ax.legend(loc="upper left", fontsize=8)

    def update(_):
        d = tel.snapshot()
        if not d["t_ms"]:
            return []
        t0 = d["t_ms"][0]
        t = [(x - t0) / 1000.0 for x in d["t_ms"]]
        lnSpL.set_data(t, [v * scale for v in d["spL"]])
        lnVL.set_data(t,  [v * scale for v in d["vL"]])
        lnSpR.set_data(t, [v * scale for v in d["spR"]])
        lnVR.set_data(t,  [v * scale for v in d["vR"]])
        lnUL.set_data(t, d["uL"]);    lnUR.set_data(t, d["uR"])
        lnDL.set_data(t, d["dacL"]);  lnDR.set_data(t, d["dacR"])
        for ax in (axL, axR, axU):
            ax.relim()
            ax.autoscale_view()
        if stop.is_set():
            plt.close(fig)
        return []

    ani = FuncAnimation(fig, update, interval=100, cache_frame_data=False)
    try:
        plt.show()
    finally:
        stop.set()
        try:
            cmdr.last_motion = None
            ser.write(b"S\n")
            ser.write(b"P 0\n")
            ser.close()
        except (serial.SerialException, OSError):
            pass
        if logf:
            logf.close()
        print("encerrado (freio enviado).")


if __name__ == "__main__":
    main()
