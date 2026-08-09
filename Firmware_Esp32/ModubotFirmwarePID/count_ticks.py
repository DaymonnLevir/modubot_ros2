#!/usr/bin/env python3
"""
Calibração de ticks_per_rev — conta os pulsos de N voltas dadas à mão.

É o PRIMEIRO passo de tudo: sem ticks_per_rev correto, a velocidade medida
(e portanto todo o PI) está com escala errada.

Procedimento:
  1. Robô desligado do driver ou com freio acionado (o script envia 'S').
  2. Marque um ponto de referência na roda e no chassi.
  3. Rode o script, gire UMA roda de cada vez, exatamente N voltas completas,
     devagar e sempre no mesmo sentido.
  4. Ctrl+C -> o script mostra ticks/volta de cada lado.

Uso:
    python count_ticks.py COM5 --turns 10
    python count_ticks.py /dev/ttyUSB0 --turns 10

Repita 2-3 vezes: o valor deve ser estável e próximo de um inteiro. Se variar
muito, há ruído de borda (revise o conversor 5V->3V3 e o pull-up externo).
"""

import argparse
import sys
import time

import serial


def main():
    ap = argparse.ArgumentParser(description="Conta ticks por volta")
    ap.add_argument("port", help="porta serial (COM5, /dev/ttyUSB0...)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--turns", type=float, default=10.0,
                    help="numero de voltas que voce vai girar (padrao 10)")
    args = ap.parse_args()

    if args.turns <= 0:
        ap.error("--turns deve ser maior que zero")

    ser = serial.Serial(args.port, args.baud, timeout=0.5)
    time.sleep(0.5)

    ser.write(b"S\n")      # freio: garante que nada seja acionado
    ser.write(b"P 0\n")    # telemetria off: só queremos as linhas 'O'
    ser.reset_input_buffer()

    totL = 0
    totR = 0
    print(f"Gire {args.turns:g} voltas em cada roda. Ctrl+C para finalizar.\n")

    try:
        while True:
            raw = ser.readline()
            if not raw:
                continue
            line = raw.decode(errors="ignore").strip()
            if not line.startswith("O "):
                continue
            parts = line.split()
            if len(parts) != 4:
                continue
            try:
                totL += int(parts[1])
                totR += int(parts[2])
            except ValueError:
                continue
            print(f"\r  ticks  L={totL:8d}   R={totR:8d}", end="", flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            ser.write(b"S\n")
            ser.close()
        except (serial.SerialException, OSError):
            pass

    print("\n")
    print(f"Total   L = {totL} ticks   R = {totR} ticks   em {args.turns:g} voltas")
    if totL:
        print(f"  ticks_per_rev ESQUERDA = {abs(totL) / args.turns:.2f}")
    if totR:
        print(f"  ticks_per_rev DIREITA  = {abs(totR) / args.turns:.2f}")
    print("\nArredonde para o inteiro mais proximo e atualize")
    print("modubot_serial_bridge/config/modubot_params.yaml.")


if __name__ == "__main__":
    main()
