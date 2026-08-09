#!/usr/bin/env python3
"""
Estima a constante de tempo efetiva (tau) a partir dos dados brutos das
campanhas de calibração (pasta samples/ de cada campanha).

MÉTODO — por que distância acumulada e não velocidade instantânea:

A velocidade por janela tem quantização grossa (1 tick / 50 ms = 0,108 m/s),
então ajustar uma exponencial na velocidade é ajustar ruído. Já a distância
acumulada usa a contagem inteira de ticks, que é exata.

Para uma resposta de 1a ordem  v(t) = v_f · (1 - e^(-t/tau)),  a distância é
    s(t) = v_f · (t - tau · (1 - e^(-t/tau)))
e para t >> tau isso vira a reta
    s(t) ≈ v_f · t - v_f · tau
cuja extensão cruza s = 0 em  t = tau.

Ou seja: ajusta-se uma reta ao trecho de regime da distância acumulada e o
ponto onde ela cruza zero no eixo do tempo é o tau efetivo. A inclinação da
mesma reta dá a velocidade de regime v_f, de graça.

"Efetivo" porque esse tau engloba tudo que atrasa a partida: constante
mecânica, atraso de transporte e eventual rampa interna do driver. Para
sintonia IMC é exatamente o número que interessa.

Uso:
    python estima_tau.py <pasta_da_campanha> [--radius 0.078] [--ticks 91]
    python estima_tau.py .../forward_new_wheel_20260806_231056
"""

import argparse
import csv
import math
import os
import statistics as st


def fit_line(pts):
    n = len(pts)
    sx = sum(x for x, _ in pts)
    sy = sum(y for _, y in pts)
    sxy = sum(x * y for x, y in pts)
    sxx = sum(x * x for x, _ in pts)
    den = n * sxx - sx * sx
    if abs(den) < 1e-12:
        return None, None
    a = (n * sxy - sx * sy) / den
    b = (sy - a * sx) / n
    return a, b


def analyse_run(path, radius, ticks_rev, t_lo, t_hi):
    """Retorna (v_f, tau) por roda para um arquivo de run."""
    per_rev = 2.0 * math.pi * radius / ticks_rev   # metros por tick
    tL = []
    cumL = 0
    tR = []
    cumR = 0
    dac = None
    for r in csv.DictReader(open(path)):
        if r["phase"] != "command":
            continue
        if dac is None:
            dac = int(r["dac_left"])
        t = float(r["elapsed_phase_s"])
        cumL += abs(int(r["delta_ticks_left"]))
        cumR += abs(int(r["delta_ticks_right"]))
        tL.append((t, cumL * per_rev))
        tR.append((t, cumR * per_rev))

    out = {}
    for name, series in (("L", tL), ("R", tR)):
        seg = [(t, s) for t, s in series if t_lo <= t <= t_hi]
        if len(seg) < 10:
            out[name] = (None, None)
            continue
        a, b = fit_line(seg)
        if not a or a <= 1e-6:
            out[name] = (None, None)
            continue
        out[name] = (a, -b / a)      # (v_f m/s, tau s)
    return dac, out


def main():
    ap = argparse.ArgumentParser(description="Estima tau das campanhas")
    ap.add_argument("campanha", help="pasta da campanha (contem samples/)")
    ap.add_argument("--radius", type=float, default=0.078)
    ap.add_argument("--ticks", type=float, default=91.0)
    ap.add_argument("--t-lo", type=float, default=2.0,
                    help="inicio do trecho de regime (s)")
    ap.add_argument("--t-hi", type=float, default=5.5,
                    help="fim do trecho de regime (s)")
    args = ap.parse_args()

    sdir = os.path.join(args.campanha, "samples")
    if not os.path.isdir(sdir):
        print(f"nao encontrei {sdir}")
        return

    por_dac = {}
    for fn in sorted(os.listdir(sdir)):
        if not fn.endswith(".csv"):
            continue
        try:
            dac, res = analyse_run(os.path.join(sdir, fn), args.radius,
                                   args.ticks, args.t_lo, args.t_hi)
        except (ValueError, KeyError, OSError):
            continue
        if dac is None:
            continue
        d = por_dac.setdefault(dac, {"L": [], "R": [], "vL": [], "vR": []})
        for name in ("L", "R"):
            v, tau = res[name]
            if v is not None and tau is not None:
                d[name].append(tau)
                d["v" + name].append(v)

    print(f"\n=== {os.path.basename(args.campanha.rstrip(os.sep))} ===")
    print(f"raio={args.radius} m  ticks/volta={args.ticks:g}  "
          f"regime={args.t_lo}-{args.t_hi}s\n")
    print(f"{'DAC':>5} {'n':>4} {'v_f L':>8} {'w_f L':>8} "
          f"{'tau L':>8} {'dp':>6} {'tau R':>8} {'dp':>6}")
    print("-" * 62)

    taus_altos = []
    for dac in sorted(por_dac):
        d = por_dac[dac]
        if not d["L"] or not d["R"]:
            continue
        vL = st.mean(d["vL"])
        tl, tr = st.mean(d["L"]), st.mean(d["R"])
        sl = st.pstdev(d["L"]) if len(d["L"]) > 1 else 0.0
        sr = st.pstdev(d["R"]) if len(d["R"]) > 1 else 0.0
        print(f"{dac:5d} {len(d['L']):4d} {vL:8.4f} {vL / args.radius:8.3f} "
              f"{tl:8.3f} {sl:6.3f} {tr:8.3f} {sr:6.3f}")
        if dac >= 26:
            taus_altos.extend(d["L"] + d["R"])

    if taus_altos:
        m = st.mean(taus_altos)
        print(f"\ntau efetivo (DAC>=26, {len(taus_altos)} amostras): "
              f"MEDIA {m:.3f} s   MEDIANA {st.median(taus_altos):.3f} s")
        print(f"\nCom lambda = tau/2, os ganhos IMC ficam:")
        print(f"   ki = 2/(a*tau)  ->  use tau = {m:.3f} s")
    print()


if __name__ == "__main__":
    main()
