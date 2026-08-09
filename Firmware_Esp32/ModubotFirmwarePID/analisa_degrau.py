#!/usr/bin/env python3
"""
Extrai as métricas de resposta ao degrau do CSV gravado pelo plot_pid.py.

Calcula, para cada degrau de setpoint e cada roda:
    overshoot (%)      pico além do valor final, normalizado pelo tamanho do degrau
    t_subida (s)       10% -> 90% do degrau
    t_acomodacao (s)   instante após o qual |erro| fica dentro da faixa (padrão ±5%)
    e_regime           setpoint - valor final  (erro de regime permanente)
    u_max / saturou    pico da saída do controlador e se bateu em 255

Uso:
    python analisa_degrau.py ensaio1.csv
    python analisa_degrau.py ensaio1.csv --radius 0.078      # tabela em m/s
    python analisa_degrau.py ensaio1.csv --band 0.02         # faixa de ±2%

Os números saem prontos para a tabela comparativa malha aberta x PI + feedforward.
"""

import argparse
import csv


def load(path):
    rows = []
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            try:
                rows.append({k: float(v) for k, v in r.items()})
            except (ValueError, TypeError):
                continue
    return rows


def find_steps(t, sp, min_delta):
    """Índices onde o setpoint muda mais que min_delta."""
    steps = []
    for i in range(1, len(sp)):
        if abs(sp[i] - sp[i - 1]) >= min_delta:
            steps.append(i)
    return steps


def analyse(t, sp, v, u, dac, i0, i1, band):
    """Métricas de um degrau na janela [i0, i1)."""
    delta = sp[i0] - sp[i0 - 1] if i0 > 0 else sp[i0]
    if abs(delta) < 1e-6:
        return None

    v0 = v[i0 - 1] if i0 > 0 else v[i0]
    seg_t, seg_v = t[i0:i1], v[i0:i1]
    if len(seg_v) < 5:
        return None

    # valor final: média dos últimos 20% da janela
    tail = max(2, len(seg_v) // 5)
    vf = sum(seg_v[-tail:]) / tail
    t0 = seg_t[0]

    # --- overshoot: pico além de vf, no sentido do degrau ---
    if delta > 0:
        peak = max(seg_v)
        over = (peak - vf) / abs(delta) * 100.0
    else:
        peak = min(seg_v)
        over = (vf - peak) / abs(delta) * 100.0
    over = max(over, 0.0)

    # --- tempo de subida 10% -> 90% ---
    lo = v0 + 0.10 * (vf - v0)
    hi = v0 + 0.90 * (vf - v0)

    def cross(level):
        for tt, vv in zip(seg_t, seg_v):
            if (delta > 0 and vv >= level) or (delta < 0 and vv <= level):
                return tt
        return None

    t_lo, t_hi = cross(lo), cross(hi)
    t_rise = (t_hi - t_lo) if (t_lo is not None and t_hi is not None) else None

    # --- tempo de acomodação: último instante fora da faixa ---
    tol = band * abs(delta)
    t_settle = None
    for tt, vv in zip(seg_t, seg_v):
        if abs(vv - vf) > tol:
            t_settle = tt
    t_settle = (t_settle - t0) if t_settle is not None else 0.0

    seg_u = u[i0:i1]
    u_peak = max(seg_u, key=abs)
    saturou = any(abs(x) >= 254.0 for x in seg_u)
    # rampa limitando: |u| pedido bem acima do DAC efetivamente aplicado
    seg_d = dac[i0:i1]
    rampa = any(abs(a) - abs(b) > 20.0 for a, b in zip(seg_u, seg_d))

    return {
        "t": t0,
        "sp": sp[i0],
        "delta": delta,
        "vf": vf,
        "over": over,
        "t_rise": t_rise,
        "t_settle": t_settle,
        "e_ss": sp[i0] - vf,
        "u_peak": u_peak,
        "sat": saturou,
        "rampa": rampa,
    }


def report(name, rows, band, scale, unit):
    t = [r["t_ms"] / 1000.0 for r in rows]
    t = [x - t[0] for x in t]
    sp = [r[f"sp{name}"] for r in rows]
    v = [r[f"v{name}"] for r in rows]
    u = [r[f"u{name}"] for r in rows]
    dac = [r[f"dac{name}"] for r in rows]

    span = max(max(sp) - min(sp), 1e-6)
    steps = find_steps(t, sp, min_delta=0.15 * span)
    if not steps:
        print(f"  roda {name}: nenhum degrau detectado")
        return

    bounds = steps + [len(sp)]
    print(f"\n  RODA {name}")
    print(f"  {'t(s)':>6} {'setpoint':>10} {'final':>9} {'over%':>7} "
          f"{'t_sub':>7} {'t_acom':>7} {'e_reg':>9}  obs")
    print("  " + "-" * 76)

    for k, i0 in enumerate(steps):
        m = analyse(t, sp, v, u, dac, i0, bounds[k + 1], band)
        if m is None:
            continue
        obs = []
        if m["sat"]:
            obs.append("SATUROU")
        if m["rampa"]:
            obs.append("rampa limita")
        rise = f"{m['t_rise']:.3f}" if m["t_rise"] is not None else "  -  "
        print(f"  {m['t']:6.1f} {m['sp'] * scale:10.3f} {m['vf'] * scale:9.3f} "
              f"{m['over']:7.1f} {rise:>7} {m['t_settle']:7.3f} "
              f"{m['e_ss'] * scale:9.4f}  {' '.join(obs)}")
    print(f"  (velocidades em {unit}; faixa de acomodacao ±{band * 100:.0f}% do degrau)")


def main():
    ap = argparse.ArgumentParser(description="Metricas de degrau do PI")
    ap.add_argument("csv", help="arquivo gravado com plot_pid.py --log")
    ap.add_argument("--band", type=float, default=0.05,
                    help="faixa de acomodacao (0.05 = ±5%%)")
    ap.add_argument("--radius", type=float,
                    help="raio da roda (m): converte rad/s -> m/s na tabela")
    args = ap.parse_args()

    rows = load(args.csv)
    if not rows:
        print("CSV vazio ou ilegivel.")
        return

    scale = args.radius if args.radius else 1.0
    unit = "m/s" if args.radius else "rad/s"

    print(f"\n=== {args.csv} — {len(rows)} amostras ===")
    for name in ("L", "R"):
        report(name, rows, args.band, scale, unit)
    print()


if __name__ == "__main__":
    main()
