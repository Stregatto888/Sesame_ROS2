"""Generatore di trotto per Sesame (fase 4): traiettorie dei piedi -> IK -> tabella di un ciclo.

Per ogni gamba si lavora nelle 2 coordinate che la gamba controlla davvero: x (avanti) e z (altezza)
nel frame `base`. La y è una conseguenza della geometria (l'anca ruota attorno a un asse verticale):
in appoggio il piede scorre un poco di lato, e lo misuriamo.

- Appoggio (stance): il piede va da x0 + S/2 a x0 - S/2 a quota z0 (il corpo avanza).
- Volo (swing): torna da x0 - S/2 a x0 + S/2 con un arco cicloidale alto h.
- Trotto: coppie diagonali fr+rl (fase 0) e fl+rr (fase 0.5), duty factor 0.5.

Uso:  ros2 run sesame_locomotion gait_planner [--hip 15 --knee 45 --stride 30 --lift 12 --period 1.0 --out FILE]
"""
import argparse
import csv
import math
import os

from sesame_locomotion.kinematics import (
    FIRMWARE_ORDER, KNEE_STRAIGHT_DEG, LEGS, LIMITS_DEG, STAND_DEG, LegKinematics)

PHASE = {'fr_leg': 0.0, 'rl_leg': 0.0, 'fl_leg': 0.5, 'rr_leg': 0.5}
SERVO_MAX_DEG_S = math.degrees(3.0)   # joint_limits.yaml: 3.0 rad/s ≈ 172 °/s
TICK_MAX_DEG = 12.0                   # passo massimo per tick previsto nel firmware


def default_urdf():
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory('sesame_description'), 'urdf', 'sesame.urdf')
    except Exception:
        return os.path.join(os.path.dirname(__file__), '..', '..', 'sesame_description', 'urdf', 'sesame.urdf')


def start_pose_deg(knee_dev, hip_dev=45.0):
    """Posa di partenza. hip_dev: angolo della gamba dal laterale (90°): 45 = stand, 0 = tutta di lato,
    90 = in asse col corpo. knee_dev: piega del ginocchio rispetto alla gamba distesa."""
    q = dict(STAND_DEG)
    q.update(R1=90 + hip_dev, R2=90 - hip_dev, L1=90 - hip_dev, L2=90 + hip_dev)
    for j, straight in KNEE_STRAIGHT_DEG.items():
        q[j] = straight - knee_dev if straight == 180 else straight + knee_dev
    return q


def foot_target(phase, x0, z0, stride, lift):
    """Posizione (x, z) del piede alla fase [0, 1) del suo ciclo. Prima metà appoggio, poi volo."""
    if phase < 0.5:
        s = phase / 0.5
        return x0 + stride / 2 - stride * s, z0
    s = (phase - 0.5) / 0.5
    x = x0 - stride / 2 + stride * (s - math.sin(2 * math.pi * s) / (2 * math.pi))
    z = z0 + lift * (1 - math.cos(2 * math.pi * s)) / 2
    return x, z


def plan(kin, knee_dev=45.0, hip_dev=15.0, stride=0.030, lift=0.012, period=1.0, rate=50):
    q0 = start_pose_deg(knee_dev, hip_dev)
    n = int(round(period * rate))
    neutral = {g: kin.foot_deg(g, q0[h], q0[k]) for g, (h, k, _) in LEGS.items()}
    seed = {g: (q0[h], q0[k]) for g, (h, k, _) in LEGS.items()}
    rows, y_slip = [], {g: [] for g in LEGS}
    for i in range(n):
        q = {}
        for g, (h, k, _) in LEGS.items():
            ph = (i / n + PHASE[g]) % 1.0
            x, z = foot_target(ph, neutral[g][0], neutral[g][2], stride, lift)
            sol = kin.ik_xz(g, x, z, seed[g])
            if sol is None:
                raise RuntimeError(f'IK fallita: {g} fase {ph:.2f} x={x * 1000:.1f} z={z * 1000:.1f} mm')
            seed[g] = sol
            q[h], q[k] = sol
            if ph < 0.5:
                y_slip[g].append(kin.foot_deg(g, *sol)[1])
        rows.append(q)
    return q0, neutral, rows, y_slip


def check(rows, rate, margin=0.0):
    """Limiti, velocità massima e passo massimo per tick (ciclo chiuso: l'ultimo torna al primo)."""
    report, ok = {}, True
    for j in FIRMWARE_ORDER:
        vals = [r[j] for r in rows]
        lo, hi = LIMITS_DEG[j]
        steps = [abs(vals[(i + 1) % len(vals)] - vals[i]) for i in range(len(vals))]
        inside = min(vals) >= lo + margin and max(vals) <= hi - margin
        vmax = max(steps) * rate
        ok &= inside and vmax <= SERVO_MAX_DEG_S and max(steps) <= TICK_MAX_DEG
        report[j] = (min(vals), max(vals), inside, max(steps), vmax)
    return ok, report


def main():
    ap = argparse.ArgumentParser(description='Pianifica un ciclo di trotto e lo verifica.')
    ap.add_argument('--knee', type=float, default=45.0, help='piega del ginocchio a riposo (°)')
    ap.add_argument('--hip', type=float, default=15.0,
                    help='angolo della gamba dal laterale a riposo (°; stand = 45)')
    ap.add_argument('--stride', type=float, default=30.0, help='lunghezza del passo (mm)')
    ap.add_argument('--lift', type=float, default=12.0, help='altezza del volo (mm)')
    ap.add_argument('--period', type=float, default=1.0, help='durata del ciclo (s)')
    ap.add_argument('--rate', type=int, default=50, help='campioni al secondo (Hz)')
    ap.add_argument('--urdf', default=default_urdf())
    ap.add_argument('--out', default='/tmp/trot_cycle.csv')
    a, _ = ap.parse_known_args()   # ignora eventuali --ros-args

    kin = LegKinematics(a.urdf)
    q0, neutral, rows, y_slip = plan(kin, a.knee, a.hip, a.stride / 1000, a.lift / 1000, a.period, a.rate)
    ok, rep = check(rows, a.rate)

    print(f'Trotto: anca {a.hip}° dal laterale, ginocchio {a.knee}°, passo {a.stride} mm, volo {a.lift} mm, ciclo {a.period} s, '
          f'{len(rows)} campioni a {a.rate} Hz -> velocità {a.stride / 1000 / (a.period / 2) * 1000:.0f} mm/s')
    print('Posa di partenza (°): ' + ', '.join(f'{j} {q0[j]:.0f}' for j in FIRMWARE_ORDER))
    print('Piedi a riposo (mm):  ' + ', '.join(
        f'{g} ({p[0] * 1000:.1f}, {p[1] * 1000:.1f}, {p[2] * 1000:.1f})' for g, p in neutral.items()))
    print(f'{"giunto":6} {"min °":>7} {"max °":>7} {"limiti":>9} {"passo max °":>12} {"vel max °/s":>12}')
    for j, (mn, mx, inside, smax, vmax) in rep.items():
        lo, hi = LIMITS_DEG[j]
        print(f'{j:6} {mn:7.1f} {mx:7.1f} {lo:4}-{hi:<4}{"" if inside else "!"} {smax:12.2f} {vmax:12.1f}')
    for g, ys in y_slip.items():
        print(f'slittamento laterale in appoggio {g}: {(max(ys) - min(ys)) * 1000:.1f} mm')
    print(f'Soglie: limiti URDF, velocità <= {SERVO_MAX_DEG_S:.0f} °/s, passo per tick <= {TICK_MAX_DEG:.0f}°')
    print('ESITO: SUPERATO' if ok else 'ESITO: NON SUPERATO')

    with open(a.out, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t_s'] + FIRMWARE_ORDER)
        for i, r in enumerate(rows):
            w.writerow([f'{i / a.rate:.3f}'] + [f'{r[j]:.2f}' for j in FIRMWARE_ORDER])
    print(f'Tabella (ordine firmware, gradi): {a.out}')
    raise SystemExit(0 if ok else 1)


if __name__ == '__main__':
    main()
