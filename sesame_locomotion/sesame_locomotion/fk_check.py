"""Confronta la FK locale (kinematics.py, letta dall'URDF) con /compute_fk di MoveIt.

Serve a legare il generatore di andatura alla catena verificata in fase 3: errore atteso ~0 mm.
Richiede move_group attivo. Uso:  ros2 run sesame_locomotion fk_check --ros-args -p samples:=200
"""
import math
import random

import rclpy

from sesame_locomotion.gait import default_urdf
from sesame_locomotion.ik_roundtrip import IkRoundtrip, robot_state
from sesame_locomotion.kinematics import LEGS, LIMITS_DEG, LegKinematics


def main():
    rclpy.init()
    node = IkRoundtrip()          # riusa client, parametri (samples, tolerance_mm, seed) e foot_pos
    node.get_logger().info('Confronto FK locale / compute_fk')
    kin = LegKinematics(default_urdf())
    worst, ok = 0.0, True
    try:
        for g, (hip, knee, link) in LEGS.items():
            errs = []
            for _ in range(node.samples):
                q = {j: random.uniform(*LIMITS_DEG[j]) for j in (hip, knee)}
                ros = node.foot_pos(robot_state(q), link)
                loc = kin.foot_deg(g, q[hip], q[knee])
                errs.append(1000.0 * math.dist(ros, loc))
            worst = max(worst, max(errs))
            ok &= max(errs) < node.tol_mm
            node.get_logger().info(f'{g}: {len(errs)} campioni, errore max {max(errs):.4f} mm')
        node.get_logger().info(('ESITO: SUPERATO' if ok else 'ESITO: NON SUPERATO') + f' (max {worst:.4f} mm)')
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(0 if ok else 1)


if __name__ == '__main__':
    main()
