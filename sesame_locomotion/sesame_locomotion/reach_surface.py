"""Superficie raggiungibile dei 4 piedi (fase 3).

Griglia regolare di (anca, ginocchio) nei limiti -> /compute_fk -> posizione del piede nel frame `base`.
Ogni chiamata calcola tutti e 4 i piedi insieme. Scrive un CSV per gamba e stampa un riepilogo.

Richiede move_group attivo:  ros2 launch sesame_bringup moveit.launch.py
Uso:  ros2 run sesame_locomotion reach_surface --ros-args -p step_deg:=3.0
"""
import csv
import math

import rclpy
from rclpy.node import Node
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetPositionFK
from sensor_msgs.msg import JointState

from sesame_locomotion.ik_roundtrip import CROUCH_DEG, LEGS, LIMITS_DEG, STAND_DEG

BASE_FRAME = 'base'
OK = 1


def lerp(lo, hi, t):
    return lo + (hi - lo) * t


class ReachSurface(Node):
    def __init__(self):
        super().__init__('reach_surface')
        self.step = self.declare_parameter('step_deg', 3.0).value
        self.prefix = self.declare_parameter('csv_prefix', '/tmp/reach_').value
        self.fk = self.create_client(GetPositionFK, 'compute_fk')
        if not self.fk.wait_for_service(timeout_sec=10.0):
            raise RuntimeError('servizio compute_fk assente: move_group è attivo?')

    def feet(self, deg):
        """deg: gradi di tutti gli 8 giunti -> {link: (x, y, z)} in metri."""
        req = GetPositionFK.Request()
        req.header.frame_id = BASE_FRAME
        req.fk_link_names = [leg[2] for leg in LEGS.values()]
        req.robot_state = RobotState(joint_state=JointState(
            name=list(deg), position=[math.radians(v) for v in deg.values()]))
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        res = fut.result()
        if res is None or res.error_code.val != OK:
            return None
        return {n: (p.pose.position.x, p.pose.position.y, p.pose.position.z)
                for n, p in zip(res.fk_link_names, res.pose_stamped)}

    def run(self):
        # Stesso numero di passi per anca e ginocchio; t in [0, 1] mappato sui limiti di ogni giunto.
        n = int(round(180.0 / self.step))
        ts = [i / n for i in range(n + 1)]
        data = {g: [] for g in LEGS}
        for th in ts:
            for tk in ts:
                deg = {}
                for hip, knee, _ in LEGS.values():
                    deg[hip] = lerp(*LIMITS_DEG[hip], th)
                    deg[knee] = lerp(*LIMITS_DEG[knee], tk)
                pos = self.feet(deg)
                if pos is None:
                    self.get_logger().error('FK fallita')
                    continue
                for g, (hip, knee, link) in LEGS.items():
                    data[g].append((deg[hip], deg[knee], *pos[link]))

        stand = self.feet(STAND_DEG)
        crouch = self.feet(CROUCH_DEG)
        log = self.get_logger()
        log.info(f'Griglia {n + 1}x{n + 1} per gamba (passo ~{self.step}°), frame {BASE_FRAME}, mm')
        for g, (hip, knee, link) in LEGS.items():
            rows = data[g]
            with open(f'{self.prefix}{g}.csv', 'w', newline='') as f:
                w = csv.writer(f)
                w.writerow(['hip_deg', 'knee_deg', 'x', 'y', 'z'])
                w.writerows(rows)
            mm = lambda k: [1000.0 * r[k] for r in rows]  # noqa: E731
            xs, ys, zs = mm(2), mm(3), mm(4)
            c = [1000.0 * v for v in crouch[link]]
            s = [1000.0 * v for v in stand[link]]
            # Corsa verticale disponibile tenendo l'anca della posa crouch (utile per l'andatura).
            hip_c = CROUCH_DEG[hip]
            col = [1000.0 * r[4] for r in rows if abs(r[0] - hip_c) <= self.step / 2]
            log.info(f'{g} ({link}): x [{min(xs):.1f}, {max(xs):.1f}]  y [{min(ys):.1f}, {max(ys):.1f}]  '
                     f'z [{min(zs):.1f}, {max(zs):.1f}]')
            log.info(f'    stand  ({s[0]:.1f}, {s[1]:.1f}, {s[2]:.1f})   '
                     f'crouch ({c[0]:.1f}, {c[1]:.1f}, {c[2]:.1f})')
            if col:
                log.info(f'    anca a {hip_c}°: z del piede [{min(col):.1f}, {max(col):.1f}]')
        log.info(f'CSV: {self.prefix}<gruppo>.csv')


def main():
    rclpy.init()
    node = ReachSurface()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
