"""Test di andata e ritorno della IK (fase 3).

Per ogni gamba: angoli casuali nei limiti -> FK (/compute_fk) -> IK (/compute_ik, seme = crouch)
-> FK della soluzione -> errore di posizione del piede. Obiettivo: errore < 1 mm.

Richiede move_group attivo:  ros2 launch sesame_bringup moveit.launch.py
Uso:  ros2 run sesame_locomotion ik_roundtrip --ros-args -p samples:=200
"""
import csv
import math
import random

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from sensor_msgs.msg import JointState

D = math.radians

# Gruppo -> (anca, ginocchio, link del piede)
LEGS = {
    'fr_leg': ('R1', 'R3', 'r3_foot'),
    'rr_leg': ('R2', 'R4', 'r4_foot'),
    'fl_leg': ('L1', 'L3', 'l3_foot'),
    'rl_leg': ('L2', 'L4', 'l4_foot'),
}

# Limiti del contratto PC/ESP32 (gradi)
LIMITS_DEG = {
    'R1': (45, 180), 'R2': (0, 135), 'L1': (0, 135), 'L2': (45, 180),
    'R3': (0, 180), 'R4': (0, 180), 'L3': (0, 180), 'L4': (0, 180),
}

STAND_DEG = {'R1': 135, 'R2': 45, 'L1': 45, 'L2': 135, 'R4': 0, 'R3': 180, 'L3': 0, 'L4': 180}
CROUCH_DEG = dict(STAND_DEG, R3=150, R4=30, L3=30, L4=150)

BASE_FRAME = 'base'
OK = 1  # MoveItErrorCodes.SUCCESS


def robot_state(deg_overrides=None, rad_overrides=None):
    """RobotState con tutti gli 8 giunti: crouch come base, poi le sostituzioni."""
    pos = {j: D(v) for j, v in CROUCH_DEG.items()}
    for j, v in (deg_overrides or {}).items():
        pos[j] = D(v)
    pos.update(rad_overrides or {})
    js = JointState(name=list(pos), position=list(pos.values()))
    return RobotState(joint_state=js)


class IkRoundtrip(Node):
    def __init__(self):
        super().__init__('ik_roundtrip')
        self.samples = self.declare_parameter('samples', 200).value
        self.margin = self.declare_parameter('margin_deg', 5.0).value
        self.tol_mm = self.declare_parameter('tolerance_mm', 1.0).value
        self.collisions = self.declare_parameter('avoid_collisions', False).value
        self.csv_path = self.declare_parameter('csv', '/tmp/ik_roundtrip.csv').value
        random.seed(self.declare_parameter('seed', 42).value)

        self.fk = self.create_client(GetPositionFK, 'compute_fk')
        self.ik = self.create_client(GetPositionIK, 'compute_ik')
        for c in (self.fk, self.ik):
            if not c.wait_for_service(timeout_sec=10.0):
                raise RuntimeError(f'servizio {c.srv_name} assente: move_group è attivo?')

    def call(self, client, req):
        fut = client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        return fut.result()

    def foot_pos(self, state, link):
        req = GetPositionFK.Request()
        req.header.frame_id = BASE_FRAME
        req.fk_link_names = [link]
        req.robot_state = state
        res = self.call(self.fk, req)
        if res is None or res.error_code.val != OK:
            return None
        p = res.pose_stamped[0].pose.position
        return (p.x, p.y, p.z)

    def solve_ik(self, group, link, target):
        req = GetPositionIK.Request()
        r = req.ik_request
        r.group_name = group
        r.ik_link_name = link
        r.robot_state = robot_state()          # seme: crouch, non la soluzione vera
        r.avoid_collisions = self.collisions
        r.timeout = Duration(sec=0, nanosec=100_000_000)
        r.pose_stamped.header.frame_id = BASE_FRAME
        r.pose_stamped.pose.position.x, r.pose_stamped.pose.position.y, \
            r.pose_stamped.pose.position.z = target
        r.pose_stamped.pose.orientation.w = 1.0  # ignorata (rotation_scale 0)
        res = self.call(self.ik, req)
        if res is None or res.error_code.val != OK:
            return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return sol

    def run(self):
        rows, summary = [], []
        for group, (hip, knee, link) in LEGS.items():
            errs, fails = [], 0
            for i in range(self.samples):
                q = {j: random.uniform(LIMITS_DEG[j][0] + self.margin, LIMITS_DEG[j][1] - self.margin)
                     for j in (hip, knee)}
                target = self.foot_pos(robot_state(q), link)
                if target is None:
                    self.get_logger().error(f'{group}: FK fallita')
                    continue
                sol = self.solve_ik(group, link, target)
                if sol is None:
                    fails += 1
                    rows.append([group, i, q[hip], q[knee], *target, '', '', 'IK_FAIL'])
                    continue
                got = self.foot_pos(robot_state(rad_overrides={hip: sol[hip], knee: sol[knee]}), link)
                if got is None:
                    fails += 1
                    continue
                err_mm = 1000.0 * math.dist(target, got)
                errs.append(err_mm)
                rows.append([group, i, q[hip], q[knee], *target,
                             math.degrees(sol[hip]), math.degrees(sol[knee]), f'{err_mm:.4f}'])
            n_ok = sum(e < self.tol_mm for e in errs)
            summary.append((group, self.samples, fails, n_ok,
                            max(errs, default=float('nan')),
                            sum(errs) / len(errs) if errs else float('nan')))

        with open(self.csv_path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['group', 'i', 'hip_deg', 'knee_deg', 'x', 'y', 'z',
                        'ik_hip_deg', 'ik_knee_deg', 'err_mm'])
            w.writerows(rows)

        log = self.get_logger()
        log.info(f'Risultati (tolleranza {self.tol_mm} mm, margine {self.margin}°, CSV {self.csv_path})')
        log.info(f'{"gruppo":8} {"campioni":>8} {"IK fallite":>10} {"< toll.":>8} {"max mm":>8} {"medio mm":>9}')
        all_ok = True
        for g, n, fails, n_ok, emax, emean in summary:
            log.info(f'{g:8} {n:8d} {fails:10d} {n_ok:8d} {emax:8.4f} {emean:9.4f}')
            all_ok &= (fails == 0 and n_ok == n)
        log.info('ESITO: SUPERATO' if all_ok else 'ESITO: NON SUPERATO (vedi CSV)')
        return all_ok


def main():
    rclpy.init()
    node = IkRoundtrip()
    try:
        ok = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(0 if ok else 1)


if __name__ == '__main__':
    main()
