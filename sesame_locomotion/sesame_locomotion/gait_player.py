"""Riproduce la tabella di un ciclo di andatura sui 4 joint_trajectory_controller.

Prima porta il robot nella prima riga della tabella in `approach_s` secondi, poi ripete il ciclo
`cycles` volte. Basta control.launch.py (MoveIt non serve).

Uso:  ros2 run sesame_locomotion gait_player --ros-args -p table:=/tmp/trot_cycle.csv -p cycles:=10
"""
import csv
import math

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from sesame_locomotion.kinematics import LEGS


def dur(t):
    sec = int(t)
    return Duration(sec=sec, nanosec=int(round((t - sec) * 1e9)))


class GaitPlayer(Node):
    def __init__(self):
        super().__init__('gait_player')
        self.table = self.declare_parameter('table', '/tmp/trot_cycle.csv').value
        self.cycles = self.declare_parameter('cycles', 10).value
        self.approach = self.declare_parameter('approach_s', 2.0).value
        self.pubs = {g: self.create_publisher(JointTrajectory, f'/{g}_controller/joint_trajectory', 10)
                     for g in LEGS}

    def load(self):
        with open(self.table) as f:
            rows = list(csv.DictReader(f))
        times = [float(r['t_s']) for r in rows]
        dt = times[1] - times[0]
        return [{j: math.radians(float(v)) for j, v in r.items() if j != 't_s'} for r in rows], dt

    def send(self):
        rows, dt = self.load()
        period = len(rows) * dt
        for g, (hip, knee, _) in LEGS.items():
            msg = JointTrajectory(joint_names=[hip, knee])
            msg.points.append(JointTrajectoryPoint(
                positions=[rows[0][hip], rows[0][knee]], time_from_start=dur(self.approach)))
            for c in range(self.cycles):
                for i, r in enumerate(rows):
                    if c == 0 and i == 0:
                        continue
                    t = self.approach + c * period + i * dt
                    msg.points.append(JointTrajectoryPoint(positions=[r[hip], r[knee]], time_from_start=dur(t)))
            # chiude l'ultimo ciclo tornando alla prima riga
            t = self.approach + self.cycles * period
            msg.points.append(JointTrajectoryPoint(positions=[rows[0][hip], rows[0][knee]], time_from_start=dur(t)))
            self.pubs[g].publish(msg)
        total = self.approach + self.cycles * period
        self.get_logger().info(f'{len(rows)} campioni per ciclo ({period:.2f} s), {self.cycles} cicli, '
                               f'durata {total:.1f} s')


def main():
    rclpy.init()
    node = GaitPlayer()
    # attende che i controllori siano iscritti ai topic
    for _ in range(50):
        if all(p.get_subscription_count() > 0 for p in node.pubs.values()):
            break
        rclpy.spin_once(node, timeout_sec=0.1)
    else:
        node.get_logger().error('controllori non trovati: control.launch.py è attivo?')
    node.send()
    rclpy.spin_once(node, timeout_sec=0.5)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
