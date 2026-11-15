#!/usr/bin/env python3
"""Mostra /joint_states come li riceverà l'ESP32: gradi, ordine del firmware, valore servo.

Uso (nel container, dopo source install/setup.bash):
    python3 joint_degrees.py

servo = 90 + direzione * (angolo - 90) + subtrim   (contratto PC/ESP32, docs/progresso.md)
DIREZIONE e SUBTRIM sono i valori di partenza: si calibrano in fase 5.
"""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

FIRMWARE_ORDER = ["R1", "R2", "L1", "L2", "R4", "R3", "L3", "L4"]
DIREZIONE = {j: +1 for j in FIRMWARE_ORDER}
SUBTRIM = {j: 0.0 for j in FIRMWARE_ORDER}
LIMITI = {"R1": (45, 180), "R2": (0, 135), "L1": (0, 135), "L2": (45, 180),
          "R4": (0, 180), "R3": (0, 180), "L3": (0, 180), "L4": (0, 180)}


class JointDegrees(Node):
    def __init__(self):
        super().__init__("joint_degrees")
        self.last = None
        self.create_subscription(JointState, "/joint_states", self.cb, 10)
        self.create_timer(0.5, self.show)

    def cb(self, msg):
        self.last = dict(zip(msg.name, msg.position))

    def show(self):
        if not self.last:
            return
        righe = []
        for j in FIRMWARE_ORDER:
            if j not in self.last:
                continue
            deg = math.degrees(self.last[j])
            servo = 90 + DIREZIONE[j] * (deg - 90) + SUBTRIM[j]
            lo, hi = LIMITI[j]
            flag = "" if lo - 0.5 <= deg <= hi + 0.5 else "  <-- FUORI LIMITE"
            righe.append(f"{j}: {deg:7.2f}°  servo {servo:7.2f}{flag}")
        print("\033[2J\033[H" + "\n".join(righe), flush=True)


def main():
    rclpy.init()
    rclpy.spin(JointDegrees())


if __name__ == "__main__":
    main()
