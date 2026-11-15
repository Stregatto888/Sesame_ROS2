"""Cinematica delle gambe di Sesame letta dall'URDF (senza ROS, solo numpy).

- FK generica: catena di giunti `base` -> piede presa da `sesame.urdf` (origin xyz/rpy, axis).
- IK nello spazio controllabile della gamba (2 giunti -> 2 coordinate): dato (x, z) del piede nel frame
  `base`, trova (anca, ginocchio); la y è una conseguenza. Newton con jacobiano numerico.

La FK è confrontata con /compute_fk di MoveIt da `fk_check` (deve dare errore ~0).
"""
import math
import xml.etree.ElementTree as ET

import numpy as np

# Gruppo -> (anca, ginocchio, link del piede)
LEGS = {
    'fr_leg': ('R1', 'R3', 'r3_foot'),
    'rr_leg': ('R2', 'R4', 'r4_foot'),
    'fl_leg': ('L1', 'L3', 'l3_foot'),
    'rl_leg': ('L2', 'L4', 'l4_foot'),
}
FIRMWARE_ORDER = ['R1', 'R2', 'L1', 'L2', 'R4', 'R3', 'L3', 'L4']
LIMITS_DEG = {
    'R1': (45, 180), 'R2': (0, 135), 'L1': (0, 135), 'L2': (45, 180),
    'R3': (0, 180), 'R4': (0, 180), 'L3': (0, 180), 'L4': (0, 180),
}
STAND_DEG = {'R1': 135, 'R2': 45, 'L1': 45, 'L2': 135, 'R4': 0, 'R3': 180, 'L3': 0, 'L4': 180}
# Il ginocchio "disteso" (piede nel punto più basso) è 180° per R3/L4 e 0° per R4/L3.
KNEE_STRAIGHT_DEG = {'R3': 180, 'L4': 180, 'R4': 0, 'L3': 0}
BASE = 'base'


def rpy_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def axis_angle(axis, a):
    k = np.asarray(axis, float) / np.linalg.norm(axis)
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(a) * K + (1 - math.cos(a)) * K @ K


def tf(R, t):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


class LegKinematics:
    def __init__(self, urdf_path):
        root = ET.parse(urdf_path).getroot()
        self.by_child = {}
        for j in root.findall('joint'):
            o = j.find('origin')
            xyz = [float(v) for v in (o.get('xyz', '0 0 0') if o is not None else '0 0 0').split()]
            rpy = [float(v) for v in (o.get('rpy', '0 0 0') if o is not None else '0 0 0').split()]
            ax = j.find('axis')
            self.by_child[j.find('child').get('link')] = {
                'name': j.get('name'), 'type': j.get('type'),
                'parent': j.find('parent').get('link'),
                'T': tf(rpy_matrix(*rpy), xyz),
                'axis': [float(v) for v in ax.get('xyz').split()] if ax is not None else [1, 0, 0],
            }

    def chain(self, link):
        out = []
        while link != BASE:
            j = self.by_child[link]
            out.append(j)
            link = j['parent']
        return out[::-1]

    def foot(self, group, q_rad):
        """Posizione del piede (m) nel frame base. q_rad: {giunto: rad}."""
        T = np.eye(4)
        for j in self.chain(LEGS[group][2]):
            T = T @ j['T']
            if j['type'] == 'revolute':
                T = T @ tf(axis_angle(j['axis'], q_rad[j['name']]), [0, 0, 0])
        return T[:3, 3].copy()

    def foot_deg(self, group, hip_deg, knee_deg):
        hip, knee, _ = LEGS[group]
        return self.foot(group, {hip: math.radians(hip_deg), knee: math.radians(knee_deg)})

    def ik_xz(self, group, x, z, seed_deg, tol=1e-6, iters=50):
        """(x, z) del piede (m) -> (anca, ginocchio) in gradi. Ritorna None se non converge."""
        q = np.array(seed_deg, float)
        target = np.array([x, z])
        h = 1e-3  # gradi, passo per il jacobiano numerico
        for _ in range(iters):
            p = self.foot_deg(group, *q)
            e = target - p[[0, 2]]
            if np.linalg.norm(e) < tol:
                return float(q[0]), float(q[1])
            J = np.empty((2, 2))
            for k in range(2):
                dq = q.copy()
                dq[k] += h
                J[:, k] = (self.foot_deg(group, *dq)[[0, 2]] - p[[0, 2]]) / h
            step = np.linalg.lstsq(J, e, rcond=None)[0]
            q += np.clip(step, -10, 10)  # al massimo 10° per iterazione
        return None
