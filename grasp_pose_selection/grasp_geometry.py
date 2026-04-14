# 几何与抓取：边界采样、物体/末端位姿变换、轮廓工具
import math
from typing import Tuple

import numpy as np

PI = math.pi


def rot2(theta: float) -> np.ndarray:
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s], [s, c]], dtype=np.float64)


def wrap_pi(a: float) -> float:
    return (a + PI) % (2 * PI) - PI


def sample_box_grasps(cx, cy, w, h, n_per_edge):
    pts, normals = [], []
    hw, hh = w / 2, h / 2
    for t in np.linspace(-hw, hw, n_per_edge, endpoint=False):
        pts.append([cx + t, cy + hh])
        normals.append([0.0, -1.0])
    for t in np.linspace(-hh, hh, n_per_edge, endpoint=False):
        pts.append([cx + hw, cy + t])
        normals.append([-1.0, 0.0])
    for t in np.linspace(hw, -hw, n_per_edge, endpoint=False):
        pts.append([cx + t, cy - hh])
        normals.append([0.0, 1.0])
    for t in np.linspace(hh, -hh, n_per_edge, endpoint=False):
        pts.append([cx - hw, cy + t])
        normals.append([1.0, 0.0])
    return np.asarray(pts), np.asarray(normals)


def sample_triangle_grasps(p0, p1, p2, n_per_edge):
    p0, p1, p2 = np.asarray(p0), np.asarray(p1), np.asarray(p2)
    c = (p0 + p1 + p2) / 3.0
    pts, normals = [], []

    def edge_samples(a, b):
        nonlocal pts, normals
        e = b - a
        le = np.linalg.norm(e)
        if le < 1e-9:
            return
        tdir = e / le
        out = np.array([-tdir[1], tdir[0]])
        if np.dot(out, c - (a + b) / 2) > 0:
            out = -out
        inn = -out
        for s in np.linspace(0.0, 1.0, n_per_edge, endpoint=False):
            pts.append(a + s * e)
            normals.append(inn.copy())

    edge_samples(p0, p1)
    edge_samples(p1, p2)
    edge_samples(p2, p0)
    return np.asarray(pts), np.asarray(normals)


def grasp_to_ee_object_transform(contact_obj, n_in_obj):
    n = np.asarray(n_in_obj, dtype=np.float64)
    n = n / (np.linalg.norm(n) + 1e-12)
    ang = math.atan2(n[1], n[0])
    R_eo = rot2(-ang)
    t_eo = -R_eo @ np.asarray(contact_obj, dtype=np.float64)
    theta_eo = -ang
    return R_eo, t_eo, theta_eo


def object_pose_world(p_ee, theta_e, R_eo, t_eo, theta_eo):
    R_e = rot2(theta_e)
    t_o = p_ee + R_e @ t_eo
    theta_o = theta_e + theta_eo
    return t_o, theta_o


def object_pose_to_ee(t_o, theta_o, R_eo, t_eo, theta_eo) -> Tuple[np.ndarray, float]:
    """由物体原点世界坐标与物体朝向，反推末端位置与末端朝向 theta_e=q1+q2。"""
    theta_e = wrap_pi(float(theta_o) - float(theta_eo))
    R_e = rot2(theta_e)
    p_ee = np.asarray(t_o, dtype=np.float64) - R_e @ np.asarray(t_eo, dtype=np.float64)
    return p_ee, theta_e


def box_boundary_world(cx, cy, w, h, theta_o):
    hw, hh = w / 2, h / 2
    loc = np.array(
        [[-hw, -hh], [hw, -hh], [hw, hh], [-hw, hh]], dtype=np.float64
    )
    R = rot2(theta_o)
    return (loc @ R.T) + np.array([cx, cy])


def polygon_boundary_world(verts_obj, theta_o, t_o):
    """任意闭合多边形（物体系顶点 N×2）→ 世界系边界点。"""
    R = rot2(theta_o)
    return (np.asarray(verts_obj, dtype=np.float64) @ R.T) + np.asarray(t_o, dtype=np.float64)


def triangle_boundary_world(verts_obj, theta_o, t_o):
    return polygon_boundary_world(verts_obj, theta_o, t_o)


def sample_polyline(points, closed, num):
    pts = np.asarray(points, dtype=np.float64)
    if closed:
        pts = np.vstack([pts, pts[0:1]])
    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    u = np.cumsum(seg)
    u0 = np.concatenate([[0.0], u])
    total = u0[-1]
    if total < 1e-9:
        return pts[:-1]
    samples = []
    for k in range(num):
        d = (k + 0.5) / num * total
        j = np.searchsorted(u0, d) - 1
        j = max(0, min(j, len(pts) - 2))
        t = (d - u0[j]) / (u0[j + 1] - u0[j] + 1e-12)
        samples.append((1 - t) * pts[j] + t * pts[j + 1])
    return np.asarray(samples)
