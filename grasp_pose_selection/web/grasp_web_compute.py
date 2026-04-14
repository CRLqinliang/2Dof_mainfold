# 网页 API 用：关节空间 J(q1,q2) + SE(2) 场 F 的 theta 切片（复用 grasp_*与 primitives2D_torch）
from __future__ import annotations

import sys
from pathlib import Path
import copy
from typing import Any, Callable, Dict, List, Tuple

import numpy as np
import torch

_PKG = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(_PKG.parent))
sys.path.insert(0, str(_PKG))

from grasp_feasibility import (  # noqa: E402
    GraspTransform,
    build_feasibility_field_from_joint_sweep,
    build_joint_space_clearance,
    clearance_breakdown_for_q,
    object_pose_from_q,
)
from grasp_geometry import (  # noqa: E402
    box_boundary_world,
    grasp_to_ee_object_transform,
    polygon_boundary_world,
    sample_polyline,
    wrap_pi,
)
from primitives2D_torch import Box, Circle  # noqa: E402
from robot2D_torch import Robot2D  # noqa: E402

PI = float(np.pi)
TWO_PI = 2.0 * PI


def interp_f_theta_plane(
    F: np.ndarray, th_coords: np.ndarray, th_des: float
) -> Tuple[np.ndarray, int, int, float]:
    """对 F[:,:,k] 在 θ 方向线性插值；返回 (Fxy, k0, k1, alpha)。"""
    nz = int(F.shape[2])
    th_des = float(th_des)
    lo, hi = float(th_coords[0]), float(th_coords[-1])
    if th_des <= lo:
        return F[:, :, 0].copy(), 0, 0, 1.0
    if th_des >= hi:
        return F[:, :, nz - 1].copy(), nz - 1, nz - 1, 1.0
    j = int(np.searchsorted(th_coords, th_des, side="right"))
    k0 = max(0, j - 1)
    k1 = min(nz - 1, j)
    if k0 == k1:
        return F[:, :, k0].copy(), k0, k0, 1.0
    t0, t1 = float(th_coords[k0]), float(th_coords[k1])
    denom = t1 - t0
    alpha = (th_des - t0) / denom if abs(denom) > 1e-12 else 0.0
    alpha = float(np.clip(alpha, 0.0, 1.0))
    out = (1.0 - alpha) * F[:, :, k0] + alpha * F[:, :, k1]
    return out.astype(np.float64), k0, k1, alpha


def make_object_boundary_fn(
    obj: Dict[str, Any], obj_surface_n: int
) -> Tuple[Callable[[float, np.ndarray], np.ndarray], Dict[str, Any]]:
    otype = obj["type"]
    if otype == "box":
        box_w = float(obj["box_w"])
        box_h = float(obj["box_h"])

        def boundary_fn(theta_o: float, t_o: np.ndarray) -> np.ndarray:
            b = box_boundary_world(t_o[0], t_o[1], box_w, box_h, theta_o)
            return sample_polyline(b, closed=True, num=obj_surface_n)

        meta = {"object_type": "box", "box_w": box_w, "box_h": box_h}
        return boundary_fn, meta
    if otype == "triangle":
        tri = np.array(obj["tri"], dtype=np.float64)
        if tri.shape != (3, 2):
            raise ValueError("object.tri must be 3x2")

        def boundary_fn(theta_o: float, t_o: np.ndarray) -> np.ndarray:
            b = polygon_boundary_world(tri, theta_o, t_o)
            return sample_polyline(b, closed=True, num=obj_surface_n)

        meta = {"object_type": "triangle", "tri": tri.tolist()}
        return boundary_fn, meta
    if otype == "polygon":
        verts = np.array(obj["verts"], dtype=np.float64)
        if verts.ndim != 2 or verts.shape[1] != 2 or verts.shape[0] < 3:
            raise ValueError("object.verts must be Nx2 with N>=3")

        def boundary_fn(theta_o: float, t_o: np.ndarray) -> np.ndarray:
            b = polygon_boundary_world(verts, theta_o, t_o)
            return sample_polyline(b, closed=True, num=obj_surface_n)

        meta = {"object_type": "polygon", "verts": verts.tolist()}
        return boundary_fn, meta
    raise ValueError("object.type must be box, triangle, or polygon")


def field_coord_ranges(L1: float, L2: float) -> Tuple[float, float, float, float, float, float]:
    R = L1 + L2
    pad = 0.08 * R
    return -R - pad, R + pad, -R - pad, R + pad, -PI, PI


def obstacles_from_payload(items: List[Dict[str, Any]], device: torch.device) -> list:
    out = []
    for o in items:
        t = o["type"]
        if t == "circle":
            out.append(
                Circle(
                    center=torch.tensor([float(o["cx"]), float(o["cy"])], device=device),
                    radius=float(o["r"]),
                    device=device,
                )
            )
        elif t == "box":
            out.append(
                Box(
                    center=torch.tensor([float(o["cx"]), float(o["cy"])], device=device),
                    w=float(o["w"]),
                    h=float(o["h"]),
                    device=device,
                )
            )
    return out


def preview_payload(payload: Dict[str, Any]) -> Dict[str, Any]:
    """降低网格与采样密度，用于拖动时的快速预览。"""
    p = copy.deepcopy(payload)
    # 上限再压低，便于 Render 免费实例在可接受时间内返回
    p["q1_n"] = max(2, min(int(p["q1_n"]), 14))
    p["q2_n"] = max(2, min(int(p["q2_n"]), 14))
    p["field_nx"] = max(2, min(int(p["field_nx"]), 10))
    p["field_ny"] = max(2, min(int(p["field_ny"]), 10))
    p["field_nz"] = max(2, min(int(p["field_nz"]), 8))
    p["robot_surface_n"] = max(8, min(int(p["robot_surface_n"]), 12))
    p["obj_surface_n"] = max(12, min(int(p["obj_surface_n"]), 16))
    return p


def default_payload() -> Dict[str, Any]:
    return {
        "link1": 2.0,
        "link2": 2.0,
        "robot_surface_n": 32,
        "obj_surface_n": 48,
        "collision_margin": 0.0,
        "q1_min": -PI,
        "q1_max": PI,
        "q1_n": 48,
        "q2_min": -PI,
        "q2_max": PI,
        "q2_n": 48,
        "field_nx": 40,
        "field_ny": 40,
        "field_nz": 20,
        "field_smooth": 0,
        "field_deposit": "cell",
        "object": {"type": "box", "box_w": 0.8, "box_h": 0.8},
        "grasp": {"contact": [0.0, 0.4], "normal": [0.0, -1.0]},
        "obstacles": [
            {"type": "circle", "cx": -2.2, "cy": -1.8, "r": 0.5},
            {"type": "box", "cx": 1.5, "cy": 1.0, "w": 0.5, "h": 0.5},
        ],
        "slice_thetas": [0.0],
    }


def compute_web_demo(payload: Dict[str, Any]) -> Dict[str, Any]:
    device = torch.device("cpu")
    q1_n = int(payload["q1_n"])
    q2_n = int(payload["q2_n"])
    fnx, fny, fnz = int(payload["field_nx"]), int(payload["field_ny"]), int(payload["field_nz"])
    if q1_n < 2 or q2_n < 2:
        raise ValueError("q1_n / q2_n 必须 >= 2")
    if fnx < 2 or fny < 2 or fnz < 2:
        raise ValueError("field_nx / field_ny / field_nz 必须 >= 2")
    gn = np.array(payload["grasp"]["normal"], dtype=np.float64).reshape(-1)
    if gn.size != 2 or float(np.linalg.norm(gn)) < 1e-9:
        raise ValueError("抓取法向必须为非零二维向量")
    L1 = float(payload["link1"])
    L2 = float(payload["link2"])
    robot_surface_n = int(payload["robot_surface_n"])
    obj_surface_n = int(payload["obj_surface_n"])
    collision_margin = float(payload["collision_margin"])

    link = torch.tensor([[L1, L2]], dtype=torch.float32, device=device)
    robot = Robot2D(
        num_joints=2,
        init_states=torch.zeros(1, 2, device=device),
        link_length=link,
        device=device,
    )
    obstacles = obstacles_from_payload(payload["obstacles"], device)

    xmin, xmax, ymin, ymax, thmin, thmax = field_coord_ranges(L1, L2)
    x_coords = np.linspace(xmin, xmax, int(payload["field_nx"]))
    y_coords = np.linspace(ymin, ymax, int(payload["field_ny"]))
    th_coords = np.linspace(thmin, thmax, int(payload["field_nz"]))
    q1_coords = np.linspace(float(payload["q1_min"]), float(payload["q1_max"]), int(payload["q1_n"]))
    q2_coords = np.linspace(float(payload["q2_min"]), float(payload["q2_max"]), int(payload["q2_n"]))

    obj = payload["object"]
    g = payload["grasp"]
    contact = np.array(g["contact"], dtype=np.float64)
    normal = np.array(g["normal"], dtype=np.float64)
    R_eo, t_eo, th_eo = grasp_to_ee_object_transform(contact, normal)
    grasp = GraspTransform(R_eo, t_eo, th_eo, "web_grasp")

    boundary_fn, meta = make_object_boundary_fn(obj, obj_surface_n)

    J = build_joint_space_clearance(
        grasp,
        L1,
        L2,
        robot,
        boundary_fn,
        obstacles,
        robot_surface_n,
        device,
        collision_margin,
        q1_coords,
        q2_coords,
    )

    F, _, _, _, _ = build_feasibility_field_from_joint_sweep(
        grasp,
        L1,
        L2,
        robot,
        boundary_fn,
        obstacles,
        robot_surface_n,
        device,
        collision_margin,
        x_coords,
        y_coords,
        th_coords,
        q1_coords,
        q2_coords,
        smooth_passes=int(payload["field_smooth"]),
        max_feasible_q_store=0,
        deposit=str(payload.get("field_deposit", "cell")),
    )

    nz = int(F.shape[2])
    F_theta_layers = [F[:, :, k].tolist() for k in range(nz)]

    slice_thetas = [float(t) for t in payload["slice_thetas"]]
    slices = []
    for th_des in slice_thetas:
        F_plane, k0, k1, alpha = interp_f_theta_plane(F, th_coords, th_des)
        slices.append(
            {
                "theta_request": th_des,
                "theta_eval": float((1.0 - alpha) * th_coords[k0] + alpha * th_coords[k1]),
                "interp_k0": k0,
                "interp_k1": k1,
                "interp_alpha": alpha,
                "F": F_plane.tolist(),
            }
        )

    return {
        "meta": meta,
        "grasp": {"contact": contact.tolist(), "normal": normal.tolist()},
        "q1_coords": q1_coords.tolist(),
        "q2_coords": q2_coords.tolist(),
        "J": J.tolist(),
        "x_coords": x_coords.tolist(),
        "y_coords": y_coords.tolist(),
        "th_coords": th_coords.tolist(),
        "F_theta_layers": F_theta_layers,
        "slices": slices,
        "feasible_joint_cells": int((J > 0).sum()),
        "feasible_se2_cells": int((F > 0).sum()),
        "se2_grid_size": int(F.size),
    }


def inspect_clearance(payload: Dict[str, Any], q1: float, q2: float) -> Dict[str, Any]:
    """对单次 (q1,q2) 返回 clearance 分解（机械臂 vs 抓取物体边界）。"""
    device = torch.device("cpu")
    L1 = float(payload["link1"])
    L2 = float(payload["link2"])
    robot_surface_n = int(payload["robot_surface_n"])
    obj_surface_n = int(payload["obj_surface_n"])
    collision_margin = float(payload["collision_margin"])

    link = torch.tensor([[L1, L2]], dtype=torch.float32, device=device)
    robot = Robot2D(
        num_joints=2,
        init_states=torch.zeros(1, 2, device=device),
        link_length=link,
        device=device,
    )
    obstacles = obstacles_from_payload(payload["obstacles"], device)

    obj = payload["object"]
    g = payload["grasp"]
    contact = np.array(g["contact"], dtype=np.float64)
    normal = np.array(g["normal"], dtype=np.float64)
    R_eo, t_eo, th_eo = grasp_to_ee_object_transform(contact, normal)
    grasp = GraspTransform(R_eo, t_eo, th_eo, "inspect")

    boundary_fn, _ = make_object_boundary_fn(obj, obj_surface_n)

    t_o, theta_o = object_pose_from_q(float(q1), float(q2), L1, L2, grasp)
    theta_o = wrap_pi(theta_o)
    return clearance_breakdown_for_q(
        float(q1),
        float(q2),
        robot,
        boundary_fn,
        theta_o,
        t_o,
        obstacles,
        robot_surface_n,
        device,
        collision_margin,
    )
