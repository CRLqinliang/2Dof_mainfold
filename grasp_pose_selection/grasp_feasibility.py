# 连续可行性标量场 F(x,y,theta)：关节角前向遍历 +障碍 SDF 采样碰撞裕度
from dataclasses import dataclass
from typing import Any, Callable, Dict, List, Optional, Tuple

import numpy as np
import torch

from grasp_geometry import object_pose_world, wrap_pi

from robot2D_torch import Robot2D


@dataclass
class GraspTransform:
    """固定抓取下物体系相对末端系的变换（由接触点与内向法向决定）。"""
    R_eo: np.ndarray
    t_eo: np.ndarray
    theta_eo: float
    name: str = ""


def min_sdf_points(obstacles, pts_world: np.ndarray, device) -> float:
    if len(pts_world) == 0:
        return float("inf")
    p = torch.as_tensor(pts_world, dtype=torch.float32, device=device)
    m = []
    for obs in obstacles:
        d = obs.signed_distance(p)
        m.append(d.view(-1))
    return torch.cat(m, dim=0).min().item()


def min_sdf_points_worst(obstacles, pts_world: np.ndarray, device):
    """返回全局最小 SDF 及其对应的世界系采样点（调试用）。"""
    if len(pts_world) == 0:
        return float("inf"), np.zeros(2)
    p = torch.as_tensor(pts_world, dtype=torch.float32, device=device)
    cols = [obs.signed_distance(p).view(-1, 1) for obs in obstacles]
    stacked = torch.cat(cols, dim=1)
    d_pt, _ = stacked.min(dim=1)
    k = int(d_pt.argmin().item())
    return float(d_pt[k].item()), pts_world[k].copy()


def blur3d_mean(H: np.ndarray, passes: int = 1) -> np.ndarray:
    V = H.astype(np.float64)
    for _ in range(passes):
        P = np.pad(V, 1, mode="edge")
        nx, ny, nz = V.shape
        acc = np.zeros_like(V)
        for di in (0, 1, 2):
            for dj in (0, 1, 2):
                for dk in (0, 1, 2):
                    acc += P[di : di + nx, dj : dj + ny, dk : dk + nz]
        V = acc / 27.0
    return V


def ee_pose_from_q(q1: float, q2: float, L1: float, L2: float) -> Tuple[np.ndarray, float]:
    """平面 2R 末端位置与末端朝向 theta_e = q1+q2。"""
    p = np.array(
        [
            L1 * np.cos(q1) + L2 * np.cos(q1 + q2),
            L1 * np.sin(q1) + L2 * np.sin(q1 + q2),
        ],
        dtype=np.float64,
    )
    return p, float(q1 + q2)


def object_pose_from_q(
    q1: float, q2: float, L1: float, L2: float, grasp: GraspTransform
) -> Tuple[np.ndarray, float]:
    p_ee, th_e = ee_pose_from_q(q1, q2, L1, L2)
    t_o, theta_o = object_pose_world(p_ee, th_e, grasp.R_eo, grasp.t_eo, grasp.theta_eo)
    return t_o, float(theta_o)


def _axis_cell_indices(val: float, coords: np.ndarray) -> List[int]:
    """连续坐标落在的网格区间 [i0,i1]（格心为 coords）；用于 cell-corner splat。"""
    n = len(coords)
    if n <= 1:
        return [0]
    lo, hi = float(coords[0]), float(coords[-1])
    step = (hi - lo) / (n - 1)
    f = (float(val) - lo) / step
    f = float(np.clip(f, 0.0, n - 1))
    i0 = int(np.floor(f))
    i1 = int(np.ceil(f))
    i0 = max(0, min(i0, n - 1))
    i1 = max(0, min(i1, n - 1))
    if i0 == i1:
        return [i0]
    return [i0, i1]


def _splat_clearance_to_cell_corners(
    F: np.ndarray,
    x: float,
    y: float,
    th: float,
    c: float,
    x_coords: np.ndarray,
    y_coords: np.ndarray,
    th_coords: np.ndarray,
    unreachable_fill: float,
) -> None:
    """把一次样本的 clearance 写入包含 (x,y,th) 的网格立方体各角点：F=max(F,c)，避免最近邻单格投影。"""
    for i in _axis_cell_indices(x, x_coords):
        for j in _axis_cell_indices(y, y_coords):
            for k in _axis_cell_indices(th, th_coords):
                old = F[i, j, k]
                if old <= unreachable_fill + 1e-9:
                    F[i, j, k] = c
                else:
                    F[i, j, k] = max(old, c)


def clearance_for_q(
    q1: float,
    q2: float,
    robot: Robot2D,
    obj_boundary_world_fn: Callable[[float, np.ndarray], np.ndarray],
    theta_o: float,
    t_o: np.ndarray,
    obstacles,
    robot_surface_n: int,
    device,
    collision_margin: float,
) -> float:
    q = torch.tensor([[q1, q2]], dtype=torch.float32, device=device)
    rk = robot.surface_points_sampler(q, n=robot_surface_n)[0].reshape(-1, 2)
    ob = obj_boundary_world_fn(theta_o, t_o)
    pts = np.vstack([rk.cpu().numpy(), ob])
    return min_sdf_points(obstacles, pts, device) - collision_margin


def clearance_breakdown_for_q(
    q1: float,
    q2: float,
    robot: Robot2D,
    obj_boundary_world_fn: Callable[[float, np.ndarray], np.ndarray],
    theta_o: float,
    t_o: np.ndarray,
    obstacles,
    robot_surface_n: int,
    device,
    collision_margin: float,
) -> Dict[str, Any]:
    """关节构型下：整体 clearance、机械臂/物体分项、瓶颈归属与最差点（世界系）。"""
    q = torch.tensor([[q1, q2]], dtype=torch.float32, device=device)
    rk = robot.surface_points_sampler(q, n=robot_surface_n)[0].reshape(-1, 2)
    rk_np = rk.cpu().numpy()
    ob = obj_boundary_world_fn(theta_o, t_o)
    d_robot = min_sdf_points(obstacles, rk_np, device)
    d_obj = min_sdf_points(obstacles, ob, device)
    pts = np.vstack([rk_np, ob])
    d_all, worst_pt = min_sdf_points_worst(obstacles, pts, device)
    c = float(d_all - collision_margin)
    cr = float(d_robot - collision_margin)
    co = float(d_obj - collision_margin)
    if d_robot < d_obj - 1e-7:
        bottleneck = "robot"
    elif d_obj < d_robot - 1e-7:
        bottleneck = "object"
    else:
        bottleneck = "tie"
    return {
        "clearance": c,
        "clearance_robot": cr,
        "clearance_object": co,
        "bottleneck": bottleneck,
        "worst_xy": [float(worst_pt[0]), float(worst_pt[1])],
    }


def build_joint_space_clearance(
    grasp: GraspTransform,
    L1: float,
    L2: float,
    robot: Robot2D,
    obj_boundary_world_fn: Callable[[float, np.ndarray], np.ndarray],
    obstacles,
    robot_surface_n: int,
    device,
    collision_margin: float,
    q1_coords: np.ndarray,
    q2_coords: np.ndarray,
) -> np.ndarray:
    """J[i,j] = 该 grasp 下 (q1_coords[i], q2_coords[j]) 的碰撞裕度（与 F 场单点定义相同）。"""
    n1, n2 = len(q1_coords), len(q2_coords)
    J = np.zeros((n1, n2), dtype=np.float64)
    for i, q1 in enumerate(q1_coords):
        for j, q2 in enumerate(q2_coords):
            t_o, theta_o = object_pose_from_q(float(q1), float(q2), L1, L2, grasp)
            theta_o = wrap_pi(theta_o)
            J[i, j] = clearance_for_q(
                float(q1),
                float(q2),
                robot,
                obj_boundary_world_fn,
                theta_o,
                t_o,
                obstacles,
                robot_surface_n,
                device,
                collision_margin,
            )
    return J


def build_feasibility_field_from_joint_sweep(
    grasp: GraspTransform,
    L1: float,
    L2: float,
    robot: Robot2D,
    obj_boundary_world_fn: Callable[[float, np.ndarray], np.ndarray],
    obstacles,
    robot_surface_n: int,
    device,
    collision_margin: float,
    x_coords: np.ndarray,
    y_coords: np.ndarray,
    th_coords: np.ndarray,
    q1_coords: np.ndarray,
    q2_coords: np.ndarray,
    smooth_passes: int = 0,
    unreachable_fill: float = -1.0,
    max_feasible_q_store: int = 400,
    deposit: str = "cell",
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, List[Tuple[float, float, float]]]:
    """
    遍历 (q1,q2)，FK 得物体位姿，将 clearance 写入 SE(2) 规则网格：
    - deposit=\"cell\"（默认）：落入的网格立方体各角点做 max(F,c)，场在格分辨率内连续、无单点投影；
    - deposit=\"nearest\"：仅最近心格 max(F,c)（旧行为）。
    未命中格保持 unreachable_fill。另缓存至多 max_feasible_q_store 条 clearance>0 的 (q1,q2,c)。
    """
    nx, ny, nz = len(x_coords), len(y_coords), len(th_coords)
    F = np.full((nx, ny, nz), unreachable_fill, dtype=np.float64)
    feasible_q: List[Tuple[float, float, float]] = []
    for q1 in q1_coords:
        for q2 in q2_coords:
            t_o, theta_o = object_pose_from_q(float(q1), float(q2), L1, L2, grasp)
            theta_o = wrap_pi(theta_o)
            c = clearance_for_q(
                float(q1),
                float(q2),
                robot,
                obj_boundary_world_fn,
                theta_o,
                t_o,
                obstacles,
                robot_surface_n,
                device,
                collision_margin,
            )
            if deposit == "nearest":
                if nx <= 1:
                    ix = 0
                else:
                    ix = int(
                        np.clip(
                            np.round(
                                (float(t_o[0]) - x_coords[0])
                                / ((x_coords[-1] - x_coords[0]) / (nx - 1))
                            ),
                            0,
                            nx - 1,
                        )
                    )
                if ny <= 1:
                    iy = 0
                else:
                    iy = int(
                        np.clip(
                            np.round(
                                (float(t_o[1]) - y_coords[0])
                                / ((y_coords[-1] - y_coords[0]) / (ny - 1))
                            ),
                            0,
                            ny - 1,
                        )
                    )
                if nz <= 1:
                    iz = 0
                else:
                    iz = int(
                        np.clip(
                            np.round(
                                (float(theta_o) - th_coords[0])
                                / ((th_coords[-1] - th_coords[0]) / (nz - 1))
                            ),
                            0,
                            nz - 1,
                        )
                    )
                old = F[ix, iy, iz]
                if old <= unreachable_fill + 1e-9:
                    F[ix, iy, iz] = c
                else:
                    F[ix, iy, iz] = max(old, c)
            else:
                _splat_clearance_to_cell_corners(
                    F,
                    float(t_o[0]),
                    float(t_o[1]),
                    float(theta_o),
                    c,
                    x_coords,
                    y_coords,
                    th_coords,
                    unreachable_fill,
                )
            if c > 0 and len(feasible_q) < max_feasible_q_store:
                feasible_q.append((float(q1), float(q2), float(c)))
    if smooth_passes > 0:
        Fs = blur3d_mean(F, passes=smooth_passes)
        unreach = F <= unreachable_fill + 1e-6
        F = np.where(unreach, F, Fs)
    return F, x_coords, y_coords, th_coords, feasible_q


def find_first_colliding_config(
    grasp: GraspTransform,
    L1: float,
    L2: float,
    robot: Robot2D,
    obj_boundary_world_fn: Callable[[float, np.ndarray], np.ndarray],
    obstacles,
    robot_surface_n: int,
    device,
    collision_margin: float,
    q1_coords: np.ndarray,
    q2_coords: np.ndarray,
) -> Optional[Tuple[float, float]]:
    """第一个 clearance<0 的关节配置（用于碰撞调试图）。"""
    for q1 in q1_coords:
        for q2 in q2_coords:
            t_o, theta_o = object_pose_from_q(float(q1), float(q2), L1, L2, grasp)
            theta_o = wrap_pi(theta_o)
            c = clearance_for_q(
                float(q1),
                float(q2),
                robot,
                obj_boundary_world_fn,
                theta_o,
                t_o,
                obstacles,
                robot_surface_n,
                device,
                collision_margin,
            )
            if c < 0:
                return float(q1), float(q2)
    return None
