# -----------------------------------------------------------------------------
# 连续 SE(2) 可行域：关节 sweep + cell 沉积；joint_space_by_grasp 等图见 grasp_visualization。
#
#   python test.py --no-show
#   python test.py --help
# -----------------------------------------------------------------------------
from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Dict, List, Tuple

import matplotlib.pyplot as plt
import numpy as np
import torch

_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(_ROOT))

from primitives2D_torch import Box, Circle  # noqa: E402

from grasp_feasibility import (  # noqa: E402
    GraspTransform,
    build_feasibility_field_from_joint_sweep,
    build_joint_space_clearance,
    find_first_colliding_config,
)
from grasp_geometry import (  # noqa: E402
    box_boundary_world,
    grasp_to_ee_object_transform,
    sample_box_grasps,
    sample_polyline,
    sample_triangle_grasps,
    triangle_boundary_world,
)
from grasp_visualization import (  # noqa: E402
    plot_collision_debug,
    plot_feasibility_isosurface,
    plot_feasible_poses_2d,
    plot_grasp_candidates_figure,
    plot_grasp_compare_theta_slice,
    plot_joint_space_by_grasp,
    plot_theta_slices,
)

from robot2D_torch import Robot2D  # noqa: E402

PI = float(np.pi)


# --- argparse 分块 -----------------------------------------------------------

def add_robot_args(p: argparse.ArgumentParser) -> None:
    p.add_argument("--device", type=str, default="cpu")
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--link1", type=float, default=2.0)
    p.add_argument("--link2", type=float, default=2.0)
    p.add_argument("--robot-surface-n", type=int, default=32)
    p.add_argument("--obj-surface-n", type=int, default=48)
    p.add_argument("--collision-margin", type=float, default=0)
    p.add_argument("--q1-n", type=int, default=72, help="q1 采样数（含端点）")
    p.add_argument("--q2-n", type=int, default=72, help="q2 采样数（含端点）")
    p.add_argument("--q1-min", type=float, default=-PI, help="q1 下限（弧度）")
    p.add_argument("--q1-max", type=float, default=PI, help="q1 上限（弧度）")
    p.add_argument("--q2-min", type=float, default=-PI, help="q2 下限（弧度）")
    p.add_argument("--q2-max", type=float, default=PI, help="q2 上限（弧度）")
    p.add_argument(
        "--max-feasible-q-store",
        type=int,
        default=400,
        help="关节 sweep 中至多缓存多少条 clearance>0 的 (q1,q2) 供 2D 示意图",
    )


def add_scene_args(p: argparse.ArgumentParser) -> None:
    p.add_argument("--circle-cx", type=float, default=-2.2)
    p.add_argument("--circle-cy", type=float, default=-1.8)
    p.add_argument("--circle-r", type=float, default=0.5)
    p.add_argument("--box-obs-cx", type=float, default=1.5)
    p.add_argument("--box-obs-cy", type=float, default=1.0)
    p.add_argument("--box-obs-w", type=float, default=0.5)
    p.add_argument("--box-obs-h", type=float, default=0.5)


def add_object_args(p: argparse.ArgumentParser) -> None:
    p.add_argument("--obj-box-w", type=float, default=0.8)
    p.add_argument("--obj-box-h", type=float, default=0.8)
    p.add_argument("--grasp-edge-samples", type=int, default=5)
    p.add_argument("--tri-p0x", type=float, default=0.0)
    p.add_argument("--tri-p0y", type=float, default=0.0)
    p.add_argument("--tri-p1x", type=float, default=0.55)
    p.add_argument("--tri-p1y", type=float, default=0.0)
    p.add_argument("--tri-p2x", type=float, default=0.25)
    p.add_argument("--tri-p2y", type=float, default=0.45)
    p.add_argument("--grasp-primary-box", type=int, default=-1, help="矩形主抓取候选索引，-1 为自动中间")
    p.add_argument("--grasp-primary-tri", type=int, default=-1)


def add_field_args(p: argparse.ArgumentParser) -> None:
    p.add_argument("--field-nx", type=int, default=52)
    p.add_argument("--field-ny", type=int, default=52)
    p.add_argument("--field-nz", type=int, default=26)
    p.add_argument("--field-smooth", type=int, default=2, help="场平滑迭代（仅对可达格）")
    p.add_argument(
        "--field-deposit",
        type=str,
        choices=("cell", "nearest"),
        default="cell",
        help="SE(2) 网格沉积：cell=落入格子的立方体角点取 max；nearest=仅最近心格",
    )
    p.add_argument("--iso-level", type=float, default=0.0, help="等值面 F=该值（0为可行边界）")
    p.add_argument("--iso-stride-theta", type=int, default=1, help="theta 方向等值线步长（越大越快越稀）")
    p.add_argument("--iso-stride-xy", type=int, default=2, help="x/y 方向等值线步长")
    p.add_argument(
        "--iso-no-fill-theta",
        action="store_true",
        help="关闭 3D 图上常 theta 截面的半透明堆叠（仅线网）",
    )
    p.add_argument("--iso-fill-alpha", type=float, default=0.12, help="theta 向堆叠面片透明度")
    p.add_argument(
        "--iso-fill-stride-theta",
        type=int,
        default=1,
        help="theta 向堆叠隔多少层（1最密，越大越快）",
    )


def add_grasp_compare_args(p: argparse.ArgumentParser) -> None:
    p.add_argument(
        "--compare-grasp-indices",
        type=str,
        default="0,4,8,12",
        help="抓取对比图用到的候选索引（逗号分隔）",
    )
    p.add_argument(
        "--joint-space-max-panels",
        type=int,
        default=6,
        help="关节空间对比图最多子图数（主 grasp×2 + 矩形对比 grasp）",
    )
    p.add_argument(
        "--no-joint-space-plot",
        action="store_true",
        help="不生成 joint_space_by_grasp.png",
    )
    p.add_argument(
        "--slice-thetas",
        type=str,
        default="0.0,0.9,-0.9",
        help="theta 切片列表（弧度，逗号分隔）",
    )
    p.add_argument("--compare-theta-slice", type=float, default=0.35)


def add_viz_args(p: argparse.ArgumentParser) -> None:
    p.add_argument("--plot-lim", type=float, default=4.5)
    p.add_argument("--n-draw-2d", type=int, default=5)
    p.add_argument("--fig3d-w", type=float, default=14.0)
    p.add_argument("--fig3d-h", type=float, default=6.0)
    p.add_argument("--fig-dpi", type=int, default=150, help="保存 PNG 的 DPI")
    p.add_argument("--iso-alpha", type=float, default=0.45, help="3D 等值线透明度")
    p.add_argument("--iso-color-box", type=str, default="steelblue")
    p.add_argument("--iso-color-tri", type=str, default="darkorange")
    p.add_argument("--iso-lw-xy", type=float, default=1.0, help="等值面 theta 层截面线宽")
    p.add_argument("--iso-lw-slice", type=float, default=0.7, help="等值面 x/y 切片线宽")
    p.add_argument("--theta-slice-cmap", type=str, default="RdYlGn")
    p.add_argument("--theta-slice-vmin", type=float, default=-0.5)
    p.add_argument("--theta-slice-vmax", type=float, default=0.5)
    p.add_argument("--slice-contour-level", type=float, default=0.0, help="切片热图上的等高线 F=该值")
    p.add_argument("--theta-slice-contour-color", type=str, default="k")
    p.add_argument("--theta-slice-contour-lw", type=float, default=1.2)
    p.add_argument("--compare-slice-cmap", type=str, default="viridis")
    p.add_argument("--compare-slice-vmin", type=float, default=-0.3)
    p.add_argument("--compare-slice-vmax", type=float, default=0.4)
    p.add_argument("--compare-slice-contour-color", type=str, default="w")
    p.add_argument("--compare-slice-contour-lw", type=float, default=1.0)
    p.add_argument("--grasp-cand-fig-w", type=float, default=11.0)
    p.add_argument("--grasp-cand-fig-h", type=float, default=5.0)
    p.add_argument("--grasp-quiver-scale", type=float, default=12.0)
    p.add_argument("--slice-panel-w", type=float, default=4.0, help="theta 切片单张子图宽度（英寸）")
    p.add_argument("--slice-panel-h", type=float, default=4.0)
    p.add_argument("--viz-grid-alpha", type=float, default=0.3, help="2D 图背景网格 alpha")
    p.add_argument("--feasible-poses-fig-size", type=float, default=7.0)
    p.add_argument("--collision-debug-fig-size", type=float, default=7.0)
    p.add_argument("--feasible-arm-sz", type=float, default=0.06)
    p.add_argument("--feasible-arm-alpha", type=float, default=0.85)
    p.add_argument("--feasible-obj-alpha", type=float, default=0.4)
    p.add_argument("--feasible-poses-cmap", type=str, default="viridis")
    p.add_argument("--out-dir", type=str, default="./results", help="输出目录（相对本脚本目录）")
    p.add_argument("--no-show", action="store_true")


def parse_args():
    p = argparse.ArgumentParser(
        description="SE(2) 可行域：关节 sweep 沉积到场 F(x,y,theta) + 可视化"
    )
    add_robot_args(p)
    add_scene_args(p)
    add_object_args(p)
    add_field_args(p)
    add_grasp_compare_args(p)
    add_viz_args(p)
    return p.parse_args()


def parse_comma_ints(s: str) -> List[int]:
    return [int(x) for x in s.split(",") if x.strip()]


def parse_comma_floats(s: str) -> List[float]:
    return [float(x) for x in s.split(",") if x.strip()]


def _resolve_out_dir(args) -> Path:
    d = Path(args.out_dir)
    if not d.is_absolute():
        d = Path(__file__).resolve().parent / d
    d.mkdir(parents=True, exist_ok=True)
    return d


def build_obstacles(args, device):
    return [
        Circle(
            center=torch.tensor([args.circle_cx, args.circle_cy], device=device),
            radius=args.circle_r,
            device=device,
        ),
        Box(
            center=torch.tensor([args.box_obs_cx, args.box_obs_cy], device=device),
            w=args.box_obs_w,
            h=args.box_obs_h,
            device=device,
        ),
    ]


def field_coord_ranges(L1: float, L2: float):
    R = L1 + L2
    pad = 0.08 * R
    return -R - pad, R + pad, -R - pad, R + pad, -PI, PI


@dataclass
class Scene:
    device: torch.device
    robot: Robot2D
    obstacles: list
    L1: float
    L2: float
    x_coords: np.ndarray
    y_coords: np.ndarray
    th_coords: np.ndarray
    q1_coords: np.ndarray
    q2_coords: np.ndarray
    out_dir: Path


def build_scene(args) -> Scene:
    device = torch.device(args.device)
    torch.manual_seed(args.seed)
    out_dir = _resolve_out_dir(args)
    L1, L2 = args.link1, args.link2
    link = torch.tensor([[L1, L2]], dtype=torch.float32, device=device)
    robot = Robot2D(
        num_joints=2,
        init_states=torch.zeros(1, 2, device=device),
        link_length=link,
        device=device,
    )
    xmin, xmax, ymin, ymax, thmin, thmax = field_coord_ranges(L1, L2)
    return Scene(
        device=device,
        robot=robot,
        obstacles=build_obstacles(args, device),
        L1=L1,
        L2=L2,
        x_coords=np.linspace(xmin, xmax, args.field_nx),
        y_coords=np.linspace(ymin, ymax, args.field_ny),
        th_coords=np.linspace(thmin, thmax, args.field_nz),
        q1_coords=np.linspace(args.q1_min, args.q1_max, args.q1_n),
        q2_coords=np.linspace(args.q2_min, args.q2_max, args.q2_n),
        out_dir=out_dir,
    )


@dataclass
class ObjectSpec:
    """单个被搬运物体：grasp + 碰撞边界采样；out_stem 用于输出文件名（box / tri）。"""

    out_stem: str
    grasp: GraspTransform
    boundary_fn: Callable[[float, np.ndarray], np.ndarray]
    is_box: bool
    iso_color: str
    theta_slice_title: str


@dataclass
class GraspSampling:
    box_w: float
    box_h: float
    tri: np.ndarray
    g_box_pts: np.ndarray
    g_box_norm: np.ndarray
    g_tri_pts: np.ndarray
    g_tri_norm: np.ndarray
    bi: int
    ti: int


def build_grasp_sampling_and_specs(args) -> Tuple[GraspSampling, List[ObjectSpec]]:
    box_w, box_h = args.obj_box_w, args.obj_box_h
    g_box_pts, g_box_norm = sample_box_grasps(
        0.0, 0.0, box_w, box_h, n_per_edge=args.grasp_edge_samples
    )
    bi = (
        len(g_box_pts) // 4
        if args.grasp_primary_box < 0
        else min(args.grasp_primary_box, len(g_box_pts) - 1)
    )
    Rb, tb, thb = grasp_to_ee_object_transform(g_box_pts[bi], g_box_norm[bi])
    grasp_box = GraspTransform(Rb, tb, thb, "box_primary")

    tri = np.array(
        [
            [args.tri_p0x, args.tri_p0y],
            [args.tri_p1x, args.tri_p1y],
            [args.tri_p2x, args.tri_p2y],
        ],
        dtype=np.float64,
    )
    g_tri_pts, g_tri_norm = sample_triangle_grasps(
        tri[0], tri[1], tri[2], n_per_edge=args.grasp_edge_samples
    )
    ti = (
        len(g_tri_pts) // 3
        if args.grasp_primary_tri < 0
        else min(args.grasp_primary_tri, len(g_tri_pts) - 1)
    )
    Rt, tt, tht = grasp_to_ee_object_transform(g_tri_pts[ti], g_tri_norm[ti])
    grasp_tri = GraspTransform(Rt, tt, tht, "tri_primary")

    def box_boundary(theta_o, t_o):
        b = box_boundary_world(t_o[0], t_o[1], box_w, box_h, theta_o)
        return sample_polyline(b, closed=True, num=args.obj_surface_n)

    def tri_boundary(theta_o, t_o):
        b = triangle_boundary_world(tri, theta_o, t_o)
        return sample_polyline(b, closed=True, num=args.obj_surface_n)

    gs = GraspSampling(box_w, box_h, tri, g_box_pts, g_box_norm, g_tri_pts, g_tri_norm, bi, ti)
    specs = [
        ObjectSpec("box", grasp_box, box_boundary, True, args.iso_color_box, "Box (joint sweep)"),
        ObjectSpec("tri", grasp_tri, tri_boundary, False, args.iso_color_tri, "Triangle (joint sweep)"),
    ]
    return gs, specs


def sweep_field_for_spec(scene: Scene, args, spec: ObjectSpec) -> Tuple[np.ndarray, list]:
    F, _, _, _, fq = build_feasibility_field_from_joint_sweep(
        spec.grasp,
        scene.L1,
        scene.L2,
        scene.robot,
        spec.boundary_fn,
        scene.obstacles,
        args.robot_surface_n,
        scene.device,
        args.collision_margin,
        scene.x_coords,
        scene.y_coords,
        scene.th_coords,
        scene.q1_coords,
        scene.q2_coords,
        smooth_passes=args.field_smooth,
        max_feasible_q_store=args.max_feasible_q_store,
        deposit=args.field_deposit,
    )
    return F, fq


def save_se2_isosurface(scene: Scene, args, F_box: np.ndarray, F_tri: np.ndarray) -> None:
    fig = plt.figure(figsize=(args.fig3d_w, args.fig3d_h))
    kw = dict(
        x_coords=scene.x_coords,
        y_coords=scene.y_coords,
        th_coords=scene.th_coords,
        iso_level=args.iso_level,
        alpha=args.iso_alpha,
        stride_theta=args.iso_stride_theta,
        stride_xy=args.iso_stride_xy,
        lw_xy=args.iso_lw_xy,
        lw_slice=args.iso_lw_slice,
        fill_theta_layers=not args.iso_no_fill_theta,
        fill_alpha=args.iso_fill_alpha,
        fill_stride_theta=args.iso_fill_stride_theta,
    )
    axb = fig.add_subplot(1, 2, 1, projection="3d")
    plot_feasibility_isosurface(
        axb, F_box, title=f"Box joint-sweep: F={args.iso_level}", color=args.iso_color_box, **kw
    )
    axt = fig.add_subplot(1, 2, 2, projection="3d")
    plot_feasibility_isosurface(
        axt, F_tri, title=f"Triangle joint-sweep: F={args.iso_level}", color=args.iso_color_tri, **kw
    )
    plt.tight_layout()
    plt.savefig(scene.out_dir / "se2_field_isosurface.png", dpi=args.fig_dpi)
    if not args.no_show:
        plt.show()
    else:
        plt.close(fig)


def save_grasp_candidates(scene: Scene, args, gs: GraspSampling, cmp_idx: List[int]) -> None:
    plot_grasp_candidates_figure(
        scene.out_dir / "grasp_candidates.png",
        gs.box_w,
        gs.box_h,
        gs.g_box_pts,
        gs.g_box_norm,
        gs.g_tri_pts,
        gs.g_tri_norm,
        gs.bi,
        gs.ti,
        cmp_idx,
        cmp_idx,
        show=not args.no_show,
        fig_w=args.grasp_cand_fig_w,
        fig_h=args.grasp_cand_fig_h,
        dpi=args.fig_dpi,
        grid_alpha=args.viz_grid_alpha,
        quiver_scale=args.grasp_quiver_scale,
    )


def build_joint_space_panels(
    scene: Scene,
    args,
    grasp_box: GraspTransform,
    grasp_tri: GraspTransform,
    box_boundary,
    tri_boundary,
    gs: GraspSampling,
    cmp_idx: List[int],
) -> Tuple[List[Tuple[str, np.ndarray]], np.ndarray, np.ndarray]:
    js_panels: List[Tuple[str, np.ndarray]] = []
    J_box_js = build_joint_space_clearance(
        grasp_box,
        scene.L1,
        scene.L2,
        scene.robot,
        box_boundary,
        scene.obstacles,
        args.robot_surface_n,
        scene.device,
        args.collision_margin,
        scene.q1_coords,
        scene.q2_coords,
    )
    js_panels.append(("Box · primary grasp", J_box_js))
    J_tri_js = build_joint_space_clearance(
        grasp_tri,
        scene.L1,
        scene.L2,
        scene.robot,
        tri_boundary,
        scene.obstacles,
        args.robot_surface_n,
        scene.device,
        args.collision_margin,
        scene.q1_coords,
        scene.q2_coords,
    )
    js_panels.append(("Triangle · primary grasp", J_tri_js))
    rem = max(0, args.joint_space_max_panels - 2)
    for idx in cmp_idx:
        if rem <= 0:
            break
        if idx < 0 or idx >= len(gs.g_box_pts):
            continue
        Rb, tb, thb = grasp_to_ee_object_transform(gs.g_box_pts[idx], gs.g_box_norm[idx])
        gj = GraspTransform(Rb, tb, thb, f"box_grasp_{idx}")
        Jj = build_joint_space_clearance(
            gj,
            scene.L1,
            scene.L2,
            scene.robot,
            box_boundary,
            scene.obstacles,
            args.robot_surface_n,
            scene.device,
            args.collision_margin,
            scene.q1_coords,
            scene.q2_coords,
        )
        js_panels.append((f"Box · grasp #{idx}", Jj))
        rem -= 1
    return js_panels, J_box_js, J_tri_js


def save_joint_space_plot(scene: Scene, args, panels: List[Tuple[str, np.ndarray]]) -> None:
    plot_joint_space_by_grasp(
        scene.out_dir / "joint_space_by_grasp.png",
        panels,
        scene.q1_coords,
        scene.q2_coords,
        show=not args.no_show,
        panel_w=args.slice_panel_w,
        panel_h=args.slice_panel_h,
        cmap=args.theta_slice_cmap,
        vmin=args.theta_slice_vmin,
        vmax=args.theta_slice_vmax,
        contour_level=args.slice_contour_level,
        contour_color=args.theta_slice_contour_color,
        contour_lw=args.theta_slice_contour_lw,
        dpi=args.fig_dpi,
    )


def save_theta_slices_for_specs(
    scene: Scene, args, results: Dict[str, Tuple[np.ndarray, list]], specs: List[ObjectSpec], thetas: List[float]
) -> None:
    kw = dict(
        x_coords=scene.x_coords,
        y_coords=scene.y_coords,
        th_coords=scene.th_coords,
        theta_list=thetas,
        show=not args.no_show,
        panel_w=args.slice_panel_w,
        panel_h=args.slice_panel_h,
        cmap=args.theta_slice_cmap,
        vmin=args.theta_slice_vmin,
        vmax=args.theta_slice_vmax,
        contour_level=args.slice_contour_level,
        contour_color=args.theta_slice_contour_color,
        contour_lw=args.theta_slice_contour_lw,
        dpi=args.fig_dpi,
    )
    for spec in specs:
        F, _ = results[spec.out_stem]
        plot_theta_slices(
            scene.out_dir / f"theta_slices_{spec.out_stem}.png",
            F,
            title_prefix=spec.theta_slice_title,
            **kw,
        )


def save_grasp_compare_slice(
    scene: Scene, args, gs: GraspSampling, box_boundary, cmp_idx: List[int]
) -> None:
    fields_cmp = []
    for idx in cmp_idx[:4]:
        if idx < 0 or idx >= len(gs.g_box_pts):
            continue
        Rb, tb, thb = grasp_to_ee_object_transform(gs.g_box_pts[idx], gs.g_box_norm[idx])
        g = GraspTransform(Rb, tb, thb, f"box_grasp_{idx}")
        Fc, _, _, _, _ = build_feasibility_field_from_joint_sweep(
            g,
            scene.L1,
            scene.L2,
            scene.robot,
            box_boundary,
            scene.obstacles,
            args.robot_surface_n,
            scene.device,
            args.collision_margin,
            scene.x_coords,
            scene.y_coords,
            scene.th_coords,
            scene.q1_coords,
            scene.q2_coords,
            smooth_passes=args.field_smooth,
            max_feasible_q_store=0,
            deposit=args.field_deposit,
        )
        fields_cmp.append((f"grasp {idx}", Fc))
    if len(fields_cmp) >= 2:
        plot_grasp_compare_theta_slice(
            scene.out_dir / "grasp_compare_slice.png",
            fields_cmp,
            scene.x_coords,
            scene.y_coords,
            scene.th_coords,
            args.compare_theta_slice,
            show=not args.no_show,
            panel_w=args.slice_panel_w,
            panel_h=args.slice_panel_h,
            cmap=args.compare_slice_cmap,
            vmin=args.compare_slice_vmin,
            vmax=args.compare_slice_vmax,
            contour_level=args.slice_contour_level,
            contour_color=args.compare_slice_contour_color,
            contour_lw=args.compare_slice_contour_lw,
            dpi=args.fig_dpi,
        )


def save_collision_debug(
    scene: Scene, args, grasp_box: GraspTransform, box_boundary, gs: GraspSampling
) -> None:
    q_dbg = find_first_colliding_config(
        grasp_box,
        scene.L1,
        scene.L2,
        scene.robot,
        box_boundary,
        scene.obstacles,
        args.robot_surface_n,
        scene.device,
        args.collision_margin,
        scene.q1_coords,
        scene.q2_coords,
    )
    if q_dbg is None:
        return
    plot_collision_debug(
        scene.out_dir / "collision_debug.png",
        scene.obstacles,
        q_dbg[0],
        q_dbg[1],
        grasp_box,
        gs.box_w,
        gs.box_h,
        gs.tri,
        True,
        scene.L1,
        scene.L2,
        scene.robot,
        box_boundary,
        scene.device,
        args.robot_surface_n,
        args.collision_margin,
        show=not args.no_show,
        fig_size=args.collision_debug_fig_size,
        plot_lim=args.plot_lim,
        dpi=args.fig_dpi,
        grid_alpha=args.viz_grid_alpha,
    )


def save_feasible_poses_for_specs(
    scene: Scene, args, results: Dict[str, Tuple[np.ndarray, list]], specs: List[ObjectSpec], gs: GraspSampling
) -> None:
    for spec in specs:
        _, feasible_q = results[spec.out_stem]
        plot_feasible_poses_2d(
            scene.out_dir / f"feasible_poses_{spec.out_stem}.png",
            scene.obstacles,
            feasible_q,
            spec.grasp,
            gs.box_w,
            gs.box_h,
            gs.tri,
            spec.is_box,
            scene.L1,
            scene.L2,
            args.n_draw_2d,
            args.plot_lim,
            not args.no_show,
            fig_size=args.feasible_poses_fig_size,
            dpi=args.fig_dpi,
            grid_alpha=args.viz_grid_alpha,
            arm_sz=args.feasible_arm_sz,
            arm_alpha=args.feasible_arm_alpha,
            obj_fill_alpha=args.feasible_obj_alpha,
            poses_cmap=args.feasible_poses_cmap,
        )


def run_demo(args):
    scene = build_scene(args)
    gs, specs = build_grasp_sampling_and_specs(args)
    box_spec, tri_spec = specs[0], specs[1]
    box_boundary, tri_boundary = box_spec.boundary_fn, tri_spec.boundary_fn

    results: Dict[str, Tuple[np.ndarray, list]] = {}
    for spec in specs:
        F, fq = sweep_field_for_spec(scene, args, spec)
        results[spec.out_stem] = (F, fq)

    F_box, _ = results["box"]
    F_tri, _ = results["tri"]
    print(
        "feasible cells box:",
        int((F_box > 0).sum()),
        "triangle:",
        int((F_tri > 0).sum()),
        "/",
        F_box.size,
    )

    save_se2_isosurface(scene, args, F_box, F_tri)

    cmp_idx = parse_comma_ints(args.compare_grasp_indices)
    save_grasp_candidates(scene, args, gs, cmp_idx)

    if not args.no_joint_space_plot:
        panels, J_box_js, J_tri_js = build_joint_space_panels(
            scene, args, box_spec.grasp, tri_spec.grasp, box_boundary, tri_boundary, gs, cmp_idx
        )
        save_joint_space_plot(scene, args, panels)
        print(
            "feasible joint cells (q1,q2): box primary",
            int((J_box_js > 0).sum()),
            "/",
            J_box_js.size,
            " tri primary",
            int((J_tri_js > 0).sum()),
            "/",
            J_tri_js.size,
        )

    thetas = parse_comma_floats(args.slice_thetas)
    save_theta_slices_for_specs(scene, args, results, specs, thetas)
    save_grasp_compare_slice(scene, args, gs, box_boundary, cmp_idx)
    save_collision_debug(scene, args, box_spec.grasp, box_boundary, gs)
    save_feasible_poses_for_specs(scene, args, results, specs, gs)


if __name__ == "__main__":
    run_demo(parse_args())
