# 连续场等值面与辅助可视化
import math
from pathlib import Path
from typing import Callable, List, Sequence, Tuple

import matplotlib.pyplot as plt
import numpy as np
import torch
import robot_plot2D
from matplotlib.colors import to_rgba
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from grasp_feasibility import GraspTransform, min_sdf_points_worst, object_pose_from_q
from grasp_geometry import box_boundary_world, triangle_boundary_world


def plot_feasibility_isosurface(
    ax,
    F: np.ndarray,
    x_coords: np.ndarray,
    y_coords: np.ndarray,
    th_coords: np.ndarray,
    iso_level: float = 0.0,
    color: str = "steelblue",
    alpha: float = 0.45,
    title: str = "",
    stride_theta: int = 1,
    stride_xy: int = 2,
    lw_xy: float = 1.0,
    lw_slice: float = 0.7,
    fill_theta_layers: bool = True,
    fill_alpha: float = 0.12,
    fill_stride_theta: int = 1,
):
    """
    在规则网格上显示 F(x,y,theta)=iso_level：theta 向可叠半透明面片 + x/y/theta 向等值线网，
    表现 SE(2)×R 中可行域的实体感（场由关节 sweep +网格 cell 角点沉积得到）。
    """
    ax.set_title(title)
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_zlabel("theta")
    if float(np.nanmax(F)) < iso_level or float(np.nanmin(F)) > iso_level:
        ax.text2D(0.05, 0.9, "No iso crossing at this level", transform=ax.transAxes)
        return
    X, Y = np.meshgrid(x_coords, y_coords, indexing="ij")
    st = max(1, stride_theta)
    fst = max(1, fill_stride_theta)
    rgba_fill = to_rgba(color, fill_alpha)
    for k in range(len(th_coords)):
        Zsl = F[:, :, k]
        zmin, zmax = float(np.nanmin(Zsl)), float(np.nanmax(Zsl))
        if zmin > iso_level or zmax < iso_level:
            continue
        want_line = k % st == 0
        want_fill = fill_theta_layers and (k % fst == 0)
        if not want_line and not want_fill:
            continue
        cs = ax.contour(
            X,
            Y,
            Zsl,
            levels=[iso_level],
            zdir="z",
            offset=th_coords[k],
            colors=color,
            linewidths=lw_xy if want_line else 0.0,
            alpha=alpha if want_line else 0.0,
        )
        if want_fill and hasattr(cs, "allsegs") and len(cs.allsegs) > 0:
            polys = []
            for seg in cs.allsegs[0]:
                if seg.shape[0] < 3:
                    continue
                polys.append(
                    np.column_stack(
                        [
                            seg[:, 0].astype(np.float64),
                            seg[:, 1].astype(np.float64),
                            np.full(seg.shape[0], float(th_coords[k]), dtype=np.float64),
                        ]
                    )
                )
            if polys:
                ax.add_collection3d(
                    Poly3DCollection(
                        polys,
                        facecolors=[rgba_fill] * len(polys),
                        edgecolors="none",
                        linewidths=0,
                    )
                )
    for i in range(0, len(x_coords), max(1, stride_xy)):
        Y2, TH2 = np.meshgrid(y_coords, th_coords, indexing="ij")
        ax.contour(
            Y2,
            TH2,
            F[i, :, :],
            levels=[iso_level],
            zdir="x",
            offset=x_coords[i],
            colors=color,
            linewidths=lw_slice,
            alpha=alpha * 0.85,
        )
    for j in range(0, len(y_coords), max(1, stride_xy)):
        X2, TH2 = np.meshgrid(x_coords, th_coords, indexing="ij")
        ax.contour(
            X2,
            TH2,
            F[:, j, :],
            levels=[iso_level],
            zdir="y",
            offset=y_coords[j],
            colors=color,
            linewidths=lw_slice,
            alpha=alpha * 0.85,
        )
    ax.set_xlim(x_coords.min(), x_coords.max())
    ax.set_ylim(y_coords.min(), y_coords.max())
    ax.set_zlim(th_coords.min(), th_coords.max())


def plot_grasp_candidates_figure(
    out_path: Path,
    box_w: float,
    box_h: float,
    g_box_pts: np.ndarray,
    g_box_norm: np.ndarray,
    g_tri_pts: np.ndarray,
    g_tri_norm: np.ndarray,
    pick_box_idx: int,
    pick_tri_idx: int,
    compare_indices_box: Sequence[int],
    compare_indices_tri: Sequence[int],
    show: bool,
    fig_w: float = 11.0,
    fig_h: float = 5.0,
    dpi: int = 150,
    grid_alpha: float = 0.3,
    quiver_scale: float = 12.0,
):
    fig, axes = plt.subplots(1, 2, figsize=(fig_w, fig_h))
    lim = max(box_w, box_h, float(np.max(g_tri_pts))) * 2.5 + 0.3

    def draw_one(ax, pts, norms, title, pick_i, cmp_idx):
        ax.set_aspect("equal")
        ax.set_xlim(-lim, lim)
        ax.set_ylim(-lim, lim)
        ax.grid(True, alpha=grid_alpha)
        ax.set_title(title)
        ax.plot(np.r_[pts[:, 0], pts[0, 0]], np.r_[pts[:, 1], pts[0, 1]], "k-", lw=1)
        ax.quiver(
            pts[:, 0],
            pts[:, 1],
            norms[:, 0],
            norms[:, 1],
            scale=quiver_scale,
            color="gray",
            width=0.004,
        )
        ax.scatter(pts[:, 0], pts[:, 1], s=18, c="lightgray", zorder=3)
        ax.scatter(
            pts[pick_i, 0],
            pts[pick_i, 1],
            s=120,
            c="red",
            marker="*",
            zorder=5,
            label="primary grasp",
        )
        for j, ci in enumerate(cmp_idx):
            if ci == pick_i or ci < 0 or ci >= len(pts):
                continue
            ax.scatter(
                pts[ci, 0],
                pts[ci, 1],
                s=55,
                c="orange",
                marker="o",
                zorder=4,
            )
        ax.legend(loc="upper right", fontsize=8)

    draw_one(
        axes[0],
        g_box_pts,
        g_box_norm,
        "Box: candidates + inward normals",
        pick_box_idx,
        compare_indices_box,
    )
    draw_one(
        axes[1],
        g_tri_pts,
        g_tri_norm,
        "Triangle: candidates + inward normals",
        pick_tri_idx,
        compare_indices_tri,
    )
    plt.tight_layout()
    plt.savefig(out_path, dpi=dpi)
    if show:
        plt.show()
    else:
        plt.close(fig)


def plot_theta_slices(
    out_path: Path,
    F: np.ndarray,
    x_coords: np.ndarray,
    y_coords: np.ndarray,
    th_coords: np.ndarray,
    theta_list: List[float],
    title_prefix: str,
    show: bool,
    panel_w: float = 4.0,
    panel_h: float = 4.0,
    cmap: str = "RdYlGn",
    vmin: float = -0.5,
    vmax: float = 0.5,
    contour_level: float = 0.0,
    contour_color: str = "k",
    contour_lw: float = 1.2,
    dpi: int = 150,
):
    n = len(theta_list)
    fig, axes = plt.subplots(1, n, figsize=(panel_w * n, panel_h), squeeze=False)
    for ax, th_des in zip(axes[0], theta_list):
        k = int(np.argmin(np.abs(th_coords - th_des)))
        sl = F[:, :, k].T
        im = ax.imshow(
            sl,
            origin="lower",
            extent=[
                x_coords.min(),
                x_coords.max(),
                y_coords.min(),
                y_coords.max(),
            ],
            aspect="auto",
            cmap=cmap,
            vmin=vmin,
            vmax=vmax,
        )
        X, Y = np.meshgrid(x_coords, y_coords, indexing="ij")
        ax.contour(
            X, Y, F[:, :, k], levels=[contour_level], colors=contour_color, linewidths=contour_lw
        )
        ax.set_title(f"{title_prefix}\n theta={th_coords[k]:.3f}")
        ax.set_xlabel("x")
        ax.set_ylabel("y")
        fig.colorbar(im, ax=ax, fraction=0.046, label="clearance")
    plt.tight_layout()
    plt.savefig(out_path, dpi=dpi)
    if show:
        plt.show()
    else:
        plt.close(fig)


def plot_joint_space_by_grasp(
    out_path: Path,
    panels: List[Tuple[str, np.ndarray]],
    q1_coords: np.ndarray,
    q2_coords: np.ndarray,
    show: bool,
    panel_w: float = 3.8,
    panel_h: float = 3.5,
    cmap: str = "RdYlGn",
    vmin: float = -0.5,
    vmax: float = 0.5,
    contour_level: float = 0.0,
    contour_color: str = "k",
    contour_lw: float = 1.2,
    dpi: int = 150,
):
    """不同 grasp 下关节空间 (q1,q2) 的 clearance 热图；黑线为 F=contour_level（默认可行边界）。"""
    n = len(panels)
    ncols = min(3, n)
    nrows = int(math.ceil(n / ncols))
    fig, axes = plt.subplots(
        nrows, ncols, figsize=(panel_w * ncols, panel_h * nrows), squeeze=False
    )
    Q1, Q2 = np.meshgrid(q1_coords, q2_coords, indexing="ij")
    for idx, (title, J) in enumerate(panels):
        r, c = divmod(idx, ncols)
        ax = axes[r][c]
        im = ax.imshow(
            J.T,
            origin="lower",
            extent=[
                float(q1_coords[0]),
                float(q1_coords[-1]),
                float(q2_coords[0]),
                float(q2_coords[-1]),
            ],
            aspect="auto",
            cmap=cmap,
            vmin=vmin,
            vmax=vmax,
        )
        ax.contour(
            Q1, Q2, J, levels=[contour_level], colors=contour_color, linewidths=contour_lw
        )
        ax.set_title(title)
        ax.set_xlabel("q1")
        ax.set_ylabel("q2")
        fig.colorbar(im, ax=ax, fraction=0.046, label="clearance")
    for k in range(len(panels), nrows * ncols):
        r, c = divmod(k, ncols)
        axes[r][c].set_visible(False)
    fig.suptitle("Joint-space feasibility by grasp (same q grid)", y=1.02, fontsize=11)
    plt.tight_layout()
    plt.savefig(out_path, dpi=dpi, bbox_inches="tight")
    if show:
        plt.show()
    else:
        plt.close(fig)


def plot_grasp_compare_theta_slice(
    out_path: Path,
    fields: List[Tuple[str, np.ndarray]],
    x_coords: np.ndarray,
    y_coords: np.ndarray,
    th_coords: np.ndarray,
    theta_des: float,
    show: bool,
    panel_w: float = 4.0,
    panel_h: float = 4.0,
    cmap: str = "viridis",
    vmin: float = -0.3,
    vmax: float = 0.4,
    contour_level: float = 0.0,
    contour_color: str = "w",
    contour_lw: float = 1.0,
    dpi: int = 150,
):
    n = len(fields)
    fig, axes = plt.subplots(1, n, figsize=(panel_w * n, panel_h), squeeze=False)
    k = int(np.argmin(np.abs(th_coords - theta_des)))
    for ax, (name, F) in zip(axes[0], fields):
        sl = F[:, :, k].T
        im = ax.imshow(
            sl,
            origin="lower",
            extent=[
                x_coords.min(),
                x_coords.max(),
                y_coords.min(),
                y_coords.max(),
            ],
            aspect="auto",
            cmap=cmap,
            vmin=vmin,
            vmax=vmax,
        )
        X, Y = np.meshgrid(x_coords, y_coords, indexing="ij")
        ax.contour(
            X, Y, F[:, :, k], levels=[contour_level], colors=contour_color, linewidths=contour_lw
        )
        ax.set_title(f"{name}\n theta={th_coords[k]:.3f}")
        ax.set_xlabel("x")
        ax.set_ylabel("y")
        fig.colorbar(im, ax=ax, fraction=0.046)
    plt.tight_layout()
    plt.savefig(out_path, dpi=dpi)
    if show:
        plt.show()
    else:
        plt.close(fig)


def plot_collision_debug(
    out_path: Path,
    obstacles,
    q1: float,
    q2: float,
    grasp: GraspTransform,
    box_w: float,
    box_h: float,
    tri: np.ndarray,
    is_box: bool,
    L1: float,
    L2: float,
    robot,
    obj_fn: Callable[[float, np.ndarray], np.ndarray],
    device,
    robot_surface_n: int,
    collision_margin: float,
    show: bool,
    fig_size: float = 7.0,
    plot_lim: float = 4.5,
    dpi: int = 150,
    grid_alpha: float = 0.3,
):
    fig, ax = plt.subplots(figsize=(fig_size, fig_size))
    for obs in obstacles:
        ax.add_patch(obs.create_patch())
    lim = plot_lim
    ax.set_aspect("equal")
    ax.set_xlim(-lim, lim)
    ax.set_ylim(-lim, lim)
    ax.grid(True, alpha=grid_alpha)

    t_o, th_obj = object_pose_from_q(q1, q2, L1, L2, grasp)
    rk = robot.surface_points_sampler(
        torch.tensor([[q1, q2]], dtype=torch.float32, device=device),
        n=robot_surface_n,
    )[0].reshape(-1, 2)
    ob = obj_fn(th_obj, t_o)
    pts = np.vstack([rk.cpu().numpy(), ob])
    dmin, worst = min_sdf_points_worst(obstacles, pts, device)

    robot_plot2D.plotArm(
        ax=ax,
        a=np.array([q1, q2]),
        d=[L1, L2],
        p=np.array([0.0, 0.0]),
        sz=0.06,
        robot_base=True,
        edgecol="k",
    )
    if is_box:
        poly = box_boundary_world(t_o[0], t_o[1], box_w, box_h, th_obj)
    else:
        poly = triangle_boundary_world(tri, th_obj, t_o)
    ax.fill(poly[:, 0], poly[:, 1], facecolor="cyan", edgecolor="k", alpha=0.35)
    ax.scatter([worst[0]], [worst[1]], c="red", s=80, zorder=6, label="min SDF sample")
    ax.set_title(
        f"Collision debug  clearance={dmin - collision_margin:.4f}\n"
        f"(min SDF={dmin:.4f}, margin={collision_margin})"
    )
    ax.legend()
    plt.tight_layout()
    plt.savefig(out_path, dpi=dpi)
    if show:
        plt.show()
    else:
        plt.close(fig)


def plot_feasible_poses_2d(
    out_path: Path,
    obstacles,
    feasible_q: List[Tuple[float, float, float]],
    grasp: GraspTransform,
    box_w: float,
    box_h: float,
    tri: np.ndarray,
    is_box: bool,
    L1: float,
    L2: float,
    n_draw: int,
    plot_lim: float,
    show: bool,
    fig_size: float = 7.0,
    dpi: int = 150,
    grid_alpha: float = 0.3,
    arm_sz: float = 0.06,
    arm_alpha: float = 0.85,
    obj_fill_alpha: float = 0.4,
    poses_cmap: str = "viridis",
):
    """用关节 sweep 中记录的 clearance>0 的 (q1,q2) 画臂与物体（FK+固定 grasp）。"""
    fig, ax = plt.subplots(figsize=(fig_size, fig_size))
    for obs in obstacles:
        ax.add_patch(obs.create_patch())
    ax.set_aspect("equal")
    ax.set_xlim(-plot_lim, plot_lim)
    ax.set_ylim(-plot_lim, plot_lim)
    ax.grid(True, alpha=grid_alpha)

    cand = sorted(feasible_q, key=lambda t: -t[2])
    if not cand:
        ax.set_title("No feasible joint samples (clearance>0)")
        plt.savefig(out_path, dpi=dpi)
        if show:
            plt.show()
        else:
            plt.close(fig)
        return

    step = max(1, len(cand) // n_draw)
    cmap = plt.get_cmap(poses_cmap)(np.linspace(0, 1, n_draw))
    drawn = 0
    for idx in range(0, len(cand), step):
        if drawn >= n_draw:
            break
        q1, q2, _ = cand[idx]
        robot_plot2D.plotArm(
            ax=ax,
            a=np.array([q1, q2]),
            d=[L1, L2],
            p=np.array([0.0, 0.0]),
            sz=arm_sz,
            facecol=cmap[drawn][:3],
            edgecol="k",
            robot_base=True,
            alpha=arm_alpha,
        )
        t_o_fk, theta_o_fk = object_pose_from_q(q1, q2, L1, L2, grasp)
        if is_box:
            poly = box_boundary_world(t_o_fk[0], t_o_fk[1], box_w, box_h, theta_o_fk)
        else:
            poly = triangle_boundary_world(tri, theta_o_fk, t_o_fk)
        ax.fill(
            poly[:, 0],
            poly[:, 1],
            facecolor=cmap[drawn],
            edgecolor="k",
            alpha=obj_fill_alpha,
        )
        drawn += 1
    ax.set_title("Feasible poses (joint sweep, FK + grasp)")
    plt.tight_layout()
    plt.savefig(out_path, dpi=dpi)
    if show:
        plt.show()
    else:
        plt.close(fig)

