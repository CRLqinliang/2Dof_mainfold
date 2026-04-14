export type Obstacle =
  | { type: "circle"; cx: number; cy: number; r: number }
  | { type: "box"; cx: number; cy: number; w: number; h: number };

export type ObjectSpec =
  | { type: "box"; box_w: number; box_h: number }
  | { type: "triangle"; tri: [number, number][] }
  | { type: "polygon"; verts: [number, number][] };

export type Payload = {
  link1: number;
  link2: number;
  robot_surface_n: number;
  obj_surface_n: number;
  collision_margin: number;
  q1_min: number;
  q1_max: number;
  q1_n: number;
  q2_min: number;
  q2_max: number;
  q2_n: number;
  field_nx: number;
  field_ny: number;
  field_nz: number;
  field_smooth: number;
  field_deposit: string;
  object: ObjectSpec;
  grasp: { contact: [number, number]; normal: [number, number] };
  obstacles: Obstacle[];
  slice_thetas: number[];
};

export type ComputeSlice = {
  theta_request: number;
  /** 插值后的等效 θ（用于显示） */
  theta_eval: number;
  interp_k0: number;
  interp_k1: number;
  interp_alpha: number;
  F: number[][];
};

export type ComputeResult = {
  meta: Record<string, unknown>;
  grasp: { contact: number[]; normal: number[] };
  q1_coords: number[];
  q2_coords: number[];
  J: number[][];
  x_coords: number[];
  y_coords: number[];
  th_coords: number[];
  /** F[:,:,k]，与 th_coords[k] 对齐，用于连续 θ 插值 */
  F_theta_layers: number[][][];
  slices: ComputeSlice[];
  feasible_joint_cells: number;
  feasible_se2_cells: number;
  se2_grid_size: number;
};

export type ComputeOk = {
  ok: true;
  data: ComputeResult;
  job: { id: string; preview: boolean; ts: number };
};

export type ComputeErr = { ok: false; error: string };

/** POST /api/inspect-clearance 返回的碰撞裕度分解 */
export type ClearanceInspect = {
  clearance: number;
  clearance_robot: number;
  clearance_object: number;
  bottleneck: "robot" | "object" | "tie";
  worst_xy: [number, number];
};

export type Snapshot = {
  id: string;
  label: string;
  payload: Payload;
  result: ComputeResult | null;
  jobPreview: boolean;
};
