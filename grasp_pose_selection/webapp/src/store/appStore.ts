import { create } from "zustand";
import type { ClearanceInspect, ComputeResult, Payload, Snapshot } from "../types";

export type AppState = {
  payload: Payload;
  status: string;
  statusKind: "info" | "ok" | "err";
  result: ComputeResult | null;
  lastJobPreview: boolean;
  trajQ1: number[];
  trajQ2: number[];
  snapshots: Snapshot[];
  dragMode: "none" | "grasp" | "obs0" | "obs1";
  /** 在 J(q1,q2) 热图上手动选取的关节角；非空时 3D 优先显示该姿态 */
  jointPick: { q1: number; q2: number; feasible: boolean } | null;
  /** 连续 θ 切片（弧度），落在 th_coords 范围内 */
  se2Theta: number;
  /** 关节手动选取时 /api/inspect-clearance 的分解结果 */
  clearanceInspect: ClearanceInspect | null;
  setPayload: (p: Partial<Payload>) => void;
  replacePayload: (p: Payload) => void;
  setStatus: (kind: AppState["statusKind"], text: string) => void;
  setResult: (r: ComputeResult | null, preview: boolean) => void;
  setTrajectory: (q1: number[], q2: number[]) => void;
  addSnapshot: (label: string) => void;
  removeSnapshot: (id: string) => void;
  setDragMode: (m: AppState["dragMode"]) => void;
  setJointPick: (p: { q1: number; q2: number; feasible: boolean } | null) => void;
  clearJointPick: () => void;
  setSe2Theta: (t: number) => void;
  setClearanceInspect: (c: ClearanceInspect | null) => void;
};

const initialPayload = (): Payload => ({
  link1: 2,
  link2: 2,
  robot_surface_n: 32,
  obj_surface_n: 48,
  collision_margin: 0,
  q1_min: -Math.PI,
  q1_max: Math.PI,
  q1_n: 48,
  q2_min: -Math.PI,
  q2_max: Math.PI,
  q2_n: 48,
  field_nx: 40,
  field_ny: 40,
  field_nz: 20,
  field_smooth: 0,
  field_deposit: "cell",
  object: { type: "box", box_w: 0.8, box_h: 0.8 },
  grasp: { contact: [0, 0.4], normal: [0, -1] },
  obstacles: [
    { type: "circle", cx: -2.2, cy: -1.8, r: 0.5 },
    { type: "box", cx: 1.5, cy: 1, w: 0.5, h: 0.5 },
  ],
  slice_thetas: [0],
});

export const useAppStore = create<AppState>((set, get) => ({
  payload: initialPayload(),
  status: "就绪",
  statusKind: "info",
  result: null,
  lastJobPreview: false,
  trajQ1: [0],
  trajQ2: [0],
  snapshots: [],
  dragMode: "none",
  jointPick: null,
  se2Theta: 0,
  clearanceInspect: null,
  setPayload: (partial) =>
    set((s) => ({
      payload: { ...s.payload, ...partial },
    })),
  replacePayload: (p) => set({ payload: p }),
  setStatus: (kind, text) => set({ statusKind: kind, status: text }),
  setResult: (r, preview) =>
    set({
      result: r,
      lastJobPreview: preview,
      jointPick: null,
      se2Theta: 0,
      clearanceInspect: null,
    }),
  setTrajectory: (q1, q2) => set({ trajQ1: q1, trajQ2: q2 }),
  addSnapshot: (label) => {
    const id = crypto.randomUUID();
    const { payload, result, lastJobPreview } = get();
    set({
      snapshots: [
        ...get().snapshots,
        { id, label, payload: structuredClone(payload), result, jobPreview: lastJobPreview },
      ],
    });
  },
  removeSnapshot: (id) =>
    set({ snapshots: get().snapshots.filter((s) => s.id !== id) }),
  setDragMode: (m) => set({ dragMode: m }),
  setJointPick: (p) => set({ jointPick: p }),
  clearJointPick: () => set({ jointPick: null, clearanceInspect: null }),
  setSe2Theta: (t) => set({ se2Theta: t }),
  setClearanceInspect: (c) => set({ clearanceInspect: c }),
}));
