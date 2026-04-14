const PI = Math.PI;

export function wrapPi(a: number) {
  const t = 2 * PI;
  return ((((a + PI) % t) + t) % t) - PI;
}

export function fk2d(L1: number, L2: number, q1: number, q2: number) {
  const j1x = L1 * Math.cos(q1);
  const j1y = L1 * Math.sin(q1);
  const eex = j1x + L2 * Math.cos(q1 + q2);
  const eey = j1y + L2 * Math.sin(q1 + q2);
  return {
    base: [0, 0] as [number, number],
    j1: [j1x, j1y] as [number, number],
    ee: [eex, eey] as [number, number],
    thetaEe: q1 + q2,
  };
}

/** 与 grasp_geometry.grasp_to_ee_object_transform 一致 */
export function graspToEeObjectTransform(contact: [number, number], normal: [number, number]) {
  const nn = Math.hypot(normal[0], normal[1]) + 1e-12;
  const nx = normal[0] / nn;
  const ny = normal[1] / nn;
  const ang = Math.atan2(ny, nx);
  const ca = Math.cos(-ang);
  const sa = Math.sin(-ang);
  const t_eo: [number, number] = [
    -(ca * contact[0] - sa * contact[1]),
    -(sa * contact[0] + ca * contact[1]),
  ];
  const theta_eo = -ang;
  return { t_eo, theta_eo };
}

/** 与 grasp_geometry.object_pose_world 一致：物体原心世界位姿 (t_o, theta_o) */
export function objectPoseWorld(
  pEe: [number, number],
  thetaE: number,
  t_eo: [number, number],
  theta_eo: number,
) {
  const ce = Math.cos(thetaE);
  const se = Math.sin(thetaE);
  const t_o: [number, number] = [
    pEe[0] + ce * t_eo[0] - se * t_eo[1],
    pEe[1] + se * t_eo[0] + ce * t_eo[1],
  ];
  return { t_o, theta_o: wrapPi(thetaE + theta_eo) };
}

export function objPointToWorld(p: [number, number], t_o: [number, number], theta_o: number) {
  const c = Math.cos(theta_o);
  const s = Math.sin(theta_o);
  return [t_o[0] + c * p[0] - s * p[1], t_o[1] + s * p[0] + c * p[1]] as [number, number];
}

/** 世界系点 → 物体系（与 objPointToWorld 互逆） */
export function worldToObjectLocal(
  t_o: [number, number],
  theta_o: number,
  p: [number, number],
): [number, number] {
  const dx = p[0] - t_o[0];
  const dy = p[1] - t_o[1];
  const c = Math.cos(-theta_o);
  const s = Math.sin(-theta_o);
  return [c * dx - s * dy, s * dx + c * dy];
}

/** 由关节角得到物体原心位姿（与后端 object_pose_from_q 一致） */
export function objectPoseFromJoints(
  L1: number,
  L2: number,
  q1: number,
  q2: number,
  contact: [number, number],
  normal: [number, number],
) {
  const k = fk2d(L1, L2, q1, q2);
  const { t_eo, theta_eo } = graspToEeObjectTransform(contact, normal);
  const { t_o, theta_o } = objectPoseWorld(k.ee, k.thetaEe, t_eo, theta_eo);
  return { t_o, theta_o, theta_eo, ee: k.ee, thetaEe: k.thetaEe };
}
