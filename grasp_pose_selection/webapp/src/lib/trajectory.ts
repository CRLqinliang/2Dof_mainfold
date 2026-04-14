import type { ComputeResult } from "../types";

/** 在相邻可行格之间加密采样（每段 substeps 个插值点）。 */
function densifyPath(
  path: [number, number][],
  q1_coords: number[],
  q2_coords: number[],
  substeps: number,
): { q1: number[]; q2: number[] } {
  const q1: number[] = [];
  const q2: number[] = [];
  if (path.length === 0) return { q1: [0], q2: [0] };
  for (let p = 0; p < path.length - 1; p++) {
    const [i0, j0] = path[p];
    const [i1, j1] = path[p + 1];
    for (let s = 0; s < substeps; s++) {
      const t = s / substeps;
      q1.push(q1_coords[i0] * (1 - t) + q1_coords[i1] * t);
      q2.push(q2_coords[j0] * (1 - t) + q2_coords[j1] * t);
    }
  }
  const [ie, je] = path[path.length - 1];
  q1.push(q1_coords[ie]);
  q2.push(q2_coords[je]);
  return { q1, q2 };
}

function smallOrbit(q1c: number, q2c: number) {
  const amp = 0.12;
  const q1: number[] = [];
  const q2: number[] = [];
  for (let s = 0; s <= 48; s++) {
    const t = (s / 48) * Math.PI * 2;
    q1.push(q1c + amp * Math.sin(t));
    q2.push(q2c + amp * Math.cos(t));
  }
  return { q1, q2 };
}

/**
 * 在 J>0 的格子上做 BFS，从第一个可行格走到「从起点 BFS 可达」的较远格，
 * 再沿格子中心折线加密。避免在关节空间对两个可行点直线插值（中间常不可行，
 * 3D 里会像穿过障碍）。
 */
export function trajectoryFromJointResult(res: ComputeResult) {
  const { J, q1_coords, q2_coords } = res;
  const n = q1_coords.length;
  const m = q2_coords.length;
  let si = -1;
  let sj = -1;
  outer: for (let i = 0; i < n; i++) {
    for (let j = 0; j < m; j++) {
      if (J[i][j] > 0) {
        si = i;
        sj = j;
        break outer;
      }
    }
  }
  if (si < 0) {
    return { q1: [0], q2: [0] };
  }

  const key = (i: number, j: number) => `${i},${j}`;
  const parent = new Map<string, string | null>();
  const q: [number, number][] = [[si, sj]];
  parent.set(key(si, sj), null);
  let best: [number, number] = [si, sj];
  let bestScore = 0;
  const dirs = [
    [1, 0],
    [-1, 0],
    [0, 1],
    [0, -1],
  ];

  while (q.length) {
    const [i, j] = q.shift()!;
    const score = (i - si) ** 2 + (j - sj) ** 2;
    if (score > bestScore) {
      bestScore = score;
      best = [i, j];
    }
    for (const [di, dj] of dirs) {
      const ni = i + di;
      const nj = j + dj;
      if (ni < 0 || nj < 0 || ni >= n || nj >= m) continue;
      if (J[ni][nj] <= 0) continue;
      const nk = key(ni, nj);
      if (parent.has(nk)) continue;
      parent.set(nk, key(i, j));
      q.push([ni, nj]);
    }
  }

  const path: [number, number][] = [];
  let cur: string | null = key(best[0], best[1]);
  const used = new Set<string>();
  while (cur != null && !used.has(cur)) {
    used.add(cur);
    const parts = cur.split(",").map(Number) as [number, number];
    path.push(parts);
    cur = parent.get(cur) ?? null;
  }
  path.reverse();

  if (path.length <= 1) {
    return smallOrbit(q1_coords[si], q2_coords[sj]);
  }

  return densifyPath(path, q1_coords, q2_coords, 8);
}
