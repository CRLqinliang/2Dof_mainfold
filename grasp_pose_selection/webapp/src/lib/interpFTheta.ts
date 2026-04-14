/** 与 grasp_web_compute.interp_f_theta_plane 一致的线性插值 */
export function interpFAtTheta(
  layers: number[][][],
  thCoords: number[],
  theta: number,
): number[][] {
  const nz = thCoords.length;
  if (nz === 0) return [];
  if (nz === 1) return layers[0];
  const lo = thCoords[0];
  const hi = thCoords[nz - 1];
  if (theta <= lo) return layers[0];
  if (theta >= hi) return layers[nz - 1];
  let j = 1;
  while (j < nz && thCoords[j] < theta) j++;
  const k0 = j - 1;
  const k1 = Math.min(nz - 1, j);
  if (k0 === k1) return layers[k0];
  const t0 = thCoords[k0];
  const t1 = thCoords[k1];
  const denom = t1 - t0;
  const a = denom > 1e-12 ? (theta - t0) / denom : 0;
  const A = layers[k0];
  const B = layers[k1];
  return A.map((row, i) => row.map((v, ii) => (1 - a) * v + a * B[i][ii]));
}
