import { useEffect, useMemo, useRef } from "react";
import { PlotHeatmap } from "./PlotHeatmap";
import { interpFAtTheta } from "../lib/interpFTheta";
import { objectPoseFromJoints } from "../lib/fk";
import { useAppStore } from "../store/appStore";

export function SE2SlicePanel() {
  const result = useAppStore((s) => s.result);
  const se2Theta = useAppStore((s) => s.se2Theta);
  const setSe2Theta = useAppStore((s) => s.setSe2Theta);
  const jointPick = useAppStore((s) => s.jointPick);
  const payload = useAppStore((s) => s.payload);

  const thKey = result
    ? `${result.th_coords.length}_${result.th_coords[0]}_${result.th_coords[result.th_coords.length - 1]}_${result.J.length}`
    : "";
  const prevKey = useRef("");
  useEffect(() => {
    if (!result?.th_coords?.length || !result.F_theta_layers?.length) {
      prevKey.current = "";
      return;
    }
    if (prevKey.current === thKey) return;
    prevKey.current = thKey;
    const t = result.th_coords;
    setSe2Theta((t[0] + t[t.length - 1]) / 2);
  }, [thKey, result, setSe2Theta]);

  if (!result?.th_coords?.length || !result.F_theta_layers?.length) return null;

  const th = result.th_coords;
  const loData = th[0];
  const hiData = th[th.length - 1];
  const loSlider = -Math.PI;
  const hiSlider = Math.PI;
  const thetaClamped = Math.max(loSlider, Math.min(hiSlider, se2Theta));
  const thetaForF = Math.max(loData, Math.min(hiData, thetaClamped));
  const Fz = useMemo(
    () => interpFAtTheta(result.F_theta_layers, th, thetaForF),
    [result.F_theta_layers, th, thetaForF],
  );

  const se2Highlight =
    jointPick &&
    (() => {
      const { t_o } = objectPoseFromJoints(
        payload.link1,
        payload.link2,
        jointPick.q1,
        jointPick.q2,
        payload.grasp.contact as [number, number],
        payload.grasp.normal as [number, number],
      );
      return { x: t_o[0], y: t_o[1], feasible: jointPick.feasible };
    })();

  return (
    <div className="se2-panel">
      <div className="se2-bar">
        <span className="se2-lab">θ 连续</span>
        <input
          type="range"
          min={loSlider}
          max={hiSlider}
          step={(hiSlider - loSlider) / 4000}
          value={thetaClamped}
          onChange={(e) => setSe2Theta(+e.target.value)}
        />
        <span className="se2-val">
          {thetaForF.toFixed(3)} rad（滑条 [−π, π]，场在 th 网格内插值）
        </span>
      </div>
      <PlotHeatmap
        title={`F(x,y) · θ 插值切片（在 th 轴线性插值）`}
        z={Fz}
        x={result.x_coords}
        y={result.y_coords}
        cellPick={
          se2Highlight
            ? {
                valueGrid: Fz,
                highlight: se2Highlight,
                interactive: false,
                onCellPick: () => {},
              }
            : undefined
        }
      />
      <div className="se2-note">
        绿/红点：当前关节下物体原心 (x,y)。θ 在离散层之间线性插值，不改变原始场分辨率。
      </div>
    </div>
  );
}
