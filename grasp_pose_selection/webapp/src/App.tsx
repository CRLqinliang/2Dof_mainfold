import { useCallback, useEffect, useMemo, useRef } from "react";
import { computeFull, computePreview, fetchDefaults, inspectClearance } from "./api/compute";
import { ComparePanel } from "./components/ComparePanel";
import { ControlPanel } from "./components/ControlPanel";
import { ObjectShapeEditor } from "./components/ObjectShapeEditor";
import { PlotHeatmap } from "./components/PlotHeatmap";
import { SE2SlicePanel } from "./components/SE2SlicePanel";
import { Workspace2D } from "./components/Workspace2D";
import { objectPoseFromJoints, wrapPi } from "./lib/fk";
import { trajectoryFromJointResult } from "./lib/trajectory";
import { useAppStore } from "./store/appStore";
import type { Payload } from "./types";

function useDebouncedPreview(payload: Payload, delay: number) {
  const setResult = useAppStore((s) => s.setResult);
  const setStatus = useAppStore((s) => s.setStatus);
  const setTrajectory = useAppStore((s) => s.setTrajectory);
  const acRef = useRef<AbortController | null>(null);
  const payloadJson = JSON.stringify(payload);

  useEffect(() => {
    const t = window.setTimeout(() => {
      if (acRef.current) acRef.current.abort();
      const ac = new AbortController();
      acRef.current = ac;
      computePreview(JSON.parse(payloadJson) as Payload, ac.signal)
        .then((out) => {
          setResult(out.data, true);
          const tr = trajectoryFromJointResult(out.data);
          setTrajectory(tr.q1, tr.q2);
          setStatus("info", "预览完成（低分辨率）");
        })
        .catch((e: Error) => {
          if (e.name === "AbortError") return;
          setStatus("err", "预览失败: " + e.message);
        });
    }, delay);
    return () => clearTimeout(t);
  }, [payloadJson, delay, setResult, setStatus, setTrajectory]);
}

export default function App() {
  const payload = useAppStore((s) => s.payload);
  const replacePayload = useAppStore((s) => s.replacePayload);
  const result = useAppStore((s) => s.result);
  const setStatus = useAppStore((s) => s.setStatus);
  const setResult = useAppStore((s) => s.setResult);
  const status = useAppStore((s) => s.status);
  const statusKind = useAppStore((s) => s.statusKind);
  const setTrajectory = useAppStore((s) => s.setTrajectory);
  const lastJobPreview = useAppStore((s) => s.lastJobPreview);
  const jointPick = useAppStore((s) => s.jointPick);
  const setJointPick = useAppStore((s) => s.setJointPick);
  const clearJointPick = useAppStore((s) => s.clearJointPick);
  const setSe2Theta = useAppStore((s) => s.setSe2Theta);
  const setClearanceInspect = useAppStore((s) => s.setClearanceInspect);

  const payloadJson = useMemo(() => JSON.stringify(payload), [payload]);

  const onJointCellPick = useCallback(
    (q1: number, q2: number, meta: { feasible: boolean; i: number; j: number }) => {
      setJointPick({ q1, q2, feasible: meta.feasible });
    },
    [setJointPick],
  );

  useDebouncedPreview(payload, 320);

  useEffect(() => {
    if (!jointPick || !result?.th_coords?.length) return;
    const { theta_o } = objectPoseFromJoints(
      payload.link1,
      payload.link2,
      jointPick.q1,
      jointPick.q2,
      payload.grasp.contact as [number, number],
      payload.grasp.normal as [number, number],
    );
    const th = result.th_coords;
    const tw = wrapPi(theta_o);
    setSe2Theta(Math.max(th[0], Math.min(th[th.length - 1], tw)));
  }, [jointPick, result, payload, setSe2Theta]);

  useEffect(() => {
    if (!jointPick) {
      setClearanceInspect(null);
      return;
    }
    const ac = new AbortController();
    const tid = window.setTimeout(() => {
      inspectClearance(JSON.parse(payloadJson) as Payload, jointPick.q1, jointPick.q2, ac.signal)
        .then(setClearanceInspect)
        .catch((e: Error) => {
          if (e.name === "AbortError") return;
          setClearanceInspect(null);
        });
    }, 120);
    return () => {
      clearTimeout(tid);
      ac.abort();
    };
  }, [jointPick, payloadJson, setClearanceInspect]);

  const onFull = useCallback(() => {
    computeFull(payload)
      .then((out) => {
        setResult(out.data, false);
        const tr = trajectoryFromJointResult(out.data);
        setTrajectory(tr.q1, tr.q2);
        const jn = out.data.q1_coords.length * out.data.q2_coords.length;
        setStatus(
          "ok",
          `全量完成 · J+ ${out.data.feasible_joint_cells}/${jn} · SE2+ ${out.data.feasible_se2_cells}/${out.data.se2_grid_size}`,
        );
      })
      .catch((e: Error) => setStatus("err", "计算失败: " + e.message));
  }, [payload, setResult, setStatus, setTrajectory]);

  const onDefaults = useCallback(() => {
    fetchDefaults()
      .then((d) => {
        replacePayload(d);
        setStatus("info", "已载入默认参数");
      })
      .catch((e: Error) => setStatus("err", "默认参数: " + e.message));
  }, [replacePayload, setStatus]);

  const statsEl =
    result &&
    (() => {
      const jn = result.q1_coords.length * result.q2_coords.length;
      const jp = jn ? (100 * result.feasible_joint_cells) / jn : 0;
      const sp =
        result.se2_grid_size > 0 ? (100 * result.feasible_se2_cells) / result.se2_grid_size : 0;
      return (
        <div className="app-status-stats">
          <span>
            J+ {result.feasible_joint_cells}/{jn} ({jp.toFixed(0)}%)
          </span>
          <span>
            SE(2)+ {result.feasible_se2_cells}/{result.se2_grid_size} ({sp.toFixed(0)}%)
          </span>
        </div>
      );
    })();

  return (
    <div className="app-layout">
      <header className="app-top">
        <h1>Grasp 交互演示</h1>
        <div className="app-top-actions">
          <button type="button" className="btn btn-primary" onClick={onFull}>
            全量计算
          </button>
          <button type="button" className="btn" onClick={onDefaults}>
            默认参数
          </button>
        </div>
      </header>

      <div className={`app-status-row ${statusKind}`}>
        <div className="app-status-msg">
          {status}
          {lastJobPreview && result ? " · 当前为预览分辨率" : ""}
        </div>
        {statsEl}
      </div>

      <div className="app-grid">
        <aside className="app-sidebar">
          <ControlPanel />
          <ObjectShapeEditor />
          <ComparePanel />
        </aside>
        <main className="app-main">
          <div className="main-split">
            <div className="main-pane main-pane--workspace">
              <h2 className="main-pane-title">工作区</h2>
              <Workspace2D />
            </div>
            <div className="main-pane main-pane--analysis">
              <h2 className="main-pane-title">关节空间与 SE(2) 切片</h2>
              {result ? (
                <>
                  <PlotHeatmap
                    title="J(q₁,q₂) · 范围 [−π, π]"
                    z={result.J}
                    x={result.q1_coords}
                    y={result.q2_coords}
                    cellPick={{
                      valueGrid: result.J,
                      highlight: jointPick
                        ? { x: jointPick.q1, y: jointPick.q2, feasible: jointPick.feasible }
                        : null,
                      onCellPick: onJointCellPick,
                    }}
                  />
                  <div className="row-joint">
                    <button
                      type="button"
                      className="btn-clear-joint"
                      disabled={!jointPick}
                      onClick={clearJointPick}
                    >
                      清除关节选取（恢复轨迹）
                    </button>
                  </div>
                  <SE2SlicePanel />
                </>
              ) : (
                <div className="placeholder-block">等待计算…</div>
              )}
            </div>
          </div>
        </main>
      </div>

      <p className="app-foot">
        开发：<code>cd webapp && npm i && npm run dev</code>，另开终端在 <code>web/</code> 下{" "}
        <code>python server.py</code>。生产静态页需设置 <code>VITE_API_BASE_URL</code> 指向已部署的后端，并用{" "}
        <code>VITE_BASE_PATH</code> 匹配 GitHub Pages 子路径。
      </p>
    </div>
  );
}
