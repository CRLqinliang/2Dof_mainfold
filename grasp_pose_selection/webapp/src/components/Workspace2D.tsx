import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import {
  fk2d,
  graspToEeObjectTransform,
  objectPoseWorld,
  objPointToWorld,
  worldToObjectLocal,
} from "../lib/fk";
import type { ObjectSpec } from "../types";
import { useAppStore } from "../store/appStore";

function clamp(v: number, a: number, b: number) {
  return Math.max(a, Math.min(b, v));
}

function objectPathWorld(object: ObjectSpec, t_o: [number, number], theta_o: number): string {
  const c = Math.cos(theta_o);
  const s = Math.sin(theta_o);
  const wxy = (x: number, y: number) => {
    const wx = c * x - s * y + t_o[0];
    const wy = s * x + c * y + t_o[1];
    return [wx, wy] as const;
  };
  if (object.type === "box") {
    const hw = object.box_w / 2;
    const hh = object.box_h / 2;
    const corners = [
      wxy(-hw, -hh),
      wxy(hw, -hh),
      wxy(hw, hh),
      wxy(-hw, hh),
    ];
    return corners.map(([x, y], i) => `${i === 0 ? "M" : "L"} ${x},${y}`).join(" ") + " Z";
  }
  if (object.type === "triangle") {
    const [a, b, c3] = object.tri;
    const p0 = wxy(a[0], a[1]);
    const p1 = wxy(b[0], b[1]);
    const p2 = wxy(c3[0], c3[1]);
    return `M ${p0[0]},${p0[1]} L ${p1[0]},${p1[1]} L ${p2[0]},${p2[1]} Z`;
  }
  const verts = object.verts;
  if (!verts.length) return "";
  const p0 = wxy(verts[0][0], verts[0][1]);
  let d = `M ${p0[0]},${p0[1]}`;
  for (let i = 1; i < verts.length; i++) {
    const pi = wxy(verts[i][0], verts[i][1]);
    d += ` L ${pi[0]},${pi[1]}`;
  }
  return d + " Z";
}

function deg(r: number) {
  return (r * 180) / Math.PI;
}

function viewSize(R: number, aspect: number, zoom: number) {
  const halfW0 = R * Math.max(1, aspect);
  const halfH0 = R * Math.max(1, 1 / aspect);
  return { w: (2 * halfW0) / zoom, h: (2 * halfH0) / zoom };
}

/** 世界系主画布：障碍编辑、抓取（物体系映射）、机械臂与物体、位姿 HUD */
export function Workspace2D() {
  const payload = useAppStore((s) => s.payload);
  const setPayload = useAppStore((s) => s.setPayload);
  const dragMode = useAppStore((s) => s.dragMode);
  const setDragMode = useAppStore((s) => s.setDragMode);
  const jointPick = useAppStore((s) => s.jointPick);
  const trajQ1 = useAppStore((s) => s.trajQ1);
  const trajQ2 = useAppStore((s) => s.trajQ2);
  const clearanceInspect = useAppStore((s) => s.clearanceInspect);

  const svgRef = useRef<SVGSVGElement>(null);
  const layerRef = useRef<SVGGElement>(null);
  const wrapRef = useRef<HTMLDivElement>(null);
  const graspDragRef = useRef<{ t_o: [number, number]; theta_o: number } | null>(null);
  const payloadRef = useRef(payload);
  useEffect(() => {
    payloadRef.current = payload;
  }, [payload]);
  useEffect(() => {
    if (dragMode !== "grasp") graspDragRef.current = null;
  }, [dragMode]);

  const [aspect, setAspect] = useState(1);
  const [view, setView] = useState({ zoom: 1, cx: 0, cy: 0 });

  const q1 = jointPick ? jointPick.q1 : trajQ1[0] ?? 0;
  const q2 = jointPick ? jointPick.q2 : trajQ2[0] ?? 0;

  const R = payload.link1 + payload.link2 + 0.5;

  useEffect(() => {
    setView({ zoom: 1, cx: 0, cy: 0 });
  }, [R]);

  useEffect(() => {
    const el = wrapRef.current;
    if (!el) return;
    const ro = new ResizeObserver(() => {
      const r = el.getBoundingClientRect();
      setAspect(r.width / Math.max(r.height, 1e-6));
    });
    ro.observe(el);
    return () => ro.disconnect();
  }, []);

  const vb = useMemo(() => {
    const { w, h } = viewSize(R, aspect, view.zoom);
    const x = view.cx - w / 2;
    const y = view.cy - h / 2;
    return `${x} ${y} ${w} ${h}`;
  }, [R, aspect, view.zoom, view.cx, view.cy]);

  const viewRef = useRef(view);
  viewRef.current = view;
  const aspectRef = useRef(aspect);
  aspectRef.current = aspect;

  useEffect(() => {
    const svg = svgRef.current;
    if (!svg) return;
    const onWheel = (e: WheelEvent) => {
      e.preventDefault();
      const wrap = wrapRef.current;
      if (wrap) {
        const r = wrap.getBoundingClientRect();
        aspectRef.current = r.width / Math.max(r.height, 1e-6);
      }
      const a = aspectRef.current;
      const v = viewRef.current;
      const { w, h } = viewSize(R, a, v.zoom);
      const x0 = v.cx - w / 2;
      const y0 = v.cy - h / 2;
      const pt = svg.createSVGPoint();
      pt.x = e.clientX;
      pt.y = e.clientY;
      const ctm = svg.getScreenCTM();
      if (!ctm) return;
      const p = pt.matrixTransform(ctm.inverse());
      const fx = (p.x - x0) / w;
      const fy = (p.y - y0) / h;
      const fac = e.deltaY > 0 ? 1.12 : 1 / 1.12;
      const newZoom = clamp(v.zoom * fac, 0.35, 48);
      const { w: nw, h: nh } = viewSize(R, a, newZoom);
      const newCx = p.x - fx * nw + nw / 2;
      const newCy = p.y - fy * nh + nh / 2;
      const nv = { zoom: newZoom, cx: newCx, cy: newCy };
      viewRef.current = nv;
      setView(nv);
    };
    svg.addEventListener("wheel", onWheel, { passive: false });
    return () => svg.removeEventListener("wheel", onWheel);
  }, [R]);

  const k = useMemo(
    () => fk2d(payload.link1, payload.link2, q1, q2),
    [payload.link1, payload.link2, q1, q2],
  );

  const gc = payload.grasp.contact as [number, number];
  const gn = payload.grasp.normal as [number, number];
  const { t_eo, theta_eo } = useMemo(
    () => graspToEeObjectTransform(gc, gn),
    [gc, gn],
  );
  const { t_o, theta_o } = useMemo(
    () => objectPoseWorld(k.ee, k.thetaEe, t_eo, theta_eo),
    [k.ee, k.thetaEe, t_eo, theta_eo],
  );

  const toWorld = useCallback((clientX: number, clientY: number) => {
    const svg = svgRef.current;
    const layer = layerRef.current;
    if (!svg || !layer) return [0, 0] as [number, number];
    const pt = svg.createSVGPoint();
    pt.x = clientX;
    pt.y = clientY;
    const m = layer.getScreenCTM();
    if (!m) return [0, 0] as [number, number];
    const p = pt.matrixTransform(m.inverse());
    return [p.x, p.y] as [number, number];
  }, []);

  const onPointerDown = useCallback(
    (e: React.PointerEvent<SVGSVGElement>) => {
      if (dragMode !== "grasp" || e.button !== 0) return;
      graspDragRef.current = { t_o: [t_o[0], t_o[1]], theta_o };
      e.currentTarget.setPointerCapture(e.pointerId);
    },
    [dragMode, t_o, theta_o],
  );

  const onPointerMove = useCallback(
    (e: React.PointerEvent<SVGSVGElement>) => {
      const [wx, wy] = toWorld(e.clientX, e.clientY);
      const p = payloadRef.current;
      if (dragMode === "grasp") {
        if ((e.buttons & 1) && !graspDragRef.current) {
          graspDragRef.current = { t_o: [t_o[0], t_o[1]], theta_o };
          e.currentTarget.setPointerCapture(e.pointerId);
        }
        if (graspDragRef.current) {
          const fr = graspDragRef.current;
          const loc = worldToObjectLocal(fr.t_o, fr.theta_o, [wx, wy]);
          setPayload({
            grasp: { contact: loc, normal: p.grasp.normal },
          });
        }
        return;
      }
      if (dragMode === "none" || !(e.buttons & 1)) return;
      if (dragMode === "obs0" && p.obstacles[0]) {
        const o = p.obstacles[0];
        const next = [...p.obstacles];
        next[0] = { ...o, cx: wx, cy: wy };
        setPayload({ obstacles: next });
        return;
      }
      if (dragMode === "obs1" && p.obstacles[1]) {
        const o = p.obstacles[1];
        const next = [...p.obstacles];
        next[1] = { ...o, cx: wx, cy: wy };
        setPayload({ obstacles: next });
      }
    },
    [dragMode, setPayload, t_o, theta_o, toWorld],
  );

  const onPointerEnd = useCallback((e: React.PointerEvent<SVGSVGElement>) => {
    if (e.currentTarget.hasPointerCapture(e.pointerId)) {
      e.currentTarget.releasePointerCapture(e.pointerId);
    }
    graspDragRef.current = null;
  }, []);

  const onResetView = useCallback(() => {
    setView({ zoom: 1, cx: 0, cy: 0 });
  }, []);

  const objPath = useMemo(
    () => objectPathWorld(payload.object, t_o, theta_o),
    [payload.object, t_o[0], t_o[1], theta_o],
  );

  const glen = Math.hypot(gn[0], gn[1]) || 1;
  const n0 = gn[0] / glen;
  const n1 = gn[1] / glen;
  const arrowLen = 0.35;
  const nScale = 0.25;
  const nx = clamp(n0 * nScale, -0.6, 0.6);
  const ny = clamp(n1 * nScale, -0.6, 0.6);
  const ee = objPointToWorld(gc, t_o, theta_o);
  const arrowEnd = objPointToWorld([gc[0] + nx * arrowLen, gc[1] + ny * arrowLen], t_o, theta_o);

  const armPath = `M ${k.base[0]},${k.base[1]} L ${k.j1[0]},${k.j1[1]} L ${k.ee[0]},${k.ee[1]}`;

  return (
    <div className="ws2d" style={{ marginBottom: 0 }}>
      <div className="ws2d-head">
        <div className="ws2d-title">
          2D 工作空间（世界系；紫十字为 t_o；滚轮缩放）
          {jointPick ? (
            <span className="ws2d-tag">
              {" "}
              · 手动 q₁={jointPick.q1.toFixed(3)} q₂={jointPick.q2.toFixed(3)}
              {jointPick.feasible ? " · 可行格" : " · 不可行格"}
            </span>
          ) : null}
        </div>
        <div className="ws2d-tools">
          <button type="button" className={dragMode === "grasp" ? "btn on" : "btn"} onClick={() => setDragMode(dragMode === "grasp" ? "none" : "grasp")}>
            拖抓取点
          </button>
          <button type="button" className={dragMode === "obs0" ? "btn on" : "btn"} onClick={() => setDragMode(dragMode === "obs0" ? "none" : "obs0")}>
            障碍1
          </button>
          <button type="button" className={dragMode === "obs1" ? "btn on" : "btn"} onClick={() => setDragMode(dragMode === "obs1" ? "none" : "obs1")}>
            障碍2
          </button>
          <button type="button" className="btn" onClick={() => setDragMode("none")}>
            退出拖拽
          </button>
          <button type="button" className="btn" onClick={onResetView}>
            重置视图
          </button>
        </div>
      </div>
      <div className="ws2d-canvas-wrap" ref={wrapRef}>
        <svg
          ref={svgRef}
          viewBox={vb}
          className="ws2d-svg"
          style={{ touchAction: "none" }}
          onPointerDown={onPointerDown}
          onPointerMove={onPointerMove}
          onPointerUp={onPointerEnd}
          onPointerCancel={onPointerEnd}
        >
          <g ref={layerRef} transform="scale(1,-1)">
            <rect x={-1e3} y={-1e3} width={2e3} height={2e3} fill="#fafbfc" />
            {payload.obstacles.map((o, idx) =>
              o.type === "circle" ? (
                <circle key={idx} cx={o.cx} cy={o.cy} r={o.r} fill="#cf222e33" stroke="#cf222e" />
              ) : (
                <g key={idx} transform={`translate(${o.cx},${o.cy})`}>
                  <rect
                    x={-o.w / 2}
                    y={-o.h / 2}
                    width={o.w}
                    height={o.h}
                    fill="#0969da22"
                    stroke="#0969da"
                  />
                </g>
              ),
            )}
            <path
              d={armPath}
              fill="none"
              stroke="#0969da"
              strokeWidth={0.07}
              strokeLinecap="round"
              strokeLinejoin="round"
            />
            <circle cx={k.base[0]} cy={k.base[1]} r={0.07} fill="#24292f" />
            <circle cx={k.j1[0]} cy={k.j1[1]} r={0.065} fill="#57606a" />
            <path d={objPath} fill="#1a7f3722" stroke="#1a7f37" strokeWidth={0.02} />
            <g stroke="#8250df" strokeWidth={0.04}>
              <line x1={t_o[0] - 0.18} y1={t_o[1]} x2={t_o[0] + 0.18} y2={t_o[1]} />
              <line x1={t_o[0]} y1={t_o[1] - 0.18} x2={t_o[0]} y2={t_o[1] + 0.18} />
            </g>
            <circle cx={ee[0]} cy={ee[1]} r={0.07} fill="#24292f" />
            <line
              x1={ee[0]}
              y1={ee[1]}
              x2={arrowEnd[0]}
              y2={arrowEnd[1]}
              stroke="#1a7f37"
              strokeWidth={0.04}
            />
          </g>
        </svg>
      </div>
      <div className="ws2d-zoom-hint">在画布上滚轮：以光标为中心缩放（x/y 世界比例一致）。</div>
      <div className="ws2d-hud-below">
        <div className="hud-t">抓取 / 物体位姿</div>
        <div>接触（物体系）: ({gc[0].toFixed(2)}, {gc[1].toFixed(2)})</div>
        <div>法向: ({gn[0].toFixed(2)}, {gn[1].toFixed(2)})</div>
        <div>θ_eo: {deg(theta_eo).toFixed(1)}°</div>
        <div>θ_obj: {deg(theta_o).toFixed(1)}°</div>
        <div>t_o: ({t_o[0].toFixed(3)}, {t_o[1].toFixed(3)})</div>
        {clearanceInspect && jointPick ? (
          <div className="hud-ins">
            <div className="hud-ins-t">裕度（手动格）</div>
            <div>总 {clearanceInspect.clearance.toFixed(3)} · 臂 {clearanceInspect.clearance_robot.toFixed(3)} · 物 {clearanceInspect.clearance_object.toFixed(3)}</div>
            <div>
              瓶颈: {clearanceInspect.bottleneck === "robot" ? "机械臂" : clearanceInspect.bottleneck === "object" ? "物体" : "持平"}
            </div>
          </div>
        ) : null}
      </div>
    </div>
  );
}
