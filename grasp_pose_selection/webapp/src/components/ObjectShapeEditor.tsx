import { useCallback, useMemo, useRef, useState } from "react";
import { useAppStore } from "../store/appStore";
import type { ObjectSpec } from "../types";

const DEFAULT_PENT: [number, number][] = [
  [0.32, 0],
  [0.1, 0.3],
  [-0.26, 0.18],
  [-0.26, -0.18],
  [0.1, -0.3],
];

function computeBBox(verts: [number, number][]) {
  let x0 = Infinity;
  let y0 = Infinity;
  let x1 = -Infinity;
  let y1 = -Infinity;
  for (const [x, y] of verts) {
    x0 = Math.min(x0, x);
    y0 = Math.min(y0, y);
    x1 = Math.max(x1, x);
    y1 = Math.max(y1, y);
  }
  const pad = 0.25;
  return { x0: x0 - pad, y0: y0 - pad, x1: x1 + pad, y1: y1 + pad };
}

/** 物体系多边形顶点编辑（专用小画布） */
export function ObjectShapeEditor() {
  const payload = useAppStore((s) => s.payload);
  const setPayload = useAppStore((s) => s.setPayload);
  const svgRef = useRef<SVGSVGElement>(null);
  const layerRef = useRef<SVGGElement>(null);
  const [dragI, setDragI] = useState<number | null>(null);

  const obj = payload.object;
  const verts = obj.type === "polygon" ? obj.verts : null;

  const bb = useMemo(() => (verts?.length ? computeBBox(verts) : { x0: -1, y0: -1, x1: 1, y1: 1 }), [verts]);
  const vb = `${bb.x0} ${bb.y0} ${bb.x1 - bb.x0} ${bb.y1 - bb.y0}`;

  const toLocal = useCallback((clientX: number, clientY: number) => {
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

  const setPoly = (next: { type: "polygon"; verts: [number, number][] }) => {
    setPayload({ object: next });
  };

  const onDown = (i: number) => (e: React.PointerEvent) => {
    if (obj.type !== "polygon") return;
    e.currentTarget.setPointerCapture(e.pointerId);
    setDragI(i);
  };

  const onMove = (e: React.PointerEvent) => {
    if (dragI === null || obj.type !== "polygon") return;
    const [lx, ly] = toLocal(e.clientX, e.clientY);
    const nv = obj.verts.map((p, k) => (k === dragI ? ([lx, ly] as [number, number]) : p));
    setPoly({ type: "polygon", verts: nv });
  };

  const onUp = (e: React.PointerEvent) => {
    if (dragI !== null && e.currentTarget.hasPointerCapture(e.pointerId)) {
      e.currentTarget.releasePointerCapture(e.pointerId);
    }
    setDragI(null);
  };

  const addVertex = () => {
    if (obj.type !== "polygon") return;
    const v = obj.verts;
    if (v.length < 2) return;
    const a = v[v.length - 1];
    const b = v[0];
    setPoly({ type: "polygon", verts: [...v, [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2]] });
  };

  const delLast = () => {
    if (obj.type !== "polygon" || obj.verts.length <= 3) return;
    setPoly({ type: "polygon", verts: obj.verts.slice(0, -1) });
  };

  if (obj.type !== "polygon") {
    return (
      <div className="panel-card ose">
        <div className="ose-h">物体形状</div>
        <p className="ose-muted">在参数中选择「多边形」后可在此编辑顶点。</p>
      </div>
    );
  }

  const d = verts.map((p, i) => `${i === 0 ? "M" : "L"} ${p[0]},${p[1]}`).join(" ") + " Z";

  return (
    <div className="panel-card ose">
      <div className="ose-h">物体形状（物体系，拖顶点）</div>
      <div className="ose-bar">
        <button type="button" className="btn" onClick={addVertex}>
          加顶点
        </button>
        <button type="button" className="btn" onClick={delLast}>
          删最后点
        </button>
      </div>
      <svg ref={svgRef} viewBox={vb} className="ose-svg">
        <g ref={layerRef} transform="scale(1,-1)">
          <path d={d} fill="#1a7f3722" stroke="#1a7f37" strokeWidth={0.02} />
          {verts.map((p, i) => (
            <circle
              key={i}
              cx={p[0]}
              cy={p[1]}
              r={0.06}
              fill="#bc4c00"
              stroke="#fff"
              strokeWidth={0.02}
              style={{ cursor: dragI === i ? "grabbing" : "grab", touchAction: "none" }}
              onPointerDown={onDown(i)}
              onPointerMove={onMove}
              onPointerUp={onUp}
              onPointerCancel={onUp}
            />
          ))}
        </g>
      </svg>
    </div>
  );
}

export function defaultPolygonObject(): ObjectSpec {
  return { type: "polygon", verts: DEFAULT_PENT.map((p) => [p[0], p[1]] as [number, number]) };
}
