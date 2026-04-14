import Plotly from "plotly.js-dist-min";
import { useEffect, useMemo, useRef } from "react";

type Props = {
  title: string;
  z: number[][];
  x: number[];
  y: number[];
  /** 格点拖动选取：valueGrid[i][j] 与 z 同索引，feasible 默认 > 0 */
  cellPick?: {
    valueGrid: number[][];
    feasibleThreshold?: number;
    highlight: { x: number; y: number; feasible: boolean } | null;
    onCellPick: (xVal: number, yVal: number, meta: { feasible: boolean; i: number; j: number }) => void;
    /** 默认 true；false 时只显示 highlight 标记，不监听拖动 */
    interactive?: boolean;
  };
};

function transpose<T>(m: T[][]): T[][] {
  if (!m.length) return [];
  return m[0].map((_, j) => m.map((row) => row[j]));
}

function nearestCell(q1: number, q2: number, q1c: number[], q2c: number[]) {
  let bi = 0;
  let bj = 0;
  let bd = Infinity;
  for (let i = 0; i < q1c.length; i++) {
    for (let j = 0; j < q2c.length; j++) {
      const d = (q1c[i] - q1) ** 2 + (q2c[j] - q2) ** 2;
      if (d < bd) {
        bd = d;
        bi = i;
        bj = j;
      }
    }
  }
  return { i: bi, j: bj };
}

type FullLayoutAxis = {
  _offset: number;
  _length: number;
  range: [number, number];
  p2c?: (pxRel: number) => number;
};

function mouseToQ1Q2(
  gd: HTMLElement,
  clientX: number,
  clientY: number,
): { q1: number; q2: number } | null {
  const fl = (gd as unknown as { _fullLayout?: { xaxis: FullLayoutAxis; yaxis: FullLayoutAxis } })
    ._fullLayout;
  if (!fl) return null;
  const xa = fl.xaxis;
  const ya = fl.yaxis;
  const rect = gd.getBoundingClientRect();
  const px = clientX - rect.left;
  const py = clientY - rect.top;
  const xl = px - xa._offset;
  const yl = py - ya._offset;
  if (xl < -2 || xl > xa._length + 2 || yl < -2 || yl > ya._length + 2) return null;

  let q1: number;
  let q2: number;
  if (typeof xa.p2c === "function" && typeof ya.p2c === "function") {
    q1 = xa.p2c(xl);
    q2 = ya.p2c(yl);
  } else {
    const xr = xa.range;
    const yr = ya.range;
    const fx = Math.min(1, Math.max(0, xl / xa._length));
    const fy = Math.min(1, Math.max(0, yl / ya._length));
    q1 = xr[0] + fx * (xr[1] - xr[0]);
    q2 = yr[1] - fy * (yr[1] - yr[0]);
  }
  return { q1, q2 };
}

function restylePickMarker(
  el: HTMLElement,
  hi: { x: number; y: number; feasible: boolean } | null,
) {
  const gd = el as unknown as { _fullLayout?: unknown };
  if (!gd._fullLayout) return;
  if (hi) {
    Plotly.restyle(
      el,
      {
        x: [[hi.x]],
        y: [[hi.y]],
        "marker.size": [16],
        "marker.opacity": [1],
        "marker.color": [hi.feasible ? "#1a7f37" : "#cf222e"],
      },
      [1],
    );
  } else {
    Plotly.restyle(
      el,
      {
        "marker.size": [0],
        "marker.opacity": [0],
      },
      [1],
    );
  }
}

function axisSpan(coords: number[]) {
  if (!coords.length) return 1;
  const lo = coords[0];
  const hi = coords[coords.length - 1];
  const s = Math.abs(hi - lo);
  return s < 1e-12 ? 1 : s;
}

export function PlotHeatmap({ title, z, x, y, cellPick }: Props) {
  const ref = useRef<HTMLDivElement>(null);
  const wrapRef = useRef<HTMLDivElement>(null);
  const jiRef = useRef(cellPick);
  jiRef.current = cellPick;
  const xyRef = useRef({ x, y });
  xyRef.current = { x, y };

  const highlight = cellPick?.highlight ?? null;
  const pickInteractive = Boolean(cellPick && cellPick.interactive !== false);
  const showPickMarker = Boolean(cellPick);

  const xSpan = useMemo(() => axisSpan(x), [x]);
  const ySpan = useMemo(() => axisSpan(y), [y]);
  const aspectStyle = useMemo(
    () => ({ aspectRatio: xSpan / ySpan } as React.CSSProperties),
    [xSpan, ySpan],
  );

  useEffect(() => {
    const wrap = wrapRef.current;
    const el = ref.current;
    if (!wrap || !el) return;
    const ro = new ResizeObserver(() => {
      Plotly.Plots.resize(el);
    });
    ro.observe(wrap);
    return () => ro.disconnect();
  }, []);

  useEffect(() => {
    const el = ref.current;
    if (!el) return;

    const zt = transpose(z);
    const traces: Record<string, unknown>[] = [
      {
        z: zt,
        x,
        y,
        type: "heatmap",
        colorscale: "RdYlGn",
        zmid: 0,
        zmin: -0.5,
        zmax: 0.5,
        colorbar: {
          title: "clearance",
          tickfont: { color: "#57606a" },
          titlefont: { color: "#24292f" },
        },
      },
    ];

    if (showPickMarker) {
      traces.push({
        type: "scatter",
        mode: "markers",
        x: [0],
        y: [0],
        marker: {
          size: 0.001,
          opacity: 0,
          color: "#1a7f37",
          symbol: "circle-open",
          line: { width: 3 },
        },
        hoverinfo: "skip",
      });
    }

    const layout: Record<string, unknown> = {
      paper_bgcolor: "#ffffff",
      plot_bgcolor: "#fafbfc",
      font: { color: "#24292f", size: 11 },
      title: { text: title, font: { size: 14, color: "#1f2328" } },
      margin: { t: 40, r: showPickMarker ? 56 : 20, b: 48, l: 56 },
      xaxis: { gridcolor: "#eaeef2", zerolinecolor: "#d8dee4", constrain: "domain" },
      yaxis: {
        gridcolor: "#eaeef2",
        zerolinecolor: "#d8dee4",
        scaleanchor: "x",
        scaleratio: 1,
        constrain: "domain",
      },
      showlegend: false,
      dragmode: pickInteractive ? false : "zoom",
      autosize: true,
    };

    const config = { responsive: true, scrollZoom: !pickInteractive };

    let cancelled = false;
    let dragging = false;
    let raf = 0;
    let lastCx = 0;
    let lastCy = 0;

    const applyPick = () => {
      const ji = jiRef.current;
      if (!ji) return;
      const { x: xq, y: yq } = xyRef.current;
      const q = mouseToQ1Q2(el, lastCx, lastCy);
      if (!q) return;
      const { i, j } = nearestCell(q.q1, q.q2, xq, yq);
      const th = ji.feasibleThreshold ?? 0;
      const feasible = ji.valueGrid[i][j] > th;
      ji.onCellPick(xq[i], yq[j], { feasible, i, j });
    };

    const schedulePick = (cx: number, cy: number) => {
      lastCx = cx;
      lastCy = cy;
      if (raf) cancelAnimationFrame(raf);
      raf = requestAnimationFrame(() => {
        raf = 0;
        applyPick();
      });
    };

    const onPointerDown = (e: PointerEvent) => {
      if (!pickInteractive || e.button !== 0) return;
      e.preventDefault();
      dragging = true;
      el.setPointerCapture(e.pointerId);
      schedulePick(e.clientX, e.clientY);
    };

    const onPointerMove = (e: PointerEvent) => {
      if (!dragging) return;
      schedulePick(e.clientX, e.clientY);
    };

    const onPointerUp = (e: PointerEvent) => {
      if (!dragging) return;
      dragging = false;
      if (el.hasPointerCapture(e.pointerId)) {
        el.releasePointerCapture(e.pointerId);
      }
    };

    Plotly.react(el, traces, layout, config).then(() => {
      if (cancelled) return;
      if (pickInteractive) {
        el.addEventListener("pointerdown", onPointerDown);
        el.addEventListener("pointermove", onPointerMove);
        el.addEventListener("pointerup", onPointerUp);
        el.addEventListener("pointercancel", onPointerUp);
      }
      restylePickMarker(el, jiRef.current?.highlight ?? null);
    });

    return () => {
      cancelled = true;
      if (raf) cancelAnimationFrame(raf);
      el.removeEventListener("pointerdown", onPointerDown);
      el.removeEventListener("pointermove", onPointerMove);
      el.removeEventListener("pointerup", onPointerUp);
      el.removeEventListener("pointercancel", onPointerUp);
      Plotly.purge(el);
    };
  }, [title, z, x, y, pickInteractive, showPickMarker]);

  useEffect(() => {
    const el = ref.current;
    if (!showPickMarker || !el) return;
    const id = requestAnimationFrame(() => {
      restylePickMarker(el, highlight);
    });
    return () => cancelAnimationFrame(id);
  }, [showPickMarker, highlight?.x, highlight?.y, highlight?.feasible]);

  const hint = pickInteractive ? (
    <div className="ph-hint">
      <strong>按住左键拖动</strong>：连续选取最近格点；右侧 <strong>2D 工作区</strong>实时显示机械臂与物体（不会因重绘打断拖动）。
    </div>
  ) : null;

  return (
    <div className="ph-wrap">
      {hint}
      <div ref={wrapRef} className="ph-aspect" style={aspectStyle}>
        <div
          ref={ref}
          className={pickInteractive ? "ph-plot-inner ph-plot--grab" : "ph-plot-inner"}
        />
      </div>
    </div>
  );
}
