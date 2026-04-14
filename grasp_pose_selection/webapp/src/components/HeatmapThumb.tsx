import { useEffect, useRef } from "react";

function cellColor(v: number) {
  if (v > 0) {
    const t = Math.min(1, v / 0.35);
    return `hsl(130,55%,${42 + t * 28}%)`;
  }
  const t = Math.min(1, -v / 0.35);
  return `hsl(0,65%,${38 + t * 22}%)`;
}

/** 用 result.J 画缩略热力图（格点即像素） */
export function HeatmapThumb({ J, label }: { J: number[][]; label: string }) {
  const ref = useRef<HTMLCanvasElement>(null);

  useEffect(() => {
    const c = ref.current;
    if (!c || !J.length || !J[0]?.length) return;
    const nx = J.length;
    const ny = J[0].length;
    c.width = nx;
    c.height = ny;
    const ctx = c.getContext("2d");
    if (!ctx) return;
    for (let i = 0; i < nx; i++) {
      for (let j = 0; j < ny; j++) {
        ctx.fillStyle = cellColor(J[i][j] ?? 0);
        ctx.fillRect(i, ny - 1 - j, 1, 1);
      }
    }
  }, [J]);

  return (
    <div className="hm-th">
      <canvas ref={ref} className="hm-th-c" title={label} />
      <style>{`
        .hm-th { width:100%; margin-top:6px; }
        .hm-th-c { width:100%; height:56px; image-rendering:pixelated; border-radius:4px; border:1px solid var(--border); display:block; }
      `}</style>
    </div>
  );
}
