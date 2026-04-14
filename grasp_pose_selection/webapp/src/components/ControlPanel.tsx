import { defaultPolygonObject } from "./ObjectShapeEditor";
import { useAppStore } from "../store/appStore";

export function ControlPanel() {
  const payload = useAppStore((s) => s.payload);
  const setPayload = useAppStore((s) => s.setPayload);

  const obj = payload.object;
  const objKind = obj.type === "box" ? "box" : obj.type === "triangle" ? "tri" : "poly";

  return (
    <div className="panel-card panel-card--flush-top">
      <h2 className="section-title">参数</h2>
      <div className="form-row">
        <label>L1/L2</label>
        <input
          type="number"
          step={0.1}
          value={payload.link1}
          onChange={(e) => setPayload({ link1: +e.target.value })}
        />
        <input
          type="number"
          step={0.1}
          value={payload.link2}
          onChange={(e) => setPayload({ link2: +e.target.value })}
        />
      </div>
      <div className="form-row" style={{ marginTop: -4, marginBottom: 10 }}>
        <span className="hint">J 网格默认范围 [−π, π]（q₁_min/max、q₂_min/max）</span>
      </div>
      <div className="form-row">
        <label>网格 q1×q2</label>
        <input
          type="number"
          value={payload.q1_n}
          onChange={(e) => setPayload({ q1_n: +e.target.value })}
        />
        <input
          type="number"
          value={payload.q2_n}
          onChange={(e) => setPayload({ q2_n: +e.target.value })}
        />
      </div>
      <div className="form-row">
        <label>场 nx,ny,nz</label>
        <input
          type="number"
          value={payload.field_nx}
          onChange={(e) => setPayload({ field_nx: +e.target.value })}
        />
        <input
          type="number"
          value={payload.field_ny}
          onChange={(e) => setPayload({ field_ny: +e.target.value })}
        />
        <input
          type="number"
          value={payload.field_nz}
          onChange={(e) => setPayload({ field_nz: +e.target.value })}
        />
      </div>
      <div className="form-row">
        <label>物体</label>
        <select
          value={objKind}
          onChange={(e) => {
            const v = e.target.value;
            if (v === "box") {
              setPayload({ object: { type: "box", box_w: 0.8, box_h: 0.8 } });
            } else if (v === "tri") {
              setPayload({
                object: {
                  type: "triangle",
                  tri: [
                    [0, 0],
                    [0.55, 0],
                    [0.25, 0.45],
                  ],
                },
              });
            } else {
              setPayload({ object: defaultPolygonObject() });
            }
          }}
        >
          <option value="box">矩形</option>
          <option value="tri">三角形</option>
          <option value="poly">多边形</option>
        </select>
      </div>
      {obj.type === "box" ? (
        <div className="form-row">
          <label>宽/高</label>
          <input
            type="number"
            step={0.05}
            value={obj.box_w}
            onChange={(e) =>
              setPayload({
                object: { type: "box", box_w: +e.target.value, box_h: obj.box_h },
              })
            }
          />
          <input
            type="number"
            step={0.05}
            value={obj.box_h}
            onChange={(e) =>
              setPayload({
                object: { type: "box", box_w: obj.box_w, box_h: +e.target.value },
              })
            }
          />
        </div>
      ) : obj.type === "triangle" ? (
        <div className="form-row" style={{ flexDirection: "column", alignItems: "stretch" }}>
          {obj.tri.map((p, idx) => (
            <div key={idx} className="form-row">
              <label>p{idx}</label>
              <input
                type="number"
                value={p[0]}
                onChange={(e) => {
                  const tri = obj.tri.map((q, k) =>
                    k === idx ? ([+e.target.value, q[1]] as [number, number]) : q,
                  );
                  setPayload({ object: { type: "triangle", tri } });
                }}
              />
              <input
                type="number"
                value={p[1]}
                onChange={(e) => {
                  const tri = obj.tri.map((q, k) =>
                    k === idx ? ([q[0], +e.target.value] as [number, number]) : q,
                  );
                  setPayload({ object: { type: "triangle", tri } });
                }}
              />
            </div>
          ))}
        </div>
      ) : null}
    </div>
  );
}
