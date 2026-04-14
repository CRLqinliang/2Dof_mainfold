import { HeatmapThumb } from "./HeatmapThumb";
import { useAppStore } from "../store/appStore";

export function ComparePanel() {
  const snapshots = useAppStore((s) => s.snapshots);
  const addSnapshot = useAppStore((s) => s.addSnapshot);
  const removeSnapshot = useAppStore((s) => s.removeSnapshot);

  return (
    <div className="panel-card" style={{ marginBottom: 0 }}>
      <div className="section-title section-title--plain">对比快照</div>
      <div className="cmp-actions">
        <button type="button" className="btn" onClick={() => addSnapshot(`snap-${snapshots.length + 1}`)}>
          保存当前
        </button>
      </div>
      <div className="cmp-grid-scroll">
        <div className="cmp-grid">
          {snapshots.length === 0 ? (
            <div className="cmp-empty">暂无快照。计算后点「保存当前」。</div>
          ) : (
            snapshots.map((s) => {
              const r = s.result;
              const jn = r ? r.q1_coords.length * r.q2_coords.length : 0;
              const se2p = r && r.se2_grid_size > 0 ? r.feasible_se2_cells / r.se2_grid_size : 0;
              return (
                <div key={s.id} className="cmp-card">
                  <div className="cmp-title">{s.label}</div>
                  <div className="cmp-meta">
                    {r ? (
                      <>
                        <div>
                          J+: {r.feasible_joint_cells}/{jn}{" "}
                          {jn ? `(${(100 * (r.feasible_joint_cells / jn)).toFixed(0)}%)` : ""}
                        </div>
                        <div>
                          SE2+: {r.feasible_se2_cells}/{r.se2_grid_size} ({(100 * se2p).toFixed(0)}%)
                        </div>
                        <div>{s.jobPreview ? "预览" : "全量"}</div>
                      </>
                    ) : (
                      <div>无结果</div>
                    )}
                  </div>
                  {r?.J?.length ? <HeatmapThumb J={r.J} label={`${s.label} J(q1,q2)`} /> : null}
                  <button type="button" className="btn-danger-outline" style={{ marginTop: 8 }} onClick={() => removeSnapshot(s.id)}>
                    删除
                  </button>
                </div>
              );
            })
          )}
        </div>
      </div>
    </div>
  );
}
