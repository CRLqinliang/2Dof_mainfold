import type { ClearanceInspect, ComputeErr, ComputeOk, Payload } from "../types";

const API_BASE = (import.meta.env.VITE_API_BASE_URL ?? "").replace(/\/+$/, "");

function apiUrl(path: string) {
  return `${API_BASE}${path}`;
}

async function readJson(r: Response): Promise<unknown> {
  const ct = r.headers.get("content-type") || "";
  if (ct.includes("application/json")) return r.json();
  return { ok: false, error: await r.text() };
}

export async function fetchDefaults(): Promise<Payload> {
  const r = await fetch(apiUrl("/api/defaults"));
  const body = (await readJson(r)) as { ok?: boolean; data?: Payload; error?: string };
  if (!body || body.ok !== true || !body.data) {
    throw new Error((body as ComputeErr).error || "defaults failed");
  }
  return body.data;
}

export async function computePreview(payload: Payload, signal?: AbortSignal): Promise<ComputeOk> {
  const r = await fetch(apiUrl("/api/compute-preview"), {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(payload),
    signal,
  });
  const body = (await readJson(r)) as ComputeOk | ComputeErr;
  if (!body || body.ok !== true) {
    throw new Error((body as ComputeErr).error || "preview failed");
  }
  return body;
}

export async function inspectClearance(
  payload: Payload,
  q1: number,
  q2: number,
  signal?: AbortSignal,
): Promise<ClearanceInspect> {
  const r = await fetch(apiUrl("/api/inspect-clearance"), {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ payload, q1, q2 }),
    signal,
  });
  const body = (await readJson(r)) as { ok?: boolean; data?: ClearanceInspect; error?: string };
  if (!body || body.ok !== true || !body.data) {
    throw new Error((body as ComputeErr).error || "inspect failed");
  }
  return body.data;
}

export async function computeFull(payload: Payload): Promise<ComputeOk> {
  const r = await fetch(apiUrl("/api/compute-full"), {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(payload),
  });
  const body = (await readJson(r)) as ComputeOk | ComputeErr;
  if (!body || body.ok !== true) {
    throw new Error((body as ComputeErr).error || "full compute failed");
  }
  return body;
}
