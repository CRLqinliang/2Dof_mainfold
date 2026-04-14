import react from "@vitejs/plugin-react";
import { defineConfig, loadEnv } from "vite";

function normalizeBase(p: string) {
  if (!p || p === "/") return "/";
  return p.endsWith("/") ? p : `${p}/`;
}

export default defineConfig(({ mode }) => {
  const env = loadEnv(mode, process.cwd(), "");
  const dev = mode === "development";
  const base = dev
    ? "/"
    : normalizeBase(env.VITE_BASE_PATH && env.VITE_BASE_PATH.length > 0 ? env.VITE_BASE_PATH : "/");

  return {
    plugins: [react()],
    base,
    server: {
      port: 5173,
      proxy: {
        "/api": "http://127.0.0.1:8765",
      },
    },
  };
});
