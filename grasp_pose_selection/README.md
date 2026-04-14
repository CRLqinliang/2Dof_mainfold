# grasp_pose_selection

## 本地

- 前端：`cd webapp && npm i && npm run dev`
- 后端：`cd web && python server.py`（默认 `8765`，Vite 已代理 `/api`）

## 后端上线（Render）

出现 `No such file ... requirements.txt` 时，说明构建在**仓库根目录**执行了，而文件在 `grasp_pose_selection/` 下。任选其一：

**方式 A：手动 Web Service**

- **Root Directory**：`grasp_pose_selection`（必填）
- **Build Command**：`pip install -r requirements.txt`
- **Start Command**：`RENDER=1 python web/server.py`

**方式 B：Blueprint**

- 使用仓库根目录的 [`render.yaml`](../render.yaml)（与 `grasp_pose_selection/render.yaml` 内容一致，`rootDir` 已指向子目录）

**方式 C：不设 Root Directory（不推荐）**

- **Build Command**：`pip install -r grasp_pose_selection/requirements.txt`
- **Start Command**：`cd grasp_pose_selection && RENDER=1 python web/server.py`

### 部署后验证

浏览器或命令行访问：`https://<你的服务域名>/api/defaults` ，应返回 JSON。

## 前端 GitHub Pages

- 仓库 **Settings → Pages**：Source 选 GitHub Actions
- 仓库 **Settings → Variables**：新增 `VITE_API_BASE_URL` = 后端根地址（无末尾 `/`，如 `https://xxx.onrender.com`）
- Workflow 默认 `VITE_BASE_PATH=/<仓库名>/`；若站点挂在用户主页根域，需改 workflow 里该变量

## 冒烟

```bash
cd grasp_pose_selection && pip install -r requirements.txt
cd web && python -c "from grasp_web_compute import default_payload; print(default_payload()['link1'])"
```
