# RoboCon — 移动机器人接口与仿真

仓库摘要：
- 本目录包含移动机器人（含机械臂）的接口草案与本地仿真示例。

关键文件：
- `interfaces_overview.md` — 模块架构 Mermaid 图与接口概览。
- `communication_matrix.md` — 详细通信矩阵（topics/services/actions、消息类型、QoS 建议、频率）。
- `track_and_grab/track_and_grab.cpp` — 本地 C++ 仿真器源码（验证传感器/运动逻辑）。

如何运行仿真示例（在工作区根执行）：

```bash
cd /home/yf/ros2_ws
g++ -std=c++17 -O2 -o src/track_and_grab/track_and_grab src/track_and_grab/track_and_grab.cpp
./src/track_and_grab/track_and_grab
```

后续建议：
- 我可以将 `communication_matrix.md` 中的自定义消息/服务/动作生成到 ROS2 包骨架（你可选择 ROS2 发行版：`humble`/`galactic`/`iron` 等）。
- 若需要文档图片，我可以把 Mermaid 图渲染为 PNG/SVG 并放入 `src/docs/`。

---
自动生成并上传（如已授权）
