# R2 设计说明与改进建议

## 已实现骨架映射

- `launch` 分类：
  - 刚开局：`COLD_START`
  - 返回重拼武器：`RETURN_WEAPON_REASSEMBLY`
  - 返回梅林补拿 KFS：`RETURN_MF_KFS`
- 三阶段流程：
  - `MC`：对齐武器架、抓端头、组装+二维码识别、等待 R1 离开
  - `MF`：入口规则、相邻抓取、平台内/外可抓判定
  - `CF`：按策略选格，中层/顶层放置，必要时触发补拿分支
- 节点通信：
  - 统一通过 `FlagBus.publish/get` 发布和消费 flag

## 当前方案中的不足（建议优先补齐）

- 缺少真实时序保证：
  - 目前用 `sleep` 和模拟 flag，实际应替换为任务完成回调（动作服务器）+ 超时监控。
- 坐标体系尚未统一：
  - 需要定义 `map -> base_link -> sensor` 的 TF 树与标定流程。
- 视觉置信度门槛缺失：
  - 需为二维码识别、方块识别设置置信度阈值与降级策略。
- 对抗区策略未纳入时间成本模型：
  - 建议将每个候选格子的“预计耗时”加入评分函数，形成 `score = points - alpha * time`。
- 重试策略不完整：
  - 目前只实现统一重试入口，需细分为“手动重试/强制重试/对抗区重试点选择”。

## 建议的工程化拆分（ROS2）

- `launch_manager_node`：决定三类 launch 模式并发布 `mode_flag`
- `mc_executor_node`：武馆阶段动作与状态回传
- `mf_planner_node`：梅林相对坐标寻路与抓取
- `cf_strategy_node`：九宫格评分、阻断优先
- `safety_guard_node`：统一犯规守卫（假 KFS/相邻规则/越界）
- `mission_state_node`：全局状态机协调

## 建议新增 flag（最小集）

- 输入：
  - `r1_left_mc`
  - `qr_detect_ok`
  - `kfs_detect_list`
  - `enemy_line_threat`
  - `forced_retry`
- 输出：
  - `phase`
  - `motion_goal`
  - `arm_action`
  - `target_cell`
  - `retry_mode`

## 与规则的关键一致性提醒

- R2 离开武馆必须晚于 R1 完全离开武馆。
- R2 在梅林只能拿 R2 KFS，且遵循相邻抓取与入口首取约束。
- R2 入对抗区必须携带至少 1 个 R2 KFS。
- 任何触发强制重试的行为，必须执行重试区与等待时间规则。
