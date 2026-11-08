# 导航执行与验证边界

核心算法为 C++17，按职责分文件，ROS 接入放在 `src/execution/`。

| 模块 | 职责 |
| --- | --- |
| `timed_trajectory.cpp` | 已验证地形边上的梯形速度、加速度和时间参数，不抄近路 |
| `path_tracker.cpp` | odom 系反馈跟踪、停车转向、低速前进上下楼、平台出口与到点确认 |
| `task_supervisor.cpp` | 任务状态、暂停、取消、有限重试、恢复超时和无进展退出 |
| `safety_arbiter.cpp` | 手动优先、任务撤权、租约、急停门控 |
| `executor_inputs.cpp` | 原子路线校验、连续 odom 位姿、定位修正跳变处理 |
| `executor_tick.cpp` | 编排核心算法与 Walk 步态请求，不直接发底盘速度 |
| `executor_output.cpp` | Action 反馈、名义时间轨迹、短时运动许可 |
| `guard_checks.cpp` | 独立检查状态、定位、路线、TF、实际矩形足迹和制动包络 |
| `dog_web/navigation_bridge.cpp` | HTTP 到 Action/任务服务适配，不承担控制算法 |

任务编排使用显式有限状态机；当前职责不需要引入额外行为树框架。
`terrain_route_validation.cpp` 独立复核旧图边的前进姿态、转向和更细支撑采样，
不通过的连接不得进入执行缓存；有限次重搜索失败会明确返回阻塞。

数据链：`/navigate_terrain` → `/navigation/planner_goal` →
`/local_navigation/terrain_route` → `/navigation/command` →
独立 `velocity_guard` → `/dog/safe_cmd_vel` → 驱动。
`/cmd_vel` 为手动输入，优先于自主任务；接管后旧任务不能自动恢复。
手动控制支持无地图建图，因此不进行地图碰撞认证。驱动自身指令超时停车仍保留。

路线包绑定地图项目、revision、目标时间戳、点、地形节点 ID、楼梯标记和障碍，
避免把不同周期的 Path 和 JSON 状态拼起来执行。缓存路线每帧重新检查当前障碍，
首次复杂搜索超预算时不允许运动，下一次使用已重验证的缓存。

任务服务：`/navigation_executor/pause`、`resume`、`cancel`（Trigger）。
急停：`/velocity_guard/estop`（SetBool）；解除需新鲜速度反馈且已停车，旧任务仍撤权。
每条自主指令最多存活 0.15s；守护进程 50Hz、执行器 20Hz。
定位、路线、扫描或反馈过期时撤销自主许可，输入恢复超时20s退出，无进展30s退出。

楼梯使用机器人已有 WALK，不增加未经 SDK 证明的楼梯模式：入口停车、对齐、
低速前进、平台停车转向、出口确认。当前按每条地形边停车，速度保守、耗时较长；
这是首版闭环，不能据此宣称已达到连续高效的实机楼梯导航。

统一启动仍用 `dog_bringup` 的 navigation 模式；独立入口：

```bash
source install/setup.bash
ros2 launch dog_navigation execution.launch.py map_root:=$PWD/data/maps map_id:=route_rect_final_r015
```

默认 `execute=false`、`execution.yaml` 中 `calibration_confirmed=false`，Web 显示状态但
不会放行自主运动。0.65m 的 base_link 接地偏移、0.3m 腹部高度及制动参数尚需标定，
不能用0.5m整机总高替代这些量。代码中的加减速度约束是软件假设，未证明实机制动性能。

验证区分：纯算法测试、真实地图理想速度反馈、隔离 DDS 节点测试、实机。
前三项不能覆盖机身俯仰、四足落点、脚滑、下楼制动和动态人体漏检；
机器人未连接时不能完成实机闭环、制动距离或楼梯通过率验证。
