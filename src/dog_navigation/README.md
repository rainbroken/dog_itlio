# dog_navigation：实时三维避障规划与重规划

本包包含地形规划、任务执行和独立安全仲裁。单独运行 `navigation.launch.py` 仅输出路径；
`execution.launch.py` 增加任务与速度链路，自主运动默认关闭。与已有地图定位分别运行。
核心代码为 C++17；地图与预验证地形边复用 dog_terrain，保持上下楼层的高度区别。
也可用 `dog_bringup mode:=navigation` 统一启动定位与本包，见根目录 README。
分文件架构、任务接口、默认开关和验证边界见 [EXECUTION.md](EXECUTION.md)。

## 源码与配置分类

| 目录 | 内容 | 配置 |
| --- | --- | --- |
| `src/global_planner/` | 整条地形图路线搜索、复用、动态边代价缓存与可执行性校验 | `config/global_planner.yaml` |
| `src/local_planner/` | 实时障碍记忆、碰撞覆盖层、已校验路径截取、时间参数化与路径跟踪 | `config/local_planner.yaml` |
| `src/decision/` | 任务状态机、恢复/重试、定位有效性门控、安全仲裁 | `config/decision.yaml` |
| `src/ros/` | 规划节点、订阅发布、TF、调度和参数接入 | `config/navigation.yaml` |
| `src/execution/` | 执行与安全节点的 ROS 接入 | `config/execution.yaml` |
| `src/common/` | ROS 转换工具与阶段计时 | 无独立配置 |

这里的决策是有限状态机，不是行为树。`LocalPlanner` 为兼容已有调用保留名称，
其完整路线搜索实现位于 `global_planner/terrain_planner.cpp`；底层图搜索仍复用
`dog_terrain`。局部部分没有独立的 DWA/MPC 轨迹优化器，路径截取不改变已校验地形边。

两个 launch 均支持 `global_planner_config:=...`、`local_planner_config:=...`；
执行 launch 额外支持 `decision_config:=...`。
规划节点依次加载全局、局部、`config`，执行节点依次加载局部、决策、`execution_config`；
后加载的同名参数优先，launch 显式参数最后覆盖。已有自定义 `config` / `execution_config`
可继续覆盖原参数，bringup 的 `navigation_config` / `execution_config` 用法保持不变。
直接运行可执行文件时须自行传入所需 `--params-file`，配置不会自动加载。

静态地形参数和碰撞尺寸取自所加载地图工程的 config；0.66 m 长、0.58 m 宽的机器人
应使用按该尺寸构建并验证的地图。在规划 YAML 重复填写尺寸无法改变已有地图的碰撞认证。
修改分类 YAML 后需要重启对应节点。

频率参数均为 Hz：`global_planner.yaml` 的 `planning_rate`（默认 5）驱动共用规划循环，
包含障碍更新、全局路线校验/必要时重搜索及局部路径截取；`processing_budget` 是该循环的
拒绝输出预算，不是定时周期。`local_planner.yaml` 的 `controller_frequency`（默认 20）
驱动跟踪与任务更新；`execution.yaml` 的 `guard_frequency`（默认 50）驱动独立安全检查和
速度输出。控制和安全循环周期须小于当前 0.15 s 的许可/制动反应时间；调高频率不保证
实时性，调低频率不延长许可或数据有效期。当前没有独立的局部轨迹优化频率。

## 启动

```bash
source /opt/ros/humble/setup.bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 colcon build --symlink-install \
  --packages-up-to dog_navigation --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

# 终端一：已有地图定位，按 dog_slam 文档提供正确的机身初始位姿。
ros2 launch dog_slam localization.launch.py \
  map_path:=/home/yuling/dog_itlio/data/maps/saved_20261107_170227

# 终端二：只启动避障与重规划，不启动硬件、LIO 或底盘驱动。
ros2 launch dog_navigation navigation.launch.py \
  map_root:=/home/yuling/dog_itlio/data/maps map_id:=saved_20261107_170227

# 终端三：替换为实际可通行地表的地图 XYZ（这里是目标地面高度）。
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: map}, pose: {position: {x: 1.5, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}'
ros2 topic echo /local_navigation/status
```

定位地图和导航地图须来自同一个地图项目；默认检查定位状态中的 `map_path` 是否为该项目目录、
目录内 `localization.pcd` 或 `cloud.pcd`。导航加载时通过 dog_terrain 校验项目哈希与版本。
本节点不热加载地图，切换项目须重启；手动替换文件时，定位与导航均须重新加载同一版本。
同名文件不会复用内存缓存，但仅同一路径检查无法证明外部进程此前加载的文件内容相同。
支持 `use_sim_time:=true` 与 `config:=/absolute/path/navigation.yaml`，回放时两节点时间应一致。

## 数据流与算法

1. 消费 `/lio/cloud_deskewed`，要求 `livox_frame` 中 FLOAT32 XYZ，扫描结束时间戳新鲜、递增。
   用扫描时刻的 TF 把传感器原点、点云转到连续 `odom` 系。保留最新扫描，默认 5 Hz 处理。
2. 在 `odom` 中维护默认 0.10 m 的三维体素记忆，水平半径 8 m、上下范围各 4 m。
   返回点产生占据证据，扫描射线产生空闲证据；同帧占据优先。只有新射线穿过、或点离开局部
   窗口才删除旧占据，不用时间到期把遮挡物直接变成空闲。空扫描、超出资源预算或处理失败不部分清图。
3. 规划时使用当前完整 `map ← odom` 修正，把观测叠加到三维地形图。
   与原始静态地图距离不超过 0.08 m 的返回按已有几何处理，防止把已验证的台阶踏面、立面
   反复当作新障碍；其余返回作为新障碍。这里没有物体类别识别或运动目标跟踪。
4. 在地形图已有边上检查新障碍，禁止创建未知地面、跳过空洞或直接连接上下楼层。
   动态障碍包络为地图竖直圆柱：旧项目取 `body_radius`，矩形项目取足迹外接圆半径，
   再加 0.12 m 余量及体素半对角线；
   加上地形配置的 `footprint_padding`；高度从地表上方 0.12 m 到 body_top 加余量。
   整条三维边使用连续圆柱扫掠检查，不会只在离散位置检查硬碰撞；距离代价仍按半体素采样。
   这能区分同 XY 的不同楼层，但不是完整机身姿态扫掠体或落足动力学检查。
5. 每周期从当前机身下方的名义接地点运行三维 A*。沿用原地形边界、坡度和台阶代价，
   增加新障碍附近的距离代价；被占据的连接禁止通过。输出整条路线，以及默认前 3 m 的局部段。
   局部段保留图边，可多包含一条跨越长度界限的已检查边；不做穿墙风险未知的捷径或平滑。

`base_ground_offset` 默认 0.65 m，仅是可配置初值，**不是实机标定结果**。
起点使用完整机身变换作用于 `[0,0,-base_ground_offset]`，保留坡道倾角，不直接把机身 Z 设成零。
该近似须按实际站姿调整，否则可能找不到地面或选错表面。目标输入始终为地形接触点高度，
与 dog_slam 的 `/initialpose` 机身高度含义不同。

## 接口契约

| 接口 | 类型与含义 |
| --- | --- |
| `/goal_pose` | PoseStamped，map 中目标地表 XYZ；朝向当前不参与规划，零时间戳允许 |
| `/map_localizer/status` | 定位 JSON，须 valid、map_ready 且项目匹配 |
| `/local_navigation/global_path` | nav_msgs/Path，带实时障碍约束的全局地表路线 |
| `/local_navigation/local_path` | nav_msgs/Path，局部地表路线；不是机身轨迹或速度指令 |
| `/local_navigation/obstacles` | PointCloud2，map 中本轮识别出的新增障碍返回 |
| `/local_navigation/status` | JSON，有效性、停车请求、数据时效、地图 revision、台阶边索引、处理耗时 |
| `/local_navigation/stop_required` | Bool，true 表示请求停止执行；本节点没有执行停车的底盘连接 |
| `/local_navigation/cancel` | Trigger，清除当前目标与路线 |

有效路线的全局/局部 Path 使用相同 `header.stamp`，状态的 `route_stamp` 与其对应；
`scan_stamp` 单独标记障碍数据时间。Pose 朝向为单位四元数占位，不表示期望机身姿态。
路径失效立即发布空 Path 与 `stop_required=true`，不会继续对外报告上一条路线可用。
起终点落在同一地形节点时报告 `goal_reached`、清除目标并请求停止；这是离散图判断，不是精确到位控制。
状态中的 `stairs_edges` 为全局路线中边的下标，仍然只是原地形图的台阶候选。

默认扫描超时 0.6 s、定位状态超时 0.8 s、TF 超时 0.4 s、处理预算 0.4 s。
定位无效/地图不匹配、扫描或 TF 过期、配准修正 TF 不新鲜、端点受阻、无绕行路线、计算超时
均撤销路径。检测 odom 跳变或时钟回退时清除局部记忆与目标，要求重新提交目标。
路径使用前须同时匹配状态与路径时间戳、检查 status.valid、valid_for_seconds 和数据时效；
状态有效不代表导航执行已启用。节点进程退出时需要下游自己的超时停车，不能依靠最后一条 Bool。

## 能力边界与验证

### 已有地图的地形重建

地形图在导入时生成；重新编译不会改写已有 `navigation.json`。
碰撞算法缓存为 v3；加载 v1/v2 项目时校验原文件后，从源点云在内存中重建，
避免复用旧离散碰撞边。原项目文件保持不变，但加载时间增加、运行时 revision 更新，
需要重新选点。下述命令可将新结果另存为独立项目，后续直接加载 v3 缓存。
当前构建器先在均匀采样点云上估计局部平面，再将实际观测点分配给平面，避免台阶立面把
踏面连成一个高度簇。支撑检查使用已分配的踏面点；原始点云仍完整用于机身碰撞。
跨台阶连接受 `max_step` 与 `max_slope_deg` 约束，不再另设硬编码的 12° 坡度上限。
这些是几何预览约束，不代表已验证实机落足或楼梯动作。

```bash
source install/setup.bash
ros2 run dog_terrain rebuild_map "$PWD/data/maps" \
  bag_20261107_203614_r015 bag_20261107_203614_r015_stairs_v2 \
  11.922 -2.174 -0.829 14.014 -4.285 -5.320 \
  --config "$PWD/data/config/terrain.json"
```

输出 ID 必须不存在，源项目与其坐标系、点云和关键帧保留。末尾六个可选参数为起终点 XYZ；
返回码 2 表示新项目已保存，但指定路线未连通，不表示可执行。旧项目不会自动迁移或覆盖。
导入服务需重启后才会使用新构建器；新项目可从 Web 地图库单独加载。
`--config` 可覆盖源项目的部分地形参数。`data/config/terrain.json` 的 `body_top=0.5`
用于地表相对的障碍高度筛选，不等于完整机身高度碰撞认证。
机器人平面包络按用户指定的 `body_length=0.66`、`body_width=0.58` 米。
Web 控制台后续导入使用相同配置；已有项目必须重建才会更新，旧圆柱项目保留原语义。

### 矩形足迹与地形构建配置

`data/config/terrain.json` 使用 0.10 m 规划网格，并将表面估计尺度从网格尺寸解耦：
法向邻域 0.10 m，仅对失败估计回退到 0.15 m；保留已通过的小尺度估计。
法向厚度上限 0.02 m，平面内点距离 0.015 m。
支撑搜索距离为 0.12 m；候选连接水平距离最多 0.25 m，仍检查沿途观测支撑与
`max_step=0.22` m，不按距离直接补边，也不补造地图点。

`terrain_tolerance=0.02` 启用局部观测地形跟随：在足迹附近，按高度差不超过
`max_step` 且中间有表面观测的候选节点寻找相连踏面。足迹支撑可以分布在多级台阶，
障碍高度筛选也相对这些局部踏面进行，避免把后一级踏面作为机身障碍。
未与当前地形局部相连的楼层、无支撑缺口、墙体和超高台阶不能据此放行。
这仍是有界距离、离散支撑采样的几何模型，不包含腿部碰撞、步态、动力学与上方净空认证。

矩形边按行进方向进行连续平移扫掠检查；机器人允许倒行，因此朝向等价模 pi。
搜索状态包含上一节点，对转弯补查旋转扫掠体（角度采样并膨胀包络以覆盖采样间隙）。
起终点仅指定 XYZ，朝向不约束；连通分量相同仍可能因转弯不通过而没有路线。
局部动态避障暂用矩形外接圆加原有余量，较静态矩形检查更保守。

缓存版本 5 保存支撑点索引，并将已通过邻域平面验证的支撑点与单格节点生成解耦；
单格点数不足或分布过窄时不生成节点，但不丢弃已观测的平面支撑。
原节点无效时，可尝试同格中心在该测量平面上的位置，须有邻近实测平面点，
并重新通过足迹支撑与碰撞检查；原本有效的节点不移动。
版本 4 项目保留原有结果，显式重建才启用上述地面覆盖改进。保存不覆盖同名项目。

地面覆盖审计位于 `data/navigation/floor_coverage/`。固定原始点云和四个近水平参考平面，
用同一批观测点满足矩形支撑、保守障碍筛选的评估位置比较各版本；不将所有水平面视为地面真值。
报告同时保留同格节点覆盖、固定 0.15 m 近邻覆盖和已有边覆盖，避免跨网格尺度比较时混淆格子对齐与
实际节点距离。最终 `floor_support_r010_v5` 对相同的 18 组地面 XYZ 查询全部通过，
0.15 m 版本为 14/18；原指定跨楼层路线也保留连通。端点仍按地图契约吸附，具体偏差见报告。
细网格的节点数和离线构建成本增加，近邻覆盖也不代表每个位置都能到达或已通过实机验证。
只读诊断工具 `inspect_map MAP_ROOT MAP_ID NODE_PAIRS_JSON` 可报告指定节点对的
碰撞见证点和缺少支撑的位置，节点编号必须来自同一地图 revision。

### 全局碰撞与间隙代价

参考旧工作空间 `dog_nav` 的“硬碰撞＋软膨胀代价＋完整路段验证”分层，保留当前的
三维多地表表达。旧工程普通全局规划是 SmacPlanner2D，楼梯依赖人工配置中心线、楼层和平台，
不能直接投影当前多层点云后宣称完成三维楼梯规划。

旧圆柱项目对每条边检查连续竖直圆柱扫掠体，水平半径为 `body_radius + footprint_padding`，
上下界为随地表路径插值的 `body_bottom/body_top`，XY 接近与高度区间必须同时满足才判碰撞。
支撑检查独立使用已观测地面，不因机身无碰撞而允许跨越未知空洞。
软代价同时考虑可通行图边界和当前高度包络内的原始障碍距离；不把另一楼层投影成当前层障碍。
全局路线返回前再次验证所选边，输出 `collision_check` 检查模型、边数与包络参数。

`footprint_padding` 默认 0m，允许显式增加非负平面安全余量；它不改变坡度、台阶或支撑阈值。
新矩形项目使用已确认的 0.66×0.58m 长宽和 0.5m 总高。腹部高度、接地偏移仍须实测；
该模型没有完整机身俯仰/横滚、四足落点或腿部扫掠动力学。
不能将可显示的地表路线当作可直接执行的机身运动轨迹。

Web `POST /api/inspect_surface` 接受 `map_id/revision/point`，返回附近表面的支撑、
机身碰撞、碰撞点坐标及连通分量。`/api/plan` 失败时同时返回 `start_surface/goal_surface`；
两端都有效但分量不同说明中间仍存在断点，不能靠放大端点吸附范围跨过。

实时观测规划层不直接控制底盘；执行与安全仲裁使用下面的独立模块。
动态障碍速度预测与物理停车验证仍未完成。
默认 5 Hz 是配置频率，不是已经证明的最坏时延保证；过载或超预算时撤销路线。

全局路线中超出局部观测范围的部分依赖静态地图；没有把未被当前雷达看到的空间证明为自由空间。
雷达遮挡、稀疏点云、盲区、小于高度阈值的障碍或静态地图误差都可能漏检。
体素记忆可能因未获得清除射线而保留已离开的物体，这是保守阻塞；不会仅超时恢复通行。
射线清除依赖正确的去畸变、传感器外参和时间同步，尚未接入逐点不确定度或多帧占据概率。
已知静态几何距离过滤也依赖定位与地图准确，阈值需要在真实数据上验证。

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 colcon test --packages-select dog_navigation --event-handlers console_direct+
colcon test-result --test-result-base build/dog_navigation --verbose
```

C++ 测试使用合成地面、重叠楼层和折返楼梯，检查绕行、边上碰撞、堵塞无路径、SE(3) 修正、
楼层隔离、静态台阶保留、射线清除与遮挡保留。DDS 测试发送合成 TF、定位状态和实时扫描，
检查路线更新、阻断后清空路径、定位失效、地图不匹配、扫描断流与取消目标。
未使用真实机器人、动态行人或真实楼梯数据验证，不能据此宣称实机避障可靠性。
