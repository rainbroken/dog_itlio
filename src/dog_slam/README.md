# dog_slam：机器人 LIO 建图与已有地图定位

`dog_slam` 管理机器人侧入口、安装标定、TF 和输出策略；`src/3rd_utils/itlio`
继续提供 IteraLIO 算法及 Livox 数据桥接。建图与已有地图定位使用独立入口；导航运动执行尚未接入。

## 编译与启动

在工作空间根目录执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to dog_slam \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch dog_slam mapping.launch.py
```

先独立启动 MID360 驱动，提供 `/livox/lidar` 和 `/livox/imu`。
入口默认启动桥接、IteraLIO、robot_state_publisher 和 lio_tf_adapter，
不启动传感器硬件驱动、底盘驱动、RViz 或相机。不要同时启动原来的
`itera_lio.launch.py` 或另一个 `dog_description` 发布者。
如需要底盘关节状态，另启 `dog_driver`，保持 `publish_odom_tf:=false`；
`/dog/odom` 只作为独立腿式里程计观测，当前不参与融合。

回放机器人安装数据：

```bash
ros2 launch dog_slam mapping.launch.py use_sim_time:=true
ros2 bag play /path/to/bag --clock
```

已有 `/interfaces/lidar`、`/interfaces/imu` 时可用 `bridge:=false`。

### 保存时的动态残影清理

机器人建图入口的 `config/mapping.yaml` 开启
`save_pcd.dynamic_filter.enabled`。参考旧工程的多帧占据/射线空闲证据，
在 IteraLIO 内对完整扫描归档进行离线清理，不按前后方向、人体位置或地图区域删点。
仅删除至少三帧射线穿过、来自不同 0.5m 观察位置网格、且累计空闲证据强于
占据证据的体素。射线在真实回波前留出距离余量，同帧占据优先；未再次观测、
被遮挡或观测不足的点保留。有评估 CSV 时，跟踪/入图失败帧不提供清理证据。
这是静态地图输出清理，不能识别所有人体，长期静止的人仍可能保留；位姿误差也可能
导致静态物体被误删。前端配准和实时障碍扫描仍保留原有行为。

默认保存两份 PCD：`cloud_static.pcd` 为过滤地图，`localization.pcd` 为定位地图。
定位地图是过滤结果中证据体素至少两帧命中的稳定观测点子集，不是另一份未过滤地图。
两者保留相同的原始坐标及 0.05m 保存分辨率；定位节点按自己的配置进行配准采样。
`localization_keyframes.csv` 是匹配坐标系的定位辅助轨迹，仍需保留；
`dynamic_filter_report.json` 记录参数、点数与耗时。0.20m 证据网格只决定动静判定；
输出 PCD 从原始 PCD 样本中选择，
**保持当前 IteraLIO 的 0.05m 保存体素大小**（当前 `voxel.map_size` 同为 0.05m）。
不把证据网格当成保存分辨率，也不使用旧工程的 0.08m 输出。
Web 的已保存 PCD 列表可选择 `cloud_static.pcd` 导入，并沿用相邻的 `localization.pcd`；
同名导入生成独立地图，不会覆盖原有地图。静态地图使用原始 LIO 的
`localization_keyframes.csv`，不能配对回环优化轨迹；精简模式只输出过滤后的原始 LIO 地图，
不额外导出优化地图及后端评估副本。

`save_pcd.keep_debug_artifacts` 默认 false：保存成功后清除原始 `cloud.pcd`、
移除点 PCD；退出保存且定位轨迹写入成功后，再清除逐帧 `.bin` 与扫描索引。
手动保存期间保留扫描归档，允许继续建图、再次保存。任一输出失败时保留恢复数据。
需要重新调整过滤参数做对照时，建图前显式设为 true，才保留这些调试文件；
精简后若需重新过滤，须从 rosbag 重跑，不能只靠最终静态点云恢复逐帧可见性。

旧的纯 LIO 完整扫描归档也可以重新清理：

```bash
source install/setup.bash
ros2 run itera_lio filter_static_map /absolute/path/to/slam/run
```

工具读取该次运行 `evaluation/parameters.yaml` 的实际保存分辨率和过滤参数；
默认不删除原始 PCD、扫描归档或轨迹；对已完成运行加 `--compact`，
会在两份新地图及定位轨迹检查成功后删除中间文件。清理在保存时执行并占用 CPU/内存，
手动保存会等待处理完成，退出时同样需要等待。机器人入口在 SIGINT 后预留 90 秒再升级 SIGTERM；
更长数据需实测耗时，不能在保存中途强制关闭。

输入桥接保留原有时间语义，点云/IMU 头分别统一为 `livox_frame` / `livox_imu`，
不旋转测量向量（驱动须提供原生传感器轴数据）。仓库 Livox 驱动原先把 IMU 头也写为
`livox_frame`，此处仅修正其命名。加速度沿用 `interfaces` 的 g → m/s² 换算；
实机需核实发布端单位和 reliable QoS。不要对已经是 m/s² 的输入再次换算。
传感器配置可用 `sensor_config:=/absolute/path/sensor.yaml` 替换，但帧名、模式和外参
由系统入口强制统一；输出里程计话题固定为 `/lio/odom`，更换输入话题须与桥接输出对应。

## TF 与外参契约

```text
map                         IteraLIO：map → odom
└── odom                    dog_slam：odom → base_link
    └── base_link           robot_state_publisher：以下固定/关节关系
        └── torso
            └── livox_frame
                └── livox_imu
```

这里树中每条边由父节点指向子节点。`map → odom` 无回环时为单位变换，
有回环时允许修正；局部控制应使用连续的 `odom`，不能直接承受 `map` 修正。
IteraLIO 默认的 `odom → imu` 与 `imu → lidar` 在本入口关闭，
避免 IMU/LiDAR 与 URDF 出现多个父节点。新增的三个 `tf.publish_*` 开关
默认均为 true，原第三方启动入口保留原行为。

安装标定的唯一来源是 `dog_description/urdf/bpx.urdf`，
或 `robot_description_file:=/absolute/path/robot.urdf` 指定的模型。
入口沿固定链计算 `T_BI`、`T_BL`，给 LIO 的标定是
`T_IL = inverse(T_BI) * T_BL`，满足 `p_I = R_IL * p_L + p_IL`。
适配节点计算 `T_OB = T_OI * inverse(T_BI)`，保留 xyz 和 roll/pitch/yaw，
使用原扫描结束时间戳；收到错误帧名、非有限位姿、非法四元数或时间戳时拒绝发布。
断流后停止更新 TF，不外推位姿；导航接入时仍须增加独立的时效与定位质量门控。

当前 URDF 的雷达安装为前移 0.25 m、上移 0.12 m、pitch 约 30°，
其 IMU 固定关节还含小幅 roll/pitch 校正。**这些是现有模型数据，不代表已经实机标定。**
原 `mid360.yaml` 的 IMU–LiDAR 旋转为单位阵，与此模型不同；
因此旧手持数据不能直接使用机器狗默认模型，须提供对应安装标定的 URDF。
模型内相机链接只保留静态结构，不引入相机订阅或融合依赖。

## 输出与使用边界

| 输出 | 坐标系/语义 |
| --- | --- |
| `/lio/odom` | `odom` 中的 `livox_imu` 原始里程计，不是 base_link 里程计 |
| `/lio/path` | `odom` 中的 IMU 轨迹 |
| `/tf` | `map → odom` 和 `odom → base_link` |
| `/lio/cloud_registered` | 当前源码使用 LIO 连续里程计变换点，但 header 标为 `map`；不可直接作为已有地图坐标点使用 |
| `/lio/cloud_deskewed` | `livox_frame` 中去畸变扫描；已有地图定位使用此输入和 URDF 安装外参 |
| `/lio/global_map` | `odom` 系累计预览，有点数上限，不是完整地图存储 |
| `/lio/optimized_map`、`/lio/optimized_path` | 后端优化结果，`map` 系，按后端事件输出 |
| `/lio/save_pcd` | `std_srvs/srv/Trigger` 几何地图保存 |

```bash
ros2 service call /lio/save_pcd std_srvs/srv/Trigger '{}'
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link livox_imu
```

默认归档和评估目录为 `~/.local/share/dog/slam`，可用 `output_dir:=...` 替换；
内部文件布局由 IteraLIO 保存器决定。默认退出时也保存几何 PCD，须正常停止并等待写盘。
本入口显式关闭图像记录和保存后 refine。点云预览与归档仍有运行和存储成本，需用实测数据评估。

本入口默认关闭回环，`loop_closure:=true` 才启用当前第三方后端参数；
参数曾面向其他数据配置，尚未在本机器狗数据上验证。后端通过 `map → odom`
表达图优化修正，并未将优化状态反馈前端。无回环的图优化输出不能视作已消除漂移。
原始 `/lio/odom` 未提供可直接用于下游融合的有效完整协方差及角速度，
本次只提供正确的机身 TF，不伪造机身速度或不确定度。

`dog_terrain` / `dog_web` 仍是离线地图与多层地表规划入口；这里没有自动将在线点云接入它们，
也没有局部动态避障、路径跟踪或异常停车闭环。已有地图定位使用下面的独立入口。
导入 PCD 前还需确认米制、Z-up、地图版本及坐标对齐；TF 接通本身不证明这些条件成立。

## 已有地图定位与重定位

```bash
source install/setup.bash
ros2 launch dog_slam localization.launch.py \
  map_path:=/home/yuling/.local/share/dog/maps/full_map
```

`map_path` 接受 PCD 文件或地图项目目录；目录优先读取 `localization.pcd`，不存在时读取
`cloud.pcd`。支持 PCL 的 ASCII、binary 和 binary_compressed PCD，要求米制、Z-up、FLOAT32 XYZ；
保留三维高度与 roll/pitch/yaw。读取选定文件，不按文件名复用地图；项目目录在此仅用于选择
几何 PCD，不校验 `dog_terrain` 的导航缓存或哈希。地图定位与导航图须使用同一坐标和版本。

此入口复用建图入口的传感器桥接、URDF 与机身 TF，关闭 LIO 后端、回环、LIO 的
`map → odom`、PCD 归档和退出保存。`map_localizer` 独占 `map → odom`；
`lio_tf_adapter` 继续发布连续的 `odom → base_link`，不将地图配准修正注入局部里程计。
不要同时运行 mapping.launch.py。已有正确配置的 LIO/TF 时可用 `start_lio:=false`，
但须自行关闭其他 `map → odom` 发布者并启用 `/lio/cloud_deskewed`。

首先提供地图中的 **base_link 机身位姿**，不是脚下地面点：

```bash
# 示例值须替换为实际机身位置；四元数应表达实际朝向/倾角。
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  '{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.65}, orientation: {w: 1.0}}}}'
ros2 topic echo /map_localizer/status
```

初始位姿零时间戳表示下一帧扫描时的位姿；非零时间戳须新鲜，模块用对应时刻的 odom
补偿到当前扫描。也可在 `config/localization.yaml` 设置 `initial_pose: [x,y,z,roll,pitch,yaw]`
并通过 `localization_config:=...` 指定配置。未初始化、配准未确认或失锁时不发布定位 TF。

流程为扫描距离过滤与体素降采样、局部三维地图裁剪、粗细两级点到点 ICP，随后检查扫描重叠率、
反向重叠率、RMSE、基于地图法向的六维几何可观性和相对预测位姿的修正量。
默认连续三帧修正一致才确认初始定位或失锁恢复。有效时以最新实际 odom TF 时间戳发布修正，
使用完整 `T_map_odom = T_map_base * inverse(T_odom_base)`。

失锁恢复在最后修正与当前 odom 预测附近进行有限 XYZ/yaw 多假设搜索；默认 XY ±1 m、
Z ±0.3 m、yaw ±0.52 rad，对排名靠前的四个假设配准，并拒绝已检查候选中的歧义。
以上是**有初值、有限范围重定位**链路。默认开启的历史关键帧全局检索与无历史地图兼容搜索见下文。
两种方式都不能排除所有重复结构的错误匹配，实机使用仍需验证地图和观察几何。

| 接口 | 用途 |
| --- | --- |
| `/map_localizer/status` | JSON：状态、valid、地图路径、拒绝原因、重叠率、残差和确认次数 |
| `/map_localizer/pose` | 有效且新鲜的地图系机身 PoseStamped；不伪造协方差 |
| `/map_localizer/map` | 加载后降采样的地图，transient_local |
| `/diagnostics` | 定位诊断 |
| `/map_localizer/relocalize` | Trigger：请求有界重定位，成功回执只表示已入队 |
| `/map_localizer/global_relocalize` | Trigger：请求地图级搜索，需要先在 YAML 开启 |
| `/map_localizer/reset` | Trigger：清除初值与有效状态 |
| `/map_localizer/reload_map` | Trigger：重新读取参数 map_path 对应文件，清除旧定位 |

```bash
ros2 service call /map_localizer/relocalize std_srvs/srv/Trigger '{}'
ros2 param set /map_localizer map_path /absolute/path/new_map.pcd
ros2 service call /map_localizer/reload_map std_srvs/srv/Trigger '{}'
```

参数除 `map_path` 配合 reload 外，在启动时读取。断流、过期结果、TF 查找失败和配准失败均
撤销有效状态；检测 odom 跳变或时钟回退后要求新初值。TF 缓冲仍可能保留历史修正，
下游必须同时检查 status.valid 和 TF 时效，不能仅以 TF 可查询判断定位正常。
地图加载失败不继续使用旧定位。单平面等退化场景会拒绝输出；参数需结合实际点云调整。
网页已接入初始位姿和重定位请求，路径执行和失锁物理停车仍未联动。

### 在网页初始化位置与朝向

先启动 `dog_bringup mode:=navigation map_id:=...`，在网页地图管理中加载相同项目。
展开右侧“定位初始化与重定位”，点击“点选位置”，选中机器人当前脚下的地表节点，
填写地图朝向与机身高度偏移后点击“提交初值”。紫色箭头表示设定的地图朝向。
0° 指向地图 +X，90° 指向 +Y，和机器人自身坐标的朝向定义需一致。

高度偏移默认 0.65 m，只是配置初值，须按实际站姿校准。网页把地面点加上沿机身 Z 轴的
高度偏移作为机身位置；roll/pitch 保留当前新鲜 odom→base_link TF 的倾角，yaw 使用填写值。
因此需要 LIO 已经产生有效机身 TF。若不满足重力方向、外参或站姿条件，应直接使用完整
SE(3) 的 `/initialpose` 或 YAML `initial_pose`，不要把地面点直接当作机身坐标。

后端检查网页地图版本与定位地图身份；“已提交”仅表示初值已发送，
只有多帧质量检查通过后网页才显示“已确认定位”。设置初值不会发送运动指令。
HTTP 接口为 POST `/api/localization/initialize`，字段是 `map_id/revision/ground_position/yaw/body_height`；
其中 yaw 为弧度，网页输入为度并转换。重定位按钮调用同前缀的 `relocalize` / `global_relocalize`。

### 历史关键帧全局重定位

默认在 `config/localization.yaml` 开启。加载地图后自动搜索，失锁后先附近恢复、再全局检索，
无需点选位置与朝向；人工初值仍优先。配置示例：

```yaml
map_localizer:
  ros__parameters:
    max_result_age: 9.0     # 全局搜索结果时效预算
    global_relocalization:
      enabled: true
      on_startup: true       # 没有初值时自动搜索
      on_lost: true          # 有界恢复连续失败后再搜索
      keyframe_candidates: 128 # 描述子检索保留的位置数，2..256
      min_overlap: 0.80       # 全局候选正向重叠率额外门限
      body_height: 0.65      # 名义机身离地高度，必须校准
      xy_step: 1.0
      height_step: 0.5
      yaw_steps: 24
      max_hypotheses: 30000
      scoring_points: 128
      refine_candidates: 12
      time_budget: 8.0      # 默认预算，按地图和硬件测试调整
      retry_interval: 3.0
      failures_before_global: 3
```

设 `global_relocalization.enabled: false` 可关闭全局重定位。
需要人工控制启动时，可设 `enabled: true, on_startup: false, on_lost: false`，
再调用服务或网页按钮：

```bash
ros2 service call /map_localizer/global_relocalize std_srvs/srv/Trigger '{}'
```

新建纯 LIO 地图保存时导出 `localization_keyframes.csv`（map/world 中的 IMU 位姿）；
原始 `cloud.pcd` 配原始关键帧，`cloud_optimized.pcd` 配
`localization_optimized_keyframes.csv`，避免混用优化前后坐标。网页“导入已保存 PCD”
将对应文件复制到地图项目，并纳入 `map.json` 的 SHA256 校验。
旧纯 LIO 会话的 `cloud.pcd` 若同目录恰有一个 `*_raw_keyframes.csv`，重新导入时也可使用；
不自动猜测旧优化地图的轨迹。相同地图 ID 仍禁止覆盖，网页同名导入会分配新 ID。
旧项目未自动修改，须重新导入原始会话才能携带历史关键帧。

有历史文件时，按历史观测位置对几何地图提取局部径向/高度占据直方图，检索与当前扫描相似的
前 `keyframe_candidates` 个位置，再枚举 yaw、近邻评分与 ICP 验证。
这些描述子由几何地图重建，不是持久化的逐帧原始扫描，也不是 Scan Context 实现。
历史 IMU 位置只用作近似机身初值，其传感器杆臂由有界 SE(3) 配准修正；
若安装杆臂超过 `max_initial_translation/max_initial_vertical`，需要按实际硬件调整门限。
历史位置按 0.5 m XY / 0.25 m Z 分桶去重，索引最多 10000 个位置。
搜索范围受建图经过位置和候选截断限制，远离历史路径时可能失败。
描述子使用二值占据以降低累计地图与单帧扫描的采样密度差异。
反向重叠率仅统计当前扫描有返回的 1° 方位/俯仰角格内、首个返回之前的地图点，
不把未观测方向与被遮挡表面纳入分母；仍保留前向残差、重叠率与几何退化门限。

没有历史文件的地图兼容原有**重力约束的地图级位置与全 yaw 候选搜索**。
要求 map/odom 的 Z 均与重力对齐，使用 odom 中的机身倾角和名义离地高度；
地图近水平表面按 XYZ 分桶产生锚点，对每个锚点枚举完整 360° 朝向。
点云近邻粗评分后，对空间/朝向分散的前若干候选进行原有粗细 ICP 和重叠率、残差、
几何退化、修正量检查。多个不同位姿得分相近则拒绝，成功候选仍须连续多帧确认才发布 TF。
人工初值优先；确认位置变化在当前机身处比较，避免受 odom 原点距离影响。

只有在指定地图中生成的锚点与 yaw 候选数、耗时都未超预算时才允许成功；
超限报告 `global_hypothesis_budget_exceeded` / `global_time_budget_exceeded`，不使用被截断搜索的结果。
细配准只验证排名靠前的有限候选，所以歧义拒绝仍不等于证明整栋建筑唯一匹配。
缺少地表返回、不同站姿/高度、重力对齐误差、遮挡或相似楼层可能导致搜索失败或误匹配；
没有运动目标跟踪或全局最优保证；默认开启也不代表已通过实机定位验证。

状态增加 `global_relocalization_enabled`、`localization_source`、`registration_duration_ms`，
以及本轮 `relocalization_scope`。失败时保持 valid=false；关闭开关时不会建立全局索引或接受全局服务。
关键帧检索的 scope 为 `historical_keyframe_search`、source 为 `historical_keyframes`；
无历史文件时 scope 为 `gravity_constrained_map_search`、source 为 `global_map_search`。
配置在启动时读取，修改 YAML 后需重启；提高 time_budget 时还须让 max_result_age 大于该预算。
既有 ARM 合成房间测试在 1 秒预算下触发超时拒绝，采用 5 秒/6 秒组合；
这不是实际建筑耗时保证。增大结果时效意味着允许更旧的配准输入，移动机器人应优先停车重定位，
并用真实扫描验证预算、运动与时效的关系。

## 验证

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 colcon test --packages-select dog_slam --event-handlers console_direct+
colcon test-result --verbose
```

测试覆盖带旋转/平移安装外参的机身位姿恢复、真实 URDF 的外参链一致性和非法传感器安装链拒绝。
另外通过 DDS 合成里程计检查机身 TF、时间戳保留和非法输入拒绝。
桥接测试检查点云/IMU 帧名、时间戳和加速度单位换算。
定位测试使用合成三维房间，检查 SE(3) 配准、高度/倾角变化、三帧确认、失锁恢复、
退化平面与错误楼层初值拒绝、三种 PCD 编码与同名文件重读；通过 DDS 检查地图 TF、
传感器安装平移、断流撤销、reset 和错误地图重载。启动配置测试检查两个模式的 TF 开关。
这些是合成测试，不代表实机定位精度或真实楼梯定位已验证。
当前机器的用户级 anyio 插件与系统 pytest 不兼容；遇到 pytest 启动错误时，在编译和测试前
设置 `export PYTHONNOUSERSITE=1`。如之前已配置跳过 Python 测试，编译时增加
`--cmake-force-configure` 重新发现测试。
实机需继续检查时间同步、静止重力、安装方向、运行时 TF 唯一发布者与断流处理，
再分别验证建图、定位和导航闭环。

当前构建仍有 `cv_bridge` 所需 OpenCV 4.5 与环境 OpenCV 4.8 的混用链接告警，
本次未调整第三方依赖；虽接入测试通过，后续仍需统一依赖并验证实测数据运行。
