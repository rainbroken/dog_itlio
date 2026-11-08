# 评估记录

默认 `evaluation.detailed: false`，新运行使用精简 CSV。历史运行文件保持原样。
每类文件仍可通过 `evaluation.outputs` 独立关闭。

| 文件 | 默认内容 |
| --- | --- |
| `frames.csv` | 30 列，逐帧记录匹配质量、跟踪状态、地图更新、主要耗时及实际回环提交 |
| `runtime.csv` | 32 列，默认每 5 秒记录输入速率、队列、累计异常、后端规模、重建及内存 |
| `odom.csv` | 38 列：时间戳、位姿、速度、重力、IMU 偏置及 18 项协方差对角线；不受 `detailed` 影响 |
| `loop_verifications.csv` | 每次候选验证的完整记录，包括拒绝原因、几何指标和修正变换 |
| `events.csv` | 输入异常、跟踪状态切换、实际回环提交、慢速地图重建 |
| `summary.json` | 整趟运行的统计、累计失败和回环计数 |
| `parameters.yaml`、`manifest.json` | 有效参数快照、坐标系、产物清单及记录模式 |

逐帧表按以下顺序排列：

- 定位：`frame_index`、`timestamp`。
- 匹配：`scan_points`、`sampled_points`、`effective_points`、`mean_residual_before`、`mean_residual_after`、`iterations`、`converged`。
- 状态：`tracking_state`、`degenerate`、`condition_number`、`map_updated`、`map_insert_reason`、`map_size`。
- 修正：`translation_correction`、`rotation_correction`。
- 耗时：`deskew_ms`、`registration_ms`、`correction_ms`、`map_update_ms`、`total_ms`、`core_end_to_end_ms`、`output_ms`、`end_to_end_ms`、`callback_to_state_publish_ms`。
- 回环：`loop_candidates`、`loop_verifications`、`loop_constraint_added`、`backend_optimized`。

位置、平面残差和位移修正的单位为 m，旋转修正为 rad，耗时为 ms，内存为 MiB。
时间戳保留原有秒值和精度，便于跨文件关联；计数、残差和耗时不代表绝对定位精度。
`loop_verifications.csv` 中候选通过不等于已入图，实际提交看 `loop_constraint_added` 或 `loop_committed` 事件。

正常地图重建只计入运行统计，不再逐次刷事件。精简模式下，观测到的新重建耗时
达到 `evaluation.slow_rebuild_ms` 时记录 `slow_map_rebuild`；默认阈值为 100 ms，
可按设备预算调整。统计来自最近一次重建快照，不能据此声称捕获了两帧之间的所有重建。
候选验证已写专用 CSV 时，不再重复写入事件表；禁用该 CSV 时仍保留验证事件。

`odom.csv` 在每个成功处理的 LiDAR 帧记录一次（包括建图初始化帧或未接受校正的帧），
时间戳为扫描结束时刻。状态和协方差取自该时刻的前端滤波结果，在继续传播扫描结束后的
IMU 样本之前保存；不是后端优化状态，也不是最新 IMU 时刻的预测状态。
`evaluation.enabled` 和 `evaluation.outputs.odometry` 仍可禁用该输出。

前 20 列顺序为 `timestamp,px,py,pz,qx,qy,qz,qw,vx,vy,vz,gx,gy,gz,bgx,bgy,bgz,bax,bay,baz`。
位置、速度、重力分别使用前端 W 坐标系（当前 ROS 输出中对应 odom 坐标系），单位为
m、m/s、m/s²；四元数表示 `R_WI`，即 IMU 到 W 的旋转；陀螺仪及加速度计偏置使用
IMU 坐标系，单位分别为 rad/s、m/s²。

接下来是 `cov_diag_0` 至 `cov_diag_17`（方差，不是标准差），不输出非对角项。
它们来自滤波器的误差状态协方差，索引顺序为：

| 索引 | 误差状态 | 分量单位 |
| --- | --- | --- |
| 0–2 | 右乘旋转扰动 xyz，`R_true = R_WI * Exp(δθ)`，局部 IMU 轴 | rad |
| 3–5 | W 系位置 xyz | m |
| 6–8 | W 系速度 xyz | m/s |
| 9–11 | IMU 系陀螺仪偏置 xyz | rad/s |
| 12–14 | IMU 系加速度计偏置 xyz | m/s² |
| 15–17 | W 系重力 xyz | m/s² |

对角项单位是对应误差分量单位的平方。它们表达滤波器内部的不确定度估计，
不能直接作为实际误差或真值精度，也不能用于恢复状态间的交叉相关性。
写入开销计入 `odometry_log_ms`（详细逐帧表）及输出总耗时。

排查细分耗时、可观性、缓存时，将配置改为：

```yaml
evaluation:
  detailed: true
```

详细模式恢复 126 列逐帧表、67 列运行表及逐次重建、候选验证事件；轨迹表保持 38 列。
两种模式都保留汇总统计、异常计数和完整回环验证表，算法行为不由该开关控制。
`manifest.json` 的 `schema_version: 4` 和 `detailed` 标明输出格式；读取 CSV 时按列名访问。
性能审计脚本对缺少的可选细分指标列出 `unavailable_metrics`，不将缺失值当作零。

## 结束扫描后自动后台 refine

联合系统的 `lio_bringup/config/lio_output.yaml` 默认设置
`save_pcd.refine_after_save: true`。它会同时启用完整局部 LiDAR 扫描与 IMU 归档。
通过 App 正常结束扫描（托管启动已有 `save_on_shutdown:=true`），或以退出保存模式
启动后正常退出，原始 PCD、位姿图、轨迹保存成功后，LIO 才启动独立后台任务。
后台任务等待原进程彻底退出、归档文件关闭，再开始读取。

仅点击 `/lio/save_pcd` 保存一个中途快照、但继续保持 LIO 运行，不会自动 refine；
应完成并结束本次扫描。强制杀进程、断电、保存失败不会被当作成功结束。
直接手动启动联合系统时，退出保存仍需显式启用：

```bash
ros2 launch lio_bringup lio_sensors.launch.py save_on_shutdown:=true
```

本轮新增链路不改变实时估计模型。后台入口为 `tools/refine_saved_run.py`，优化器为
`tools/refine_plane_map.py`，逐帧刚体重拼接为 `tools/plane_refinement_common.py`。
流程为完整输入检查、全视场扫描增广、平面重新提取、任意方向平面与位姿联合优化、
单次全局坐标旋转、点云完整性与未拟合扫描一致性检查。采用
`native / Gaussian / 0.0005 m/√100 ms` 配置，陀螺仪正则与此前一致。
不增加固定重力模长、平层或平行墙约束，不重新做逐点去畸变，不反馈实时前端。
初值和原生约束取本次实际保存的后端图；不会额外自动执行 `/lio/refine_revisits`，
所以不能宣称与此前手动重访细化后的最佳文件逐位或精度等价。

正常成果目录新增：

```text
<本次时间戳目录>/
  cloud_color.pcd / cloud_color_optimized.pcd  原来保存的成果，继续保留
  full_deskewed_points.bin                   全视场局部扫描
  full_deskewed_scans.csv                    扫描索引及位姿/外参
  refinement_imu.bin                        7 列 little-endian float64，时间已对齐 LiDAR
  refinement_archive.json                  归档进程与时间轴信息
  refinement_job.json                      当前后台任务状态
  refinement.log                           后台日志
  cloud_color_refined.pcd                   检查通过后原子发布的新增成果
  refined/<后处理时间戳>/                   本次输入准备、优化报告、检查及独立结果
```

`refinement_job.json` 记录 `waiting_for_exit`、`queued`、`preparing`、`extracting`、
`optimizing`、`exporting`、`complete` 或 `failed`，以及输出路径和错误原因。
只有优化收敛、源点及强度/RGB 保持、逐扫描刚体检查和未拟合扫描评估成功后，
才发布顶层 refined PCD。后台失败保留原始成果和诊断，不用原始 PCD 冒充 refined。
`complete` 代表处理链路成功，**不代表地面整图 2 cm 或真值精度通过验收**。
项目 API 会返回 `refinement` 状态；活跃任务期间禁止重命名/删除源工程，已有不可变
成果仍可下载。任务完成后新增 PCD 出现在项目文件列表，默认主成果选择不变。

任务降低 CPU 调度优先级，数值库限制为单线程，同一存储根目录通过文件锁串行求解。
归档增加磁盘空间与写入：例如 2 万点/帧、10 Hz、每点 16 字节时，额外扫描归档约
3.2 MB/s，10 分钟约 1.9 GB，另有 IMU 和后处理输入。离线计算仍占用内存和 CPU；
后台执行不等于没有资源成本。系统重启不会自动恢复被中断的求解。

需要关闭自动功能时，将联合系统配置的 `save_pcd.refine_after_save` 改为 `false` 并
重启；若还需要以后手动处理，启动时加 `refinement_archive:=true`。独立 `itera_lio`
启动默认不开自动后处理，可在其输出配置中自行启用上述开关及退出保存。

对已经完整结束、保留所需归档的目录，也可手动重新处理：

```bash
ros2 run itera_lio refine_saved_run.py /绝对路径/本次时间戳目录 --publish-latest
```

单独一份 PCD 或过去没有完整扫描/IMU 归档的普通目录，不能恢复相同的后处理输入；
入口会拒绝缺失、截断、传感器丢帧/断档、未处理完的输入及仍在运行的 LIO。
旧验证目录若有完整 `input/` 与 `imu_samples.npy`，入口也支持复算。

2026-11-01 已用 1080 的前 60 秒验证实际自动流程：600 帧 LiDAR、12002 条 IMU
全部处理，579 帧去畸变输出（一次初始化阶段失败），无容量丢弃或 IMU 间隙重置。
节点保存后自动启动后台任务，得到 509121 点 refined PCD；578 次彩色扫描的强度/RGB
逐位保留，最大单扫描刚体检查误差约 3.95 µm。测试目录为
`data/refine_workflow_validation_20261101_180105/replay_auto/20261101_180612_229/`。
这是链路与导出完整性验证，不是完整 1080 精度或地面验收。
