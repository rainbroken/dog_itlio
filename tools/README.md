# 导航离线诊断

`record.sh` 录制运行现场；`analyze_navigation_bag.py` 绘制实际录制的速度、任务阶段和轨迹。
参数导出失败时，分析结果会标明偏移和跟踪阈值的回退来源，不能当成完整的运行参数快照。

`benchmark_navigation_motion.cpp` 用真实保存地图、指定障碍快照、当前规划与控制代码做闭环模型回归，
不会发布 ROS 指令。先按工作空间约定完整编译，再编译诊断工具：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
c++ -std=c++17 -O2 tools/benchmark_navigation_motion.cpp \
  -Isrc/dog_navigation/include -Iinstall/dog_terrain/include \
  -I/usr/include/eigen3 -I/usr/include/pcl-1.12 \
  -Linstall/dog_terrain/lib -Linstall/dog_navigation/lib \
  -Wl,-rpath,"$PWD/install/dog_terrain/lib" \
  -Wl,-rpath,"$PWD/install/dog_navigation/lib" \
  -lnavigation_execution -llocal_planner -ldog_terrain \
  -o /tmp/dog-motion-benchmark
/tmp/dog-motion-benchmark data/maps floor_support_r010_v5 \
  rosbag/navigation_20261108_162743_E6TXXK/analysis/motion_robustness_cases.json \
  > /tmp/dog-motion-results.json
```

输入为 JSON 数组，每项含 `name`、三维接地点 `start` / `goal`、`initial_yaw`（弧度）及
地图坐标下的 `obstacles`；可指定 `response_lag`、`gait_phase`、`lookahead_distance`、
`continuous_turn_angle`。未给航向时假定 0，不能据此重建原任务的初始朝向。

模型采用 20 Hz 控制、一阶速度响应及小幅步态横摆扰动，完整检查矩形碰撞体、动态障碍、
控制器预测位置及期望/反馈速度的刹停包络。地面高度取已认证路线插值，忽略机身俯仰、
足端接触和脚滑；不模拟 DDS 迟延、路线租约缺口或真实执行器性能。
地图障碍快照保持静止，因而不能验证移动行人避障。

输出含 XY/yaw 指令、合成反馈、跟踪误差及计算耗时。
`smooth` 要求模型到达目标、行走开始后途中零线速累计不超过 0.25 秒、
移动期间 XY 指令加速度不超限，且无原始指令认证不一致或 100 ms 控制计算超时。
初始原地对齐、最终到达停稳/朝向调整单独计时；
起停跨越 0.21 m/s 死区和安全急停不计为正常加速度平滑。
所有结果均标记 `physical_robot_verified: false`，不能代替新录制的实机验证。

`benchmark_dynamic_obstacles.cpp` 使用相同的头文件、库路径及 `-llocal_planner -ldog_terrain`
编译，参数为 `ROOT ID sx sy sz gx gy gz`。它在缓存路线的内部节点依次新增 0.30 m
宽的箱体，连续规划三次并移除障碍，输出耗时和失败原因；不发布 ROS，不模拟机器人运动。
失败时另做限时三秒的完整转向搜索作诊断，超时不能视为已经证明无路。
节点级测试还覆盖实时点云新增障碍绕行、封路停车、正向清障后不重发目标自动恢复。

已有 `motion_final_results.json` 和原始分析 CSV 时，可生成标明实测/合成反馈的对比图：

```bash
python3 tools/plot_navigation_motion.py rosbag/navigation_20261108_162743_E6TXXK/analysis
```
