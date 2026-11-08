# interfaces

ROS 2 bridge from `livox_ros_driver2/msg/CustomMsg` to
`sensor_msgs/msg/PointCloud2` for Livox Mid-360 data.

The output point layout retains all fields from `CustomPoint`:

| Field | PointCloud2 type | Offset |
| --- | --- | ---: |
| `x`, `y`, `z` | `FLOAT32` | 0, 4, 8 |
| `reflectivity` | `UINT8` | 12 |
| `tag` | `UINT8` | 13 |
| `line` | `UINT8` | 14 |
| `offset_time` | `UINT32` | 16 |

`offset_time` remains the nanosecond offset relative to the Livox packet time.
The original message header (timestamp and frame ID) is copied unchanged unless
the optional `frame_id` parameter is set. The Livox IMU acceleration fields are
converted from g to m/s^2 before publication; set `imu_acceleration_scale` to
`1.0` when the input is already in SI units.

## Build and run

Install/source `livox_ros_driver2`, then run:

```bash
colcon build --packages-select interfaces
source install/setup.bash
ros2 launch interfaces livox_bridge.launch.py
```

Default topics:

- Input: `/livox/lidar` (`livox_ros_driver2/msg/CustomMsg`)
- Lidar output: `/interfaces/lidar` (`sensor_msgs/msg/PointCloud2`)
- IMU input: `/livox/imu` (`sensor_msgs/msg/Imu`)
- IMU output: `/interfaces/imu` (`sensor_msgs/msg/Imu`)

Override parameters from the command line when needed:

```bash
ros2 run interfaces livox_to_pointcloud2_node --ros-args \
  -p input_topic:=/livox/lidar \
  -p output_topic:=/interfaces/lidar \
  -p imu_input_topic:=/livox/imu \
  -p imu_output_topic:=/interfaces/imu
```
