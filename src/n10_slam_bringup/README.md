# n10_slam_bringup

N10 麦克纳姆轮小车的 ROS 2 上位机包。包内提供底盘串口节点、麦克纳姆轮运动学、键盘遥控节点，以及建图、导航和定位启动文件。

完整的系统说明、硬件说明、固件端说明和两端协议见：

`F:\ros2\README.md`

## 功能

- 通过 UART8 向 STM32 下发四轮目标转速。
- 接收 STM32 遥测，发布里程计、IMU、轮速和状态标志。
- 发布 `odom` 到 `base_link` 的 TF。
- 接入 N10 串口雷达和 slam_toolbox。
- 提供 Nav2 建图、导航和已保存地图定位启动文件。
- 提供按固定频率重复发布命令的麦克纳姆轮键盘遥控节点。

## 节点和话题

### base_serial_node

可执行文件：

```bash
ros2 run n10_slam_bringup base_serial_node
```

订阅：

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/Twist` | 底盘目标速度 |

发布：

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/odom` | `nav_msgs/Odometry` | 轮式里程计和陀螺仪航向 |
| `/imu/data_raw` | `sensor_msgs/Imu` | 偏航角和偏航角速度 |
| `/wheel_rpm` | `std_msgs/Float32MultiArray` | 四轮实测转速 |
| `/base/status_flags` | `std_msgs/UInt16` | STM32 状态位 |

TF：

- `odom` 到 `base_link`
- 固定频率发布，停车时也不会停止。

### mecanum_keyboard

可执行文件：

```bash
ros2 run n10_slam_bringup mecanum_keyboard
```

发布：

- 发布 `/cmd_vel`
- 以 20 赫兹重复发送当前命令
- 退出时发送零速度

## 文件说明

| 文件 | 作用 |
| --- | --- |
| `base_protocol.py` | 串口帧编码、解析和遥测解码 |
| `base_serial_node.py` | 底盘 ROS 2 节点 |
| `mecanum.py` | 麦克纳姆轮正逆运动学 |
| `mecanum_keyboard_node.py` | 键盘遥控节点 |
| `launch/n10_slam.launch.py` | 雷达、底盘、TF、slam_toolbox、RViz |
| `launch/n10_navigation.launch.py` | 建图加 Nav2 |
| `launch/n10_localization_navigation.launch.py` | 已保存地图加 AMCL 和 Nav2 |
| `config/base_serial.yaml` | 底盘几何、编码器、方向和超时参数 |
| `config/slam_toolbox.yaml` | slam_toolbox 参数 |
| `config/nav2_params.yaml` | Nav2、AMCL、代价地图和速度限制 |
| `udev/99-ros-car-serial.rules` | 雷达和底盘的固定串口别名 |
| `test/test_protocol_and_kinematics.py` | 协议和运动学单元测试 |

## 底盘参数

主要参数位于 `config/base_serial.yaml`。

| 参数 | 当前值 | 说明 |
| --- | --- | --- |
| `port` | `/dev/robot_base` | 底盘串口别名 |
| `baud_rate` | 115200 | 串口波特率 |
| `wheel_diameter` | 0.075 | 车轮直径，单位米 |
| `half_length` | 0.080 | 轮心到车体中心的纵向距离 |
| `half_width` | 0.080 | 轮心到车体中心的横向距离 |
| `counts_per_revolution` | 60000 | 车轮转一圈的编码器计数 |
| `max_wheel_rpm` | 200 | 单轮最大目标转速 |
| `command_rate` | 20 | 命令发送频率，单位赫兹 |
| `cmd_vel_timeout` | 0.30 | 速度命令超时，单位秒 |
| `telemetry_timeout` | 0.50 | 遥测超时，单位秒 |
| `use_imu_yaw_for_odom` | true | 使用陀螺仪航向增量 |
| `motor_signs` | [1, 1, 1, 1] | 四轮命令方向 |
| `encoder_signs` | [1, 1, 1, 1] | 四轮反馈方向 |

代码中的 `half_length` 和 `half_width` 默认值为 0.040，实际启动会加载 YAML 中的 0.080。修改轮距时以 YAML 为准。

## 启动

建图：

```bash
ros2 launch n10_slam_bringup n10_slam.launch.py
```

建图加 Nav2：

```bash
ros2 launch n10_slam_bringup n10_navigation.launch.py
```

已保存地图定位和导航：

```bash
ros2 launch n10_slam_bringup n10_localization_navigation.launch.py \
  map:=/root/my_robot_ws/maps/n10_map.yaml
```

键盘遥控：

```bash
ros2 run n10_slam_bringup mecanum_keyboard \
  --ros-args -p speed:=0.10 -p turn:=0.35
```

## 串口协议

UART8 帧格式：

```text
0xAA
命令 1 字节
长度 1 字节
数据 N 字节
校验 1 字节
```

校验值为命令、长度和数据逐字节相加后的低 8 位。

| 命令 | 值 | 数据 | 说明 |
| --- | --- | --- | --- |
| 四轮转速 | 0x10 | 4 个 int16，小端，单位 RPM | 顺序为左前、右前、左后、右后 |
| 停止 | 0x22 | 无 | 四轮停机 |
| 遥测 | 0x35 | 36 字节 | 固件每 20 毫秒发送 |

遥测字段和状态位见根目录 README。

## 构建和测试

在工作区根目录执行：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select n10_slam_bringup
source install/setup.bash
```

运行单元测试：

```bash
cd src/n10_slam_bringup
python3 -m unittest discover -s test -v
```

测试覆盖：

- 四轮转速帧编码
- 噪声和错误校验后的帧同步
- 遥测字段布局
- 麦克纳姆轮正逆运动学
- 轮速限幅比例

## 标定顺序

1. 抬起底盘，确认四个车轮都能按命令转动。
2. 发送小的正向前进速度，确认四个车轮方向一致。
3. 检查 `/wheel_rpm`，前进时四个值都应为正。
4. 修正 `motor_signs` 和 `encoder_signs`。
5. 确认车轮转一圈约为 60000 个编码器计数。
6. 测量轮心到车体中心的 X/Y 距离，更新 `half_length` 和 `half_width`。
7. 测量雷达安装位置，更新启动文件中的静态 TF。
8. 检查 `/odom`、`/imu/data_raw` 和 TF 后再开始建图。

## 注意

- 修改 `config/*.yaml` 后需要重新启动 launch 文件。
- 修改 Python 源码后，使用 `--symlink-install` 构建时通常不需要重新编译，但要重新启动节点。
- 雷达和底盘的串口别名依赖 udev 规则，换 USB 口后仍应保持固定名称。
- 蓝牙手动控制直接连接 STM32，不经过本包。
- 云台控制不在本包范围内。
