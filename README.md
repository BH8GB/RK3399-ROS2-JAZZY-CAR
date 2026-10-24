# N10 麦克纳姆轮小车系统

本工作区是 N10 麦克纳姆轮小车的上位机工程，配套下位机固件位于：

`C:\Users\EricE\Downloads\ros2_car`

两端通过 UART8 连接，共同完成底盘运动、里程计、建图和导航。上位机运行 ROS 2，负责运动学解算、串口协议、里程计、TF、N10 雷达接入、SLAM 和 Nav2。下位机运行 STM32H743VIT6 固件，负责电机闭环、编码器采集、陀螺仪读取和底盘安全保护。

## 系统组成

| 部分 | 内容 |
| --- | --- |
| 下位机主控 | STM32H743VIT6，480MHz，FreeRTOS |
| 底盘 | 四轮麦克纳姆轮，直流电机加增量编码器 |
| 云台 | 二轴步进云台，A4988 驱动，MT6701 编码器 |
| 姿态 | 单轴陀螺仪，输出偏航角和角速度 |
| 上位机 | ROS 2 Jazzy，Python 3 |
| 雷达 | 镭神 N10 串口雷达 |
| 建图 | slam_toolbox |
| 导航 | Nav2，AMCL，DWB 局部规划器，NavFn 全局规划器 |

## 数据链路

控制链路：

```text
ROS 2 控制端
  /cmd_vel
    base_serial_node
      麦克纳姆轮逆运动学
        UART8 0x10 四轮目标转速
          STM32 速度环
            四个直流电机
```

反馈链路：

```text
四个编码器加单轴陀螺仪
  STM32 遥测帧 0x35，周期 20 毫秒
    base_serial_node
      /odom
      /imu/data_raw
      /wheel_rpm
      /base/status_flags
      TF odom 到 base_link
```

感知和导航链路：

```text
N10 雷达
  /scan
    slam_toolbox 或 AMCL
      map 到 odom 的 TF
        Nav2 路径规划和速度控制
          /cmd_vel
```

## 目录结构

```text
my_robot_ws/
  src/
    n10_slam_bringup/       自研上位机包
      n10_slam_bringup/
        base_protocol.py    串口帧编解码
        base_serial_node.py 底盘 ROS 2 节点
        mecanum.py          麦克纳姆轮运动学
        mecanum_keyboard_node.py 键盘遥控节点
      launch/               建图、导航、定位启动文件
      config/               底盘、SLAM、Nav2 参数
      rviz/                 RViz 配置
      udev/                 串口设备别名规则
      test/                 协议和运动学单元测试
    lslidar_driver/         厂商 N10 雷达驱动
    lslidar_msgs/           厂商雷达消息定义
```

固件工程结构、引脚表和云台协议见：

`C:\Users\EricE\Downloads\ros2_car\README.md`

## 硬件连接

### 串口

| 链路 | 下位机引脚 | 参数 | 用途 |
| --- | --- | --- | --- |
| UART8 | PE0 RX，PE1 TX | 115200 8N1 | ROS 2 底盘通信 |
| USART3 | PD9 RX，PD8 TX | 115200 8N1 | 陀螺仪 |
| USART1 | PA10 RX，PA9 TX | 115200 8N1 | 蓝牙手动控制 |
| UART5 | PC12 TX | 115200 | 调试波形输出 |

UART 电平为 3.3V。USB 转串口模块的 TX 接下位机 RX，RX 接下位机 TX，双方必须共地。

### 稳定设备名

Linux 下不要依赖会变化的 `ttyUSB0` 和 `ttyUSB1`。安装 udev 规则后使用固定别名：

- `/dev/n10_lidar`：N10 雷达，USB `1a86:7523`
- `/dev/robot_base`：STM32 UART8，USB `1a86:55d4`，序列号 `556B031183`

安装命令：

```bash
sudo cp src/n10_slam_bringup/udev/99-ros-car-serial.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

重新插拔设备后检查：

```bash
ls -l /dev/n10_lidar /dev/robot_base
```

### 雷达安装位置

`n10_slam.launch.py` 中的静态 TF 为：

- `base_link` 到 `laser`
- x 为 0.110 米
- y 为 0
- z 为 0.160 米
- roll、pitch、yaw 均为 0

实际安装位置变化后，必须同步修改启动文件中的静态 TF，否则建图和导航会出现偏移。

## 底盘参数

`src/n10_slam_bringup/config/base_serial.yaml` 是上位机底盘参数的主配置。

| 参数 | 当前值 | 说明 |
| --- | --- | --- |
| `wheel_diameter` | 0.075 | 车轮直径，单位米 |
| `half_length` | 0.080 | 轮心到车体中心的纵向距离，单位米 |
| `half_width` | 0.080 | 轮心到车体中心的横向距离，单位米 |
| `counts_per_revolution` | 60000 | 车轮转一圈的编码器计数 |
| `max_wheel_rpm` | 200 | 单轮最大目标转速 |
| `command_rate` | 20 | 速度命令发送频率，单位赫兹 |
| `cmd_vel_timeout` | 0.30 | 速度命令超时，单位秒 |
| `telemetry_timeout` | 0.50 | 遥测超时，单位秒 |
| `use_imu_yaw_for_odom` | true | 使用陀螺仪偏航增量修正里程计航向 |
| `motor_signs` | [1, 1, 1, 1] | 四轮命令方向修正 |
| `encoder_signs` | [1, 1, 1, 1] | 四轮编码器方向修正 |

注意：`base_serial_node.py` 中 `half_length` 和 `half_width` 的代码默认值是 0.040，实际启动时会加载 `base_serial.yaml` 中的 0.080。修改轮距参数时以 YAML 为准。

## UART8 协议

帧格式：

```text
帧头 0xAA
命令 1 字节
长度 1 字节
数据 N 字节
校验 1 字节
```

校验值为命令、长度和数据逐字节相加后的低 8 位。

| 命令 | 值 | 数据 | 说明 |
| --- | --- | --- | --- |
| 四轮转速 | 0x10 | 4 个 int16，小端，单位 RPM | 顺序为左前、右前、左后、右后，范围正负 250 |
| 停止 | 0x22 | 无 | 四轮停机并清空 PID |
| 遥测 | 0x35 | 36 字节 | 固件每 20 毫秒主动发送 |

遥测数据布局：

| 偏移 | 类型 | 内容 |
| --- | --- | --- |
| 0 | uint32 | 下位机时间戳，单位毫秒 |
| 4 | int32 x 4 | 四轮累计编码器计数 |
| 20 | int16 x 4 | 四轮实测转速，单位 RPM |
| 28 | int16 | 偏航角，放大 10 倍 |
| 30 | int16 | 偏航角速度，放大 10 倍 |
| 32 | uint16 | 状态标志 |
| 34 | uint16 | 堵转事件累计次数 |

状态标志：

| 位 | 值 | 含义 |
| --- | --- | --- |
| 0 | 0x0001 | 陀螺仪数据有效 |
| 1 | 0x0002 | 当前有运动命令 |
| 2 | 0x0004 | 300 毫秒命令超时，底盘已停机 |
| 3 | 0x0008 | UART8 接收错误已恢复 |
| 4 | 0x0010 | 堵转保护已触发，所有电机输出关闭 |

## 构建

### 下位机固件

使用 Keil MDK-ARM 打开：

`C:\Users\EricE\Downloads\ros2_car\MDK-ARM\H743VIT6.uvprojx`

编译后生成：

`C:\Users\EricE\Downloads\ros2_car\MDK-ARM\H743VIT6\H743VIT6.hex`

连接调试器后下载固件。工程使用 ARMCC V5.06，切换编译器前先确认 HAL 和 FreeRTOS 的兼容性。

### 上位机 ROS 2

以下命令按 ROS 2 Jazzy 编写，其他发行版请替换对应的 setup 路径。

```bash
cd /root/my_robot_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

主要依赖：

- `navigation2`
- `nav2_bringup`
- `slam_toolbox`
- `python3-serial`
- `lslidar_driver`
- `lslidar_msgs`

## 运行

### 建图

```bash
ros2 launch n10_slam_bringup n10_slam.launch.py
```

启动内容：

- N10 雷达驱动
- 底盘串口节点
- `base_link` 到 `laser` 的静态 TF
- slam_toolbox
- RViz2

常用参数：

```bash
ros2 launch n10_slam_bringup n10_slam.launch.py \
  use_rviz:=true \
  use_base:=true \
  base_port:=/dev/robot_base \
  lidar_port:=/dev/n10_lidar \
  resolution:=0.05
```

### 建图同时运行 Nav2

```bash
ros2 launch n10_slam_bringup n10_navigation.launch.py
```

这个启动文件同时运行建图和 Nav2，适合在未知环境中边建图边测试导航。正式使用已保存地图时，应使用定位启动文件。

### 已保存地图的定位和导航

```bash
ros2 launch n10_slam_bringup n10_localization_navigation.launch.py \
  map:=/root/my_robot_ws/maps/n10_map.yaml
```

启动内容：

- N10 雷达驱动
- 底盘串口节点
- 静态 TF
- AMCL 定位
- Nav2 导航
- RViz2

默认地图路径是 `/root/my_robot_ws/maps/n10_map.yaml`，文件不存在时需要先修改参数或先保存地图。

### 键盘遥控

```bash
ros2 run n10_slam_bringup mecanum_keyboard \
  --ros-args -p speed:=0.10 -p turn:=0.35
```

按键：

- `i` 或 `,`：前进或后退
- `j` 或 `l`：左转或右转
- 大写 `J` 或 `L`：左移或右移
- `k` 或空格：停止
- `q` 或 `z`：整体加速或减速
- `w` 或 `x`：调整线速度
- `e` 或 `c`：调整角速度
- `Ctrl-C`：停止并退出

键盘节点以 20 赫兹重复发布命令，避免底盘 300 毫秒命令超时。

### 蓝牙手动控制

蓝牙模块接 STM32 USART1，使用 115200 8N1，发送以 CRLF 结尾的 ASCII 命令。常用命令：

```text
M 30 30 30 30
M -30 -30 -30 -30
M 0 0 0 0
STOP
PING
STATUS
HELP
```

四个数值的顺序固定为左前、右前、左后、右后，单位为目标转速 RPM。蓝牙手动通道不受 UART8 的 300 毫秒超时保护约束，测试时仍需保持底盘周围安全。

## 标定

首次上车前抬起底盘，发送较小的正向前进速度，确认四个车轮都向前转，并且 `/wheel_rpm` 的四个值都为正。

### 电机方向

如果某个车轮反转，修改 `base_serial.yaml` 中对应的 `motor_signs`。

### 编码器方向

如果某个车轮前进时实测转速为负，修改对应的 `encoder_signs`。如果固件端编码器方向本身接反，还需要同步修改 STM32 `encoder.h` 中的方向常量，然后重新编译和烧录。

### 轮径和编码器计数

确认车轮转一圈的编码器计数约为 60000。如果不同，需要同时修改：

- `base_serial.yaml` 中的 `counts_per_revolution`
- STM32 固件中的编码器计数常量

### 轮距

`half_length` 和 `half_width` 都表示轮心到车体中心的距离，不是车体外壳尺寸。当前值为 0.080 米。Nav2 的 `footprint` 使用车体外轮廓，当前为 0.25 米乘 0.23 米，两者含义不同，不要混用。

### 陀螺仪航向

`use_imu_yaw_for_odom` 为 true 时，里程计航向使用陀螺仪偏航增量，轮式里程计主要提供平移。陀螺仪方向或零漂异常时，先检查 `/imu/data_raw` 和固件串口数据，再调整固件中的航向符号。

## 调试

### 检查话题

```bash
ros2 topic echo /wheel_rpm
ros2 topic echo /odom
ros2 topic echo /imu/data_raw
ros2 topic echo /base/status_flags
ros2 topic echo /scan
```

### 检查 TF

```bash
ros2 run tf2_tools view_frames
```

正常 TF 链：

```text
map 到 odom
odom 到 base_link
base_link 到 laser
```

### 检查串口

```bash
ls -l /dev/robot_base /dev/n10_lidar
sudo usermod -aG dialout $USER
```

修改用户组后需要重新登录。

### 常见问题

- 串口打不开：检查 udev 规则、设备别名和 `dialout` 权限。
- 有串口但没有遥测：检查波特率、TX/RX 交叉和 STM32 是否在发送 0x35 帧。
- 底盘不动：检查 `/cmd_vel` 是否有数据，`/base/status_flags` 是否触发超时或堵转保护。
- 里程计漂移：检查轮径、轮距、编码器计数和陀螺仪方向。
- 雷达数据不动：检查 `/dev/n10_lidar` 和 `lsn10.yaml` 中的串口配置。
- 建图漂移：检查静态 TF、雷达安装位置和轮式里程计方向。

## 已知限制

- `lslidar_driver` 和 `lslidar_msgs` 是厂商代码，本项目只使用其接口，没有修改内部实现。
- 蓝牙手动控制直接连接 STM32 USART1，不经过 ROS 2，也不受 UART8 的 300 毫秒超时保护。
- 底盘只有单轴陀螺仪，`/imu/data_raw` 只提供偏航角和偏航角速度，不提供完整三轴姿态。
- 云台由 STM32 的 USART1 手动协议控制，当前 ROS 2 上位机只控制底盘，不控制云台。
- 建图过程中不要抬起机器人。轮式里程计不会记录抬起动作，扫描匹配可能因此漂移。
- `n10_localization_navigation.launch.py` 的默认地图路径需要提前准备，否则地图服务器会启动失败。

## 相关文档

- 上位机包说明：`src/n10_slam_bringup/README.md`
- 下位机固件说明：`C:\Users\EricE\Downloads\ros2_car\README.md`