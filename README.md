# Ranger Mini 3.0 建图与导航子系统

本项目是 Jetson 上的 ROS 1 导航工作区，供主线 VLM 项目作为底层导航能力调用。
当前阶段按“底盘驱动、建图、保存地图、人工初始化定位、目标点导航”的开发范围交接；
不包含 VLM 决策、语义目标解析或自动重定位管理。

## 1. 开发状态与边界

| 能力 | 当前情况 |
| --- | --- |
| Ranger Mini 3.0 底盘 | 已接入 CAN、速度控制、里程计及 TF |
| Helios 点云转二维扫描 | 已接入高度过滤、投影及车体范围剔除 |
| gmapping 建图、地图保存 | 已完成流程并在本机使用 |
| AMCL + move_base 导航 | 已接入；用户已验证 V3 是正确车型，进入主线集成阶段 |
| 无界面启动 | 已支持 `rviz:=false` |
| VLM 调用封装 | 尚未实现，由主线集成 |
| 开机自动初始化、定位质量门禁、丢失后重定位 | 尚未实现 |

“阶段交接”不表示所有环境下完成可靠性或安全验收。弧线转向时曾出现扫描与地图
不一致，先前误用 Mini V1 是重要问题；切换 V3 后改善，但主线仍应对实际路线、
转弯、目标取消和定位失效进行回归验证。最新点云高度范围也需要现场验证。

## 2. 环境与目录

- Jetson，Ubuntu 18.04 ARM64，ROS Melodic，ROS 工具使用系统 Python 2。
- 实车为 **Ranger Mini 3.0**，必须使用 `ranger_mini_v3`。
- CAN：`can0`，500000 bit/s。
- 雷达：RoboSense Helios 16P，驱动在独立工作区 `~/rslidar_ws`，不在本仓库中。
- 地图默认路径：`~/maps/ranger_map.yaml`，对应 PGM 文件也不在本仓库中。
- 网络和 ROS 环境需预先可用；本仓库不部署路由、网卡或 mDNS 配置。

| 目录 | 职责 |
| --- | --- |
| `src/ranger_ros` | 底盘 ROS 节点、消息、启动入口 |
| `src/ugv_sdk` | 底层 CAN 编解码和车型适配 |
| `src/ranger_mapping` | 扫描转换、车体过滤、外参、gmapping、建图 RViz |
| `src/ranger_navigation` | 地图服务、AMCL、规划器、代价地图、导航 RViz |
| `BUILD.md` | 克隆后的构建说明 |

依赖源码直接纳入仓库，不需要初始化 Git 子模块。构建产物不提交。
V1/V2 兼容实现仍保留，不代表本车使用它们；早期 V1 特殊换算和符号修改不作用于 V3。
通用 `ranger.launch` 和节点缺省车型也已改为 V3。

## 3. 启动顺序

分别在对应终端运行。不要通过主线反复创建第二套底盘、雷达或导航节点。

### 3.1 环境和底盘

```bash
source /opt/ros/melodic/setup.bash
source ~/ranger_ws/devel/setup.bash --extend
ip -details link show can0
```

仅在每次上电后 `can0` 尚未以 500000 bit/s 启用时执行一次：

```bash
rosrun ranger_bringup bringup_can2usb.bash
```

关闭旧底盘驱动及其他速度发布者，再启动：

```bash
roslaunch ranger_bringup ranger_mini_v3.launch publish_odom_tf:=true
```

确认运行车型：

```bash
rosparam get /ranger_base_node/robot_model
```

应为 `ranger_mini_v3`。底盘保留 50 Hz 命令刷新和 0.6 秒指令超时保护，
但这不是独立急停系统，也不能代替主线的目标取消逻辑。

### 3.2 雷达

```bash
source /opt/ros/melodic/setup.bash
source ~/rslidar_ws/devel/setup.bash --extend
roslaunch rslidar_sdk start.launch rviz:=false
```

底盘、雷达和导航跨终端运行时必须连接同一个 ROS master；必要时叠加 source
所需工作区。ROS_IP/ROS_HOSTNAME 必须是本机有效地址，不能使用已失效的 DHCP 地址。

### 3.3 已有地图导航

停止 gmapping 和键盘/遥控速度发布者。先做不向底盘输出导航速度的预览：

```bash
source /opt/ros/melodic/setup.bash
source ~/ranger_ws/devel/setup.bash --extend
roslaunch ranger_navigation navigation.launch enable_motion:=false
```

在 RViz 用 **2D Pose Estimate** 设置初始位置和朝向，检查激光与地图对齐。
预览输出到 `/navigation/cmd_vel`，不代表其他控制源被禁止。

实车导航：停止预览，在空旷区域、急停可用且无其他控制源的前提下启动：

```bash
roslaunch ranger_navigation navigation.launch enable_motion:=true
```

重启导航后再次初始化位姿。可用 **2D Nav Goal** 手动验证。

主线无界面部署入口：

```bash
roslaunch ranger_navigation navigation.launch \
  enable_motion:=true rviz:=false \
  map_file:=/home/jetson/maps/ranger_map.yaml
```

**无界面启动不会自动完成定位。** 主线需提供已知初始位姿并验证定位后才发目标。
`start_scan:=false` 仅用于扫描转换、过滤及雷达静态 TF 已由别处启动的情况，
否则保持默认值；不能重复发布同一 TF。

## 4. VLM 主线集成契约

### 4.1 职责划分

VLM 负责决策“去哪里”，主线将语义目的地解析为当前地图中的 `(x, y, yaw)`。
本子系统负责定位、路径规划、避障和底盘执行。当前没有语义地点库、HTTP API、
ROS 2 接口或已经实现的 VLM 导航桥接节点。

建议主线持有一个长期运行的 ROS 1 Action 客户端，而不是每次执行 shell 启停导航。
非 ROS/Python 3 的 VLM 进程可通过独立 ROS 桥接进程调用，桥接层的传输协议尚待主线设计。

### 4.2 ROS 接口

| 接口 | 类型 | 用途 |
| --- | --- | --- |
| `/move_base` | `move_base_msgs/MoveBaseAction` | 推荐的导航目标、反馈、结果、取消入口 |
| `/move_base_simple/goal` | `geometry_msgs/PoseStamped` | RViz/临时目标；不推荐作为主线任务管理接口 |
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | 初始化 AMCL 位姿及协方差，frame 为 `map` |
| `/amcl_pose` | `geometry_msgs/PoseWithCovarianceStamped` | 地图坐标系下的定位结果 |
| `/ranger_base_node/odom` | `nav_msgs/Odometry` | 里程计 |
| `/helios/scan_filtered` | `sensor_msgs/LaserScan` | 建图、定位、避障共用二维扫描 |
| `/rslidar_points` | `sensor_msgs/PointCloud2` | 外部雷达驱动提供的原始点云 |
| `/cmd_vel` | `geometry_msgs/Twist` | 底盘速度；主线导航任务不应直接写入 |

目标单位是米、弧度；`target_pose.header.frame_id = "map"`，时间戳使用当前 ROS 时间。
平面朝向四元数为 `(0, 0, sin(yaw/2), cos(yaw/2))`，不要使用全零四元数。
`map` 原点来自所选地图，不是车辆每次开机的原点，也不是 VLM 相机坐标。

### 4.3 任务生命周期

主线适配器应至少实现：

1. 等待 Action 服务、地图、扫描和完整 TF 可用。
2. 初始化或确认定位；当前没有自动输出“定位可信”的布尔接口。
3. 为上层任务维护独立任务 ID，并关联 Action goal。
4. 发目标并监听反馈、终态，不能把“已发出”或 `Got new plan` 当成成功。
5. 区分 `SUCCEEDED`、`ABORTED`、`PREEMPTED`、`REJECTED`、`LOST`，向 VLM 返回明确结果。
6. 在取消、上层任务超时或主线退出时取消对应 goal；不得假设客户端退出会自动停车。
7. 新任务替换旧任务时明确执行抢占策略，不让多个客户端争夺控制权。

主线可以自行设置任务超时，但本项目尚无统一的任务超时 API。取消后还应检查机器人
实际停止；发生异常使用物理急停，不做无上限自动重试。

运维用“取消全部目标”命令（主线应优先取消自己的 goal）：

```bash
rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID '{}'
```

**重要：底盘指令超时不等于导航任务超时。** 主线失联但 move_base 仍在运行时，
move_base 可能继续为当前目标发布速度；0.6 秒底盘保护不会因此自动触发。
如需要失联停车，主线需增加看门狗或速度仲裁/门禁，目前尚未实现。

### 4.4 TF 所有权

```text
map --AMCL--> odom --V3 底盘--> base_link --静态外参--> rslidar
```

建图时 `map -> odom` 改由 gmapping 发布，因此不能同时运行 AMCL 和 gmapping。
主线若加入机器人模型发布器，也不能重复发布 `base_link -> rslidar`。

## 5. 当前参数与安全限制

| 项目 | 当前值 |
| --- | --- |
| 机器人外形 | 长 0.72 m，宽 0.50 m，`base_link` 为车体中心 |
| 雷达外参 | 平移 `(0.2767, 0, 0.661)` m，绕 z 轴旋转 180° |
| 点云高度截取 | 相对 `rslidar` 原点 `[-0.7285, +0.20]` m |
| 扫描距离范围 | 0.5 到 20 m |
| 自身点剔除框 | `base_link` 下 x 为 ±0.455 m、y 为 ±0.345 m |
| 代价地图 footprint | x 为 ±0.36 m、y 为 ±0.25 m，padding 0.03 m |
| 膨胀半径 | 0.10 m |
| 全局/局部规划器 | GlobalPlanner / DWAPlannerROS |
| 导航最大线速度/角速度 | 0.20 m/s / 0.25 rad/s |
| 到点容差 | 位置 0.15 m，朝向 0.15 rad |
| 重规划/控制频率 | 1 Hz / 5 Hz |
| 振荡检测 | 60 秒，平移距离阈值 0.15 m |
| 自动恢复动作 | 禁用；不自动执行旋转/倒车恢复 |

当前导航不采样倒车或横移。DWA 的运动假设与底盘阿克曼/自转模式切换之间仍需现场验证。
自身点剔除框不等于碰撞 footprint，可能同时剔除紧贴车体的真实障碍。
高度截取不是地面分割，扩大截取范围可能引入地面点，也可能与旧地图的观测不一致。
二维扫描无法保证检测落差、所有低矮障碍或盲区；本项目不提供安全认证避障能力。

## 6. 建图、保存与测试

保留底盘和雷达，停止导航：

```bash
roslaunch ranger_mapping gmapping.launch
mkdir -p ~/maps
rosrun map_server map_saver -f ~/maps/ranger_map
```

保存命令在另一个已 source 环境的终端运行。部署时一起携带 YAML 和 PGM；
主线应记录地图版本，避免旧坐标目标误用于新地图。

离线配置测试，不启动底盘：

```bash
python2 ~/ranger_ws/src/ranger_ros/ranger_base/tests/v3_launch_test.py
```

隔离 ROS master 下的导航启动测试，不连接真实底盘：

```bash
python2 ~/ranger_ws/src/ranger_navigation/tests/navigation_smoke.py
```

这些测试验证启动参数、模拟数据下的地图/TF/规划服务和预览输出隔离，不等于实车验收。
主线接入至少应覆盖：人工初始定位、直线/转弯到点、最终朝向、目标抢占、取消、
超时、障碍阻挡、扫描丢失、TF 中断、主线失联和错误地图。

## 7. 后续工作

已讨论但未实现：固定停车区域附近的初始位姿自动加载、带不确定度的 AMCL 初始化、
定位质量判断及不可信时禁止导航。全局重定位、绑架恢复、失联停车和速度仲裁也不在
本次交接实现范围内。主线不能将“节点启动成功”视为“已定位且允许运动”。
