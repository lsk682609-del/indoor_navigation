# 参考 snn 室内实车方案接入松灵小车

2026-11-03。本机源码与交接材料分析。用户已授权实施，接入代码与独立实车入口已新增，
实际配置、构建与验证情况见 [实车运行说明](../real_robot/README.md)。下文保留原分析依据；
定位器仍需接入原实车工作空间，尚未启动真实底盘测试。
本文件取代此前以旧 nav、out_car 为主参考的迁移结论。out_car 保留为室外实车接口参考。

## 结论

采用 snn 中的 Scout Mini 驱动和室内激光地图定位方案，保留 indoor_navigation
当前路网规划、Theta*、RPP、窄门控制及其优化。不要用 snn 的旧导航源码整套覆盖当前工程。
交接材料明确记录第三阶段“3D SLAM 建图与定位（已完成）”，包含室内定位演示及
IMU + ESKF + NDT 地图匹配的说明；第四阶段 RTK 融合另列为进行中。
因此，室内接入应围绕已有地图定位开展，不把室外 RTK 链路作为室内运行前提。

## 已核实的参考入口

- [交接材料：室内建图定位](/home/lxh/snn/项目工作交接/技术分享/技术分享材料.md:132)。
- [Scout Mini 启动脚本](/home/lxh/snn/项目工作交接/nav/scripts/start_scout_mini.sh)：
  调用 scout_base/scout_mini_base.launch.py，历史 CAN 接口为 can_usb。
- [Scout Mini launch](/home/lxh/snn/项目工作交接/nav/src/driver/scout_ros2/scout_base/launch/scout_mini_base.launch.py)：
  默认 can0、非全向 Scout Mini、实车模式，odom/base_link 坐标系。
- [底盘通信实现](/home/lxh/snn/项目工作交接/nav/src/driver/scout_ros2/scout_base/include/scout_base/scout_messenger.hpp:47)：
  订阅绝对话题 /cmd_vel，类型 geometry_msgs/msg/Twist；发布轮式 Odometry，twist 来自底盘速度反馈。
- [定位启动脚本](/home/lxh/snn/项目工作交接/nav/scripts/launch_slam.sh)：
  调用 laser_slam_3d/localization.launch.py。
- [定位配置](/home/lxh/snn/项目工作交接/nav/src/robot_launch/config/slam.yaml)：
  Livox CustomMsg、IMU、地图路径、传感器外参、inc_ndt 与重定位相关设置。

扫描 snn 下的 package.xml，未找到 laser_slam_3d 包。现有目录包含定位脚本、配置与
方案说明，但未包含该定位器的完整可构建工程。需要接入原室内实车定位工作空间；
不能从文档推断其实际发布的 Odometry 话题、位姿参考点及 TF，须在该工作空间核实。
旧脚本 /home/nhy/ww/... 路径也需改为部署机器上的实际路径。

## 当前工程需要修改什么

| 部分 | snn 提供的依据 | 当前工程的适配工作 |
| --- | --- | --- |
| 底盘控制 | Scout 驱动直接接收 Twist | 当前 /cmd_vel 类型已匹配；接入 scout_base、scout_msgs、ugv_sdk，并配置实际 CAN 接口 |
| 定位 | 已有点云地图上的激光/IMU 定位 | 接入原定位工作空间，确认 map 坐标、初始化与定位质量输出 |
| 速度反馈 | Scout Odometry 包含车体速度 | 单独接给控制器；不要将轮式积分位姿直接当成地图定位 |
| 点云 | MID360 launch 使用 CustomMsg | 保持定位器所需消息，另转换为导航使用的 PointCloud2，核对帧和时间戳 |
| TF | Scout 可发布 odom → base_link | 为每条变换指定唯一发布者，关闭 bridge 的重复动态 TF |
| 车体几何 | 旧 nx 配置使用 0.60 × 0.50 m footprint | 按实车完整轮廓、参考点与雷达安装外参重设；旧配置不是实测尺寸证明 |
| 运行环境 | ROS 2 Humble 实车节点 | 单独实车入口、use_sim_time=false、统一 domain/命名空间、真实地图资源 |

### 1. 定位输入和速度反馈分开

当前 [navigation.launch.py](/home/lxh/indoor_navigation/src/indoor_bringup/launch/navigation.launch.py)
用同一个 odom_topic 同时覆盖 bridge 的定位输入和控制器的速度输入。
实车应拆成 localization_odom_topic 和 velocity_odom_topic 两个参数（待新增）。
Scout 的 odom_topic_name 可以设成 wheel_odom，供控制器读取车体速度；
定位话题使用定位器实际输出，且必须与路网处于同一地图坐标系。
避免底盘和定位器同时发布默认 /odom 而混入两种数据。

当前 bridge 的 onOdom 会重新标注 map/base_link，并以位置差分覆盖 twist，
这不能完成 odom 到 map 的坐标变换，也不应覆盖已有的可靠车体速度。
应使用真实 TF 变换与明确的位姿参考点；仅在定位器输出传感器位姿时补偿安装外参，
已经是 base_link 的位姿不再重复扣除 body_offset。

### 2. TF 选择一种一致结构

若定位器支持发布 map → odom，可采用：

```text
map --定位器--> odom --Scout 驱动--> base_link --实测外参--> livox_frame
```

若原定位器直接发布 map → base_link，则保留这条定位 TF，
关闭 Scout 的 odom → base_link TF，仍保留轮式 Odometry 消息作为速度反馈。
Scout C++ 支持 publish_tf 参数，但现有 scout_mini_base.launch.py 未暴露并传递它，
需要补充 launch 参数或在实车入口直接创建带此参数的 Node。
两种结构都需要处理当前 bridge 无条件发布 map → base_link 的行为。
当前 publish_base_tf 开关只控制 base_link → base_footprint 静态变换，不能关闭上述动态 TF。

### 3. 保留定位所需点云格式，给导航提供 PointCloud2

[MID360 launch](/home/lxh/snn/项目工作交接/nav/src/driver/livox_ros_driver2/launch/msg_MID360_launch.py)
设置 xfer_format=1、frame_id=livox_frame；slam.yaml 也选择 Livox CustomMsg。
当前导航 bridge 接收 PointCloud2，不能仅改话题名连接 CustomMsg。
宜保留定位输入，在旁路转换出传感器坐标系下的 PointCloud2，供障碍物与窄门检测共用。
不要直接把世界坐标系中已配准的点云改名为 lidar 帧。

当前工程的 simulation/livox_ros_driver2 是消息包占位实现；实车要选择完整驱动，
构建时避免与 snn 中同名完整驱动重复或遮蔽。SDK、雷达网络地址及实际安装外参按实车设置。
仿真曾修正的射线起点偏移不应应用到真实雷达数据。

### 4. 几何、避障和地图

保留期望速度 1.0 m/s、膨胀半径 0.5 m 以及当前窄门与碰撞检测优化。
1.0 m/s 是期望上限，转弯、接近障碍、窄门及终点仍由控制器降速。
不要复制 snn 旧配置中的 inflation_radius=1.0、lethal_cost=200、body_offset_x=-0.40、
require_nav_sat_fix=true 等旧车/旧定位参数。

按实车重新设置 footprint、padding、门宽范围、障碍物高度与雷达外参；
RPP 中仍有约 0.23 m 后悬和 0.275 m 前悬的固定值，需与实车 footprint 统一。
点云地图、栅格背景和路网使用同一原点、方向、米制尺度与初始位姿；
不再使用仿真的环形路网或 SDF 背景代表真实环境。

### 5. 单独实车启动与验收

新增实车入口与配置，组合 Scout、完整雷达驱动、原室内定位器、当前导航核心及真实地图。
所有节点关闭仿真时间；按部署方式统一 ROS_DOMAIN_ID 和 TF 话题。
当前 env.sh 会清除已有 overlay 且设置 ROS_LOCALHOST_ONLY=1，
实车需单独组织依赖环境；跨机器运行时再调整 localhost 限制。

先验证静止定位、消息类型与 TF 唯一性，再验证底盘速度反馈与导航输出的一致性，
随后完成空走廊、转弯、障碍绕行及实际宽度窄门的闭环测试。
补充定位失效、点云超时与控制退出时的停车处理；当前 bridge 的超时检查主要记录日志，
不能将日志报警视为已经实现完整停车链路。

以上是源码分析结论，尚未执行实车通信或运动测试。
