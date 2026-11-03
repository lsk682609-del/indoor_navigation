# 以 out_car 为基线适配室内小车

> 用户进一步指定参考 snn 中的室内实车方案，最新接入结论见
> [SNN_REAL_ROBOT_MIGRATION.md](SNN_REAL_ROBOT_MIGRATION.md)。本文保留为室外实车程序的对照记录。

2026-11-03。用户确认 out_car 是此前已经做过室外 SLAM 与导航实车测试的程序。
本次为本机源码对照分析，未启动底盘或更改运行配置。

## 当前结论

复用 out_car 的实际雷达、SLAM 和底盘接口，保留 indoor_navigation 已修复的
路网跟踪、Theta*、窄门和停顿处理，新增小车专用实车配置。
在传感器、控制协议和计算平台与原测试一致的前提下，主要工作是几何参数适配、
接口接通和室内定位坐标处理，无需重新搭建整套硬件链路。

当前 out_car 中同时保留了旧车、SCAN、相机坡度方案和新车激光方案。
应按启动脚本实际选择的文件分析，不能仅查看 nx_net_nav.yaml 或旧文档。

## 已核实的实际启动路径

1. [SLAM run.sh](/home/lxh/out_car/nanhu-huibo-slam/run.sh:35)
   启动 rslidar_sdk/car1_start.py、RTK 串口和
   fast_livo_with_bievr/car1_rslidar_rtk_mapping.launch.py。
2. [RoboSense 配置](/home/lxh/out_car/WS_ROS2/catkin_robosense/src/rslidar_sdk/config/config.yaml)
   使用 RSAIRY，发布 /car1/rslidar_points 和 /car1/rslidar_imu_data。
3. [导航 3588_launch.sh](/home/lxh/out_car/nav/scripts/3588_launch.sh:13)
   设 ROS_DOMAIN_ID=77，实际调用 3588_new_car_net_nav.launch.py，默认 namespace=car1。
4. [新车 launch](/home/lxh/out_car/nav/src/robot_launch/launch/3588_new_car_net_nav.launch.py)
   默认加载 new_car_lidar_rpp.yaml，定位与底盘在外部启动。
5. [新车配置](/home/lxh/out_car/nav/src/robot_launch/config/new_car_lidar_rpp.yaml)
   bridge 订阅 combined_odom 和 rslidar_points，输出 odom_local 与 cur_lid。
   控制器默认输出 geometry_msgs/TwistStamped。

相应的预期数据流为：

```text
Airy 点云＋IMU → fast_livo_with_bievr → /car1/combined_odom
                     ↓                         ↓
                 地图/定位              导航位姿和车体系速度
Airy 原始点云 → bridge → /car1/cur_lid → 局部代价地图
真实室内路网 → 路网规划＋Theta*＋RPP＋窄门 → 底盘速度接口
```

这套源码已经提供导航可用的位姿与速度基础。
[DebugInfo::publishFrame](/home/lxh/out_car/nanhu-huibo-slam/src/fast_livo_with_bievr/src/common/DebugInfo.cpp:444)
处理 IMU→base 的杆臂速度并转换到车体系；publishStandardOutput 同时发布 /car1/odom、
/car1/combined_odom，包含 twist。不能再沿用先前对其他 SLAM 版本“没有控制速度”的判断。

但当前 indoor_navigation 与 out_car 的 input_bridge 都会重新用位置差分覆盖 twist。
接入这个已含有效车体系速度的 combined_odom 时，宜增加保留输入 twist 的模式，
避免覆盖正确的速度，尤其避免将世界系差分速度误标为车体系。

## 尺寸和外参如何替换

以下小车列是当前仿真/室内配置的数值，不能直接视为实车测量结果。

| 项目 | out_car 当前新车配置 | 室内小车配置/修改方向 |
| --- | --- | --- |
| 包络长度 | 1.045 m | 当前仿真包络 X 范围总长 0.505 m；实车按测量写入 |
| 包络宽度 | 0.780 m | 当前仿真 0.460 m；实车包括轮胎、保险杠及突出载荷 |
| base_link 参考点 | 前主动轮轴中点地面投影 | 小车需明确控制参考点，定位、控制和 footprint 用同一点 |
| 前后边界 | X=[-0.755,+0.290] m | 当前室内 X=[-0.230,+0.275] m；依实车和参考点替换 |
| 左右边界 | Y=±0.390 m | 当前室内 Y=±0.230 m；依实车替换 |
| 雷达平移 | launch 为 (0.240,0,0.713) m | 仿真为 (0.240,0,0.300) m；实际安装位置应测量 |
| 雷达 pitch | launch 实际默认 0.3490 rad | 当前仿真为 0；实车按安装角设置 |
| body_offset | 新车配置三轴均为 0，定位已输出 base 位姿 | 如果继续输出同一小车 base 参考点，保持 0，避免重复补偿 |
| 膨胀/通行 | 新车 inflation_radius=0.7，lethal_cost=200 | 保留室内版 0.5 和 253 的已测策略，再结合实车包络验证 |

源码注释与参数有个别不一致：新车注释说 padding=0.03，当前实际值却是 0.0；
雷达注释说倾角 0，launch 实际默认却是 0.3490。适配时以当前生效值和实测安装为依据。

SLAM 中 [car1_rslidar_rtk.yaml](/home/lxh/out_car/nanhu-huibo-slam/src/fast_livo_with_bievr/config/car1_rslidar_rtk.yaml)
还包含 base_frame.lidar_position=[0.45,0,0]，需要与新的车体参考点一起核对。
同一雷达/IMU组件的内部相对安装若没有变化，相关内部外参可以沿用；
组件相对车体的安装位置、雷达高度、IMU→base 杆臂变化则需要更新。

室内控制器还有车头 0.275 m、车尾 0.23 m 的源码常量，须参数化或从 footprint 推导，
使前后净空检测与 YAML 轮廓同步。窄门最小宽度和停车距离也应按新车包络与误差设置。

## 室内与室外之间需要处理的差异

### RTK 和地图原点

当前室外配置 rtk.enable=true、frontend.enable=true；SLAM run.sh 也会启动 RTK。
LIVMapper 在 RTK 开启但启动标定未完成时会等待标定，不发布后续正常帧输出。
室内无可靠 RTK 时应为该副本配置 rtk.enable=false，并关闭/调整依赖 RTK 的前端和建图业务。

源码保留无 RTK 的 LIO 处理和位姿发布路径；但无 RTK 后的局部原点与原室外 ENU 不再天然一致。
应建立室内定位地图与路网的共同坐标，不能照搬室外经纬度或仅将 world/enu 改名为 map。
固定场景反复运行需要地图定位/重定位或明确的初始位姿对齐，是否已覆盖当前室内场地须实测。

### 消息类型与 TF 约定

out_car 新车入口默认 TwistStamped，当前室内 ControllerServer 实际发布 Twist。
如果小车沿用带时间戳的驱动接口，需迁入 out_car 的可选消息类型输出支持；
如果选用的小车驱动接收 Twist，保留当前输出并显式对齐话题即可。
只在 YAML 增加 enable_stamped_cmd_vel=true 不会让未实现该功能的室内代码切换消息类型。

out_car SLAM 发布的车体 frame 是 car1/base_link；导航则把 TF 放到 /car1/tf、/car1/tf_static，
使用 map、base_link、lidar 这些 frame 名。应完整复用这套隔离约定，或统一调整全部节点；
当前室内仿真的全局 TF 用法不能与它零散混用，RViz 也须接相应 TF。

现有室内脚本默认 ROS 域 88，out_car 实车脚本为 77；接入时统一各实车进程和目标发送端。
原脚本的 /home/firefly 路径及 fastshm.xml 取决于车载部署环境，需要按目标机实际路径加载。

### 保留室内改进

保留当前室内版的目标弧长限制、绕障段复用、速度限幅修复、对准防抖、临时段续接、
带 padding 的足迹验证及窄门感知/通行逻辑，不整体覆盖成较旧控制器。
相机坡度/坑洞层不因 out_car 存在就全部启用；当前新车入口本身也只启用激光障碍层和膨胀层。
此前修正的 Livox ODE 距离只作用于仿真插件，不应把补偿加到实际 Airy 雷达数据上。

## 最小改动清单

1. 新增小车实车 launch/参数/环境入口，引用 out_car 的硬件和定位工作空间，保留仿真入口。
2. 按实测值替换 footprint、控制参考点、雷达安装外参和 SLAM 的车体杆臂参数。
3. bridge 接 /car1/combined_odom，并增加保留有效输入 twist 的模式；控制器读对应的 odom_local。
4. 按实际底盘接口统一 Twist 或 TwistStamped，以及命名空间、TF 话题和 ROS 域。
5. 无 RTK 的室内运行使用对应配置，建立真实室内地图和路网坐标对应关系。
6. 将前后净空常量与 footprint 联动，验证低速直线、旋转、窄门及定位/点云中断行为。

按用户已确认的实车测试背景，驱动和 SLAM 链路可以作为复用基础。
当前仓库仍有多套并存的入口；文中“当前新车配置”指 3588_launch.sh 现时指向的文件，
不推定每一次既往实车测试都使用了这些完全相同的参数，也未将任何配置写入实车。
