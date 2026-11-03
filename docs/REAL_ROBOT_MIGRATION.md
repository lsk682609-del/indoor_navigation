# 基于 /home/lxh/nav 的室内导航实车迁移分析

> 用户最新指定参考 `/home/lxh/snn` 中的室内实车方案。
> 最新结论见 [SNN_REAL_ROBOT_MIGRATION.md](SNN_REAL_ROBOT_MIGRATION.md)，室外对照见
> [OUT_CAR_MIGRATION.md](OUT_CAR_MIGRATION.md)。下文保留的是旧 `/home/lxh/nav`
> 目录的检查记录，其中关于缺失硬件链路、SLAM 版本和速度输出的判断不能直接套到 out_car。

分析日期：2026-11-03。以当前本机源码为依据；未连接底盘、未发送速度，也未改动运行配置。

## 结论

保留 indoor_navigation 中已经修复和验证的路网规划、Theta*、RPP、窄门控制代码，
参考 nav 的实车启动组合，重新接入实际雷达、定位和松灵底盘。
迁移重点是定位/路网坐标、车体速度、TF、硬件接口和实车几何，不能仅替换 /odom 或改 use_sim_time。

## 本机 nav 实际提供了什么

| 入口 | 源码表现 | 迁移判断 |
| --- | --- | --- |
| [nx_net_nav_slam.launch.py](/home/lxh/nav/src/robot_launch/launch/nx_net_nav_slam.launch.py:69) | 组合 RoboSense 驱动、FAST-LIVO2、导航、ros2can，默认实时时钟 | 可参考其组成；它启动 fastlivo_mapping，不能据此认为已提供历史地图重定位 |
| [nx_net_nav.launch.py](/home/lxh/nav/src/robot_launch/launch/nx_net_nav.launch.py:23) | MID360＋MAVROS＋导航＋底盘，use_sim_time 默认 true | 不能直接作为室内实车入口；需要替换定位来源、统一时钟并检查 TF |
| [3588_net_nav.launch.py](/home/lxh/nav/src/robot_launch/launch/3588_net_nav.launch.py:53) | 命名空间内的导航和底盘，TF 话题也命名空间化，不含雷达和 SLAM | 可参考分机部署，但要接入外部定位/点云，并统一 TF 话题 |
| [nx_net_nav.yaml](/home/lxh/nav/src/robot_launch/config/nx_net_nav.yaml:178) | 相对话题 rslidar_points、odom、odom_local，旧外参 body_offset_x=-0.40、z=0.50 | 属于特定安装配置，不能直接套到当前小车 |

当前 /home/lxh/nav/src 和 install 未包含上述入口引用的 ros2can、fast_livo、rslidar_sdk。
本机其他目录能找到相关源码，但仍需确认小车上实际使用的版本。
例如 [co-search 中的底盘驱动](/home/lxh/下载/co-search/co-search-main/src/drivers_ros2/ros2can/src/run.cpp:296)
和 [FAST-LIVO2](/home/lxh/slam/src/LIVMapper.cpp:1781)。这些目录作为依赖参考，不等同于已验证能驱动这台松灵底盘。

## 必须修改的接口

### 1. 固定地图定位与路网坐标

当前仿真位姿来自 Gazebo 真值，路网与 world 天然同坐标。
实车须先在真实场地建立定位地图，再在同一坐标系中编辑路网。
定位地图（PCD/栅格等）和拓扑路网是不同资源：前者用于估计车辆在哪，后者定义走哪些走廊。
路网格式仍沿用现有 lla_mode:0 的米制分段格式。nav 里的实测路网如与当前场地及定位地图
同版、同坐标，可以复用；否则需要重新采集或正确变换节点，不能沿用仿真矩形环路。

[桥接 onOdom](/home/lxh/indoor_navigation/src/navigation2/bridge/nav2_input_bridge/src/input_bridge_node.cpp:245)
只是修改 frame_id 并处理杆臂，未将重启后原点变化的局部里程计变换到固定地图。
需要真实的地图定位，或显式估计并应用 map 与 LIO 世界之间的旋转、平移。
重启、换出生位姿后必须验证同一物理位置仍对应相同路网坐标。

nav 的 launch_slam.sh 仍引用旧路径和 laser_slam_3d；launch_net_slam.sh 传入 map_data_path，
但当前 nx_net_nav_slam.launch.py 未声明或转交该参数。因此不能把旧脚本当作已完成地图加载。
本机 /home/lxh/slam 有 initial_pose 和 LIO/RTK 输出逻辑，仍需另行验证所选定位版本的地图加载、
匹配及失效状态；室内不应依赖 GPS/MAVROS 才能形成导航位姿。

### 2. 分开接定位位姿与控制速度

当前 [navigation.launch.py](/home/lxh/indoor_navigation/src/indoor_bringup/launch/navigation.launch.py:22)
用同一个 odom_topic 同时配置 bridge 输入和 controller_server 输入，适用于当前 Gazebo /odom。
实车应拆为 localization_odom_topic 和 control_odom_topic：

- 定位位姿：已经对齐真实地图、参考点是 base_link。
- 控制速度：车体系的实际 vx、vy、wz，优先来自底盘反馈或连续里程计。

桥接程序目前用原输入位置差分计算世界系速度，却将 child_frame_id 标为 base_link。
车辆朝地图 +Y 以 0.3 m/s 行驶时，世界系速度是 (0,0.3)，车体系应为 (0.3,0)。
必须把速度旋转到车体系；若做差分，还要对修正后的车体位置差分，处理杆臂和时间间隔。
不要把地图重定位的位姿跳变差分成控制速度。

本机 /home/lxh/slam 的 statePoseToBase 已做 IMU→车体转换并发布 base_link 位姿，
再沿用 nav 的 -0.40/0.50 偏移会重复补偿。
其映射后 /car1/odom 不可默认认为含有效控制速度；高频传播分支的速度也明确是世界/ENU 表达。

查到的 ros2can/run.cpp 虽然解码 0x221 速度反馈，但只存入 velocity_struct_，
未发布导航可直接使用的 Odometry/Twist 速度话题。应在实际选用的驱动中补充发布，
或接入已经具备正确反馈接口的底盘驱动。

### 3. TF 只能有一个明确的发布来源

可保留简化链 map→base_link→lidar，也可采用 map→odom→base_link→lidar。
前者须保证唯一位姿发布者已输出地图系车体位姿；后者由地图定位和连续里程计分别负责两段。
不能用静态零变换代替真实的地图与里程计对齐。

本机 FAST-LIVO2 默认可发布 enu→base_link；当前 bridge 又会发布 map→base_link。
直接并用会让同一子坐标系出现冲突，需要新增桥接动态 TF 开关或绕过其位姿发布部分。
现有 publish_base_tf:=false **仅关闭 base_link→base_footprint 的静态节点**，
并不会关闭 bridge 发布的 map→base_link。

雷达外参须实测平移和旋转。nav 的 RoboSense、MID360 入口和当前仿真使用了三套不同外参，
不能按话题名猜测。bridge 当前只改点云 frame_id，不旋转/平移点坐标；
已转到世界系的 cloud_registered 也不能改名后当作雷达原始坐标点云。

### 4. 统一命名空间和硬件消息

建议先按单车 car1 整理接口，以下为待实现的接口约定，不是已可执行的配置：

| 用途 | 建议接口 | 关键约束 |
| --- | --- | --- |
| 定位 | 实际定位输出，例如 /car1/odom | 确认 frame、车体参考点与地图对齐 |
| 控制反馈 | /car1/control_odom | twist 必须是车体系实际速度 |
| 避障原始点云 | 实际驱动的 PointCloud2 | XYZ 字段、正确 frame、时间戳与 QoS |
| 导航消费点云 | /car1/cur_lid | costmap 与窄门控制订阅同一条有效数据 |
| 控制输出 | /car1/cmd_vel_nav→停车/限速层→/car1/cmd_vel | 最终与底盘实际订阅匹配 |
| TF | 统一 /tf、/tf_static，或完整一致的单车 TF 话题 | 不混用 nav/3588 的命名空间 TF 与当前全局 TF |

nav 的 nx_net_nav_slam 节点主要在根命名空间，bridge 配置的 odom 实际解析为 /odom，
而本机 FAST-LIVO2 的默认输出是 /car1/odom，需要显式接通。
当前室内导航输出 /cmd_vel；本机 co-search 的 car_driver.launch.py 会把相对 cmd_vel
放到 /car1/cmd_vel，也需要显式映射。nx_net_nav_slam 直接启动根节点的底盘方式又不同。

若用 RoboSense，参考 nav 的点云+IMU驱动链；若用 MID360，
SLAM 可需要 CustomMsg，而当前 bridge 输入需要 PointCloud2，可能需要开启对应发布模式或增加转换。
室内目录里的 livox_ros_driver2 **只有消息定义，没有真实驱动或 SDK**；
实车构建须避免它覆盖同名完整驱动包，不能直接拿它启动 MID360。

### 5. 底盘接口、几何和停车约束

确认实际松灵型号、控制协议、CAN 设备和反馈能力。
所查 ros2can 使用 libcanbus/USB-CAN，0x111 下发、0x221 反馈，并在驱动内做 ×1000 单位换算。
导航侧继续使用 m/s 和 rad/s，不要重复换算；这个驱动并非仅配置一个 can0 名字即可通用。
其 CMake 在缺 libcanbus/libusb/libudev 时会跳过 car_driver 目标，构建成功不代表有可用底盘程序。

实测车体与外伸设备的包络，修改 footprint、padding 和雷达安装外参。
另外 [控制器](/home/lxh/indoor_navigation/src/navigation2/controller/nav2_a_star_rpp/src/regulated_pure_pursuit_controller.cpp:1490)
还有车尾 0.23 m、车头 0.275 m 的源码常量，须参数化或从 footprint 推导，
只修改 YAML 的 footprint 不会同步修正前后净空检测。
窄门最小宽度、0.4 m 高度筛选、停车距离和允许定位误差也应按真实包络、传感器与场地重定。
0.70 m 的仿真通过结果不能直接作为实车可通过宽度。

初次实车调试可单独设较低的速度上限（例如 0.2 m/s），根据制动和跟踪记录逐步调回目标速度。
需要核查底盘既有的命令超时和急停，建立定位/雷达失效、导航进程退出时的停车链路。
当前 bridge 的 checkDataStatus 超时仅发布日志；所查 ros2can 文件未见 cmd_vel 超时置零，
底盘固件是否自带 watchdog 尚待核实，不能以日志报警替代停车验证。

### 6. 启动、部署和环境

新建 real_robot.launch.py、real_robot.yaml、env_real.sh、run_real.sh，保留现有仿真入口。
实车 use_sim_time=false，不启动 Gazebo、模型生成、仿真传感器插件和仿真 world 背景。
RViz 只显示真实路网、定位/实测地图及实时传感器；当前 world Marker 不是真实场地地图。

当前 env.sh 清除其他工作空间环境、设 ROS 域 88 和 ROS_LOCALHOST_ONLY=1，
实车环境应显式加载已确认的硬件/定位与导航工作空间，并避免加载同名的旧 nav 控制器。
单机可保持 localhost；分机时需统一域、开放相应 ROS 网络、统一 TF 话题并同步时钟。
本机 3588 启动脚本还默认 car6，不能与当前固定 car1 的 launch/goal.sh 混用。

只迁移源码和资源，在车载目标架构上重新构建；不要复制本机 build/install。
旧 nav 的部分 CMake 默认引用 aarch64 fmt，当前室内副本默认系统 fmt，仍需核对目标机依赖。
硬件版应排除仿真包，使用与实车兼容的完整驱动和消息定义。

## 推荐实施顺序

1. 确认实际底盘型号、雷达、车载计算机、定位程序版本和工作空间。
2. 先联通驱动与反馈，核对 cmd_vel 的话题、单位、方向、停车和真实车速。
3. 联通定位与点云，确认外参、时间、TF 唯一发布来源，修正 bridge 的位姿/速度职责。
4. 完成真实地图与路网的坐标绑定；测试重启、不同朝向启动和重定位后是否仍对齐。
5. 不接自动运动输出先观察路网、位置、点云、局部路径和实际包络的一致性。
6. 再验证低速直线、转弯、普通绕障、窄门，以及数据中断后的停车。

当前只完成源码分析。未执行实车驱动、CAN 操作或速度发布，不能把仿真验证等同于实车验收。
