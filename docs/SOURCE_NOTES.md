# 来源和独立性

2026-11-02 从本机现有 `nanhu-research-navigation` 源码创建独立副本。

2026-11-03 新增 `real_robot/drivers_ws/src`，复制自本机 `snn/项目工作交接/nav/src/driver`：
scout_base、scout_msgs、ugv_sdk 与完整 livox_ros_driver2。原 snn 目录未修改。
实车通过独立驱动工作空间与 `install_real` 构建，仿真消息包仍保留在 src/simulation 中。
实车入口、点云转换、速度输出检查和接入测试为本次新增；原室内定位包需外部工作空间提供。

- `src/navigation2`：保留自定义消息、路网规划、状态机、RPP/Theta*、costmap 和日志等包。
  将原有两个 fmt 选择项的默认值设为使用系统库，避免依赖旧 ARM fmt 文件。
  后续修复了本副本中 Theta* 的地图边界访问、搜索状态清理和队列结束判断；
  验证见 VALIDATION.md 与 tests/theta_star_regression。旧工作空间未同步修改。
  箱体复测后进一步修正普通室内绕障的目标弧长、历史汇入索引、无解停车分支及
  LOS/折线代价一致性；调整仅作用于本工作空间副本。
- `src/simulation/vehicle_gazebo_sim`：保留原小车 URDF、相机模型和仿真入口。
- `src/simulation/livox_laser_simulation_RO2`、`realsense_gazebo_plugin`：复制原仿真插件源码。
- `src/simulation/livox_ros_driver2`：仅保留原两种消息及版权许可，创建消息专用构建；
  不包含 Livox 硬件驱动和 SDK。不能用本消息包代替实车雷达驱动。
- `src/indoor_bringup`、根目录脚本和测试：本次新增。
- 初始控制器参数与 RViz 视图来自原 gazebo_net_nav 配置，室内配置降低速度，
  改为平面雷达障碍层并关闭场景特定的门区伺服。
  箱体测试后修复规划/跟踪衔接，之后按用户要求将膨胀半径改为 0.5 m、最高速度改为 1.0 m/s。
  参考本机 `out_car/nav/src/navigation2/controller/nav2_a_star_rpp` 和
  `下载/MotionLab/src/nav2_a_star_rpp` 的同路绕障段保留、253 阈值复检及路径平滑策略。
  室内副本保留 140 的规划缓冲，已生成绕障段采用 253 执行阈值并补充带 padding、
  随候选航向旋转的完整轮廓检查；平滑最大偏移限制 0.15 m。
  后续根据现场日志修复对准状态切换、前向净空限速逐周期累乘和临时绕障段末端等待，
  改动仅在室内副本；原 out_car 和 MotionLab 未同步修改。

原工程未修改。源码不是外部符号链接，不复制旧 build/install/log。
窄门修复进一步采用原 `nanhu-research-navigation/src/robot_launch/config/gazebo_net_nav.yaml`
的 253 规划阈值与门框测距三阶段伺服；室内自动感知门心，保留低速上限、
新鲜点云检查和完整轮廓预测，并用里程计纠正锁定直行航向。
修正本副本 Livox ODE 点云漏加射线起始距离的问题，避免把障碍错误地向内移动 0.10 m。
门区只绕过膨胀代价，真实碰撞保护保留；普通绕障仍不使用原固定门场景的无解强行跟踪分支。
新构建仅使用 /opt/ros/humble、系统依赖和本工程安装目录。
第三方文件保留其原有版权和许可声明，分别以对应包的声明为准。
