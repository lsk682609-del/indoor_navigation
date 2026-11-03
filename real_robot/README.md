# Scout Mini 室内实车运行

本目录复制 snn 的 Scout、ugv_sdk 和完整 Livox 驱动，接入当前室内导航核心。
实车安装在 `../install_real`，硬件驱动安装在 `drivers_ws/install`，不会覆盖仿真安装目录。
默认实车 ROS 域为 **89**，仿真仍为 88；所有实车终端使用本目录 env.sh。

## 构建

```bash
cd /home/lxh/indoor_navigation
./real_robot/build.sh
```

需要已有 ROS 2 Humble、Livox SDK2、PCL 及 Scout SDK 的 Asio 头文件。
本机缺少的 libasio-dev 已下载并解压到 `real_robot/deps`，未修改系统安装；
Git 仓库不包含该缓存目录，克隆到其他机器后可安装 libasio-dev：

```bash
sudo apt install libasio-dev
```

本机本次构建输出见 `results/real_robot_build.log`；运行日志不纳入 Git。

## 部署前配置

1. 修改 `src/indoor_bringup/config/scout_hardware.yaml`：实际 CAN 口、话题和雷达安装外参。
   can_usb 及 `(0.16, 0, 0.25)`、roll=0.007、pitch=0.519 来自 snn，必须核对实际安装。
   本入口不创建 CAN 接口；沿用原实车已配置的 SocketCAN 环境。
2. 修改 `src/indoor_bringup/config/real_nav.yaml`：footprint 与 padding。
   当前 0.60 × 0.50 m 来自 snn 旧车配置，尚未按本次实车测量；前后探测边缘随 footprint 自动更新。
   默认期望速度 1.0 m/s，膨胀半径 0.5 m，保留 Theta* 和窄门控制。
3. 准备真实点云地图、原定位器及与其同坐标的路网。路网使用现有自定义格式，`lla_mode: 0`，
   XYZ 单位米。不要把仿真环形路网作为实车地图。需要栅格背景时准备对应地图 YAML。
4. 准备匹配实际雷达/主机地址的 MID360 JSON。驱动源码内的示例 JSON 仅供修改参考。

snn 中 `launch_slam.sh` 引用 laser_slam_3d，但未包含该包源码。
将原室内定位工作空间放到部署机，并在每个实车终端按实际路径设置：

```bash
export INDOOR_LOCALIZATION_SETUP=/实际定位工作空间/install/setup.bash
source real_robot/env.sh
```

如果定位器已独立运行，可不设置该变量；保证 ROS 域、主机通信设置和时间基准一致。
默认本机通信；分机部署时，各终端一致设置 `INDOOR_REAL_LOCALHOST_ONLY=0` 和实际域号。
定位器需要加载原点云地图并完成初始定位。确认输出的实际 Odometry 话题、位姿参考点、
TF 结构后再传给本入口。局部轮式积分或无地图约束的起点坐标不能直接作为历史路网 map 定位。

## TF 方式

| tf_mode | 定位器发布 | Scout 发布 | bridge 发布 |
| --- | --- | --- | --- |
| map_base（默认） | map → base_link | 不发布动态 TF | 不发布动态 TF |
| map_odom | map → odom | odom → base_link | 不发布动态 TF |
| bridge | 地图位姿消息，不发布指向 base_link 的动态 TF | 不发布动态 TF | map → base_link |

定位器的输出帧不同于 map 时，必须存在真实、已知的坐标变换；bridge 会查询 TF，
不会把 world/odom 简单改名成 map。输出参考点是雷达/IMU 时，也需要该帧到 base_link 的 TF。
严格模式禁止 body_offset 与 TF 外参叠加补偿。

默认发布 base_link → livox_frame 实测外参位置的静态 TF；若原定位器/URDF 已发布，
传 `publish_lidar_tf:=false`。base_link → base_footprint 为单位静态变换，需确保其与实车参考点约定一致；
原 URDF 已发布这条变换时传 `publish_base_tf:=false`。

## 启动

以下路径和定位话题均需替换为实际值。先启动并初始化原室内定位器，再执行：

```bash
./real_robot/run.sh \
  topo_map_path:=/实际地图/室内路网.yaml \
  localization_odom_topic:=/原定位器实际里程计话题 \
  lidar_config:=/实际配置/MID360_config.json \
  tf_mode:=map_base
```

若雷达或底盘已由原实车入口启动，分别加 `start_lidar:=false`、`start_base:=false`，
确保其话题、轮速输入与 TF 开关与本配置一致，避免重复启动。
已直接提供传感器帧 PointCloud2 时，加 `convert_livox:=false`，将 hardware 配置中的
navigation_cloud 设为实际 PointCloud2 话题。不要给导航输入改名后的世界坐标点云。

如果希望入口一起启动定位器，先在 hardware 配置的 localization 中填入真实包、launch
及它实际支持的地图等参数，再加 `start_localization:=true`。缺失的包不会被本工程自动补造。
定位器不发布栅格地图时，可加 `map_yaml:=/实际地图/map.yaml` 启动地图服务；
已有 /map 发布者时不传该参数。RViz 显示真实路网、栅格地图与实时障碍，不显示仿真墙体。

另开实车终端下发目标：

```bash
cd /home/lxh/indoor_navigation
./real_robot/goal.sh 3 0
./real_robot/goal.sh --cancel
```

坐标为实际 map 坐标。脚本报告导航请求是否受理，到达状态查看 `/car1/navigation_status`。
实车使用 `real_robot/goal.sh`，原 `scripts/goal.sh` 继续服务仿真域 88。

## 速度输出和测试

```text
控制器 /car1/nav_cmd_vel → velocity_guard → /cmd_vel → Scout
定位器 → bridge /car1/odom_local ─────────────┘
Scout /wheel_odom → 控制器速度反馈 + velocity_guard
Livox CustomMsg → 转换 → bridge /car1/cur_lid → 避障 + velocity_guard
```

guard 同时检查定位、轮速、过滤点云和动态地图 TF，默认数据时效 0.5 秒，指令时效 0.25 秒；
任何一项未就绪、超时、帧错误或输入异常时输出零速度，恢复后需要新控制指令。
速度超限时同比例缩放线速度与角速度，保留控制器已检查的转弯曲率。
状态话题为 `/velocity_guard/status`。定位器提供 Bool 健康状态时，在 hardware 的
guard.localization_health_topic 中填写，用于配准失败但仍持续发布位姿的情况；
未配置时只能判断消息、帧和 TF 时效，不能证明地图匹配质量。
Scout 驱动另有 0.3 秒指令超时归零，防止 guard 进程退出后沿用历史指令。

无硬件接口测试（只启动转换、bridge 和 guard，速度输出固定为 /real_test/output）：

```bash
source real_robot/env.sh
ROS_DOMAIN_ID=187 python3 real_robot/test_interfaces.py
ROS_DOMAIN_ID=186 python3 real_robot/test_navigation.py
```

本次验证：4 个硬件依赖包和 17 个导航包编译通过，27 项接口检查通过；
合成代价地图的窄门/车体尺寸回归通过；完整实车入口在虚拟运动闭环中接受目标并到达，
最终复测中 3 m 目标终点误差 0.195 m、最高速度 1.000 m/s。测试未启动真实底盘或雷达。
使用原仿真消息包的独立构建也通过，未覆盖正在运行的仿真安装结果。
结果位于 `results/real_robot_interfaces.log`、`results/real_robot_footprint.log`、
`results/real_robot_navigation.log`；完整入口测试结束时各节点正常退出。

首次实车验证依次检查静止定位与 TF、轮速方向、空走廊、转弯、绕障及实际门宽通行。
当前自动检查不等于实车闭环测试，1.0 m/s 为配置的期望上限。

## 本次复制与修改

驱动来源：`/home/lxh/snn/项目工作交接/nav/src/driver`，保留原许可证。
本目录只修改复制后的 Scout：暴露 TF 开关、指令超时归零、退出控制循环时归零；
ugv_sdk 显式声明 cmake 构建类型。原 snn 源码保持不变。
导航核心增加严格帧变换/TF 开关，速度输入与定位输入拆开，前后边缘读取 footprint。
分析依据见 [SNN 实车迁移分析](../docs/SNN_REAL_ROBOT_MIGRATION.md)。
