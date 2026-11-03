# 松灵小车室内路网导航

独立 ROS 2 Humble 工作空间，使用现有仿真的四轮滑移转向松灵小车模型。
本目录包含导航核心、模型、雷达/相机仿真插件、室内世界、路网、启动和测试工具。
源码为实际副本，构建及运行不需要 source 原来的 nanhu 或 out_car 工作空间。

默认提供**可运行的室内仿真导航**：Gazebo 里程计位姿 + 平面路网 SNN 规划 +
雷达局部代价地图 + Theta* 绕障 + RPP 跟踪。默认最高速度 1.0 m/s、膨胀半径 0.5 m；
弯道和障碍附近自动减速，Theta* 绕障路径经车身轮廓检查后平滑并复用。
窄通道自动识别两侧边缘、居中对准并以 0.25 m/s 穿过，保留完整车身碰撞检查。
已新增参考 snn 的 Scout Mini 实车入口，包含底盘驱动、完整 Livox 驱动、点云转换及数据超时停车。
基于历史地图的定位器通过原实车工作空间接入，本目录尚未包含该定位器源码。
实车构建、地图/外参配置与启动方式见 [实车运行说明](real_robot/README.md)。

仓库保存源码、配置、地图、启动脚本和测试。`build*`、`install*`、日志、运行结果及
本机下载的依赖缓存不入库，克隆后需要重新构建。源码许可与来源见 [NOTICE.md](NOTICE.md)。
当前实车接入已通过虚拟闭环测试，尚未完成实车闭环；新版外部路网编辑器的命令兼容适配仍待完成。

## 环境与首次构建

使用当前电脑已有的 Ubuntu 22.04、ROS 2 Humble、Gazebo Classic 11 和开发依赖。

```bash
cd /home/lxh/indoor_navigation
./scripts/build.sh
./scripts/check.sh
```

启动前会检查各节点的安装路径和执行权限；缺失时直接提示，不会启动到一半再关闭界面。

脚本先载入系统 ROS，再使用本目录的安装结果，主动清除其他 ROS 工作空间的路径。
Livox 依赖仅保留仿真需要的消息定义，不需要安装 Livox 硬件 SDK。
更换电脑时可先检查依赖：

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
```

依赖由系统安装，本目录不包含 Ubuntu/ROS/Gazebo 本身。保留了原导航库中的包名，
请分别打开终端使用各工作空间。更换目录后重新编译，不要直接搬用旧 build/install。

## 一键启动

终端 A：

```bash
cd /home/lxh/indoor_navigation
./scripts/run_sim.sh
```

默认打开 Gazebo 和 RViz，车辆从 `(0,0)` 朝 +X 出发。无界面测试：

```bash
./scripts/run_sim.sh gui:=false use_rviz:=false
```

此前保存的箱体、圆柱和窄通道测试布局已纳入源码，可以显式选择：

```bash
./scripts/run_sim.sh \
  world:="$PWD/src/indoor_bringup/worlds/indoor_narrow_passage.world"
```

Gazebo 中临时增删的障碍不会自动写回该文件。结束已有实例后再切换场景。

RViz 默认俯视完整场景，青色线为完整路网、黄色圆点为节点、白色文字为节点编号，
灰色为室内墙体，红色为本次选择的规划路线。无需先发送目标，完整路网即会显示。
背景只是世界几何的显示，不替代导航的实时雷达避障。

旧仿真已经运行时，可另开终端补开新版地图显示（无需重启 Gazebo）：

```bash
cd /home/lxh/indoor_navigation
./scripts/run_display.sh
```

之后整体重启仿真时，先结束单独的显示进程；新版 run_sim.sh 已自动包含显示功能。

默认使用本机 ROS 域 88、Gazebo master 端口 11355，与旧工程的域 77/默认 Gazebo 端口分开。
同一实例只启动一次；同一个 master 端口不能同时运行两个仿真。
启动时会检查同域的旧导航进程及当前端口的 Gazebo，发现残留后立即停止新启动，
避免叠加两套导航和场景背景。结束实例请在启动终端按 Ctrl+C；仅关闭 Gazebo/RViz
窗口不等于结束完整实例。Gazebo 服务端退出时，新启动的整套实例也会结束。
所有脚本自动使用相同环境。如需修改，在每个终端一致设置 `INDOOR_ROS_DOMAIN_ID`
和 `INDOOR_GAZEBO_MASTER_URI`。默认只允许本机 ROS 通信。

## 下发目标与自动测试

终端 B：

```bash
cd /home/lxh/indoor_navigation
# 等待节点激活、下发目标、等待到达并保存 JSON。
./scripts/goal.sh 3 0

# 按配置连续测试直线、转弯和返回。
./scripts/goal.sh 0 0
./scripts/test.sh

# 小车位于原点附近时，在 (3,0) 自动放置箱体并导航至 (6,0)。
# 检查 Theta* 绕障路径与终点误差，结束后移除箱体。
./scripts/test.sh --obstacle

# 取消正在执行的任务。
./scripts/cancel.sh
```

对截图中的 1 m 箱体，可关闭旧仿真后运行 `./scripts/run_avoidance.sh`，
再用 `./scripts/goal.sh 0 4` 和 `./scripts/goal.sh 0 0` 测试两方向绕障。
自动回归可直接执行 `./scripts/validate.sh --avoidance-isolated`，
它会启动单独的无界面实例，不与当前窗口的目标点冲突，并记录真值轨迹和车体净距。

X 和 Y 必须用空格分开。`./scripts/goal.sh 3.0` 只有一个参数，
正确写法是 `./scripts/goal.sh 3 0` 或 `./scripts/goal.sh 3.0 0`。
命令会立即打印等待就绪、目标受理和行驶状态，随后等待到达。

自动测试默认额外读取 Gazebo 的 `simple_car` 真值，成功条件为导航状态 SUCCEEDED、
收到非空规划路径且真值终点误差不超过 0.3 m。超时或失败会取消任务并返回非零退出码。
这项测试没有证明整段路线零碰撞，也没有验证定位失效或动态障碍工况。
结果保存在 `results/navigation_*.json`。

`./scripts/validate.sh` 会先执行静态检查、全部节点对规划，再执行行驶测试。
`./scripts/validate.sh --obstacle` 执行同样的前置检查及单独箱体绕障测试。
从原点启动的新仿真可用 `./scripts/validate.sh --all` 一次完成箱体绕障、回原点和基础路线回归。

使用 RViz 的 `2D Goal Pose` 也可下发目标，话题为 `/goal_pose`，Fixed Frame 使用 `map`。
目标应放在走廊路网上；中央封闭区域不是导航目的地。

测试 C++ 规划器的全部 12×12 对节点路径：

```bash
source scripts/env.sh
python3 tests/planner_matrix.py
```

检查状态：

```bash
source scripts/env.sh
ros2 topic echo /car1/navigation_status
ros2 run tf2_ros tf2_echo map base_link
```

`state=3` 表示成功，4 表示取消，5 表示失败。服务受理成功不等于到达。
`source scripts/env.sh` 只改变当前终端，不会修改 `.bashrc`。

## 常用修改位置

| 内容 | 文件 |
| --- | --- |
| 室内速度、车体轮廓、感知和控制参数 | `src/indoor_bringup/config/indoor_nav.yaml` |
| 路网节点和边，局部 XYZ 米制坐标 | `src/indoor_bringup/maps/indoor_topology.yaml` |
| 室内场景 | `src/indoor_bringup/worlds/indoor.world` |
| 世界生成器 | `scripts/generate_world.py` |
| 自动测试目标 | `src/indoor_bringup/config/test_routes.yaml` |
| 仿真整体启动 | `src/indoor_bringup/launch/simulation.launch.py` |
| 完整路网、节点编号与地图背景 | `src/indoor_bringup/scripts/road_network_display.py` |
| 单独启动导航、外部输入接口 | `src/indoor_bringup/launch/navigation.launch.py` |
| 小车模型 | `src/simulation/vehicle_gazebo_sim/urdf/simple_car.urdf.xacro` |
| 路网规划 | `src/navigation2/planner/nav2_net_planner` |
| RPP / Theta* 控制器 | `src/navigation2/controller/nav2_a_star_rpp` |

```text
indoor_navigation/
├── src/indoor_bringup/     室内启动、地图、配置和世界
├── src/navigation2/       从现有程序复制的导航核心
├── src/simulation/        原小车模型和仿真插件
├── scripts/               编译、启动、目标和取消入口
├── tests/                 静态几何检查和运行测试
├── docs/                  开发与测试说明
└── results/               运行报告和日志
```

来源及适配范围见 [SOURCE_NOTES.md](docs/SOURCE_NOTES.md)。本次实际验证记录见
[VALIDATION.md](docs/VALIDATION.md)。

换电脑可使用[统一迁移与验收工具](https://github.com/lsk682609-del/robotics-workstation-restore)，其中包含固定版本、依赖安装、数据备份清单和验收入口。
