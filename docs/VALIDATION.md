# 验证记录

## 2026-11-03：重复实例与场景背景修复

现场存在旧窄门场景的残留导航栈和新默认场景的导航栈，同名节点各两份，背景发布者两份；
新导航生命周期启动失败，RViz 保留了 Gazebo 中已经不存在的旧障碍物标记。
关闭旧残留 launch，保留新 Gazebo，重新初始化当前导航生命周期；规划器、控制器、状态机均激活。
确认当前背景只有 6 个默认场景标记，背景发布者及速度发布者各 1 个，里程计和点云持续更新。

启动入口增加同域残留导航及 Gazebo master 冲突检查，Gazebo 服务端退出时结束新启动的整套实例。
背景与路网标记每次发布先清除旧标记，防止切换场景残留已删除对象。
7 项进程冲突回归通过；实际尝试重复启动被立即拒绝，没有创建新子节点。
恢复快照见 `results/recovered_scene_20261103.json`，拒绝日志见 `results/duplicate_launch_guard.log`。

## 2026-11-03：实车入口接入测试

- 实车驱动 4 包、导航核心 17 包编译通过，安装与仿真分开。
- `real_robot/test_interfaces.py`：27 项通过。验证 CustomMsg 转换、时间戳、NaN 过滤、
  实际坐标变换、传感器参考点补偿、保留车体速度、关闭重复 TF、各输入超时归零及退出归零。
- 车体轮廓回归：更改车体尺寸后前后净空探测同步更新，原 0.70 m 窄门和碰撞保护回归通过。
- `real_robot/test_navigation.py`：关闭全部硬件驱动，以合成定位/雷达及虚拟运动模型运行
  完整实车入口，最终复测目标 `(3,0)` 导航成功，误差 0.195 m，最高速度 1.000 m/s；节点正常退出。
- 测试使用独立 ROS 域及 `/real_test/output`，没有连接底盘或执行真实运动。
- 原仿真静态资源检查通过；新增组件对原仿真消息包的独立构建通过，原正在运行的仿真安装目录未覆盖。

日志见 `results/real_robot_interfaces.log`、`results/real_robot_footprint.log`、
`results/real_robot_navigation.log`。实际地图定位、CAN 收发与实车制动仍需实车验证。

验证日期：2026-11-02。

## 当前版本：自动窄门通行与仿真雷达距离修复

现场双箱体净宽约 1.098 m，但 costmap 通道软代价达到 200–245，被原 140 阈值封闭。
参考本机 `nanhu-research-navigation/src/robot_launch/config/gazebo_net_nav.yaml`：
改为 253 规划阈值，启用 ALIGN/CROSS/EXIT 点云门框伺服，不配置原固定门心。
穿门上限 0.25 m/s，入门前检查整段带 padding 的车身通行空间；穿门期间继续检查
新鲜点云、前向净空、当前足迹及预测足迹。门框丢失后用里程计纠正锁定航向。

同时发现 Livox ODE 射线从 minDist=0.10 m 处起算，发布点云却直接使用距射线起点的
contact.depth，漏加了 minDist，使障碍看起来偏近。修正后，静止点云与独立已知的
门框碰撞表面比较，三个窄门场景中位误差均 <0.00001 m，95% 分位误差均 <0.00004 m。
这是理想仿真几何一致性检查，不代表实物雷达精度。

最终运行 `./scripts/test_narrow_suite.sh` 全部通过：

| 场景 | 结果 | 耗时 (s) | 关键最小采样净距 (m) |
| --- | --- | ---: | --- |
| 0.70 m 门洞，(8,0)→(12,0) | SUCCEEDED，实际进入 CROSS | 12.769 | 上框 0.1004，下框 0.0992 |
| 当前双箱体和圆柱现场，(8,0)→(12,8) | SUCCEEDED，穿门并绕柱 | 29.169 | 上箱 0.2390，下箱 0.2708，圆柱 0.2794 |
| 0.45 m 门洞，两侧封至墙体 | 18 s 观察窗内拒绝进入，无 CROSS、无接触 | 18.005 | 所有场景障碍 >1.02 |
| 原 1 m 箱体，(9,0)→(0,4) | SUCCEEDED | 18.513 | 0.1631 |
| 原 1 m 箱体，(0,4)→(0,0) | SUCCEEDED | 10.712 | 0.2063 |
| 原 0.5 m 箱体，(0,0)→(6,0) | SUCCEEDED | 11.767 | 0.2825 |

0.45 m 测试在观察窗结束后主动取消任务，验证的是不进入不可通行门洞，
不代表已实现堵路后自动换环路或自动结束任务。正向通行测试同时要求状态成功、真值终点
误差 ≤0.3 m、两侧门框及全部盒体/圆柱的带 padding 车身净距 >0.02 m；
并直接订阅 `/log/record` 核实实际执行 CROSS，不能仅凭生成路径判为通过。
净距为 Gazebo 真值轨迹的采样检查，并非连续时间接触监测。

直接控制器回归还覆盖：0.70 m 门感知、过窄/离路线/过期点云拒绝、253 地面允许穿越、
新致命障碍打断预测运动；以往的速度限幅、防抖及临时路径续接回归均通过。
实际 C++ 路网规划再次通过 144/144 节点对检查。

- [最终报告汇总](../results/narrow_passage/summary.json)
- [实际轨迹图](../results/narrow_passage/trajectories.png)
- [现场完整模型位姿](../results/narrow_passage/live_models.json)
- [重建的完整现场](../results/narrow_passage/live_scene.world)
- [最终测试参数](../results/narrow_passage/tested_indoor_nav.yaml)

## 上一版：缩短绕障停顿

用户现场日志显示 Theta* 搜索完成耗时中位数为 4.28 ms（5 次完整触发/完成记录，
最大 6.28 ms），主要等待来自 0.12 rad 附近反复触发原地对准、净空系数逐周期
乘到上一拍输出，以及截断的临时绕障段停在末端后等待进度检查超时。
现场日志已保存在 [before_live_simulation.log](../results/avoidance_latency/before_live_simulation.log)。

修复三个控制分支，保留最高 1.0 m/s、膨胀半径 0.5 m、前向停车保护和轮廓碰撞预测。
新增直接控制器回归通过：恒定净空下目标速度正常收敛而不累乘、曲率和加速度限幅、
对准进入/退出角分离、小角度抖动不反复停车、临时路径及时续接且最终目标继续减速。
重新编译成功，144/144 节点对规划和以下四项独立 Gazebo 行驶测试通过：

| 场景 | 上一版耗时 (s) | 当前耗时 (s) | 最长实测平移停顿 (s) | 最小车体净距 (m) |
| --- | ---: | ---: | ---: | ---: |
| 1 m 箱体，(9,0)→(0,4) | 39.867 | 24.716 | 0.291 | 0.2129 |
| 1 m 箱体，(0,4)→(0,0) | 16.469 | 12.621 | 0.422 | 0.2882 |
| 0.5 m 箱体，(0,0)→(6,0) | 10.819 | 11.763 | 0.639 | 0.2189 |
| 新增上方箱体，(0,8)→(12,8) | 未做相同起点全程计时 | 14.619 | 0.080 | 0.2599 |

最长平移停顿按 /odom 实测平移速度 <0.03 m/s 的连续墙钟时间统计，
不等于原地转向指令持续时间；滑移转向中车体仍有平移时不计为停止。
耗时包括初始姿态调整，不是纯搜索耗时。小箱体全程本轮略慢，不声称每种情况都提速。
四项均收到 Theta* 路径、最终到达、指令达到 1 m/s，真值终点误差均小于 0.204 m。
上方测试按采集到的 unit_box_0 位置 `(6.161,8.807)` 重建静态 1 m 箱体，
复测相同障碍位置，本次未出现原现场的进度超时恢复。
本轮上方仿真轨迹未触发临时段提前续接分支，其触发条件由直接控制器回归单独覆盖。

- [三场景对比和报告路径](../results/avoidance_latency/comparison.json)
- [三场景实际轨迹](../results/avoidance_latency/standard/trajectories.png)
- [上方走廊实际报告](../results/avoidance_latency/upper/navigation_20261102_194416_571035.json)
- [现场模型位姿](../results/avoidance_latency/live_models_before.json)
- [现场重建世界](../results/avoidance_latency/live_scene.world)

复现命令：

```bash
./scripts/test_detour_footprint.sh
./scripts/validate.sh --avoidance-isolated --output results/avoidance_latency/standard
./scripts/validate.sh --avoidance-isolated --upper-corridor \
  --world results/avoidance_latency/live_scene.world --output results/avoidance_latency/upper
```

## 上一版：0.5 m 膨胀、1.0 m/s 速度与绕障优化

参考本机 out_car 与 MotionLab 的路径复用和平滑策略，区分规划缓冲与绕障执行检查，
补充随航向旋转的 padded footprint 复检，启用最大偏移 0.15 m 的平滑。
20 Hz 控制、速度相关预瞄及匹配速度的加减速/前向减速距离见 DEVELOPMENT.md。
下方旧版本记录保留原参数，仅本节代表当前版本。

重新编译控制器和 bringup 成功；144/144 实际 C++ 路网节点对规划通过。
ASan/UBSan 的 Theta* 边界、不可达及 100 次失败恢复检查通过。
新增控制器直接调用回归通过：软代价变化可复用路径、254 新障碍使路径失效、
中心可通行但前角碰撞时拒绝、候选航向变化、车身轮廓越界和未知格拒绝。

独立 Gazebo 实际行驶三项全部 SUCCEEDED，最大线速度指令均达到 **1.0 m/s**：

| 场景 | 旧耗时 (s) | 当前耗时 (s) | 当前真值误差 (m) | 当前车体最小采样净距 (m) |
| --- | ---: | ---: | ---: | ---: |
| 1 m 箱体，(9,0)→(0,4) | 103.722 | 39.867 | 0.1988 | 0.2167 |
| 1 m 箱体，(0,4)→(0,0) | 43.717 | 16.469 | 0.1985 | 0.2907 |
| 0.5 m 箱体，(0,0)→(6,0) | 65.214 | 10.819 | 0.1957 | 0.2705 |

耗时分别减少 61.6%、62.3%、83.4%。由真值轨迹计算，第一项行驶距离从 17.26 m
降至 13.20 m，累计航向变化从 14.45 rad 降至 4.54 rad；第三项距离从 7.73 m
降至 6.22 m，累计航向变化从 8.24 rad 降至 3.21 rad，反复转向减少。
这是全部参数与算法调整的综合结果，未进行逐项消融对比。
净距为车身矩形含 padding 与实际箱体矩形的采样分离距离；当前测试仅覆盖这些静态箱体。

- [真实轨迹图](../results/avoidance_1mps/trajectories.png)
- [完整前后对比](../results/avoidance_1mps/comparison.json)
- [汇总与对应 JSON 报告](../results/avoidance_1mps/summary.json)
- [测试参数快照](../results/avoidance_1mps/tested_indoor_nav.yaml)
- [完整仿真日志](../results/avoidance_1mps/simulation.log)
- [车身角部及路径复用回归](../results/detour_footprint_1mps.log)
- [Theta* 地址/未定义行为检查](../results/theta_1mps_regression.log)

## 构建及独立性

- 从系统 ROS 环境构建，21 个包全部完成（约 3 分 52 秒）。
- `scripts/check.sh` 通过：启动参数可解析，自定义导航和模型包均来自本目录安装结果。
- 生成的 setup 链仅包含系统 ROS 和本工作空间，不依赖旧工程的 install。
- 新旧 `simple_car.urdf.xacro` 的 SHA256 完全相同，保留原松灵仿真模型。
- Python 语法与 Bash 语法检查通过。

构建日志：[build.log](../results/build.log)；资源及安装检查：[check.log](../results/check.log)。

## 地图和规划

12 节点、12 双向边，全图连通。路网中心线至矩形墙体最小采样距离 1.900 m，
车体外接半径加 padding 0.379 m。运行中的 C++ SNN 规划器完成 144/144 组节点对规划，
包括同起终点，并校验返回路径首尾位置。

## 最终参数下的运行结果

运行命令：`./scripts/run_sim.sh gui:=false use_rviz:=false`，然后执行
`./scripts/validate.sh --all`。每项要求导航状态 SUCCEEDED、非空规划路径、
Gazebo 真值终点误差不大于 0.3 m；箱体测试还要求收到非空 Theta* 路径。

| 测试 | 目标 (m) | 墙钟耗时 (s) | Gazebo 真值终点误差 (m) | 结果 |
| --- | --- | ---: | ---: | --- |
| 箱体绕障 | (6,0) | 46.020 | 0.1990 | 通过，收到 104 点 Theta* 路径 |
| 移除箱体后回原点 | (0,0) | 36.917 | 0.2024 | 通过 |
| 直线 | (3,0) | 21.213 | 0.1974 | 通过 |
| 多段与转弯 | (12,8) | 67.520 | 0.1978 | 通过 |
| 返回起点 | (0,0) | 84.820 | 0.1968 | 通过 |

所有任务最大线速度指令为 0.30 m/s。前向停车保护和运动碰撞预测均开启。
运行结束已停止测试仿真，可通过 run_sim.sh 重新启动。

- [完整验证输出](../results/validation_20261102_170410.log)
- [箱体绕障报告](../results/navigation_20261102_170503_266694.json)
- [回原点报告](../results/navigation_20261102_170541_616324.json)
- [三段路线报告](../results/navigation_20261102_170836_607097.json)

## 调整依据及覆盖边界

初次箱体测试在较小膨胀缓冲下失败：虽生成绕行路径，但小车在进入绕行前被停车逻辑拦停，
随后进度检查失败。室内配置改为 1.0 m 膨胀半径、2.0 衰减系数、规划阈值 100，
并关闭额外路径平滑后，同场景箱体测试和基础路线全部通过。原工程与算法源码未因此修改。

本次验证覆盖默认室内场景及单个静态箱体，不包含实车定位、动态行人、定位/点云失效注入、
自动重定位和全程碰撞接触监测。到达与绕障路径检查不构成整段路线零碰撞的独立证明。

## 完整路网显示与目标命令修复

新增完整路网显示节点、默认俯视 RViz 配置和 `scripts/run_display.sh`。
完整路网与室内背景使用 Reliable / Transient Local 的 MarkerArray 发布，
不依赖导航任务触发。RViz 配置同时显示完整路网和本次规划路线。

执行 `./scripts/validate.sh --display`，在独立 ROS 域 188 验证通过：

- 默认地图生成 12 个节点、12 个编号、12 条边，以及 5 个墙体/建筑块和地面。
- 单向边显示方向箭头。
- 后启动的订阅者在未发目标、未执行定时重发时仍可收到完整显示数据。
- `goal.sh 3.0` 在创建 ROS 节点前退出，并提供中文的双坐标用法。
- `goal.sh 3 0` 向隔离的测试服务正确提交 map 坐标 (3,0)，打印目标受理/状态并生成报告。

最后一项使用模拟导航服务测试命令行接口，不向用户当前运行的仿真发送任务，
也不作为新增的实车或仿真行驶验证。上述更新未修改规划和运动控制算法。

## 启动权限及地图边界修复

发现 `road_network_display.py` 源文件缺少执行权限，ament 的符号链接安装使 launch_ros
无法把它解析为可执行程序，从而触发整套启动流程关闭。已修正源文件执行权限，
并在 run_sim.sh、run_display.sh 和 check.sh 中加入与 launch_ros 相同的可执行程序检查。
显示节点的回归测试现在通过实际安装路径启动 display.launch.py，验证发布及正常关闭。

实际交互运行还暴露了 Theta* 在局部地图边界附近的崩溃。补充起终点坐标转换失败检查、
地图边界访问检查，并重置每次搜索的指针状态；修正搜索队列耗尽/同格终点的返回逻辑。
独立 ASan/UBSan 回归覆盖孤立起点、不可达墙体、100 次失败/恢复循环、地图外坐标、
边界 LOS、同格终点及窗口尺寸变化。修复前复现越界崩溃，修复后全部通过。

日志：[修复前](../results/theta_failure_before_fix.log)、
[修复后](../results/theta_failure_after_fix.log)、
[可执行程序检查](../results/check_startup_fix.log)。

修复后实际打开 Gazebo 与 RViz，从 `(9,8)` 朝 +X 出发，执行到 `(9,0)` 的转弯路线：
SUCCEEDED，耗时 63.414 s，Gazebo 真值终点误差 0.1985 m，最大线速度指令 0.30 m/s。
实际 C++ 路网规划器再次通过 144/144 节点对检查，运行期间未出现控制器退出。
本次保留界面供继续测试。
报告：[navigation_20261102_183126_848107.json](../results/navigation_20261102_183126_848107.json)。

## 截图中的大箱体绕障修复

采集用户运行中的模型位置和真实 costmap，箱体为 Gazebo unit_box，中心约
`(0.251,1.637)`、边长 1 m。原阈值 100 使该 costmap 中目标不可达；
阈值 140 能找到通路。相同 costmap 已在实际 C++ Theta* 中离线重放验证。

同时修复了偏离路网后从历史起点选汇入点、局部目标跨越过长路网弧长，以及无解时
继续跟踪已受阻全局路径的问题。普通室内导航关闭门区专用近点汇入，
局部目标限制在前方 4 m；停车保护、碰撞预测与后退次数限制仍开启。

运行 `./scripts/validate.sh --avoidance-isolated`，在 ROS 域 189 / Gazebo master 11356
启动独立真实仿真，避免 GUI 目标覆盖。以下三项全部成功，最大线速度指令 0.30 m/s：

| 场景 | 起点→目标 (m) | 耗时 (s) | 终点真值误差 (m) | 车体至箱体最小采样净距 (m) |
| --- | --- | ---: | ---: | ---: |
| 1 m 箱体，长距离接近后绕行 | (9,0)→(0,4) | 103.722 | 0.1986 | 0.5503 |
| 1 m 箱体，反向绕行 | (0,4)→(0,0) | 43.717 | 0.1991 | 0.5567 |
| 0.5 m 箱体 | (0,0)→(6,0) | 65.214 | 0.2001 | 0.6262 |

净距根据 Gazebo 模型真值位姿、包含 padding 的实际车体多边形与箱体矩形计算，
不是中心线距或规划器自报距离。每项要求到达、收到 Theta* 绕障路径及最小采样净距 >0.02 m。
这次不包含连续接触监测或任意新障碍尺寸/位置的全面验证。

- [实际轨迹图](../results/isolated_avoidance/trajectories.png)
- [汇总](../results/isolated_avoidance/summary.json)
- [1 m 正向报告](../results/isolated_avoidance/navigation_20261102_185954_643631.json)
- [1 m 反向报告](../results/isolated_avoidance/navigation_20261102_190039_843074.json)
- [0.5 m 报告](../results/isolated_avoidance/navigation_20261102_190151_064520.json)
- [原现场 costmap 重放及地址检查](../results/avoidance_unit_test.log)
