# 钢镚四足机器狗上层 ROS2 架构方案

# 摘要：核心判断与选型一览

本方案为智身科技「钢镚」四足机器狗（12 自由度，开放运动/状态/视频流 SDK）设计上层 ROS2 软件架构，运行于 RK3588S 主控机（Ubuntu 22.04 / aarch64），感知主传感器为 Livox Mid-360S。核心判断：采用 **ROS2 Humble** + 四层解耦架构（驱动接入 → 状态估计与感知 → 导航规划 → 应用交互）；底盘接入通过**智身 SDK 桥接节点**（genisom_bridge）；SLAM 采用 **FAST-LIO2**；导航采用 **Nav2**；仿真采用智身开源 **MATRiX** 平台。

| 决策点 | 本方案选型 | 依据简注 |
|-|-|-|
| ROS 发行版 | **ROS2 Humble** | Ubuntu 22.04 官方支持，LTS 支持期长，apt 直接安装 |
| 底盘接入 | **genisom_bridge（智身 SDK 桥接）** | 钢镚开放整机 SDK：运动接口、状态接口、视频流接口，走以太网 |
| 雷达驱动 | **livox_ros_driver2（≥1.2.6）** | 官方驱动原生支持 Mid-360S（MID360s_config.json），适配 Humble |
| 定位建图 | **FAST-LIO2** | 激光-惯性实时里程计与建图，与 Mid-360S 内置 IMU 匹配度高 |
| 状态融合 | **robot_localization（EKF）** | 融合机身 IMU、腿部里程计、激光里程计，输出稳定 /odom |
| 导航 | **Nav2（MPPI 局部控制）** | 开源可维护；MPPI 对四足运动与台阶通过更友好 |
| 仿真验证 | **MATRiX（智身开源仿真）** | MuJoCo+UE5，数据接口与 ROS2 消息兼容，真机未到时先行开发 |

**下一步建议：**先完成 M0 环境搭建与 M1 底盘桥接，其中第一步是获取「L1 Maker 资源包」核验 SDK 形态（详见第 7 节）。

# 背景与目标

## 平台现状

**主控机（已实测确认）：**RK3588S，8 核 CPU，7.7G 内存，105G 可用磁盘；Ubuntu 22.04.5 LTS（aarch64）；Python 3.10；linaro 免密 sudo；ROS 尚未安装，网络可达 packages.ros.org（ROS2 仓库返回 200）；板载 can0 接口（足式底盘不涉及 CAN 通信，保留备用）。

**机器人本体：**智身科技「钢镚 L1」（ZSL-1）仿生四足机器人，12 自由度，标配 IMU；行业首个标配 AI 强化学习运控算法的量产机器人，支持 16cm 台阶持续攀爬、40° 极限爬坡、8kg 持续行走负载、IP54 防护；开放全整机 SDK（运动接口、视频流接口、状态接口）与通信协议；功能拓展接口含以太网口、USB、SBUS、UART；EDU 系列提供高达 100 TOPS 算力。

**感知传感器：**Livox Mid-360S：360° 水平 ×（-7°\~+52°）垂直视场，20 万点/秒，10Hz 典型帧率，40m@10% 反射率典型探测，内置 IMU（ICM40609），100BASE-TX 以太网，支持 PTPv2 / GPS 同步，IP67，265g。

## 范围与边界

- **本期做：**上层 ROS2 软件栈——驱动接入、状态估计、定位建图、导航规划、应用交互与仿真联调。
- **本期不做：**关节级运控算法（由机身内置强化学习运控完成）、硬件本体改造、遥控器固件。
- **传感器范围：**Mid-360S 为感知主传感器；机身相机仅做图传接入；GNSS 视室外定位需求后续评估。

# 总体架构

架构分四层，另有仿真与工具链支撑。数据上行：硬件 → 驱动接入层（ROS 化）→ 状态估计与感知层（定位建图）→ 导航规划层（Nav2）→ 应用交互层；控制下行：应用/导航 → genisom_bridge → 机身执行。

<html5-block alt="钢镚四足机器狗 ROS2 上层分层架构与数据流示意图，含待核验项标注" data-ref="html5_1"></html5-block>

图中橙色虚线标注两项待确认：智身 SDK 协议形态、雷达外参标定。Nav2 代价地图可直接订阅点云话题，数据流不强制经过感知层。

# 分层设计

## 驱动接入层

职责：把硬件数据 ROS 化，把 ROS 指令送到底盘。包含三个节点：

- **livox_ros_driver2（≥1.2.6）：**官方驱动，原生支持 Mid-360S；使用 MID360s_config.json 配置，提供 rviz_MID360s_launch.py 启动示例；输出点云与 IMU 话题。
- **genisom_bridge（智身 SDK 桥接）：**订阅 /cmd_vel 映射到 SDK 运动接口（速度 + 步态/动作），将状态接口转换为 /odom/leg、/imu/data_raw、/joint_states、/robot_state，将视频流接口转发为图像话题。待核验：SDK 是否自带 ROS 封装、通信协议与端口、是否提供里程计与关节角。
- **时间同步：**Mid-360S 支持 PTPv2 / GPS 同步，需评估主控机网卡 PTP 能力；不可用时采用软件同步与时间戳校正。

## 状态估计与感知层

职责：回答“我在哪、周围是什么”。

- **FAST-LIO2：**激光-惯性里程计与实时建图，与 Mid-360S 内置 IMU 配合；备选 Point-LIO2、LIO-SAM。
- **robot_localization（EKF）：**融合机身 IMU、腿部里程计（来自 SDK 状态接口）、激光里程计，输出 /odom 与 /odom/filtered。
- **外参标定：**lidar ↔ imu ↔ base_link 的静态变换，先按 CAD 量测初值，再用标定工具精化。

## 导航规划层

职责：给定目标点，规划路径并输出安全速度指令。

- **Nav2 组件：**全局规划（NavFn / Smac 规划器）、代价地图（global / local，3D 点云投影 2D）、行为树（BT Navigator 编排与恢复）、局部控制。
- **局部控制器：**优先 MPPI（对四足运动学与台阶通过更友好）；备选 Regulated Pure Pursuit。
- **四足适配要点：**代价地图膨胀半径与步态速度匹配；最大速度/加速度按步态设定；16cm 台阶通过性需体现在 costmap 与路径生成中；跌倒/恢复行为接入机身状态接口。

## 应用与交互层

职责：人机接口与任务编排。

- **遥控 / 上位机：**手柄（joy）、Web / 桌面端、RViz2 可视化。
- **任务调度状态机：**巡检 / 巡航 / 应急等场景编排与切换。
- **视频流 / 图传：**机身相机画面推流与显示。

# 关键接口与数据流

## Topic 清单（设计建议，最终以 SDK 核验结果为准）

| 话题 | 消息类型 | 方向 | 说明 |
|-|-|-|-|
| /cmd_vel | geometry_msgs/Twist | 输入 | 速度指令，genisom_bridge 消费 |
| /goal_pose | geometry_msgs/PoseStamped | 输入 | 导航目标点 |
| /livox/lidar | sensor_msgs/PointCloud2 | 输出 | Mid-360S 点云（livox driver） |
| /livox/imu | sensor_msgs/Imu | 输出 | 雷达内置 IMU |
| /imu/data_raw | sensor_msgs/Imu | 输出 | 机身 IMU（SDK 状态接口） |
| /odom/leg | nav_msgs/Odometry | 输出 | 腿部里程计（待核验 SDK 是否提供） |
| /odom | nav_msgs/Odometry | 输出 | EKF 融合后的里程计 |
| /joint_states | sensor_msgs/JointState | 输出 | 12 关节角（驱动 tf 与可视化） |
| /robot_state | 自定义消息 | 输出 | 电量 / 模式 / 步态 / 错误码 |
| /map | nav_msgs/OccupancyGrid | 输出 | FAST-LIO2 建图结果 / 代价地图 |
| /global_costmap、/local_costmap | nav2_msgs/Costmap | 输出 | Nav2 代价地图 |
| /plan | nav_msgs/Path | 输出 | 全局路径 |
| /video | sensor_msgs/Image | 输出 | 机身相机图传（image_transport） |
| /tf、/tf_static | tf2_msgs/TFMessage | 输出 | 坐标变换 |

## tf 树

```text
camera_init ← FAST-LIO2 全局系（SLAM 启动原点）
   ↑ FAST-LIO2: /Odometry + TF(camera_init→body)
body（雷达/IMU 共体系，≈livox_frame）
   ↑ genisom_bridge 静态 TF（base_link→livox_frame，外参待标定）
base_link ← 狗 SDK 车体系（前X 左Y 上Z）
   ↑ genisom_bridge 动态 TF（odom→base_link，10Hz）
odom ← 狗 SDK 上电原点系
```

## 坐标与单位约定

速度单位 m/s，角速度 rad/s，角度 rad；base_link 取机身几何中心，x 轴朝前、y 轴朝左、z 轴向上；所有节点统一使用 ROS2 时间（sensor time 优先）。

# 关键选型与取舍

| 决策点 | 本方案 | 备选 | 取舍依据 |
|-|-|-|-|
| ROS 发行版 | ROS2 Humble | ROS1 Noetic | 22.04 官方支持、长期维护；Noetic 在 22.04 需源码编译或容器，维护成本高 |
| 定位建图 | FAST-LIO2 | slam_toolbox（2D 降维）、Point-LIO2 | FAST-LIO2 原生利用 Mid-360S 3D 点云与 IMU，鲁棒性与精度匹配；slam_toolbox 需降维，丢失地形信息 |
| 导航框架 | Nav2 | 智身 RoamerX / rmx_lite | Nav2 开源可维护、可调参数；rmx_lite 为厂商闭源框架（IROS 2025 冠军队伍采用），若追求开箱可评估，本方案默认 Nav2 |
| 局部控制 | MPPI | Regulated Pure Pursuit、DWB | MPPI 采样式控制更适应四足运动学与台阶通过 |
| 仿真平台 | MATRiX | Gazebo / Isaac Sim | 智身官方开源，含钢镚本体模型与 ROS2 兼容接口，真机未到时开发验证成本最低 |
| 状态融合 | robot_localization | 手写 EKF | 成熟、可配置、社区维护 |

**取舍说明：**ROS2 而非 ROS1——主控机系统为 Ubuntu 22.04，Humble 为官方支持版本且 LTS 到 2027，社区与工具链（Nav2、Foxglove）均以 ROS2 为主。FAST-LIO2 而非 2D SLAM——四足机器狗场景存在台阶、坡道等三维地形，2D 降维会丢失通过性信息；FAST-LIO2 提供 6DOF 里程计，天然支撑后续复杂地形规划。Nav2 而非厂商框架——保持上层自主可控；若后续厂商 RoamerX 提供稳定开箱能力，可作为对照方案再评估。

# 实施路线

| 阶段 | 内容 | 验收标准 |
|-|-|-|
| **M0 环境搭建** | 安装 ROS2 Humble、colcon、工具链；搭建工作区；接入 Mid-360S 与 livox_ros_driver2 | ros2 doctor 通过；雷达点云在 RViz2 正常显示 |
| **M1 底盘桥接** | 获取 L1 Maker 资源包核验 SDK；实现 genisom_bridge：cmd_vel→运动接口、状态→odom/imu/joint | RViz2 中 tf 树正确；键盘/手柄控制行走；里程计与 IMU 话题正常 |
| **M2 标定与同步** | lidar↔imu↔base_link 外参标定；PTPv2 / 软件时间同步 | 点云与机身姿态对齐；多传感器时间戳一致 |
| **M3 定位建图** | 集成 FAST-LIO2，联调 EKF 融合 | 室内/室外建图质量合格；定位漂移可接受 |
| **M4 自主导航** | Nav2 部署与四足参数适配（costmap、MPPI、行为树） | 目标点自主导航、避障、台阶/坡道可通过 |
| **M5 应用与仿真** | 任务状态机、图传/遥操；MATRiX 仿真-真机联调 | 仿真与真机行为一致；完整巡检任务闭环 |

**当前优先：**M0 + M1。第一步是获取「L1 Maker 资源包」（智身官方下载中心提供），核验 SDK 提供的接口、协议与示例，M1 才能落地。

# 风险与待确认事项

| 事项 | 影响 | 待办 |
|-|-|-|
| 智身 SDK 形态与协议（最大不确定项） | 决定 genisom_bridge 实现方式与里程计/关节数据可得性 | 下载 L1 Maker 资源包，核对 SDK 文档、通信协议、是否提供 ROS 封装 |
| 主控机连接拓扑 | 确定以太网接线、IP 规划、雷达接口 | 确认 RK3588S 为外挂主控还是机身 EDU 板；雷达接网口或 USB |
| 雷达安装位姿与外参 | 影响点云对齐与定位精度 | 确定安装位置，量测初值 + 标定工具精化 |
| 算力预算 | RK3588S 8 核跑 FAST-LIO2 + Nav2 需实测 | M0 阶段做点云/里程计负载压测；必要时评估 100 TOPS EDU 板 |
| GNSS 需求 | 影响室外大范围定位方案（L2 已含 LiDAR+GNSS 厘米级定位） | 确认作业场景是否包含开阔室外长距离任务 |
| 时间同步 | PTPv2 依赖网卡能力 | 评估主控机以太网 PHY 的 PTP 支持；不可用则软件同步 |

# 参考来源

- [智身科技官网：钢镚 L1 产品页（12 自由度、8kg 负载、40° 爬坡、SDK 开放、以太网/USB/SBUS/UART 接口）](https://www.genisomai.com/product-robot/L1)
- [智身科技下载中心：钢镚 L1 使用手册、EDU 介绍、L1 Maker 资源包](https://www.genisomai.com/download.html)
- [智身科技官网首页（产品矩阵、RoamerX 智航系统）](https://www.genisomai.com/)
- [新华网江苏频道：IROS 2025 四足挑战赛冠军队伍采用钢镚 L1 + RoamerX/rmx_lite 导航框架](http://www.js.xinhuanet.com/20251024/1af9b1dbaa634716b8c84b7866c93ae9/c.html)
- [机器人大讲堂：MATRiX 仿真平台开源，数据接口与 ROS2 兼容](https://www.leaderobot.com/news/6550)
- [OpenELAB：Mid-360S 与 Mid-360 规格对比（40m@10%、200k 点/秒、IMU、PTPv2、驱动支持矩阵）](https://openelab.io/blogs/learn-2/livox-mid-360-vs-mid-360s-whats-the-difference-and-which-should-you-choose)
- [OpenELAB：Mid-360S + livox_ros_driver2 + FAST-LIO 部署指南](https://openelab.io/blogs/learn/livox-mid360s-ros2-sdk2-fast-lio-mapping-guide)
- [livox_ros_driver2 仓库（Ubuntu 22.04 / ROS2 Humble 构建说明）](https://gitee.com/xlhou/livox_ros_driver2)

# 落地实测与坐标系统一（M1–M3）

## 里程碑落地状态

| 里程碑 | 状态 | 关键产物与实测结论 |
|-|-|-|
| M0 环境 | ✅ 完成 | ROS2 Humble Desktop + livox_ros_driver2（Mid-360S）编译通过；修复出厂 apt-mark hold（2368 包）、换 nju 全量源、libjsoncpp 降级 |
| M1 网络/SDK | ✅ 完成 | eth0 双静态 IP（192.168.1.150 雷达网段 + 192.168.168.150 狗网段）；AgiBot D1 Edu Ultra 实为智元 ZSL-1，SDK 通讯打通（checkConnect=True，电量/姿态/关节真实数据）；狗侧三处配置已改并备份 |
| M2 桥接节点 | ✅ 完成 |  |
| M3 SLAM | ✅ 完成 | FAST-LIO2（ROS2 分支）编译并运行，/Odometry /cloud_registered /path 输出正常，IMU 姿态收敛；启动须用 ros2 launch（直接 run 缺 extrinsic 参数会段错误） |

## 坐标系统一方案

**全局系选择：**以 FAST-LIO2 的 `camera_init`（SLAM 启动原点）为全局定位系，供 Nav2 建图与导航使用；狗 SDK 的 `odom` 作为局部运动先验（可经 robot_localization 融合或作 fallback）。

- `camera_init`：FAST-LIO2 全局系，启动时原点即当前位姿
- `body`：FAST-LIO2 车体系（雷达/内置 IMU 共体，语义等价 `livox_frame`）
- `base_link`：狗 SDK 车体系，前 X / 左 Y / 上 Z（与 REP-103 一致）
- `odom`：狗 SDK 上电原点系，bridge 10Hz 发布动态 TF

## 外参标定：几何量测法（已完成）

标定目标：`base_link → livox_frame` 的 6 自由度静态外参，写入 genisom_bridge 的 launch 参数。FAST-LIO2 侧 `extrinsic_T/R` 保持 mid360.yaml 出厂值（Mid-360 内置 IMU 与雷达共体，出厂已标定，无需改动）。

| 量测项 | 符号 | 参考点与方向 | 实测值 |
|-|-|-|-|
| 前后平移 | x (m) | 雷达中心相对 base_link 原点，狗头方向为 + | 0.25 |
| 左右平移 | y (m) | 左侧为 + | 0.00 |
| 高度 | z (m) | 向上为 +；取站立标称姿态（趴下时 0.17 不作外参） | 0.45 |
| 横滚 | roll (rad) | 绕 X 轴 | 0.00 |
| 俯仰 | pitch (rad) | 绕 Y 轴，抬头为 +；实测前倾 45° 取负值 | -0.785 |
| 偏航 | yaw (rad) | 绕 Z 轴 | 0.00 |

**量测参考：**base_link 原点取狗躯干几何中心（前后腿中点、左右对称面、地面为基准）；若无法精确确定 SDK 原点，先按躯干中心近似，标定后现场微调。写入位置：genisom_bridge/launch/bridge.launch.py 的 lidar_x/y/z 与 lidar_roll/pitch/yaw（弧度），改后重启节点即可生效。

# 开发状态与后续任务（M0–M5 实测追踪）

## 一、当前开发进度

| 阶段 | 内容 | 状态 | 验证情况 |
|-|-|-|-|
| M0 环境搭建 | ROS2 Humble + 依赖修复（鱼香ROS一键安装、apt源修复） | ✅ 完成 | ros-humble-desktop 安装成功，ffmpeg/rosbridge 依赖就绪 |
| M1 网络与 SDK | 有线双网段（雷达 192.168.1.150 + 狗 192.168.168.150）、AgiBot SDK 获取配置与 demo 跑通 | ✅ 完成 | 雷达 192.168.1.179、狗 192.168.168.168:43997 可达；SDK 连接即夺控制权（遥控器失效属正常） |
| M2 底盘桥接 | genisom_bridge（SDK→ROS2 桥接，standUp 方法名修复） | ✅ 完成 | cmd_vel→SDK→狗真实移动验证；站立/趴下等动作服务可用 |
| M3 建图与坐标 | FAST-LIO2（mid360.yaml）编译运行；外参几何量测标定；TF 树全链 | ✅ 完成 | /cloud_registered 10Hz；map→odom→camera_init→body→base_link 可解；外参 T=(0.25,0,0.45) RPY=(0,-45°,0) |
| M4 导航 | Nav2 接入：/scan 10Hz 进 costmap、/cmd_vel 通路、目标下发/取消 | ⚠️ 部分完成 | Nav2 节点全 active、可下发目标，但 amcl 位姿剧烈跳变（地图质量差），导航闭环未打通 |
| M5 网页上位机 | 首页 + 建图/导航/地图/设置 四子页 SPA；后端代理 + 管理 API | ✅ 部署完成（待现场实测） | http://192.168.122.152:8080；JS库本地化；建图保存全自动链路已接入；/api/maps、/api/sysinfo 验证通过 |

## 二、当前系统运行状态

- 访问入口：页面 http://192.168.122.152:8080（电脑浏览器）；管理 API :8090；ROS WebSocket :9090
- 运行服务：livox 驱动 / genisom_bridge / FAST-LIO2 / lio_tf_bridge / rosbridge / web_ops 全部运行中（导航 Nav2 未启动）
- 地图：maps/map.pgm（139×332 @ 0.05m，早期建图，墙线不闭合含噪点，需重建）
- 外参：base_link→livox_frame T=(0.25, 0, 0.45)m、RPY=(0, -45°, 0)（站立标称、前倾低头）；body→base_link 逆变换已配
- SDK 关键机制：SDK 连接期间遥控器失效（文档 FAQ 确认）；SDK 指令中断约 3s 自动趴下（安全机制）

## 三、遗留问题

1. M4 导航闭环未打通：amcl 定位跳变（yaw -83°\~-125°\~-43°、位置跳 0.3m），根因为地图质量差（墙线不闭合、噪点多），需重建地图后复测
2. 视频流未接入：狗相机 RTSP 绑定狗 AP 网段（192.168.234.1:8554），主控有线网段访问不通，前端为占位提示
3. 建图保存全自动链路（SIGINT→PCD→PGM）已脚本化但未在现场实测
4. SDK 更多机身参数（关节角度/步态/温度等）尚未接入上位机设置页
5. 地图编辑保存后 Nav2 热切换逻辑待实测（需重启 Nav2 生效）

## 四、后续任务清单

### P0 导航闭环（当前主线）

- [ ] 现场重建地图：上位机「建图」页 开始建图 → 场地内匀速遛狗覆盖 → 保存为 2D 地图，验证一键保存链路并产出闭合墙线新地图

- [ ] M4 导航闭环复测：加载新地图 → 设定初始位姿 → 下发导航目标 → 验证执行/到达/取消

- [ ] 上位机现场实测：连接、动作面板（站立/趴下）、手动控制、急停按钮全流程

### P1 功能完善

- [ ] 视频流接入：主控加 192.168.234.x 路由到狗 AP 或狗侧配置 RTSP 绑定 eth0，前端视频面板启用

- [ ] 地图管理实测：地图页编辑（画笔/橡皮）→ 保存 → 导航页切换新图 → Nav2 使用验证

### P2 扩展能力

- [ ] SDK 机身参数接入设置页（关节角度、步态参数、温度等，按 SDK 接口扩展）

- [ ] 多机/编队扩展：如后续部署多台，上位机支持多机选择与独立控制

- [ ] 远程诊断完善：日志归档、异常告警、服务自恢复

## 五、操作安全约束

<callout emoji="💡">
移动类操作必须现场监护（此前试验曾险些撞物）；手动控制限速 0.3m/s；SDK 连接期间遥控器失效，恢复遥控需停止 bridge；建图/导航前确认场地安全。
</callout>