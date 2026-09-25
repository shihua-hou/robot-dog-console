# 钢镚四足机器狗上位机（robot_ws）

智元 D1 Edu-Ultra + Livox Mid-360S + FAST-LIO2 + Nav2 + Web 上位机。

## 架构简述

- **底盘**：`genisom_bridge` → AgiBot HighLevel SDK（`publish_odom_tf:=false`，导航时不发 SDK 里程计）
- **建图**：Livox → FAST-LIO2 → `map_building_node`（`/map_building` 实时 2D 预览）→ 保存时 `pcd2pgm` → `maps/*.pgm`
- **定位/导航**：`/scan`（pointcloud_to_laserscan）+ AMCL + Nav2；里程计来自 `lio_tf_bridge`（`/odom` / `/odom_nav`）
- **上位机**：`web_ui/` 静态页 + rosbridge `:9090` + `web_ops_node.py` `:8090`

## 一键启动

```bash
bash /home/linaro/robot_ws/start_ui_stack.sh
```

### 开机自启（网页控制台）

重启后自动拉起静态页 `:8080`、`web_ops` `:8090`、`rosbridge` `:9090`（不自动开建图整链）：

```bash
sudo bash /home/linaro/robot_ws/scripts/install_web_console_service.sh
```

常用：

```bash
systemctl status dog-web-console
sudo systemctl restart dog-web-console
sudo systemctl disable dog-web-console   # 取消自启
```

浏览器请用主控 **WLAN（wlan0）** IP，不要用 eth0 / 交换机网段：

```text
http://192.168.122.152:8080/          # 示例：本机 wlan0
ws://192.168.122.152:9090             # rosbridge
```

`eth0` 上的 `192.168.1.x` / `192.168.168.x` 是给 Livox / 狗 SDK 用的。

界面交互（参考 wheeltec v2）：
- 首页驾驶舱入口
- 建图/导航全屏舞台
- **左摇杆 = 移动（vx/vy）**，**右摇杆 = 转向（wz）**
- 导航：武装按钮后，地图上按住拖朝向

## 常用脚本

| 脚本 | 作用 |
|------|------|
| `start_all.sh` | Livox + SDK 桥 + FAST-LIO + lio_tf + map_building |
| `start_scan_node.sh` | `/cloud_registered` → `/scan` |
| `nav2_start.sh [map.yaml]` | 启动 Nav2（会自动拉起 scan） |
| `start_ui_stack.sh` | 以上 + rosbridge + web_ops + HTTP 8080 |
| `scripts/start_web_console.sh` | 仅网页三件套（8080/8090/9090） |
| `scripts/install_web_console_service.sh` | 安装 `dog-web-console` 开机自启 |

## 上位机流程

1. **建图**：建图页 → 开始建图 → 遥控覆盖场地 → 看 `/map_building` 预览 → 保存为 2D 地图  
2. **定位**：导航页选地图 → 启动 Nav2 → 地图上拖拽设初始位姿 → 等待 AMCL  
3. **导航**：设目标（含朝向）→ 开始导航；急停零速并取消目标  

## 网络提示

- 狗端有线：`192.168.168.168`（SDK）/ 本机口常为 `192.168.168.150`
- Livox：见 `src/livox_ros_driver2/config/MID360s_config.json`
- 上位机浏览器访问板子局域网 IP（设置页可改）

## 编译

```bash
cd /home/linaro/robot_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select genisom_bridge lio_tf_bridge nav2_tools
source install/setup.bash
```
