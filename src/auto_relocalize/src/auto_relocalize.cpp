/*
 * auto_relocalize: 设初始位姿后用 ICP 把激光贴到地图黑障碍，再交给 AMCL。
 *
 * 流程:
 *   1. 用户「设初始位姿」→ /initialpose → 本节点收到 seed
 *   2. 当前 /scan 与地图占用格点云做 2D-ICP（多航向初值）
 *   3. 用命中率/未知占比校验，通过则发布精修位姿
 *   4. 启动不做全局瞎搜；无初值时提示用户设姿
 */
#include <cmath>
#include <vector>
#include <mutex>
#include <algorithm>

#include <Eigen/Geometry>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

struct Candidate {
  double x = 0, y = 0, yaw = 0;
  double fitness = 1e9;  // ICP 均方距离，越小越好
  double hit = 0.0;
  double unk = 0.0;
  bool ok = false;
};

class AutoRelocalize : public rclcpp::Node {
public:
  AutoRelocalize() : Node("auto_relocalize") {
    min_hit_ratio_ = declare_parameter<double>("min_hit_ratio", 0.55);
    max_unk_ratio_ = declare_parameter<double>("max_unk_ratio", 0.30);
    hit_dist_ = declare_parameter<double>("hit_dist", 0.22);
    max_beam_range_ = declare_parameter<double>("max_beam_range", 8.0);
    local_radius_ = declare_parameter<double>("local_radius", 4.0);
    search_xy_ = declare_parameter<double>("search_xy", 1.5);      // 初值附近平移搜索半径
    xy_step_ = declare_parameter<double>("xy_step", 0.20);
    icp_max_corr_ = declare_parameter<double>("icp_max_corr", 0.55);
    icp_max_iter_ = declare_parameter<int>("icp_max_iter", 50);
    icp_fitness_max_ = declare_parameter<double>("icp_fitness_max", 0.10);
    yaw_span_deg_ = declare_parameter<double>("yaw_span_deg", 45.0);
    yaw_step_deg_ = declare_parameter<double>("yaw_step_deg", 8.0);
    watchdog_en_ = declare_parameter<bool>("watchdog_en", true);
    watchdog_hit_ = declare_parameter<double>("watchdog_hit", 0.40);
    watchdog_count_ = declare_parameter<int>("watchdog_count", 4);

    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", rclcpp::QoS(1).transient_local().reliable(),
        [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) { onMap(msg); });
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lk(scan_mutex_);
          scan_ = msg;
        });
    seed_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", 10,
        [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg) {
          onSeedPose(msg);
        });
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", 10);
    status_pub_ = create_publisher<std_msgs::msg::String>("/relocalize_status", 10);
    srv_ = create_service<std_srvs::srv::Trigger>(
        "relocalize",
        [this](std_srvs::srv::Trigger::Request::ConstSharedPtr,
               std_srvs::srv::Trigger::Response::SharedPtr res) {
          Candidate c;
          bool ok = false;
          if (have_seed_) {
            ok = refineNear(c, seed_x_, seed_y_, seed_yaw_);
          } else if (lookupCurrentPose(seed_x_, seed_y_, seed_yaw_)) {
            have_seed_ = true;
            ok = refineNear(c, seed_x_, seed_y_, seed_yaw_);
          }
          res->success = ok;
          if (ok) {
            publishPose(c);
            res->message = "对齐成功 hit=" + std::to_string(c.hit) +
                           " fitness=" + std::to_string(c.fitness);
            publishStatus("aligned");
          } else {
            res->message =
                "附近未贴墙 — 请粗略设初值(真位姿在附近即可)，松手后平移+旋转搜索";
            publishStatus("need_init");
          }
        });

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    watchdog_timer_ = create_wall_timer(std::chrono::seconds(3),
                                        [this]() { watchdogCheck(); });

    RCLCPP_INFO(get_logger(),
        "重定位就绪: 初值附近平移±%.1fm + 旋转搜索, 需红激光贴黑墙 hit≥%.0f%%",
        search_xy_, min_hit_ratio_ * 100.0);
    publishStatus("need_init");
  }

private:
  using Cloud = pcl::PointCloud<pcl::PointXYZ>;

  void publishStatus(const std::string &s) {
    std_msgs::msg::String m;
    m.data = s;
    status_pub_->publish(m);
  }

  bool haveScan() {
    std::lock_guard<std::mutex> lk(scan_mutex_);
    return scan_ != nullptr;
  }

  bool lookupCurrentPose(double &x, double &y, double &yaw) {
    try {
      auto tf = tf_buffer_->lookupTransform("map", "base_footprint", tf2::TimePointZero);
      x = tf.transform.translation.x;
      y = tf.transform.translation.y;
      const auto &q = tf.transform.rotation;
      yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                       1.0 - 2.0 * (q.y * q.y + q.z * q.z));
      return true;
    } catch (...) {
      try {
        auto tf = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
        x = tf.transform.translation.x;
        y = tf.transform.translation.y;
        const auto &q = tf.transform.rotation;
        yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z));
        return true;
      } catch (...) {
        return false;
      }
    }
  }

  void onSeedPose(geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg) {
    if ((now() - last_self_pub_).seconds() < 1.5) return;
    // web_ops 连发 3 次 /initialpose：合并为一次精修
    if ((now() - last_seed_handle_).seconds() < 2.0) return;
    last_seed_handle_ = now();
    const auto &q = msg->pose.pose.orientation;
    seed_x_ = msg->pose.pose.position.x;
    seed_y_ = msg->pose.pose.position.y;
    seed_yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                           1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    have_seed_ = true;
    RCLCPP_INFO(get_logger(), "收到初值 (%.2f, %.2f, %.0f°) → 附近平移+旋转贴墙",
                seed_x_, seed_y_, seed_yaw_ * 180.0 / M_PI);
    if (!map_ready_ || !haveScan()) {
      publishStatus("need_scan");
      return;
    }
    Candidate c;
    if (refineNear(c, seed_x_, seed_y_, seed_yaw_)) {
      publishPose(c);
      publishStatus("aligned");
    } else {
      RCLCPP_WARN(get_logger(),
          "附近未找到贴墙位姿(best hit=%.0f%% unk=%.0f%%)，请微调初值朝向/位置后再试",
          c.hit * 100.0, c.unk * 100.0);
      publishStatus("seed_weak");
    }
  }

  void watchdogCheck() {
    if (!watchdog_en_ || !map_ready_ || !haveScan() || !have_seed_) return;
    if (!ever_aligned_) return;  // 首次贴墙成功前不跑看门狗
    if ((now() - last_reloc_time_).seconds() < 20.0) return;
    double x, y, yaw;
    if (!lookupCurrentPose(x, y, yaw)) return;
    auto pts = beams(400);
    if (pts.size() < 25) return;
    auto sp = scoreHits(pts, x, y, yaw);
    if (sp.hit >= watchdog_hit_ && sp.unk <= max_unk_ratio_) {
      low_score_cnt_ = 0;
      return;
    }
    if (++low_score_cnt_ < watchdog_count_) return;
    low_score_cnt_ = 0;
    RCLCPP_WARN(get_logger(), "跟踪命中偏低(hit=%.0f%%)，ICP 再精修", sp.hit * 100.0);
    Candidate c;
    if (refineNear(c, x, y, yaw)) {
      publishPose(c);
      publishStatus("aligned");
    } else {
      publishStatus("need_init");
    }
  }

  void onMap(nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
    map_ = msg;
    const int w = msg->info.width, h = msg->info.height;
    const float res = msg->info.resolution;
    const float INF = 1e9f;
    dist_.assign((size_t)w * h, INF);
    for (int i = 0; i < w * h; ++i)
      if (msg->data[i] >= 65) dist_[i] = 0.0f;
    const float s = res, diag = res * 1.41421356f;
    for (int y = 0; y < h; ++y)
      for (int x = 0; x < w; ++x) {
        float &d = dist_[(size_t)y * w + x];
        if (x > 0) d = std::min(d, dist_[(size_t)y * w + x - 1] + s);
        if (y > 0) d = std::min(d, dist_[(size_t)(y - 1) * w + x] + s);
        if (x > 0 && y > 0) d = std::min(d, dist_[(size_t)(y - 1) * w + x - 1] + diag);
        if (x < w - 1 && y > 0) d = std::min(d, dist_[(size_t)(y - 1) * w + x + 1] + diag);
      }
    for (int y = h - 1; y >= 0; --y)
      for (int x = w - 1; x >= 0; --x) {
        float &d = dist_[(size_t)y * w + x];
        if (x < w - 1) d = std::min(d, dist_[(size_t)y * w + x + 1] + s);
        if (y < h - 1) d = std::min(d, dist_[(size_t)(y + 1) * w + x] + s);
        if (x < w - 1 && y < h - 1) d = std::min(d, dist_[(size_t)(y + 1) * w + x + 1] + diag);
        if (x > 0 && y < h - 1) d = std::min(d, dist_[(size_t)(y + 1) * w + x - 1] + diag);
      }

    map_cloud_.reset(new Cloud);
    map_cloud_->points.reserve(w * h / 8);
    for (int gy = 0; gy < h; ++gy) {
      for (int gx = 0; gx < w; ++gx) {
        if (msg->data[(size_t)gy * w + gx] < 65) continue;
        pcl::PointXYZ p;
        p.x = msg->info.origin.position.x + (gx + 0.5f) * res;
        p.y = msg->info.origin.position.y + (gy + 0.5f) * res;
        p.z = 0.f;
        map_cloud_->points.push_back(p);
      }
    }
    map_cloud_->width = map_cloud_->points.size();
    map_cloud_->height = 1;
    map_cloud_->is_dense = true;

    // 体素降采样，加速 ICP
    if (map_cloud_->size() > 2000) {
      Cloud::Ptr filtered(new Cloud);
      pcl::VoxelGrid<pcl::PointXYZ> vg;
      vg.setInputCloud(map_cloud_);
      vg.setLeafSize(0.08f, 0.08f, 0.08f);
      vg.filter(*filtered);
      map_cloud_ = filtered;
    }
    map_ready_ = !map_cloud_->empty();
    RCLCPP_INFO(get_logger(), "地图障碍点云 %zu 点 (ICP目标)", map_cloud_->size());
  }

  std::vector<std::pair<float, float>> beams(int n) {
    sensor_msgs::msg::LaserScan::ConstSharedPtr scan;
    {
      std::lock_guard<std::mutex> lk(scan_mutex_);
      scan = scan_;
    }
    std::vector<std::pair<float, float>> pts;
    if (!scan) return pts;
    const int total = (int)scan->ranges.size();
    const int stride = std::max(1, total / n);
    for (int i = 0; i < total; i += stride) {
      const float r = scan->ranges[i];
      if (!std::isfinite(r) || r < scan->range_min || r > max_beam_range_) continue;
      const float a = scan->angle_min + i * scan->angle_increment;
      pts.emplace_back(r * std::cos(a), r * std::sin(a));
    }
    return pts;
  }

  struct HitScore { double hit = 0, unk = 0; };

  HitScore scoreHits(const std::vector<std::pair<float, float>> &pts,
                     double x, double y, double yaw) const {
    HitScore out;
    if (!map_ || pts.empty()) return out;
    const auto &info = map_->info;
    const int w = info.width, h = info.height;
    const double c = std::cos(yaw), s = std::sin(yaw);
    int hits = 0, unk = 0;
    for (const auto &p : pts) {
      const double wx = x + c * p.first - s * p.second;
      const double wy = y + s * p.first + c * p.second;
      const int gx = (int)((wx - info.origin.position.x) / info.resolution);
      const int gy = (int)((wy - info.origin.position.y) / info.resolution);
      if (gx < 0 || gx >= w || gy < 0 || gy >= h) { ++unk; continue; }
      const size_t idx = (size_t)gy * w + gx;
      if (map_->data[idx] < 0) { ++unk; continue; }
      if (dist_[idx] <= hit_dist_) ++hits;
    }
    out.hit = (double)hits / pts.size();
    out.unk = (double)unk / pts.size();
    return out;
  }

  static Eigen::Matrix4f poseMat(double x, double y, double yaw) {
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    const float c = std::cos(yaw), s = std::sin(yaw);
    T(0, 0) = c; T(0, 1) = -s; T(0, 3) = (float)x;
    T(1, 0) = s; T(1, 1) = c;  T(1, 3) = (float)y;
    return T;
  }

  static void matToPose(const Eigen::Matrix4f &T, double &x, double &y, double &yaw) {
    x = T(0, 3);
    y = T(1, 3);
    yaw = std::atan2(T(1, 0), T(0, 0));
  }

  Cloud::Ptr cropMap(double cx, double cy, double radius) const {
    Cloud::Ptr out(new Cloud);
    if (!map_cloud_) return out;
    const float r2 = (float)(radius * radius);
    out->points.reserve(map_cloud_->size() / 4);
    for (const auto &p : map_cloud_->points) {
      const float dx = p.x - (float)cx, dy = p.y - (float)cy;
      if (dx * dx + dy * dy <= r2) out->points.push_back(p);
    }
    out->width = out->points.size();
    out->height = 1;
    out->is_dense = true;
    return out;
  }

  Cloud::Ptr scanCloudBody(const std::vector<std::pair<float, float>> &pts) const {
    Cloud::Ptr cloud(new Cloud);
    cloud->points.reserve(pts.size());
    for (const auto &p : pts) {
      pcl::PointXYZ q;
      q.x = p.first; q.y = p.second; q.z = 0.f;
      cloud->points.push_back(q);
    }
    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = true;
    return cloud;
  }

  bool refineNear(Candidate &best, double sx, double sy, double syaw) {
    best = Candidate{};
    if (!map_ready_ || !haveScan()) {
      RCLCPP_WARN(get_logger(), "地图或激光未就绪");
      return false;
    }
    auto pts = beams(600);
    if (pts.size() < 30) {
      RCLCPP_WARN(get_logger(), "有效激光过少(%zu)", pts.size());
      return false;
    }

    const double span = yaw_span_deg_ * M_PI / 180.0;
    const double ystep = yaw_step_deg_ * M_PI / 180.0;
    const auto t0 = now();

    // —— 阶段1：初值附近平移 + 旋转穷举，目标=红激光尽量打在黑障碍上 ——
    Candidate coarse;
    coarse.fitness = 1e9;
    int evals = 0;
    for (double dx = -search_xy_; dx <= search_xy_ + 1e-9; dx += xy_step_) {
      for (double dy = -search_xy_; dy <= search_xy_ + 1e-9; dy += xy_step_) {
        for (double dyaw = -span; dyaw <= span + 1e-9; dyaw += ystep) {
          const double x = sx + dx, y = sy + dy, yaw = syaw + dyaw;
          auto hs = scoreHits(pts, x, y, yaw);
          ++evals;
          // 排序：命中高、未知低优先
          const double rank = -hs.hit + 0.5 * hs.unk;
          const double best_rank = -coarse.hit + 0.5 * coarse.unk;
          if (!coarse.ok || rank < best_rank) {
            coarse.x = x; coarse.y = y; coarse.yaw = yaw;
            coarse.hit = hs.hit; coarse.unk = hs.unk;
            coarse.fitness = 1.0 - hs.hit;
            coarse.ok = true;
          }
        }
      }
    }
    if (!coarse.ok) {
      RCLCPP_WARN(get_logger(), "附近搜索无候选");
      return false;
    }
    RCLCPP_INFO(get_logger(),
        "粗搜(%d次): 最佳 hit=%.0f%% unk=%.0f%% @ (%.2f,%.2f,%.0f°)",
        evals, coarse.hit * 100.0, coarse.unk * 100.0,
        coarse.x, coarse.y, coarse.yaw * 180 / M_PI);

    // —— 阶段2：以粗搜最优为初值做 ICP，再平移微调 ——
    Cloud::Ptr source = scanCloudBody(pts);
    Cloud::Ptr target = cropMap(coarse.x, coarse.y, local_radius_);
    best = coarse;
    if (target->size() >= 40) {
      // 粗搜前几名附近再跑几次 ICP（含粗搜最优）
      std::vector<Candidate> seeds = {coarse};
      for (double dx : {-0.25, 0.0, 0.25}) {
        for (double dy : {-0.25, 0.0, 0.25}) {
          for (double dyaw : {-0.15, 0.0, 0.15}) {
            Candidate s = coarse;
            s.x += dx; s.y += dy; s.yaw += dyaw;
            seeds.push_back(s);
          }
        }
      }
      for (const auto &seed : seeds) {
        Eigen::Matrix4f guess = poseMat(seed.x, seed.y, seed.yaw);
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setInputSource(source);
        icp.setInputTarget(target);
        icp.setMaxCorrespondenceDistance((float)icp_max_corr_);
        icp.setMaximumIterations(icp_max_iter_);
        icp.setTransformationEpsilon(1e-8);
        icp.setEuclideanFitnessEpsilon(1e-6);
        Cloud aligned;
        icp.align(aligned, guess);
        if (!icp.hasConverged()) continue;
        double x, y, yaw;
        matToPose(icp.getFinalTransformation(), x, y, yaw);
        auto hs = scoreHits(pts, x, y, yaw);
        const double fitness = icp.getFitnessScore();
        const double rank = -hs.hit + 0.5 * hs.unk + 0.3 * fitness;
        const double best_rank = -best.hit + 0.5 * best.unk + 0.3 * best.fitness;
        if (rank < best_rank) {
          best.x = x; best.y = y; best.yaw = yaw;
          best.hit = hs.hit; best.unk = hs.unk;
          best.fitness = fitness;
          best.ok = true;
        }
      }
    }

    const double dt = (now() - t0).seconds();
    // 核心验收：绝大部分红点必须贴黑墙
    const bool pass = (best.hit >= min_hit_ratio_) && (best.unk <= max_unk_ratio_);
    if (!pass) {
      RCLCPP_WARN(get_logger(),
          "拒绝: hit=%.0f%%(需≥%.0f%%) unk=%.0f%%(需≤%.0f%%) fitness=%.3f (%.1fs) "
          "— 红激光未大部分对齐黑障碍",
          best.hit * 100.0, min_hit_ratio_ * 100.0,
          best.unk * 100.0, max_unk_ratio_ * 100.0, best.fitness, dt);
      best.ok = false;
      return false;
    }
    RCLCPP_INFO(get_logger(),
        "对齐成功: x=%.2f y=%.2f yaw=%.1f° hit=%.0f%% unk=%.0f%% fitness=%.3f (%.1fs)",
        best.x, best.y, best.yaw * 180 / M_PI, best.hit * 100.0, best.unk * 100.0,
        best.fitness, dt);
    return true;
  }

  // 保留旧名兼容内部若有残留调用
  bool refineIcp(Candidate &best, double sx, double sy, double syaw) {
    return refineNear(best, sx, sy, syaw);
  }

  void publishPose(const Candidate &c) {
    geometry_msgs::msg::PoseWithCovarianceStamped msg;
    msg.header.frame_id = "map";
    msg.header.stamp = now();
    msg.pose.pose.position.x = c.x;
    msg.pose.pose.position.y = c.y;
    msg.pose.pose.orientation.z = std::sin(c.yaw / 2);
    msg.pose.pose.orientation.w = std::cos(c.yaw / 2);
    msg.pose.covariance[0] = msg.pose.covariance[7] = 0.15 * 0.15;
    msg.pose.covariance[35] = 0.10 * 0.10;
    last_self_pub_ = now();
    last_reloc_time_ = now();
    ever_aligned_ = true;
    pose_pub_->publish(msg);
    seed_x_ = c.x; seed_y_ = c.y; seed_yaw_ = c.yaw;
    have_seed_ = true;
  }

  double min_hit_ratio_, max_unk_ratio_, hit_dist_, max_beam_range_;
  double local_radius_, search_xy_, xy_step_;
  double icp_max_corr_, icp_fitness_max_;
  double yaw_span_deg_, yaw_step_deg_, watchdog_hit_;
  int icp_max_iter_, watchdog_count_;
  bool watchdog_en_;
  bool map_ready_ = false, have_seed_ = false, ever_aligned_ = false;
  int low_score_cnt_ = 0;
  double seed_x_ = 0, seed_y_ = 0, seed_yaw_ = 0;
  rclcpp::Time last_reloc_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_self_pub_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_seed_handle_{0, 0, RCL_ROS_TIME};

  nav_msgs::msg::OccupancyGrid::ConstSharedPtr map_;
  Cloud::Ptr map_cloud_;
  std::vector<float> dist_;
  sensor_msgs::msg::LaserScan::ConstSharedPtr scan_;
  std::mutex scan_mutex_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr seed_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AutoRelocalize>());
  rclcpp::shutdown();
  return 0;
}
