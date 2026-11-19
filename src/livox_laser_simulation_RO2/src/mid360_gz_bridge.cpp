#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <mutex>
#include <random>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <gz/msgs/laserscan.pb.h>
#include <gz/transport/Node.hh>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <livox_ros_driver2/msg/custom_point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include "ros2_livox/mid360_resampler.hpp"

namespace ros2_livox
{

namespace
{
// Binary-identical to livox_ros_driver2 LivoxPointXyzrtlt (26 bytes packed)
#pragma pack(push, 1)
struct LivoxPointXyzrtlt
{
  float x;
  float y;
  float z;
  float intensity;
  uint8_t tag;
  uint8_t line;
  double timestamp;  // absolute sim timestamp in ns
};
#pragma pack(pop)
static_assert(sizeof(LivoxPointXyzrtlt) == 26, "LivoxPointXyzrtlt must be 26 bytes packed");
}  // namespace

class Mid360GzBridge : public rclcpp::Node
{
public:
  Mid360GzBridge()
  : Node("mid360_gz_bridge")
  {
    gz_topic_ = declare_parameter<std::string>("gz_topic", "/livox/lidar/gz_scan");
    custom_topic_ = declare_parameter<std::string>("custom_topic", "/livox/lidar");
    pointcloud_topic_ = declare_parameter<std::string>("pointcloud_topic", "/livox/lidar/pointcloud");
    frame_id_ = declare_parameter<std::string>("frame_id", "livox_frame");
    csv_file_ = declare_parameter<std::string>("csv_file", "");

    scan_rate_ = declare_parameter<double>("scan_rate", 10.0);
    subframes_per_scan_ = std::max(1, static_cast<int>(declare_parameter<int>("subframes_per_scan", 5)));
    points_per_scan_ = std::max(1, static_cast<int>(declare_parameter<int>("points_per_scan", 20000)));
    line_count_ = std::max(1, static_cast<int>(declare_parameter<int>("line_count", 4)));
    downsample_ = std::max(1, static_cast<int>(declare_parameter<int>("downsample", 1)));

    min_range_ = declare_parameter<double>("min_range", 0.1);
    max_range_ = declare_parameter<double>("max_range", 40.0);
    noise_mean_ = declare_parameter<double>("noise_mean", 0.0);
    noise_stddev_ = std::max(0.0, declare_parameter<double>("noise_stddev", 0.02));
    // Optional Mid-360 datasheet near-range noise (0.0 disables, keeping uniform noise_stddev)
    range_noise_near_ = std::max(0.0, declare_parameter<double>("range_noise_near", 0.0));
    near_dist_ = declare_parameter<double>("near_dist", 0.2);
    far_dist_ = std::max(near_dist_ + 1e-3, declare_parameter<double>("far_dist", 10.0));
    // Optional angular noise (deg, 1 sigma per axis, compensated for grid quantization)
    angular_noise_rad_ = std::max(0.0, declare_parameter<double>("angular_noise_deg", 0.0)) * M_PI / 180.0;
    false_alarm_rate_ = std::clamp(declare_parameter<double>("false_alarm_rate", 0.0), 0.0, 1.0);
    // Optional reflectivity-dependent max range (0.0 disables, using flat max_range)
    const double range_10 = declare_parameter<double>("range_10", 0.0);
    const double range_80 = declare_parameter<double>("range_80", 0.0);
    if (range_10 > 0.0 && range_80 > 0.0)
    {
      range_10_ = range_10;
      range_exp_ = std::log(range_80 / range_10) / std::log(8.0);
    }

    edge_jump_min_m_ = declare_parameter<double>("edge_jump_min_m", 0.20);
    edge_jump_rel_ = declare_parameter<double>("edge_jump_rel", 0.10);

    lidar_id_ = static_cast<uint8_t>(std::clamp<int>(declare_parameter<int>("lidar_id", 0), 0, 255));
    default_reflectivity_ = std::clamp(declare_parameter<double>("default_reflectivity", 100.0), 0.0, 255.0);
    publish_pointcloud_ = declare_parameter<bool>("publish_pointcloud", true);
    filter_invalid_pointcloud_ = declare_parameter<bool>("filter_invalid_pointcloud", true);

    // Fixed default seed (5489) ensures reproducible simulation noise; 0 uses random_device
    const int64_t seed = declare_parameter<int64_t>("seed", 5489);
    if (seed == 0)
    {
      rng_.seed(std::random_device{}());
    }
    else
    {
      rng_.seed(static_cast<std::mt19937::result_type>(seed));
    }

    if (csv_file_.empty())
    {
      const std::string share_dir =
          ament_index_cpp::get_package_share_directory("ros2_livox_simulation");
      csv_file_ = share_dir + "/scan_mode/mid360.csv";
    }

    std::string err;
    if (!resampler_.loadCsv(csv_file_, line_count_, &err))
    {
      RCLCPP_FATAL(get_logger(), "Failed to load Mid-360 scan pattern: %s", err.c_str());
      throw std::runtime_error(err);
    }

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    custom_pub_ = create_publisher<livox_ros_driver2::msg::CustomMsg>(custom_topic_, qos);
    if (publish_pointcloud_)
    {
      cloud2_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(pointcloud_topic_, qos);
    }

    const std::size_t expected_pts = static_cast<std::size_t>(points_per_scan_ / downsample_ + 1);
    custom_msg_.points.reserve(expected_pts);
    cloud2_points_.reserve(expected_pts);
    initCloud2Template();

    if (!gz_node_.Subscribe(gz_topic_, &Mid360GzBridge::onLaserScan, this))
    {
      const std::string msg = "Failed to subscribe to Gazebo topic: " + gz_topic_;
      RCLCPP_FATAL(get_logger(), "%s", msg.c_str());
      throw std::runtime_error(msg);
    }

    RCLCPP_INFO(
        get_logger(),
        "Mid360GzBridge ready: gz_topic=%s -> custom=%s (pointcloud=%s), "
        "csv_rays=%zu, points/scan=%d, subframes=%d, downsample=%d, "
        "range=[%.2f, %.2f]m, noise_std=%.4fm, seed=%ld",
        gz_topic_.c_str(),
        custom_topic_.c_str(),
        publish_pointcloud_ ? pointcloud_topic_.c_str() : "<disabled>",
        resampler_.size(),
        points_per_scan_,
        subframes_per_scan_,
        downsample_,
        min_range_,
        max_range_,
        noise_stddev_,
        static_cast<long>(seed));
  }

  ~Mid360GzBridge() override
  {
    gz_node_.Unsubscribe(gz_topic_);
  }

private:
  void initCloud2Template()
  {
    cloud2_msg_.header.frame_id = frame_id_;
    cloud2_msg_.height = 1;
    cloud2_msg_.is_bigendian = false;
    cloud2_msg_.point_step = sizeof(LivoxPointXyzrtlt);

    const std::pair<const char *, std::pair<uint32_t, uint8_t>> field_specs[] = {
        {"x", {0, sensor_msgs::msg::PointField::FLOAT32}},
        {"y", {4, sensor_msgs::msg::PointField::FLOAT32}},
        {"z", {8, sensor_msgs::msg::PointField::FLOAT32}},
        {"intensity", {12, sensor_msgs::msg::PointField::FLOAT32}},
        {"tag", {16, sensor_msgs::msg::PointField::UINT8}},
        {"line", {17, sensor_msgs::msg::PointField::UINT8}},
        {"timestamp", {18, sensor_msgs::msg::PointField::FLOAT64}},
    };

    cloud2_msg_.fields.clear();
    cloud2_msg_.fields.reserve(7);
    for (const auto &spec : field_specs)
    {
      sensor_msgs::msg::PointField f;
      f.name = spec.first;
      f.offset = spec.second.first;
      f.datatype = spec.second.second;
      f.count = 1;
      cloud2_msg_.fields.push_back(f);
    }
  }

  void onLaserScan(const gz::msgs::LaserScan &_msg)
  {
    if (!rclcpp::ok())
    {
      return;
    }

    const int h_count = static_cast<int>(_msg.count());
    const int v_count = static_cast<int>(_msg.vertical_count());
    if (h_count < 2 || v_count < 2 || _msg.ranges_size() < h_count * v_count)
    {
      return;
    }

    SphericalGridView grid;
    grid.ranges = _msg.ranges().data();
    grid.intensities = (_msg.intensities_size() >= h_count * v_count)
                           ? _msg.intensities().data()
                           : nullptr;
    grid.count = h_count;
    grid.vertical_count = v_count;
    grid.angle_min = _msg.angle_min();
    grid.angle_max = _msg.angle_max();
    grid.vertical_angle_min = _msg.vertical_angle_min();
    grid.vertical_angle_max = _msg.vertical_angle_max();
    grid.range_min = _msg.range_min();
    grid.range_max = _msg.range_max();

    const double da = (grid.angle_max - grid.angle_min) / static_cast<double>(h_count - 1);
    const double de = (grid.vertical_angle_max - grid.vertical_angle_min) /
                      static_cast<double>(v_count - 1);
    const double ang_var = angular_noise_rad_ * angular_noise_rad_;
    const double el_sigma = (angular_noise_rad_ > 0.0)
                                ? std::sqrt(std::max(0.0, ang_var - de * de / 12.0))
                                : 0.0;

    const int64_t stamp_ns =
        static_cast<int64_t>(_msg.header().stamp().sec()) * 1000000000LL +
        static_cast<int64_t>(_msg.header().stamp().nsec());

    std::lock_guard<std::mutex> lock(mutex_);

    // Reset accumulator cleanly if sim time jumped backwards (e.g. world reset)
    if (stamp_ns < last_stamp_ns_)
    {
      subframe_idx_ = 0;
    }
    last_stamp_ns_ = stamp_ns;

    if (subframe_idx_ == 0)
    {
      frame_start_ns_ = stamp_ns;
      custom_msg_.points.clear();
      cloud2_points_.clear();
    }

    // Sub-frame slice [k0, k1) of the current 10 Hz scan (matching livox_gz_sim)
    const int k0 = static_cast<int>(
        static_cast<int64_t>(points_per_scan_) * subframe_idx_ / subframes_per_scan_);
    const int k1 = static_cast<int>(
        static_cast<int64_t>(points_per_scan_) * (subframe_idx_ + 1) / subframes_per_scan_);
    const int k_start = ((k0 + downsample_ - 1) / downsample_) * downsample_;

    const uint32_t subframe_offset_ns =
        static_cast<uint32_t>(std::max<int64_t>(0, stamp_ns - frame_start_ns_));
    const double synthetic_step_ns =
        (scan_rate_ > 0.0 && points_per_scan_ > 0)
            ? (1e9 / scan_rate_ / static_cast<double>(points_per_scan_))
            : 5000.0;

    const auto &rays = resampler_.rays();
    const std::size_t n_rays = rays.size();
    const bool need_cloud2 = publish_pointcloud_ && (cloud2_pub_ != nullptr);

    for (int k = k_start; k < k1; k += downsample_)
    {
      const std::size_t ray_idx = (frame_cursor_ + static_cast<std::size_t>(k)) % n_rays;
      const PrecomputedRay &ray = rays[ray_idx];

      double dist = 0.0;
      double refl = 0.0;

      if (false_alarm_rate_ > 0.0 && uniform_dist_(rng_) < false_alarm_rate_)
      {
        const double fa_max = std::min(max_range_, range_10_ > 0.0 ? range_10_ : max_range_);
        dist = min_range_ + uniform_dist_(rng_) * std::max(0.0, fa_max - min_range_);
        refl = std::floor(uniform_dist_(rng_) * 10.0);
      }
      else
      {
        const SampleResult sample = Mid360Resampler::sampleRay(
            grid,
            ray,
            min_range_,
            max_range_,
            default_reflectivity_,
            edge_jump_min_m_,
            edge_jump_rel_);
        dist = sample.range;
        refl = sample.reflectivity;

        if (dist > 0.0 && range_10_ > 0.0)
        {
          const double r_eff = std::clamp(refl, 1.0, 100.0);
          const double max_for_refl = range_10_ * std::pow(r_eff / 10.0, range_exp_);
          if (dist > max_for_refl)
          {
            dist = 0.0;
            refl = 0.0;
          }
        }
      }

      if (dist > 0.0)
      {
        double sigma = noise_stddev_;
        if (range_noise_near_ > 0.0)
        {
          const double t = std::clamp((dist - near_dist_) / (far_dist_ - near_dist_), 0.0, 1.0);
          sigma = range_noise_near_ + t * (noise_stddev_ - range_noise_near_);
        }
        if (sigma > 0.0 || noise_mean_ != 0.0)
        {
          dist += noise_mean_ + sigma * unit_gauss_(rng_);
        }
        if (dist < min_range_ || dist >= max_range_)
        {
          dist = 0.0;
          refl = 0.0;
        }
      }

      float px = 0.0f;
      float py = 0.0f;
      float pz = 0.0f;
      if (dist > 0.0)
      {
        if (angular_noise_rad_ > 0.0)
        {
          const double cos_el = std::max(std::cos(static_cast<double>(ray.el)), 0.1);
          const double dh = da * cos_el;
          const double az_sigma =
              std::sqrt(std::max(0.0, ang_var - dh * dh / 12.0)) / cos_el;
          const double el = static_cast<double>(ray.el) + el_sigma * unit_gauss_(rng_);
          const double az = static_cast<double>(ray.az) + az_sigma * unit_gauss_(rng_);
          px = static_cast<float>(dist * std::cos(el) * std::cos(az));
          py = static_cast<float>(dist * std::cos(el) * std::sin(az));
          pz = static_cast<float>(dist * std::sin(el));
        }
        else
        {
          const float r = static_cast<float>(dist);
          px = r * ray.dir_x;
          py = r * ray.dir_y;
          pz = r * ray.dir_z;
        }
      }

      const uint32_t pt_offset_ns = (subframes_per_scan_ > 1)
                                        ? subframe_offset_ns
                                        : static_cast<uint32_t>(
                                              static_cast<double>(k) * synthetic_step_ns);
      const uint8_t refl_u8 = (dist > 0.0) ? static_cast<uint8_t>(std::clamp(refl, 0.0, 255.0)) : 0U;

      livox_ros_driver2::msg::CustomPoint pt;
      pt.x = px;
      pt.y = py;
      pt.z = pz;
      pt.reflectivity = refl_u8;
      pt.tag = 0U;
      pt.line = ray.line;
      pt.offset_time = pt_offset_ns;
      custom_msg_.points.push_back(pt);

      if (need_cloud2 && (!filter_invalid_pointcloud_ || dist > 0.0))
      {
        cloud2_points_.push_back(LivoxPointXyzrtlt{
            px,
            py,
            pz,
            static_cast<float>(refl_u8),
            0U,
            ray.line,
            static_cast<double>(frame_start_ns_ + static_cast<int64_t>(pt_offset_ns)),
        });
      }
    }

    ++subframe_idx_;
    if (subframe_idx_ < subframes_per_scan_)
    {
      return;
    }

    // Complete scan assembled: advance pattern cursor by one full scan sweep
    frame_cursor_ = (frame_cursor_ + static_cast<std::size_t>(points_per_scan_)) % n_rays;
    subframe_idx_ = 0;

    builtin_interfaces::msg::Time ros_stamp;
    ros_stamp.sec = static_cast<int32_t>(frame_start_ns_ / 1000000000LL);
    ros_stamp.nanosec = static_cast<uint32_t>(frame_start_ns_ % 1000000000LL);

    custom_msg_.header.stamp = ros_stamp;
    custom_msg_.header.frame_id = frame_id_;
    custom_msg_.timebase = static_cast<uint64_t>(std::max<int64_t>(0, frame_start_ns_));
    custom_msg_.point_num = static_cast<uint32_t>(custom_msg_.points.size());
    custom_msg_.lidar_id = lidar_id_;
    custom_msg_.rsvd = {0, 0, 0};

    custom_pub_->publish(custom_msg_);

    if (need_cloud2)
    {
      cloud2_msg_.header.stamp = ros_stamp;
      cloud2_msg_.header.frame_id = frame_id_;
      cloud2_msg_.width = static_cast<uint32_t>(cloud2_points_.size());
      cloud2_msg_.row_step = cloud2_msg_.width * cloud2_msg_.point_step;
      cloud2_msg_.is_dense = filter_invalid_pointcloud_;
      cloud2_msg_.data.resize(cloud2_msg_.row_step);
      if (!cloud2_points_.empty())
      {
        std::memcpy(cloud2_msg_.data.data(), cloud2_points_.data(), cloud2_msg_.row_step);
      }
      cloud2_pub_->publish(cloud2_msg_);
    }
  }

  std::string gz_topic_;
  std::string custom_topic_;
  std::string pointcloud_topic_;
  std::string frame_id_;
  std::string csv_file_;

  double scan_rate_{10.0};
  int subframes_per_scan_{5};
  int points_per_scan_{20000};
  int line_count_{4};
  int downsample_{1};

  double min_range_{0.1};
  double max_range_{40.0};
  double noise_mean_{0.0};
  double noise_stddev_{0.02};
  double range_noise_near_{0.0};
  double near_dist_{0.2};
  double far_dist_{10.0};
  double angular_noise_rad_{0.0};
  double false_alarm_rate_{0.0};
  double range_10_{0.0};
  double range_exp_{0.0};
  double edge_jump_min_m_{0.20};
  double edge_jump_rel_{0.10};

  uint8_t lidar_id_{0};
  double default_reflectivity_{100.0};
  bool publish_pointcloud_{true};
  bool filter_invalid_pointcloud_{true};

  Mid360Resampler resampler_;
  gz::transport::Node gz_node_;
  rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr custom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud2_pub_;

  std::mutex mutex_;
  std::mt19937 rng_{5489u};
  std::normal_distribution<double> unit_gauss_{0.0, 1.0};
  std::uniform_real_distribution<double> uniform_dist_{0.0, 1.0};
  std::size_t frame_cursor_{0};
  int subframe_idx_{0};
  int64_t frame_start_ns_{0};
  int64_t last_stamp_ns_{0};

  livox_ros_driver2::msg::CustomMsg custom_msg_;
  sensor_msgs::msg::PointCloud2 cloud2_msg_;
  std::vector<LivoxPointXyzrtlt> cloud2_points_;
};

}  // namespace ros2_livox

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  try
  {
    auto node = std::make_shared<ros2_livox::Mid360GzBridge>();
    rclcpp::spin(node);
  }
  catch (const std::exception &ex)
  {
    RCLCPP_ERROR(rclcpp::get_logger("mid360_gz_bridge"), "Fatal error: %s", ex.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
