#ifndef ROS2_LIVOX_MID360_RESAMPLER_HPP_
#define ROS2_LIVOX_MID360_RESAMPLER_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <string>
#include <vector>

namespace ros2_livox
{

struct PrecomputedRay
{
  float az{0.0f};     // Azimuth in [-pi, pi] rad (+X = 0, +Y = +pi/2)
  float el{0.0f};     // Elevation in rad (+Z > 0, -Z < 0; el = pi/2 - zenith)
  float dir_x{1.0f};  // Unit ray direction X
  float dir_y{0.0f};  // Unit ray direction Y
  float dir_z{0.0f};  // Unit ray direction Z
  uint8_t line{0};    // Mid-360 laser line index (0..3)
};

struct SphericalGridView
{
  const double *ranges{nullptr};
  const double *intensities{nullptr};
  int count{0};
  int vertical_count{0};
  double angle_min{-M_PI};
  double angle_max{M_PI};
  double vertical_angle_min{-0.1291543646475804};  // -7.4 deg
  double vertical_angle_max{0.9145525280450288};   // +52.4 deg
  double range_min{0.1};
  double range_max{40.0};
};

struct SampleResult
{
  double range{0.0};         // 0.0 means invalid / no return
  double reflectivity{0.0};  // 0..255 (from <laser_retro> intensity or default)
};

class Mid360Resampler
{
public:
  static double wrapToPi(double rad)
  {
    constexpr double kTwoPi = 2.0 * M_PI;
    double a = std::fmod(rad + M_PI, kTwoPi);
    if (a < 0.0)
    {
      a += kTwoPi;
    }
    return a - M_PI;
  }

  bool loadCsv(
      const std::string &csv_path,
      int line_count = 4,
      std::string *error_msg = nullptr)
  {
    std::ifstream file(csv_path);
    if (!file.is_open())
    {
      if (error_msg)
      {
        *error_msg = "Cannot open CSV file: " + csv_path;
      }
      return false;
    }

    rays_.clear();
    rays_.reserve(800000);

    std::string line;
    // Skip header line ("Time/s,Azimuth/deg,Zenith/deg")
    if (!std::getline(file, line))
    {
      if (error_msg)
      {
        *error_msg = "Empty CSV file: " + csv_path;
      }
      return false;
    }

    constexpr double kDeg2Rad = M_PI / 180.0;
    const std::size_t safe_lines = static_cast<std::size_t>(std::max(1, line_count));
    std::size_t row_idx = 0;
    while (std::getline(file, line))
    {
      if (line.empty())
      {
        continue;
      }
      const char *p = line.c_str();
      char *end1 = nullptr;
      (void)std::strtod(p, &end1);  // Time/s index column
      if (end1 == p || *end1 != ',')
      {
        continue;
      }
      char *end2 = nullptr;
      const double az_deg = std::strtod(end1 + 1, &end2);
      if (end2 == end1 + 1 || *end2 != ',')
      {
        continue;
      }
      char *end3 = nullptr;
      const double zenith_deg = std::strtod(end2 + 1, &end3);
      if (end3 == end2 + 1)
      {
        continue;
      }

      const double az = wrapToPi(az_deg * kDeg2Rad);
      // Matches legacy livox_points_plugin.cpp and livox_gz_sim:
      //   el = pi/2 - zenith_rad, dir = [cos(el)*cos(az), cos(el)*sin(az), sin(el)]
      const double el = M_PI_2 - zenith_deg * kDeg2Rad;
      const double cos_el = std::cos(el);

      PrecomputedRay ray;
      ray.az = static_cast<float>(az);
      ray.el = static_cast<float>(el);
      ray.dir_x = static_cast<float>(cos_el * std::cos(az));
      ray.dir_y = static_cast<float>(cos_el * std::sin(az));
      ray.dir_z = static_cast<float>(std::sin(el));
      ray.line = static_cast<uint8_t>(row_idx % safe_lines);
      rays_.push_back(ray);
      ++row_idx;
    }

    if (rays_.empty())
    {
      if (error_msg)
      {
        *error_msg = "No valid scan rows parsed from CSV: " + csv_path;
      }
      return false;
    }
    return true;
  }

  void setRaysForTest(std::vector<PrecomputedRay> rays)
  {
    rays_ = std::move(rays);
  }

  const std::vector<PrecomputedRay> &rays() const { return rays_; }
  std::size_t size() const { return rays_.size(); }

  /// \brief Edge-aware bilinear resampling of a spherical gpu_lidar range grid
  ///        plus nearest-pixel reflectivity (<laser_retro>) lookup.
  ///
  /// Uses bilinear interpolation across smooth surfaces to reduce angular
  /// quantization error to sub-millimeter scale, and falls back to nearest-pixel
  /// lookup across depth discontinuities or sky boundaries to avoid ghost
  /// "flying pixels".
  static SampleResult sampleRay(
      const SphericalGridView &grid,
      const PrecomputedRay &ray,
      double min_range,
      double max_range,
      double default_reflectivity = 100.0,
      double edge_jump_min_m = 0.20,
      double edge_jump_rel = 0.10)
  {
    SampleResult res;
    if (!grid.ranges || grid.count < 2 || grid.vertical_count < 2)
    {
      return res;
    }

    if (ray.el < grid.vertical_angle_min || ray.el > grid.vertical_angle_max)
    {
      return res;
    }

    const double h_span = grid.angle_max - grid.angle_min;
    const double v_span = grid.vertical_angle_max - grid.vertical_angle_min;
    if (h_span <= 0.0 || v_span <= 0.0)
    {
      return res;
    }

    const double h_step = h_span / static_cast<double>(grid.count - 1);
    const double v_step = v_span / static_cast<double>(grid.vertical_count - 1);

    const double u = std::clamp(
        (static_cast<double>(ray.az) - grid.angle_min) / h_step,
        0.0,
        static_cast<double>(grid.count - 1));
    const double v = std::clamp(
        (static_cast<double>(ray.el) - grid.vertical_angle_min) / v_step,
        0.0,
        static_cast<double>(grid.vertical_count - 1));

    const int h0 = std::min(static_cast<int>(std::floor(u)), grid.count - 1);
    const int h1 = std::min(h0 + 1, grid.count - 1);
    const int v0 = std::min(static_cast<int>(std::floor(v)), grid.vertical_count - 1);
    const int v1 = std::min(v0 + 1, grid.vertical_count - 1);

    const double fu = u - static_cast<double>(h0);
    const double fv = v - static_cast<double>(v0);

    const int hn = (fu >= 0.5) ? h1 : h0;
    const int vn = (fv >= 0.5) ? v1 : v0;
    const int nearest_idx = vn * grid.count + hn;

    const double eff_min = std::max(min_range, grid.range_min);
    const double eff_max = std::min(max_range, grid.range_max);

    const auto isValid = [eff_min, eff_max](double r) -> bool {
      return std::isfinite(r) && r >= eff_min && r < eff_max;
    };

    const double r00 = grid.ranges[v0 * grid.count + h0];
    const double r01 = grid.ranges[v0 * grid.count + h1];
    const double r10 = grid.ranges[v1 * grid.count + h0];
    const double r11 = grid.ranges[v1 * grid.count + h1];

    double dist = 0.0;
    if (isValid(r00) && isValid(r01) && isValid(r10) && isValid(r11))
    {
      const double r_lo = std::min({r00, r01, r10, r11});
      const double r_hi = std::max({r00, r01, r10, r11});
      const double jump_thresh = std::max(edge_jump_min_m, edge_jump_rel * r_lo);
      if ((r_hi - r_lo) <= jump_thresh)
      {
        const double r0 = r00 + fu * (r01 - r00);
        const double r1 = r10 + fu * (r11 - r10);
        dist = r0 + fv * (r1 - r0);
      }
      else
      {
        const double rn = grid.ranges[nearest_idx];
        dist = isValid(rn) ? rn : 0.0;
      }
    }
    else
    {
      const double rn = grid.ranges[nearest_idx];
      dist = isValid(rn) ? rn : 0.0;
    }

    if (dist > 0.0)
    {
      res.range = dist;
      const double retro = grid.intensities ? grid.intensities[nearest_idx] : 0.0;
      res.reflectivity = std::round(
          (std::isfinite(retro) && retro > 0.0)
              ? std::min(retro, 255.0)
              : std::clamp(default_reflectivity, 0.0, 255.0));
    }
    return res;
  }

  static double sampleRange(
      const SphericalGridView &grid,
      const PrecomputedRay &ray,
      double min_range,
      double max_range,
      double edge_jump_min_m = 0.20,
      double edge_jump_rel = 0.10)
  {
    return sampleRay(grid, ray, min_range, max_range, 100.0, edge_jump_min_m, edge_jump_rel).range;
  }

private:
  std::vector<PrecomputedRay> rays_;
};

}  // namespace ros2_livox

#endif  // ROS2_LIVOX_MID360_RESAMPLER_HPP_
