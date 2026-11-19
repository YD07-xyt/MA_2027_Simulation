#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string>
#include <vector>

#include "ros2_livox/mid360_resampler.hpp"

namespace ros2_livox
{

TEST(Mid360ResamplerTest, WrapToPiMatchesExpectedAngles)
{
  EXPECT_NEAR(Mid360Resampler::wrapToPi(0.0), 0.0, 1e-9);
  EXPECT_NEAR(Mid360Resampler::wrapToPi(M_PI_2), M_PI_2, 1e-9);
  EXPECT_NEAR(Mid360Resampler::wrapToPi(3.0 * M_PI_2), -M_PI_2, 1e-9);
  EXPECT_NEAR(Mid360Resampler::wrapToPi(2.0 * M_PI), 0.0, 1e-9);
  EXPECT_NEAR(std::abs(Mid360Resampler::wrapToPi(M_PI)), M_PI, 1e-9);
}

TEST(Mid360ResamplerTest, CsvRayDirectionMatchesLegacyEulerConvention)
{
  const std::string tmp_csv =
      (std::filesystem::temp_directory_path() / "test_mid360_sample.csv").string();
  {
    std::ofstream ofs(tmp_csv);
    ofs << "Time/s,Azimuth/deg,Zenith/deg\n";
    ofs << "1,0.0,90.0\n";       // +X horizontal
    ofs << "2,90.0,90.0\n";      // +Y horizontal
    ofs << "3,180.0,37.838\n";   // -X looking up at +52.162 deg
    ofs << "4,270.0,97.212\n";   // -Y looking down at -7.212 deg
  }

  Mid360Resampler resampler;
  std::string err;
  ASSERT_TRUE(resampler.loadCsv(tmp_csv, 4, &err)) << err;
  ASSERT_EQ(resampler.size(), 4u);

  // Row 0: +X horizontal
  EXPECT_NEAR(resampler.rays()[0].dir_x, 1.0f, 1e-5f);
  EXPECT_NEAR(resampler.rays()[0].dir_y, 0.0f, 1e-5f);
  EXPECT_NEAR(resampler.rays()[0].dir_z, 0.0f, 1e-5f);
  EXPECT_EQ(resampler.rays()[0].line, 0u);

  // Row 1: +Y horizontal
  EXPECT_NEAR(resampler.rays()[1].dir_x, 0.0f, 1e-5f);
  EXPECT_NEAR(resampler.rays()[1].dir_y, 1.0f, 1e-5f);
  EXPECT_NEAR(resampler.rays()[1].dir_z, 0.0f, 1e-5f);
  EXPECT_EQ(resampler.rays()[1].line, 1u);

  // Row 2: -X looking UP (+Z > 0)
  const double el2 = (90.0 - 37.838) * M_PI / 180.0;
  EXPECT_NEAR(resampler.rays()[2].dir_x, -std::cos(el2), 1e-5);
  EXPECT_NEAR(resampler.rays()[2].dir_y, 0.0, 1e-5);
  EXPECT_NEAR(resampler.rays()[2].dir_z, std::sin(el2), 1e-5);
  EXPECT_EQ(resampler.rays()[2].line, 2u);

  // Row 3: -Y looking DOWN (-Z < 0)
  const double el3 = (90.0 - 97.212) * M_PI / 180.0;
  EXPECT_NEAR(resampler.rays()[3].dir_x, 0.0, 1e-5);
  EXPECT_NEAR(resampler.rays()[3].dir_y, -std::cos(el3), 1e-5);
  EXPECT_NEAR(resampler.rays()[3].dir_z, std::sin(el3), 1e-5);
  EXPECT_EQ(resampler.rays()[3].line, 3u);
}

TEST(Mid360ResamplerTest, SmoothBilinearEdgeRejectionAndReflectivity)
{
  const int H = 361;
  const int V = 61;
  std::vector<double> ranges(static_cast<std::size_t>(H * V), 5.0);
  std::vector<double> intensities(static_cast<std::size_t>(H * V), 0.0);

  SphericalGridView grid;
  grid.ranges = ranges.data();
  grid.intensities = intensities.data();
  grid.count = H;
  grid.vertical_count = V;
  grid.angle_min = -M_PI;
  grid.angle_max = M_PI;
  grid.vertical_angle_min = -7.4 * M_PI / 180.0;
  grid.vertical_angle_max = 52.4 * M_PI / 180.0;
  grid.range_min = 0.1;
  grid.range_max = 40.0;

  // Smooth linear ramp on left half (h < 180): r = 3.0 + 0.01 * h, laser_retro unset (0)
  // Sharp wall jump on right half (h >= 180): r = 12.0, laser_retro = 199.98 (~200)
  for (int v = 0; v < V; ++v)
  {
    for (int h = 0; h < H; ++h)
    {
      ranges[v * H + h] = (h < 180) ? (3.0 + 0.01 * h) : 12.0;
      intensities[v * H + h] = (h < 180) ? 0.0 : 199.98;
    }
  }

  // Smooth region halfway between h=100 (4.00m) and h=101 (4.01m), fallback reflectivity=100
  PrecomputedRay smooth_ray;
  smooth_ray.az = static_cast<float>(grid.angle_min + 100.5 * (2.0 * M_PI / (H - 1)));
  smooth_ray.el = 0.0f;
  const SampleResult r_smooth = Mid360Resampler::sampleRay(grid, smooth_ray, 0.1, 40.0, 100.0);
  EXPECT_NEAR(r_smooth.range, 4.005, 1e-6);
  EXPECT_DOUBLE_EQ(r_smooth.reflectivity, 100.0);

  // Sharp jump region between h=179 (4.79m) and h=180 (12.0m):
  // Must NOT produce ghost midpoint (~8.4m); must snap to nearest pixel and its retro value.
  PrecomputedRay near_left;
  near_left.az = static_cast<float>(grid.angle_min + 179.3 * (2.0 * M_PI / (H - 1)));
  near_left.el = 0.0f;
  const SampleResult s_left = Mid360Resampler::sampleRay(grid, near_left, 0.1, 40.0, 100.0);
  EXPECT_NEAR(s_left.range, 4.79, 1e-6);
  EXPECT_DOUBLE_EQ(s_left.reflectivity, 100.0);

  PrecomputedRay near_right;
  near_right.az = static_cast<float>(grid.angle_min + 179.7 * (2.0 * M_PI / (H - 1)));
  near_right.el = 0.0f;
  const SampleResult s_right = Mid360Resampler::sampleRay(grid, near_right, 0.1, 40.0, 100.0);
  EXPECT_NEAR(s_right.range, 12.0, 1e-6);
  EXPECT_DOUBLE_EQ(s_right.reflectivity, 200.0);

  // Sky (inf) returns 0.0 invalid point and 0 reflectivity
  ranges[0] = std::numeric_limits<double>::infinity();
  PrecomputedRay sky_ray;
  sky_ray.az = static_cast<float>(grid.angle_min);
  sky_ray.el = static_cast<float>(grid.vertical_angle_min);
  const SampleResult s_sky = Mid360Resampler::sampleRay(grid, sky_ray, 0.1, 40.0, 100.0);
  EXPECT_DOUBLE_EQ(s_sky.range, 0.0);
  EXPECT_DOUBLE_EQ(s_sky.reflectivity, 0.0);
}

}  // namespace ros2_livox
