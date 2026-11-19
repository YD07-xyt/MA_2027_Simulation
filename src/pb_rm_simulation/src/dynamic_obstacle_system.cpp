// Copyright (c) 2026 MA_2027_Simulation Authors
// SPDX-License-Identifier: Apache-2.0

#include <algorithm>
#include <chrono>
#include <cmath>
#include <sstream>
#include <string>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Pose.hh>

namespace pb_rm_simulation
{

struct Waypoint
{
  double time{0.0};
  gz::math::Vector3d pos{0.0, 0.0, 0.0};
  double yaw{0.0};
};

/// \brief Gazebo Harmonic kinematic trajectory system for dynamic arena obstacles.
///
/// Supports two trajectory modes configured directly in SDF:
///   - <mode>ellipse</mode>: <radius_x>, <radius_y>, <period>, <phase>
///   - <mode>waypoints</mode>: repeated <waypoint>t x y z [yaw]</waypoint> elements
///
/// Moves the parent <model> via Model::SetWorldPoseCmd so the obstacle remains
/// visible to both DART collision and OGRE2 gpu_lidar depth rendering.
class DynamicObstacleSystem
  : public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate,
    public gz::sim::ISystemReset
{
public:
  void Configure(
      const gz::sim::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      gz::sim::EntityComponentManager &_ecm,
      gz::sim::EventManager & /*_eventMgr*/) override
  {
    model_ = gz::sim::Model(_entity);
    if (!model_.Valid(_ecm))
    {
      gzerr << "[DynamicObstacleSystem] Plugin must be attached to a <model>." << std::endl;
      return;
    }

    if (_sdf)
    {
      if (_sdf->HasElement("mode"))
      {
        mode_ = _sdf->Get<std::string>("mode");
      }
      if (_sdf->HasElement("relative"))
      {
        relative_ = _sdf->Get<bool>("relative");
      }
      if (_sdf->HasElement("loop"))
      {
        loop_ = _sdf->Get<bool>("loop");
      }
      if (_sdf->HasElement("radius_x"))
      {
        radius_x_ = _sdf->Get<double>("radius_x");
      }
      if (_sdf->HasElement("radius_y"))
      {
        radius_y_ = _sdf->Get<double>("radius_y");
      }
      if (_sdf->HasElement("period"))
      {
        period_ = std::max(1e-3, _sdf->Get<double>("period"));
      }
      if (_sdf->HasElement("phase"))
      {
        phase_ = _sdf->Get<double>("phase");
      }

      if (_sdf->HasElement("waypoint"))
      {
        auto wp_elem = const_cast<sdf::Element *>(_sdf.get())->GetElement("waypoint");
        while (wp_elem)
        {
          const std::string text = wp_elem->Get<std::string>();
          std::istringstream iss(text);
          Waypoint wp;
          if (iss >> wp.time >> wp.pos.X() >> wp.pos.Y() >> wp.pos.Z())
          {
            if (!(iss >> wp.yaw))
            {
              wp.yaw = 0.0;
            }
            waypoints_.push_back(wp);
          }
          else
          {
            gzwarn << "[DynamicObstacleSystem] Ignoring malformed <waypoint>: '" << text << "'"
                   << std::endl;
          }
          wp_elem = wp_elem->GetNextElement("waypoint");
        }
        std::sort(
            waypoints_.begin(),
            waypoints_.end(),
            [](const Waypoint &a, const Waypoint &b) { return a.time < b.time; });
      }
    }

    if (const auto *pose_comp = _ecm.Component<gz::sim::components::Pose>(model_.Entity()))
    {
      initial_pose_ = pose_comp->Data();
      has_initial_pose_ = true;
    }

    configured_ = true;
    gzmsg << "[DynamicObstacleSystem] Model '" << model_.Name(_ecm)
          << "' configured: mode=" << mode_
          << ", relative=" << (relative_ ? "true" : "false")
          << ", loop=" << (loop_ ? "true" : "false")
          << ", waypoints=" << waypoints_.size() << std::endl;
  }

  void Reset(
      const gz::sim::UpdateInfo & /*_info*/,
      gz::sim::EntityComponentManager &_ecm) override
  {
    if (configured_ && has_initial_pose_)
    {
      model_.SetWorldPoseCmd(_ecm, initial_pose_);
    }
  }

  void PreUpdate(
      const gz::sim::UpdateInfo &_info,
      gz::sim::EntityComponentManager &_ecm) override
  {
    if (!configured_ || _info.paused)
    {
      return;
    }

    if (!has_initial_pose_)
    {
      if (const auto *pose_comp = _ecm.Component<gz::sim::components::Pose>(model_.Entity()))
      {
        initial_pose_ = pose_comp->Data();
        has_initial_pose_ = true;
      }
      else
      {
        return;
      }
    }

    const double sim_t = std::chrono::duration<double>(_info.simTime).count();
    gz::math::Vector3d offset_pos(0.0, 0.0, 0.0);
    double offset_yaw = 0.0;

    if (mode_ == "ellipse")
    {
      constexpr double kTwoPi = 2.0 * M_PI;
      const double theta = kTwoPi * (sim_t / period_) + phase_;
      offset_pos.Set(radius_x_ * std::cos(theta), radius_y_ * std::sin(theta), 0.0);
    }
    else if (mode_ == "waypoints" && !waypoints_.empty())
    {
      evaluateWaypoints(sim_t, offset_pos, offset_yaw);
    }
    else
    {
      return;
    }

    gz::math::Pose3d target_pose;
    if (relative_)
    {
      const gz::math::Quaterniond q_yaw(0.0, 0.0, offset_yaw);
      target_pose.Set(initial_pose_.Pos() + offset_pos, initial_pose_.Rot() * q_yaw);
    }
    else
    {
      target_pose.Set(offset_pos, gz::math::Quaterniond(0.0, 0.0, offset_yaw));
    }

    model_.SetWorldPoseCmd(_ecm, target_pose);
  }

private:
  void evaluateWaypoints(double sim_t, gz::math::Vector3d &out_pos, double &out_yaw) const
  {
    if (waypoints_.size() == 1)
    {
      out_pos = waypoints_.front().pos;
      out_yaw = waypoints_.front().yaw;
      return;
    }

    const double t_start = waypoints_.front().time;
    const double t_end = waypoints_.back().time;
    const double duration = t_end - t_start;

    double t = sim_t;
    if (duration > 1e-6)
    {
      if (loop_)
      {
        t = t_start + std::fmod(std::max(0.0, sim_t - t_start), duration);
      }
      else
      {
        t = std::clamp(sim_t, t_start, t_end);
      }
    }
    else
    {
      out_pos = waypoints_.front().pos;
      out_yaw = waypoints_.front().yaw;
      return;
    }

    for (std::size_t i = 0; i + 1 < waypoints_.size(); ++i)
    {
      const Waypoint &a = waypoints_[i];
      const Waypoint &b = waypoints_[i + 1];
      if (t >= a.time && t <= b.time)
      {
        const double dt = b.time - a.time;
        const double alpha = (dt > 1e-9) ? std::clamp((t - a.time) / dt, 0.0, 1.0) : 0.0;
        out_pos = a.pos + (b.pos - a.pos) * alpha;
        out_yaw = a.yaw + (b.yaw - a.yaw) * alpha;
        return;
      }
    }

    out_pos = waypoints_.back().pos;
    out_yaw = waypoints_.back().yaw;
  }

  gz::sim::Model model_{gz::sim::kNullEntity};
  bool configured_{false};
  bool has_initial_pose_{false};
  gz::math::Pose3d initial_pose_{gz::math::Pose3d::Zero};

  std::string mode_{"ellipse"};
  bool relative_{true};
  bool loop_{true};
  double radius_x_{1.0};
  double radius_y_{2.0};
  double period_{60.0};
  double phase_{0.0};
  std::vector<Waypoint> waypoints_;
};

}  // namespace pb_rm_simulation

GZ_ADD_PLUGIN(
    pb_rm_simulation::DynamicObstacleSystem,
    gz::sim::System,
    pb_rm_simulation::DynamicObstacleSystem::ISystemConfigure,
    pb_rm_simulation::DynamicObstacleSystem::ISystemPreUpdate,
    pb_rm_simulation::DynamicObstacleSystem::ISystemReset)
