#include <algorithm>
#include <chrono>
#include <cmath>
#include <mutex>
#include <string>

#include <gz/common/Console.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/twist.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Inertial.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/transport/Node.hh>

namespace pb_rm_simulation
{

namespace
{
double clampValue(double value, double limit)
{
  return std::clamp(value, -limit, limit);
}

double rampToward(double current, double target, double max_delta)
{
  const double delta = target - current;
  if (std::abs(delta) <= max_delta)
  {
    return target;
  }
  return current + std::copysign(max_delta, delta);
}

double normalizeAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}
}  // namespace

/// \brief Force/torque-driven omnidirectional (holonomic) chassis controller
/// for Gazebo Harmonic.
///
/// Unlike velocity-teleporting planar_move plugins that overwrite vz = 0 every
/// tick and corrupt IMU linear acceleration, this system applies bounded
/// chassis-tangent drive forces and yaw torque via AddWorldWrench while leaving
/// vertical motion and terrain pitch/roll entirely to gravity and wheel contact.
/// When zero velocity is commanded, a gentle tangent position+yaw hold brake
/// eliminates static creep on sloped terrain (e.g., RMUC2027 1.5 deg spawn pad).
class OmniChassisSystem
  : public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
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
      gzerr << "[OmniChassisSystem] Must be attached to a model entity.\n";
      return;
    }

    if (_sdf)
    {
      if (_sdf->HasElement("topic"))
        topic_ = _sdf->Get<std::string>("topic");
      if (_sdf->HasElement("link_name"))
        link_name_ = _sdf->Get<std::string>("link_name");
      if (_sdf->HasElement("kp_linear"))
        kp_linear_ = _sdf->Get<double>("kp_linear");
      if (_sdf->HasElement("ki_linear"))
        ki_linear_ = _sdf->Get<double>("ki_linear");
      if (_sdf->HasElement("kp_angular"))
        kp_angular_ = _sdf->Get<double>("kp_angular");
      if (_sdf->HasElement("ki_angular"))
        ki_angular_ = _sdf->Get<double>("ki_angular");
      if (_sdf->HasElement("static_hold_kp"))
        static_hold_kp_ = _sdf->Get<double>("static_hold_kp");
      if (_sdf->HasElement("static_hold_ki"))
        static_hold_ki_ = _sdf->Get<double>("static_hold_ki");
      if (_sdf->HasElement("static_yaw_kp"))
        static_yaw_kp_ = _sdf->Get<double>("static_yaw_kp");
      if (_sdf->HasElement("max_linear_vel"))
        max_linear_vel_ = _sdf->Get<double>("max_linear_vel");
      if (_sdf->HasElement("max_angular_vel"))
        max_angular_vel_ = _sdf->Get<double>("max_angular_vel");
      if (_sdf->HasElement("max_linear_accel"))
        max_linear_accel_ = _sdf->Get<double>("max_linear_accel");
      if (_sdf->HasElement("max_angular_accel"))
        max_angular_accel_ = _sdf->Get<double>("max_angular_accel");
      if (_sdf->HasElement("max_force"))
        max_force_ = _sdf->Get<double>("max_force");
      if (_sdf->HasElement("max_torque"))
        max_torque_ = _sdf->Get<double>("max_torque");
      if (_sdf->HasElement("cmd_timeout"))
        cmd_timeout_ = _sdf->Get<double>("cmd_timeout");
    }

    // Resolve target link; URDF fixed-joint lumping can merge base_link into
    // the canonical root link (e.g. "mesh"), so fall back gracefully.
    gz::sim::Entity link_entity = model_.LinkByName(_ecm, link_name_);
    if (link_entity == gz::sim::kNullEntity)
    {
      link_entity = model_.CanonicalLink(_ecm);
    }
    if (link_entity == gz::sim::kNullEntity)
    {
      gzerr << "[OmniChassisSystem] Failed to resolve drive link '" << link_name_
            << "' or canonical link on model '" << model_.Name(_ecm) << "'.\n";
      return;
    }

    link_ = gz::sim::Link(link_entity);
    link_.EnableVelocityChecks(_ecm, true);

    const std::string resolved_name = link_.Name(_ecm).value_or("<unnamed>");
    if (!node_.Subscribe(topic_, &OmniChassisSystem::onTwist, this))
    {
      gzerr << "[OmniChassisSystem] Failed to subscribe to topic [" << topic_ << "].\n";
      return;
    }

    configured_ = true;
    gzmsg << "[OmniChassisSystem] Controlling link [" << resolved_name
          << "] on model [" << model_.Name(_ecm)
          << "] via topic [" << topic_ << "].\n";
  }

  void PreUpdate(
      const gz::sim::UpdateInfo &_info,
      gz::sim::EntityComponentManager &_ecm) override
  {
    if (!configured_ || _info.paused)
    {
      return;
    }

    const double dt =
        std::chrono::duration<double>(_info.dt).count();
    if (dt <= 0.0 || dt > 0.1)
    {
      return;
    }

    if (!inertia_initialized_)
    {
      computeModelInertia(_ecm);
      inertia_initialized_ = true;
    }

    const auto pose_opt = link_.WorldPose(_ecm);
    const auto lin_vel_opt = link_.WorldLinearVelocity(_ecm);
    const auto ang_vel_opt = link_.WorldAngularVelocity(_ecm);
    if (!pose_opt || !lin_vel_opt || !ang_vel_opt)
    {
      return;
    }

    const double sim_time =
        std::chrono::duration<double>(_info.simTime).count();

    double cmd_vx = 0.0;
    double cmd_vy = 0.0;
    double cmd_wz = 0.0;
    {
      std::lock_guard<std::mutex> lock(cmd_mutex_);
      if (cmd_received_)
      {
        if (cmd_stamp_pending_)
        {
          last_cmd_sim_time_ = sim_time;
          cmd_stamp_pending_ = false;
        }
        const bool timed_out =
            (cmd_timeout_ > 0.0) && ((sim_time - last_cmd_sim_time_) > cmd_timeout_);
        if (!timed_out)
        {
          cmd_vx = clampValue(cmd_vx_, max_linear_vel_);
          cmd_vy = clampValue(cmd_vy_, max_linear_vel_);
          cmd_wz = clampValue(cmd_wz_, max_angular_vel_);
        }
      }
    }

    // Smooth velocity targets with realistic acceleration bounds.
    const double max_dv = max_linear_accel_ * dt;
    const double max_dw = max_angular_accel_ * dt;
    ref_vx_ = rampToward(ref_vx_, cmd_vx, max_dv);
    ref_vy_ = rampToward(ref_vy_, cmd_vy, max_dv);
    ref_wz_ = rampToward(ref_wz_, cmd_wz, max_dw);

    const gz::math::Pose3d &pose = *pose_opt;
    const gz::math::Vector3d &v_world = *lin_vel_opt;
    const gz::math::Vector3d &w_world = *ang_vel_opt;

    const gz::math::Vector3d x_b = pose.Rot().RotateVector(gz::math::Vector3d::UnitX);
    const gz::math::Vector3d y_b = pose.Rot().RotateVector(gz::math::Vector3d::UnitY);
    const gz::math::Vector3d z_b = pose.Rot().RotateVector(gz::math::Vector3d::UnitZ);

    // Do not apply drive forces if chassis has flipped over.
    if (z_b.Z() < 0.5)
    {
      vel_int_x_ = 0.0;
      vel_int_y_ = 0.0;
      vel_int_w_ = 0.0;
      pos_hold_active_ = false;
      yaw_hold_active_ = false;
      return;
    }

    const double vx_curr = v_world.Dot(x_b);
    const double vy_curr = v_world.Dot(y_b);
    const double vz_curr = v_world.Dot(z_b);
    const double wz_curr = w_world.Dot(z_b);
    const bool grounded_hint = std::abs(vz_curr) < 0.25;

    // Chassis-tangent gravity feedforward eliminates downhill rollback on
    // slow slope starts and allows static hold to latch cleanly on steep ramps.
    const double g_ff_x = grounded_hint ? (9.81 * x_b.Z()) : 0.0;
    const double g_ff_y = grounded_hint ? (9.81 * y_b.Z()) : 0.0;

    // Linear tangent force control (X, Y in chassis frame; Z left purely to gravity/contact).
    double fx = 0.0;
    double fy = 0.0;
    const bool linear_active =
        (std::hypot(cmd_vx, cmd_vy) > 1e-4) || (std::hypot(ref_vx_, ref_vy_) > 1e-4);

    if (linear_active)
    {
      pos_hold_active_ = false;
      hold_int_x_ = 0.0;
      hold_int_y_ = 0.0;

      const double err_vx = ref_vx_ - vx_curr;
      const double err_vy = ref_vy_ - vy_curr;

      // Drop opposing integral on reversal and bleed inactive-axis integral so
      // wall/ramp-edge contact cannot leave persistent opposing force.
      if (cmd_vx * vel_int_x_ < 0.0)
      {
        vel_int_x_ = 0.0;
      }
      else if (std::abs(cmd_vx) < 1e-4 && std::abs(ref_vx_) < 1e-4)
      {
        vel_int_x_ *= std::max(0.0, 1.0 - 5.0 * dt);
      }
      if (cmd_vy * vel_int_y_ < 0.0)
      {
        vel_int_y_ = 0.0;
      }
      else if (std::abs(cmd_vy) < 1e-4 && std::abs(ref_vy_) < 1e-4)
      {
        vel_int_y_ *= std::max(0.0, 1.0 - 5.0 * dt);
      }

      const double max_vel_int = (ki_linear_ > 1e-6 && total_mass_ > 1e-6)
          ? std::min(0.5, (0.5 * max_force_) / (total_mass_ * ki_linear_))
          : 0.0;

      // Conditional integration (clamping anti-windup) within traction circle.
      const double fx_pre = total_mass_ * (g_ff_x + kp_linear_ * err_vx + ki_linear_ * vel_int_x_);
      const double fy_pre = total_mass_ * (g_ff_y + kp_linear_ * err_vy + ki_linear_ * vel_int_y_);
      const bool linear_saturated = std::hypot(fx_pre, fy_pre) >= max_force_;
      if (!linear_saturated || (err_vx * fx_pre < 0.0))
      {
        vel_int_x_ = clampValue(vel_int_x_ + err_vx * dt, max_vel_int);
      }
      if (!linear_saturated || (err_vy * fy_pre < 0.0))
      {
        vel_int_y_ = clampValue(vel_int_y_ + err_vy * dt, max_vel_int);
      }

      const double ax = g_ff_x + kp_linear_ * err_vx + ki_linear_ * vel_int_x_;
      const double ay = g_ff_y + kp_linear_ * err_vy + ki_linear_ * vel_int_y_;
      fx = total_mass_ * ax;
      fy = total_mass_ * ay;
    }
    else
    {
      vel_int_x_ = 0.0;
      vel_int_y_ = 0.0;

      if (!pos_hold_active_ && grounded_hint && std::hypot(vx_curr, vy_curr) < 0.08)
      {
        pos_hold_active_ = true;
        hold_pos_ = pose.Pos();
        hold_int_x_ = 0.0;
        hold_int_y_ = 0.0;
      }

      if (pos_hold_active_)
      {
        const gz::math::Vector3d dp = hold_pos_ - pose.Pos();
        const double ex = dp.Dot(x_b);
        const double ey = dp.Dot(y_b);
        hold_int_x_ = clampValue(hold_int_x_ + ex * dt, 0.25);
        hold_int_y_ = clampValue(hold_int_y_ + ey * dt, 0.25);

        const double ax =
            g_ff_x + static_hold_kp_ * ex + static_hold_ki_ * hold_int_x_ - kp_linear_ * vx_curr;
        const double ay =
            g_ff_y + static_hold_kp_ * ey + static_hold_ki_ * hold_int_y_ - kp_linear_ * vy_curr;
        fx = total_mass_ * ax;
        fy = total_mass_ * ay;
      }
      else
      {
        // Braking cleanly down to zero before latching static hold.
        fx = total_mass_ * (g_ff_x - kp_linear_ * vx_curr);
        fy = total_mass_ * (g_ff_y - kp_linear_ * vy_curr);
      }
    }

    // Clamp combined horizontal traction circle to max_force_.
    const double f_mag = std::hypot(fx, fy);
    if (f_mag > max_force_ && f_mag > 1e-9)
    {
      const double scale = max_force_ / f_mag;
      fx *= scale;
      fy *= scale;
    }

    // Yaw torque control about chassis normal axis z_b.
    double tau_z = 0.0;
    const double current_yaw = std::atan2(x_b.Y(), x_b.X());
    const bool yaw_active =
        (std::abs(cmd_wz) > 1e-4) || (std::abs(ref_wz_) > 1e-4);

    if (yaw_active)
    {
      yaw_hold_active_ = false;
      const double err_w = ref_wz_ - wz_curr;
      if (cmd_wz * vel_int_w_ < 0.0)
      {
        vel_int_w_ = 0.0;
      }
      const double max_vel_int_w = (ki_angular_ > 1e-6 && total_izz_ > 1e-6)
          ? std::min(0.5, (0.5 * max_torque_) / (total_izz_ * ki_angular_))
          : 0.0;
      const double tau_pre = total_izz_ * (kp_angular_ * err_w + ki_angular_ * vel_int_w_);
      const bool yaw_saturated = std::abs(tau_pre) >= max_torque_;
      if (!yaw_saturated || (err_w * tau_pre < 0.0))
      {
        vel_int_w_ = clampValue(vel_int_w_ + err_w * dt, max_vel_int_w);
      }
      tau_z = total_izz_ * (kp_angular_ * err_w + ki_angular_ * vel_int_w_);
    }
    else
    {
      vel_int_w_ = 0.0;
      if (!yaw_hold_active_ && grounded_hint && std::abs(wz_curr) < 0.08)
      {
        yaw_hold_active_ = true;
        hold_yaw_ = current_yaw;
      }

      if (yaw_hold_active_)
      {
        const double yaw_err = normalizeAngle(hold_yaw_ - current_yaw);
        tau_z = total_izz_ * (static_yaw_kp_ * yaw_err - kp_angular_ * wz_curr);
      }
      else
      {
        tau_z = -total_izz_ * kp_angular_ * wz_curr;
      }
    }

    tau_z = clampValue(tau_z, max_torque_);

    const gz::math::Vector3d force_world = fx * x_b + fy * y_b;
    const gz::math::Vector3d torque_world = tau_z * z_b;

    link_.AddWorldWrench(_ecm, force_world, torque_world, com_offset_);
  }

private:
  void onTwist(const gz::msgs::Twist &_msg)
  {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    cmd_vx_ = _msg.linear().x();
    cmd_vy_ = _msg.linear().y();
    cmd_wz_ = _msg.angular().z();
    cmd_received_ = true;
    cmd_stamp_pending_ = true;
  }

  void computeModelInertia(const gz::sim::EntityComponentManager &_ecm)
  {
    double mass_sum = 0.0;
    double izz_sum = 0.0;

    const auto base_pose_opt = link_.WorldPose(_ecm);
    const gz::math::Vector3d base_pos =
        base_pose_opt ? base_pose_opt->Pos() : gz::math::Vector3d::Zero;

    if (const auto *inertial =
            _ecm.Component<gz::sim::components::Inertial>(link_.Entity()))
    {
      com_offset_ = inertial->Data().Pose().Pos();
    }

    for (const auto &l_entity : model_.Links(_ecm))
    {
      const auto *inertial =
          _ecm.Component<gz::sim::components::Inertial>(l_entity);
      if (!inertial)
      {
        continue;
      }
      const double m = inertial->Data().MassMatrix().Mass();
      const double izz = inertial->Data().MassMatrix().Izz();
      mass_sum += m;

      double r2 = 0.0;
      gz::sim::Link l(l_entity);
      if (const auto lp = l.WorldPose(_ecm))
      {
        const gz::math::Vector3d d = lp->Pos() - base_pos;
        r2 = d.X() * d.X() + d.Y() * d.Y();
      }
      izz_sum += izz + m * r2;
    }

    if (mass_sum > 0.1)
    {
      total_mass_ = mass_sum;
    }
    if (izz_sum > 0.01)
    {
      total_izz_ = izz_sum;
    }

    gzmsg << "[OmniChassisSystem] Effective chassis mass=" << total_mass_
          << " kg, Izz=" << total_izz_
          << " kg*m^2, CoM offset=(" << com_offset_.X() << ", "
          << com_offset_.Y() << ", " << com_offset_.Z() << ").\n";
  }

  gz::sim::Model model_{gz::sim::kNullEntity};
  gz::sim::Link link_{gz::sim::kNullEntity};
  gz::transport::Node node_;
  bool configured_{false};
  bool inertia_initialized_{false};

  std::string topic_{"/cmd_vel"};
  std::string link_name_{"base_link"};

  double kp_linear_{22.0};
  double ki_linear_{25.0};
  double kp_angular_{30.0};
  double ki_angular_{40.0};
  double static_hold_kp_{120.0};
  double static_hold_ki_{80.0};
  double static_yaw_kp_{80.0};
  double max_linear_vel_{5.0};
  double max_angular_vel_{10.0};
  double max_linear_accel_{8.0};
  double max_angular_accel_{25.0};
  double max_force_{150.0};
  double max_torque_{30.0};
  double cmd_timeout_{0.0};

  double total_mass_{10.2};
  double total_izz_{0.12};
  gz::math::Vector3d com_offset_{gz::math::Vector3d::Zero};

  std::mutex cmd_mutex_;
  double cmd_vx_{0.0};
  double cmd_vy_{0.0};
  double cmd_wz_{0.0};
  bool cmd_received_{false};
  bool cmd_stamp_pending_{false};
  double last_cmd_sim_time_{0.0};

  double ref_vx_{0.0};
  double ref_vy_{0.0};
  double ref_wz_{0.0};

  double vel_int_x_{0.0};
  double vel_int_y_{0.0};
  double vel_int_w_{0.0};

  bool pos_hold_active_{false};
  gz::math::Vector3d hold_pos_{gz::math::Vector3d::Zero};
  double hold_int_x_{0.0};
  double hold_int_y_{0.0};

  bool yaw_hold_active_{false};
  double hold_yaw_{0.0};
};

}  // namespace pb_rm_simulation

GZ_ADD_PLUGIN(
    pb_rm_simulation::OmniChassisSystem,
    gz::sim::System,
    pb_rm_simulation::OmniChassisSystem::ISystemConfigure,
    pb_rm_simulation::OmniChassisSystem::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(
    pb_rm_simulation::OmniChassisSystem,
    "pb_rm_simulation::OmniChassisSystem")
