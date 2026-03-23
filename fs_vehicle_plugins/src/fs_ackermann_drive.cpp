#include "fs_vehicle_plugins/fs_ackermann_drive.hpp"

#include <algorithm>
#include <cmath>

#include <gz/plugin/Register.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/Util.hh>

namespace fs_plugins
{

// ─────────────────────────────────────────────────────────────────────────────
// Helpers
// ─────────────────────────────────────────────────────────────────────────────
static void ensureVelCmd(gz::sim::EntityComponentManager & ecm,
                         gz::sim::Entity joint)
{
  if (!ecm.Component<gz::sim::components::JointVelocityCmd>(joint))
    ecm.CreateComponent(joint, gz::sim::components::JointVelocityCmd({0.0}));
}

static void ensurePosFeedback(gz::sim::EntityComponentManager & ecm,
                               gz::sim::Entity joint)
{
  if (!ecm.Component<gz::sim::components::JointPosition>(joint))
    ecm.CreateComponent(joint, gz::sim::components::JointPosition({0.0}));
}

// ─────────────────────────────────────────────────────────────────────────────
// ISystemConfigure
// ─────────────────────────────────────────────────────────────────────────────
void FsAckermannDrive::Configure(
  const gz::sim::Entity & entity,
  const std::shared_ptr<const sdf::Element> & sdf,
  gz::sim::EntityComponentManager & ecm,
  gz::sim::EventManager & /*eventMgr*/)
{
  model_ = gz::sim::Model(entity);
  if (!model_.Valid(ecm)) {
    gzerr << "[FsAckermannDrive] Invalid model entity\n";
    return;
  }

  // ── Read SDF parameters ──────────────────────────────────────────────────
  wheel_radius_    = sdf->Get<double>("wheel_radius",    0.2032).first;
  wheel_base_      = sdf->Get<double>("wheel_base",      1.58  ).first;
  wheel_sep_       = sdf->Get<double>("wheel_sep",       1.4   ).first;
  max_steer_       = sdf->Get<double>("max_steer",       0.5   ).first;
  steer_kp_        = sdf->Get<double>("steer_kp",        8.0   ).first;
  max_steer_rate_  = sdf->Get<double>("max_steer_rate",  3.0   ).first;
  max_accel_       = sdf->Get<double>("max_accel",       4.0   ).first;
  max_decel_       = sdf->Get<double>("max_decel",       8.0   ).first;
  const std::string topic = sdf->Get<std::string>("topic", "/cmd_vel").first;

  // ── Resolve joint entities ───────────────────────────────────────────────
  left_rear_joint_   = model_.JointByName(ecm, "left_rear_wheel_joint");
  right_rear_joint_  = model_.JointByName(ecm, "right_rear_wheel_joint");
  left_steer_joint_  = model_.JointByName(ecm, "left_steering_hinge_joint");
  right_steer_joint_ = model_.JointByName(ecm, "right_steering_hinge_joint");

  if (left_rear_joint_   == gz::sim::kNullEntity ||
      right_rear_joint_  == gz::sim::kNullEntity ||
      left_steer_joint_  == gz::sim::kNullEntity ||
      right_steer_joint_ == gz::sim::kNullEntity)
  {
    gzerr << "[FsAckermannDrive] Failed to find one or more joints. "
          << "Check joint names in URDF.\n";
    return;
  }

  // ── Create control & feedback components ────────────────────────────────
  for (auto j : {left_rear_joint_, right_rear_joint_,
                  left_steer_joint_, right_steer_joint_})
  {
    ensureVelCmd(ecm, j);
  }
  for (auto j : {left_steer_joint_, right_steer_joint_})
    ensurePosFeedback(ecm, j);

  // ── Subscribe to cmd_vel ─────────────────────────────────────────────────
  node_.Subscribe(topic, &FsAckermannDrive::OnCmdVel, this);

  gzmsg << "[FsAckermannDrive] Loaded. topic=" << topic
        << "  wheel_radius=" << wheel_radius_
        << "  wheel_base="   << wheel_base_
        << "  wheel_sep="    << wheel_sep_
        << "  max_steer="    << max_steer_
        << "  max_accel="    << max_accel_
        << "  max_decel="    << max_decel_ << "\n";

  configured_ = true;
}

// ─────────────────────────────────────────────────────────────────────────────
// ISystemPreUpdate  — runs every physics tick, before integration
// ─────────────────────────────────────────────────────────────────────────────
void FsAckermannDrive::PreUpdate(
  const gz::sim::UpdateInfo & info,
  gz::sim::EntityComponentManager & ecm)
{
  if (!configured_ || info.paused) return;

  // Time step in seconds (0.008 s at 125 Hz physics)
  const double dt = std::chrono::duration<double>(info.dt).count();

  // Snapshot desired state set by the command callback
  double target_speed, target_steer;
  {
    std::lock_guard<std::mutex> lk(mutex_);
    target_speed = target_speed_;
    target_steer = target_steer_;
  }

  // ── 1. Speed ramping ─────────────────────────────────────────────────────
  // Advance current_speed_ toward target at max_accel or max_decel rate.
  // Decelerating = we want a smaller speed magnitude, or crossing through zero.
  // This prevents the instant speed jumps that make teleop feel jerky.
  {
    const double error = target_speed - current_speed_;
    const bool decelerating = (error * current_speed_ < 0.0) ||
                              (std::abs(target_speed) < std::abs(current_speed_));
    const double rate = decelerating ? max_decel_ : max_accel_;
    const double step = rate * dt;

    if (std::abs(error) <= step)
      current_speed_ = target_speed;   // close enough — snap to target
    else
      current_speed_ += std::copysign(step, error);
  }

  // ── 2. Rear wheel drive ──────────────────────────────────────────────────
  const double wheel_vel = current_speed_ / wheel_radius_;
  for (auto j : {left_rear_joint_, right_rear_joint_}) {
    auto * cmd = ecm.Component<gz::sim::components::JointVelocityCmd>(j);
    if (cmd) cmd->Data()[0] = wheel_vel;
  }

  // ── 3. Per-wheel Ackermann steer angles ──────────────────────────────────
  // For centre steer angle δ, the turning radius is R = L / tan(δ).
  // Inner wheel (tighter arc) must turn more than outer to avoid tyre scrub:
  //   δ_inner = atan( L / (R − w/2) )
  //   δ_outer = atan( L / (R + w/2) )
  // The left hinge is inner for a left turn (δ > 0) and vice versa.
  // The formula handles both signs correctly because tan and atan are odd.
  double left_target, right_target;
  if (std::abs(target_steer) < 1e-6) {
    left_target = right_target = 0.0;
  } else {
    const double R      = wheel_base_ / std::tan(target_steer);
    const double half_w = wheel_sep_ / 2.0;
    left_target  = std::atan(wheel_base_ / (R - half_w));
    right_target = std::atan(wheel_base_ / (R + half_w));
  }
  left_target  = std::clamp(left_target,  -max_steer_, max_steer_);
  right_target = std::clamp(right_target, -max_steer_, max_steer_);

  // ── 4. Steering hinge P-velocity controller ──────────────────────────────
  // Read current hinge angle, compute error, push a proportional velocity.
  // The physics engine integrates velocity → smooth, realistic steering feel.
  const double targets[2] = {left_target, right_target};
  const gz::sim::Entity hinges[2] = {left_steer_joint_, right_steer_joint_};

  for (int i = 0; i < 2; ++i) {
    double current_angle = 0.0;
    const auto * pos = ecm.Component<gz::sim::components::JointPosition>(hinges[i]);
    if (pos && !pos->Data().empty())
      current_angle = pos->Data()[0];

    const double error   = targets[i] - current_angle;
    const double vel_cmd = std::clamp(steer_kp_ * error,
                                      -max_steer_rate_, max_steer_rate_);

    auto * cmd = ecm.Component<gz::sim::components::JointVelocityCmd>(hinges[i]);
    if (cmd) cmd->Data()[0] = vel_cmd;
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// gz-transport callback  (called on gz internal thread — use mutex)
// ─────────────────────────────────────────────────────────────────────────────
void FsAckermannDrive::OnCmdVel(const gz::msgs::Twist & msg)
{
  const double v = msg.linear().x();
  const double w = msg.angular().z();

  // Ackermann inverse kinematics for the centre steering angle:
  //   δ = atan( L · ω / |v| )
  // At very low speed, fall back to a direct ω→δ mapping to avoid ÷0.
  double steer = 0.0;
  if (std::abs(v) > 0.05) {
    steer = std::atan2(wheel_base_ * w, std::abs(v));
  } else {
    // Scale so max angular.z ≈ wheel_base_ rad/s maps to max_steer
    steer = std::clamp(w * (wheel_base_ / 2.0), -max_steer_, max_steer_);
  }
  steer = std::clamp(steer, -max_steer_, max_steer_);

  std::lock_guard<std::mutex> lk(mutex_);
  target_speed_ = v;
  target_steer_ = steer;
}

}  // namespace fs_plugins

// ── Plugin registration ──────────────────────────────────────────────────────
GZ_ADD_PLUGIN(
  fs_plugins::FsAckermannDrive,
  gz::sim::System,
  gz::sim::ISystemConfigure,
  gz::sim::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(fs_plugins::FsAckermannDrive, "fs_plugins::FsAckermannDrive")
