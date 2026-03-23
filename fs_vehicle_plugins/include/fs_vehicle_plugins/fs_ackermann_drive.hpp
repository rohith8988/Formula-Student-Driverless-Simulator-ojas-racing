#pragma once

#include <memory>
#include <mutex>
#include <string>

#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/twist.pb.h>

namespace fs_plugins
{

/**
 * FsAckermannDrive
 * ─────────────────
 * Gazebo Harmonic (gz-sim 8) System plugin that drives the FS car via
 * geometry_msgs/Twist commands forwarded through ros_gz_bridge.
 *
 * What it does each physics step:
 *   1. Ramps current speed toward the target at configurable accel/decel rates
 *   2. Commands rear drive wheels via JointVelocityCmd
 *   3. Computes per-wheel Ackermann steer angles (inner > outer)
 *   4. Tracks each steer angle via a P-velocity controller on the hinge joints
 *
 * Ackermann geometry recap:
 *   For a centre steer angle δ and turning radius R = L/tan(δ):
 *     δ_inner = atan( L / (R − w/2) )   ← turns more (smaller radius)
 *     δ_outer = atan( L / (R + w/2) )   ← turns less (larger radius)
 *   Setting the same angle on both hinges causes tyre scrub; this plugin
 *   computes separate angles.
 *
 * SDF parameters (all optional, defaults shown):
 *   <topic>          /cmd_vel   gz-transport topic (gz.msgs.Twist)
 *   <wheel_radius>   0.2032     metres
 *   <wheel_base>     1.58       metres (front-to-rear axle distance)
 *   <wheel_sep>      1.4        metres (left-to-right kingpin distance)
 *   <max_steer>      0.5        radians (~28 deg), joint hard limit
 *   <steer_kp>       8.0        proportional gain for hinge P-controller
 *   <max_steer_rate> 3.0        rad/s, velocity clamp on hinge command
 *   <max_accel>      4.0        m/s², forward acceleration limit
 *   <max_decel>      8.0        m/s², braking / deceleration limit
 */
class FsAckermannDrive
  : public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
{
public:
  FsAckermannDrive() = default;
  ~FsAckermannDrive() override = default;

  // ISystemConfigure — called once when the model is loaded
  void Configure(
    const gz::sim::Entity & entity,
    const std::shared_ptr<const sdf::Element> & sdf,
    gz::sim::EntityComponentManager & ecm,
    gz::sim::EventManager & eventMgr) override;

  // ISystemPreUpdate — called every physics step before integration
  void PreUpdate(
    const gz::sim::UpdateInfo & info,
    gz::sim::EntityComponentManager & ecm) override;

private:
  // gz-transport callback (runs on a separate thread)
  void OnCmdVel(const gz::msgs::Twist & msg);

  gz::sim::Model model_{gz::sim::kNullEntity};
  gz::transport::Node node_;

  // Joint entities (resolved in Configure)
  gz::sim::Entity left_rear_joint_{gz::sim::kNullEntity};
  gz::sim::Entity right_rear_joint_{gz::sim::kNullEntity};
  gz::sim::Entity left_steer_joint_{gz::sim::kNullEntity};
  gz::sim::Entity right_steer_joint_{gz::sim::kNullEntity};

  // Parameters
  double wheel_radius_{0.2032};
  double wheel_base_{1.58};
  double wheel_sep_{1.4};        // kingpin-to-kingpin distance
  double max_steer_{0.5};
  double steer_kp_{8.0};
  double max_steer_rate_{3.0};   // rad/s
  double max_accel_{4.0};        // m/s²
  double max_decel_{8.0};        // m/s²

  // Desired state set by the command callback (mutex-protected)
  double target_speed_{0.0};
  double target_steer_{0.0};
  std::mutex mutex_;

  // Current (ramped) speed — only touched in PreUpdate, no mutex needed
  double current_speed_{0.0};

  bool configured_{false};
};

}  // namespace fs_plugins
