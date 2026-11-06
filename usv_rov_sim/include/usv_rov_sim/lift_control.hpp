#ifndef USV_ROV_SIM__LIFT_CONTROL_HPP_
#define USV_ROV_SIM__LIFT_CONTROL_HPP_

#include <algorithm>
#include <chrono>
#include <cmath>

namespace usv_rov_sim
{
// Studio-derived travel and rate; actuator masses/gains are simulation estimates.
struct LiftParameters
{
  double travel{0.418 * 0.65};
  double maxSpeed{0.418 * 0.25};
  double acceleration{0.25};
  double movingMass{5.0};
  double positionGain{1000.0};
  double velocityGain{120.0};
  double maxForce{150.0};
  double timeoutSeconds{0.4};
  bool Valid() const
  {
    for (double v : {travel, maxSpeed, acceleration, movingMass, positionGain,
                     velocityGain, maxForce, timeoutSeconds})
      if (!std::isfinite(v) || v <= 0) return false;
    return true;
  }
};

struct LiftCommand
{
  double direction{0};  // -1 raises, +1 lowers, zero holds measured position.
  std::chrono::steady_clock::time_point receivedAt{};
  bool valid{false};
};

struct LiftState
{
  double targetPosition{0};
  double targetVelocity{0};
  bool initialized{false};
  bool moving{false};
};

// Output is generalized force along the joint's downward axis. It never writes
// a pose, a joint position or an instantaneous physical velocity.
inline double ComputeLiftForce(const LiftParameters &p, LiftState &state,
  const LiftCommand &command, double position, double velocity,
  double gravityAlongAxis, double dt, std::chrono::steady_clock::time_point now)
{
  if (!p.Valid() || !std::isfinite(position) || !std::isfinite(velocity) ||
      !std::isfinite(gravityAlongAxis) || !std::isfinite(dt) || dt <= 0)
    return 0;
  const double age = std::chrono::duration<double>(now - command.receivedAt).count();
  const bool active = command.valid && std::isfinite(command.direction) &&
    command.direction != 0 && age >= 0 && age <= p.timeoutSeconds;
  if (!state.initialized || state.moving != active)
  {
    // Release, expiry and invalid input each capture the measured position once.
    // Repeated stop heartbeats must not let gravity slowly move the hold target.
    state.targetPosition = std::clamp(position, 0.0, p.travel);
    state.targetVelocity = 0;
    state.initialized = true;
  }
  state.moving = active;
  if (active)
  {
    const double direction = std::clamp(command.direction, -1.0, 1.0);
    const double remaining = direction > 0 ? p.travel - state.targetPosition : state.targetPosition;
    const double limit = std::min(p.maxSpeed, std::sqrt(2 * p.acceleration * std::max(0.0, remaining)));
    const double desired = direction * limit;
    state.targetVelocity += std::clamp(desired - state.targetVelocity,
      -p.acceleration * dt, p.acceleration * dt);
    state.targetPosition = std::clamp(state.targetPosition + state.targetVelocity * dt, 0.0, p.travel);
    if ((state.targetPosition == 0 && state.targetVelocity < 0) ||
        (state.targetPosition == p.travel && state.targetVelocity > 0))
      state.targetVelocity = 0;
  }
  // Gravity compensation follows vessel roll/pitch. PD feedback supplies the
  // physical acceleration/braking force and maintains height after release.
  const double force = -p.movingMass * gravityAlongAxis +
    p.positionGain * (state.targetPosition - position) +
    p.velocityGain * (state.targetVelocity - velocity);
  return std::isfinite(force) ? std::clamp(force, -p.maxForce, p.maxForce) : 0;
}
}  // namespace usv_rov_sim
#endif
