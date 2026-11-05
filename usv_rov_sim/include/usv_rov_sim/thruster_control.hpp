#ifndef USV_ROV_SIM__THRUSTER_CONTROL_HPP_
#define USV_ROV_SIM__THRUSTER_CONTROL_HPP_

#include <algorithm>
#include <chrono>
#include <cmath>

namespace usv_rov_sim
{
using SteadyClock = std::chrono::steady_clock;

// Estimated simulation-only control limits. These are not ME120 calibration.
struct ControlParameters
{
  double maxSpeed{1.0};
  double maxYawRate{0.6};
  double maxThrust{40.0};
  double surgeGain{120.0};
  double yawGain{60.0};
  double portY{0.55};
  double starboardY{-0.55};
  double timeoutSeconds{0.4};

  bool Valid() const
  {
    const double values[] = {maxSpeed, maxYawRate, maxThrust, surgeGain,
      yawGain, portY, starboardY, timeoutSeconds};
    for (const double value : values) if (!std::isfinite(value)) return false;
    return maxSpeed > 0 && maxYawRate > 0 && maxThrust > 0 &&
      surgeGain > 0 && yawGain > 0 && portY > starboardY && timeoutSeconds > 0;
  }
};

struct VelocityCommand
{
  double surge{0.0};
  double yawRate{0.0};
  SteadyClock::time_point receivedAt{};
  bool valid{false};
};

struct ThrustPair
{
  double port{0.0};
  double starboard{0.0};
};

inline ThrustPair ComputeThrust(const ControlParameters &p,
  const VelocityCommand &command, double bodySurge, double bodyYawRate,
  SteadyClock::time_point now)
{
  const double age = std::chrono::duration<double>(now - command.receivedAt).count();
  if (!p.Valid() || !command.valid || !std::isfinite(command.surge) ||
      !std::isfinite(command.yawRate) || !std::isfinite(bodySurge) ||
      !std::isfinite(bodyYawRate) || age < 0 || age > p.timeoutSeconds)
    return {};

  // An explicit stop cuts power; hydrodynamic damping brings the vessel to rest.
  if (command.surge == 0.0 && command.yawRate == 0.0) return {};
  const double targetSurge = std::clamp(command.surge, -p.maxSpeed, p.maxSpeed);
  const double targetYaw = std::clamp(command.yawRate, -p.maxYawRate, p.maxYawRate);
  const double force = p.surgeGain * (targetSurge - bodySurge);
  const double moment = p.yawGain * (targetYaw - bodyYawRate);
  const double separation = p.portY - p.starboardY;

  // Fx = port + starboard; Mz = -portY*port - starboardY*starboard.
  ThrustPair output{(-moment - p.starboardY * force) / separation,
                   (moment + p.portY * force) / separation};
  if (!std::isfinite(output.port) || !std::isfinite(output.starboard)) return {};
  const double peak = std::max(std::abs(output.port), std::abs(output.starboard));
  if (peak > p.maxThrust)
  {
    const double scale = p.maxThrust / peak;
    output.port *= scale;
    output.starboard *= scale;
  }
  return output;
}
}  // namespace usv_rov_sim
#endif
