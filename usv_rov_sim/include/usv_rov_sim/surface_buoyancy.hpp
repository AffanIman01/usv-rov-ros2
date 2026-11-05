#ifndef USV_ROV_SIM__SURFACE_BUOYANCY_HPP_
#define USV_ROV_SIM__SURFACE_BUOYANCY_HPP_

#include <algorithm>
#include <cmath>
#include <gz/math/Box.hh>
#include <gz/math/Plane.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>

namespace usv_rov_sim
{
struct BuoyancyWrench
{
  gz::math::Vector3d force{0, 0, 0};
  gz::math::Vector3d torque{0, 0, 0};
  double submergedVolume{0};
  bool valid{false};
};

// Flat water at world z=surfaceZ. The two disjoint fluid regions are expressed
// as air over the full box plus (water-air) over the clipped submerged part.
// Torque is about linkOriginWorld, matching Link::AddWorldWrench.
inline BuoyancyWrench ComputeBoxBuoyancy(const gz::math::Vector3d &size,
  const gz::math::Pose3d &collisionWorldPose,
  const gz::math::Vector3d &linkOriginWorld,
  const gz::math::Vector3d &gravity, double surfaceZ,
  double waterDensity = 1000.0, double airDensity = 1.225)
{
  BuoyancyWrench result;
  if (!size.IsFinite() || size.X() <= 0 || size.Y() <= 0 || size.Z() <= 0 ||
      !collisionWorldPose.IsFinite() || !linkOriginWorld.IsFinite() ||
      !gravity.IsFinite() || !std::isfinite(surfaceZ) ||
      !std::isfinite(waterDensity) || !std::isfinite(airDensity) ||
      waterDensity <= 0 || airDensity < 0 || waterDensity < airDensity)
    return result;

  const gz::math::Boxd box(size);
  const double fullVolume = box.Volume();
  const auto normalLocal = collisionWorldPose.Rot().RotateVectorReverse(gz::math::Vector3d::UnitZ);
  const gz::math::Planed plane(normalLocal, surfaceZ - collisionWorldPose.Pos().Z());
  const double rawVolume = box.VolumeBelow(plane);
  if (!std::isfinite(fullVolume) || !std::isfinite(rawVolume)) return result;
  result.submergedVolume = std::clamp(rawVolume, 0.0, fullVolume);
  result.force = -airDensity * fullVolume * gravity;
  result.torque = (collisionWorldPose.Pos() - linkOriginWorld).Cross(result.force);
  if (result.submergedVolume > 0.0)
  {
    const auto centerLocal = box.CenterOfVolumeBelow(plane);
    if (!centerLocal || !centerLocal->IsFinite()) return {};
    const auto centerWorld = collisionWorldPose.Pos() + collisionWorldPose.Rot().RotateVector(*centerLocal);
    const auto forceWater = -(waterDensity - airDensity) * result.submergedVolume * gravity;
    result.force += forceWater;
    result.torque += (centerWorld - linkOriginWorld).Cross(forceWater);
  }
  result.valid = result.force.IsFinite() && result.torque.IsFinite();
  return result;
}
}  // namespace usv_rov_sim
#endif
