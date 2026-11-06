#ifndef USV_ROV_SIM__SURFACE_BUOYANCY_HPP_
#define USV_ROV_SIM__SURFACE_BUOYANCY_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>
#include <utility>
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

namespace detail
{
struct SubmergedBox
{
  double volume{0};
  gz::math::Vector3d centroid{0, 0, 0};
  bool valid{false};
};

// Clip the six box faces against n.dot(point) <= offset, close the cut face,
// and sum positive tetrahedra about an interior point. Unlike the analytic
// inclusion-exclusion formula, this never divides by a tiny normal component.
// Every intersection is a convex combination of two bounded box vertices.
inline SubmergedBox ClipSubmergedBox(const gz::math::Vector3d &size,
  const gz::math::Vector3d &normal, double offset)
{
  using Vec = gz::math::Vector3d;
  SubmergedBox result;
  const Vec half = size * 0.5;
  const double fullVolume = size.X() * size.Y() * size.Z();
  const double extent = std::abs(normal.X()) * half.X() +
    std::abs(normal.Y()) * half.Y() + std::abs(normal.Z()) * half.Z();
  if (!std::isfinite(fullVolume) || !std::isfinite(extent)) return result;
  if (offset <= -extent) { result.valid = true; return result; }
  if (offset >= extent) { result.volume = fullVolume; result.valid = true; return result; }

  const std::array<Vec, 8> vertices{{
    {-half.X(), -half.Y(), -half.Z()}, {half.X(), -half.Y(), -half.Z()},
    {half.X(), half.Y(), -half.Z()}, {-half.X(), half.Y(), -half.Z()},
    {-half.X(), -half.Y(), half.Z()}, {half.X(), -half.Y(), half.Z()},
    {half.X(), half.Y(), half.Z()}, {-half.X(), half.Y(), half.Z()}}};
  const std::array<std::array<int, 4>, 6> indices{{
    {{0, 1, 2, 3}}, {{4, 5, 6, 7}}, {{0, 1, 5, 4}},
    {{3, 2, 6, 7}}, {{0, 3, 7, 4}}, {{1, 2, 6, 5}}}};
  std::vector<std::vector<Vec>> faces;
  std::vector<Vec> cap;
  const double tolerance = 1e-12 * size.Max();
  auto addCap = [&](const Vec &point) {
    for (const auto &existing : cap)
      if ((point - existing).Length() <= tolerance) return;
    cap.push_back(point);
  };
  for (const auto &face : indices)
  {
    std::vector<Vec> clipped;
    for (std::size_t edge = 0; edge < face.size(); ++edge)
    {
      const Vec &a = vertices[face[edge]];
      const Vec &b = vertices[face[(edge + 1) % face.size()]];
      const double da = normal.Dot(a) - offset, db = normal.Dot(b) - offset;
      const bool aInside = da <= 0, bInside = db <= 0;
      if (aInside) clipped.push_back(a);
      if (aInside != bInside)
      {
        // Opposite signs prevent cancellation in the denominator.
        const double fraction = std::clamp(da / (da - db), 0.0, 1.0);
        const Vec point = a + fraction * (b - a);
        clipped.push_back(point);
        addCap(point);
      }
    }
    if (clipped.size() >= 3) faces.push_back(std::move(clipped));
  }
  if (cap.size() >= 3)
  {
    Vec center{0, 0, 0};
    for (const auto &point : cap) center += point;
    center /= static_cast<double>(cap.size());
    const Vec reference = std::abs(normal.X()) < 0.8 ? Vec::UnitX : Vec::UnitY;
    const Vec axisU = normal.Cross(reference).Normalized();
    const Vec axisV = normal.Cross(axisU);
    std::sort(cap.begin(), cap.end(), [&](const Vec &a, const Vec &b) {
      const Vec ra = a - center, rb = b - center;
      return std::atan2(ra.Dot(axisV), ra.Dot(axisU)) <
             std::atan2(rb.Dot(axisV), rb.Dot(axisU));
    });
    faces.push_back(std::move(cap));
  }
  Vec interior{0, 0, 0};
  std::size_t count = 0;
  for (const auto &face : faces) for (const auto &point : face) { interior += point; ++count; }
  if (count == 0) { result.valid = true; return result; }
  interior /= static_cast<double>(count);
  Vec moment{0, 0, 0};
  for (const auto &face : faces)
    for (std::size_t triangle = 1; triangle + 1 < face.size(); ++triangle)
    {
      const Vec &a = face[0], &b = face[triangle], &c = face[triangle + 1];
      const double volume = std::abs((a - interior).Dot((b - interior).Cross(c - interior))) / 6.0;
      result.volume += volume;
      moment += volume * (interior + a + b + c) * 0.25;
    }
  if (!std::isfinite(result.volume) || result.volume < 0 ||
      result.volume > fullVolume * (1.0 + 1e-10)) return {};
  if (result.volume > 0)
  {
    result.centroid = moment / result.volume;
    if (!result.centroid.IsFinite()) return {};
    for (int axis = 0; axis < 3; ++axis)
      if (std::abs(result.centroid[axis]) > half[axis] + tolerance) return {};
  }
  result.volume = std::min(result.volume, fullVolume);
  result.valid = true;
  return result;
}
}  // namespace detail

// Flat water at world z=surfaceZ. Air acts over the full box; the additional
// water-air force acts at the clipped submerged centroid. Torque is about the
// link origin, matching Link::AddWorldWrench. No pose/velocity override is used.
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

  const double fullVolume = size.X() * size.Y() * size.Z();
  auto normal = collisionWorldPose.Rot().RotateVectorReverse(gz::math::Vector3d::UnitZ);
  for (int axis = 0; axis < 3; ++axis)
    if (std::abs(normal[axis]) < 1e-14) normal[axis] = 0;
  const double normalLength = normal.Length();
  if (!std::isfinite(fullVolume) || !std::isfinite(normalLength) || normalLength <= 0) return result;
  const auto clipped = detail::ClipSubmergedBox(size, normal / normalLength,
    (surfaceZ - collisionWorldPose.Pos().Z()) / normalLength);
  if (!clipped.valid) return result;
  result.submergedVolume = clipped.volume;
  result.force = -airDensity * fullVolume * gravity;
  result.torque = (collisionWorldPose.Pos() - linkOriginWorld).Cross(result.force);
  if (result.submergedVolume > 0)
  {
    const auto centerWorld = collisionWorldPose.Pos() +
      collisionWorldPose.Rot().RotateVector(clipped.centroid);
    const auto forceWater = -(waterDensity - airDensity) * result.submergedVolume * gravity;
    result.force += forceWater;
    result.torque += (centerWorld - linkOriginWorld).Cross(forceWater);
  }
  result.valid = result.force.IsFinite() && result.torque.IsFinite();
  return result;
}
}  // namespace usv_rov_sim
#endif
