#include <usv_rov_sim/surface_buoyancy.hpp>
#include <iostream>
#include <limits>
#include <array>
#include <stdexcept>

using namespace usv_rov_sim;
void Require(bool pass, const char *message) { if (!pass) throw std::runtime_error(message); }
bool Near(double a, double b, double tolerance=1e-8) { return std::abs(a-b) <= tolerance; }
int main()
{
  const gz::math::Vector3d size(2, 0.3, 0.3), gravity(0, 0, -9.81), origin(0, 0, 0);
  const double volume = 0.18;
  auto wet = ComputeBoxBuoyancy(size, gz::math::Pose3d(0,0,-1,0,0,0), origin, gravity, 0);
  Require(wet.valid && Near(wet.submergedVolume, volume) && Near(wet.force.Z(), 1000*volume*9.81), "Fully submerged box must displace water.");
  auto dry = ComputeBoxBuoyancy(size, gz::math::Pose3d(0,0,1,0,0,0), origin, gravity, 0);
  Require(dry.valid && Near(dry.submergedVolume, 0) && Near(dry.force.Z(), 1.225*volume*9.81), "Dry box must displace air only.");
  auto half = ComputeBoxBuoyancy(size, gz::math::Pose3d::Zero, origin, gravity, 0);
  Require(half.valid && Near(half.submergedVolume, volume/2) && Near(half.force.Z(), (1000+1.225)*volume/2*9.81), "Half-submerged box buoyancy must be exact.");
  Require(half.torque.Length() < 1e-8, "Level centered box must have no pitch or roll torque.");
  auto pitch = ComputeBoxBuoyancy(size, gz::math::Pose3d(0,0,0,0,0.08,0), origin, gravity, 0);
  auto negativePitch = ComputeBoxBuoyancy(size, gz::math::Pose3d(0,0,0,0,-0.08,0), origin, gravity, 0);
  Require(pitch.valid && negativePitch.valid && pitch.torque.Y() < 0 && negativePitch.torque.Y() > 0,
    "Partially submerged pitched box must produce restoring torque in both directions.");
  Require(Near(pitch.torque.Y(), -negativePitch.torque.Y()), "Pitch restoring torques must be symmetric.");
  // A broad waterplane is roll-stable; a narrow square-section box with its
  // CG at the geometric center need not be. The vessel uses separated hulls.
  auto roll = ComputeBoxBuoyancy({2,1,0.3}, gz::math::Pose3d(0,0,0,0.08,0,0), origin, gravity, 0);
  Require(roll.valid && roll.torque.X() < 0, "Positive roll must produce negative restoring torque.");
  auto yawedPitch = ComputeBoxBuoyancy(size, gz::math::Pose3d(0,0,0,0,0.08,1.5707963267948966), origin, gravity, 0);
  Require(yawedPitch.valid && yawedPitch.torque.X() > 0 && std::abs(yawedPitch.torque.Y()) < 1e-8,
    "Buoyancy torque must rotate with heading.");
  const gz::math::Vector3d displacedOrigin(3,4,2);
  auto translated = ComputeBoxBuoyancy(size, gz::math::Pose3d(3,4,2,0,0.08,0), displacedOrigin, gravity, 2);
  Require(translated.valid && Near(translated.force.Z(), pitch.force.Z()) && Near(translated.torque.Y(), pitch.torque.Y()),
    "Changing world origin and water height together must preserve local wrench.");
  // Regression: a nearly zero roll component made the old analytic centroid
  // produce ~2.26e18 Nm of roll torque at this recorded wall-contact pose.
  const gz::math::Vector3d hullSize(2.2, 0.28, 0.25);
  const gz::math::Pose3d recorded(8.8946983262828461, -8.6219629171740981e-18,
    -0.024389508943275004, 1.1345221308171029e-16,
    -0.022311951229055924, -1.5527909492448632e-17);
  gz::math::Vector3d summedForce, summedTorque;
  for (double y : {-0.54, 0.54})
  {
    const auto pose = recorded * gz::math::Pose3d(0, y, -0.125, 0, 0, 0);
    auto wrench = ComputeBoxBuoyancy(hullSize, pose, recorded.Pos(), gravity, 0);
    Require(wrench.valid && wrench.submergedVolume <= 0.1540000001,
      "Recorded nearly-axis-aligned case must have physical displaced volume.");
    Require(wrench.torque.Length() <= wrench.force.Length() * 1.5,
      "Buoyancy centroid must stay within its collision box.");
    summedForce += wrench.force; summedTorque += wrench.torque;
  }
  Require(summedForce.Z() > 2900 && summedForce.Z() <= 3021.480001 &&
      std::abs(summedTorque.X()) < 1e-7 && std::abs(summedTorque.Y()) < 100,
    "Recorded symmetric hulls must have bounded pitch and negligible roll torque.");

  // Independent geometry identities catch finite but inaccurate volume/center
  // calculations, including the old inclusion-exclusion cancellation error.
  for (double tiny : {0.0, 1e-16, -1e-16, 1e-12, -1e-12, 1e-8, -1e-8,
                      1e-6, -1e-6, 1e-4, -1e-4, 1e-2, -1e-2})
  {
    const auto rotation = gz::math::Quaterniond(tiny, -0.022311951229055924, 0.3);
    const auto normal = rotation.RotateVectorReverse(gz::math::Vector3d::UnitZ);
    const double extent = std::abs(normal.X()) * hullSize.X()/2 +
      std::abs(normal.Y()) * hullSize.Y()/2 + std::abs(normal.Z()) * hullSize.Z()/2;
    const double full = hullSize.X()*hullSize.Y()*hullSize.Z();
    auto centerCut = detail::ClipSubmergedBox(hullSize, normal, 0);
    Require(centerCut.valid && Near(centerCut.volume, full/2, 1e-11),
      "Any plane through a centered box must cut exactly half its volume.");
    for (double fraction : {0.0, 1e-10, 0.2, 0.8, 0.9999999999, 1.0})
    {
      auto below = detail::ClipSubmergedBox(hullSize, normal, fraction*extent);
      auto complement = detail::ClipSubmergedBox(hullSize, normal, -fraction*extent);
      Require(below.valid && complement.valid &&
          Near(below.volume + complement.volume, full, 1e-10),
        "Complementary cuts must conserve displaced volume near every axis.");
      Require((below.volume*below.centroid - complement.volume*complement.centroid).Length() < 1e-10,
        "Complementary cuts must conserve the first moment of a centered box.");
    }
    const gz::math::Pose3d submerged(0, 0, -2, tiny, 0.7, -0.4);
    auto fullyWet = ComputeBoxBuoyancy(hullSize, submerged, submerged.Pos(), gravity, 0);
    Require(fullyWet.valid && Near(fullyWet.submergedVolume, full) && fullyWet.torque.Length() < 1e-8,
      "Fully submerged rotated boxes must act at their geometric center.");
    const gz::math::Pose3d almostLevel(0, 0, 0, tiny, -0.022311951229055924, 0);
    const auto reference = ComputeBoxBuoyancy(hullSize, gz::math::Pose3d(0,0,0,0,-0.022311951229055924,0), origin, gravity, 0);
    auto perturbed = ComputeBoxBuoyancy(hullSize, almostLevel, origin, gravity, 0);
    Require(perturbed.valid && (perturbed.force-reference.force).Length() < 1e-8 &&
        (perturbed.torque-reference.torque).Length() < 1e-7 + 1000*std::abs(tiny),
      "Buoyancy wrench must vary continuously as roll crosses zero.");
  }
  auto nan = ComputeBoxBuoyancy(size, gz::math::Pose3d::Zero, origin, gravity, std::numeric_limits<double>::quiet_NaN());
  Require(!nan.valid, "Nonfinite fluid surface must fail.");
  auto invalid = ComputeBoxBuoyancy({-1,1,1}, gz::math::Pose3d::Zero, origin, gravity, 0);
  Require(!invalid.valid, "Invalid collision size must fail.");
  std::cout << "PASS: wet/dry/half displacement, pitch and roll restoring torque, heading rotation, translation invariance near-axis clipping regression, complement moments and invalid geometry.\n";
}
