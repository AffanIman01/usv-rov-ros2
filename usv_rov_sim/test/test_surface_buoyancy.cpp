#include <usv_rov_sim/surface_buoyancy.hpp>
#include <iostream>
#include <limits>
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
  auto nan = ComputeBoxBuoyancy(size, gz::math::Pose3d::Zero, origin, gravity, std::numeric_limits<double>::quiet_NaN());
  Require(!nan.valid, "Nonfinite fluid surface must fail.");
  auto invalid = ComputeBoxBuoyancy({-1,1,1}, gz::math::Pose3d::Zero, origin, gravity, 0);
  Require(!invalid.valid, "Invalid collision size must fail.");
  std::cout << "PASS: wet/dry/half displacement, pitch and roll restoring torque, heading rotation, translation invariance and invalid geometry.\n";
}
