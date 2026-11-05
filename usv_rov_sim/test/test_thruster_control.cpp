#include <usv_rov_sim/thruster_control.hpp>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

using namespace usv_rov_sim;
void Require(bool pass, const char *message) { if (!pass) throw std::runtime_error(message); }
bool Near(double a, double b) { return std::abs(a - b) < 1e-9; }
int main()
{
  const ControlParameters p;
  const auto now = SteadyClock::time_point{} + std::chrono::seconds(10);
  auto command = [&](double surge, double yaw) { return VelocityCommand{surge, yaw, now, true}; };
  auto compute = [&](VelocityCommand c, double speed=0.0, double yaw=0.0)
    { return ComputeThrust(p, c, speed, yaw, now); };
  auto forward = compute(command(0.5, 0));
  Require(Near(forward.port, 30) && Near(forward.starboard, 30), "Forward must drive both thrusters equally.");
  auto reverse = compute(command(-0.5, 0));
  Require(Near(reverse.port, -30) && Near(reverse.starboard, -30), "Reverse must reverse both forces.");
  auto left = compute(command(0, 0.3));
  const double leftMoment = -p.portY * left.port - p.starboardY * left.starboard;
  Require(left.port < 0 && left.starboard > 0 && Near(leftMoment, 18), "Positive yaw must turn left.");
  auto right = compute(command(0, -0.3));
  Require(right.port > 0 && right.starboard < 0, "Negative yaw must turn right.");
  auto mixed = compute(command(1, 0.6));
  Require(Near(std::max(std::abs(mixed.port), std::abs(mixed.starboard)), 40), "Saturation must obey per-thruster limit.");
  const double mixedForce = mixed.port + mixed.starboard;
  const double mixedMoment = -p.portY * mixed.port - p.starboardY * mixed.starboard;
  Require(Near(mixedMoment / mixedForce, 36.0 / 120.0), "Saturation must preserve force/moment ratio.");
  auto excessive = compute(command(1e6, 1e6));
  Require(Near(excessive.port, mixed.port) && Near(excessive.starboard, mixed.starboard), "Velocity targets must clamp before force computation.");
  auto atTarget = compute(command(0.5, 0.2), 0.5, 0.2);
  Require(Near(atTarget.port, 0) && Near(atTarget.starboard, 0), "No correction required at exact target velocity.");
  auto overspeed = compute(command(0.2, 0), 1.0, 0);
  Require(overspeed.port < 0 && overspeed.starboard < 0, "Velocity feedback must oppose overspeed.");
  auto stop = compute(command(0, 0), 0.8, 0.2);
  Require(Near(stop.port, 0) && Near(stop.starboard, 0), "Explicit stop cuts thrust and lets drag coast.");
  auto expired = command(0.5, 0.3); expired.receivedAt -= std::chrono::milliseconds(401);
  auto timeout = compute(expired);
  Require(Near(timeout.port, 0) && Near(timeout.starboard, 0), "Lost stream must cut both thrusters.");
  auto validAgain = compute(command(0.5, 0));
  Require(validAgain.port > 0, "Fresh input must resume after timeout.");
  auto pauseExpired = ComputeThrust(p, command(0.5, 0), 0, 0, now + std::chrono::seconds(10));
  Require(Near(pauseExpired.port, 0) && Near(pauseExpired.starboard, 0), "Wall time must expire commands even if simulation is paused.");
  auto invalid = command(std::numeric_limits<double>::quiet_NaN(), 0);
  auto rejected = compute(invalid);
  Require(Near(rejected.port, 0) && Near(rejected.starboard, 0), "NaN input must never enter physics.");
  auto invalidState = compute(command(0.5, 0), std::numeric_limits<double>::infinity());
  Require(Near(invalidState.port, 0) && Near(invalidState.starboard, 0), "Nonfinite feedback must cut thrust.");
  auto unused = compute(VelocityCommand{});
  Require(Near(unused.port, 0) && Near(unused.starboard, 0), "No command at startup means no thrust.");
  auto future = command(0.5, 0); future.receivedAt += std::chrono::seconds(1);
  Require(Near(compute(future).port, 0), "A negative input age must be rejected.");
  auto bad = p; bad.starboardY = bad.portY;
  Require(!bad.Valid() && Near(ComputeThrust(bad, command(0.5, 0), 0, 0, now).port, 0), "Zero arm separation must fail safely.");
  std::cout << "PASS: forward, reverse, yaw signs, mixing, saturation, target limits, feedback, stop, timeout, pause expiry and invalid data.\n";
  return 0;
}
