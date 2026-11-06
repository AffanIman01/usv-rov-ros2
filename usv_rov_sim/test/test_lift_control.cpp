#include <usv_rov_sim/lift_control.hpp>
#include <iostream>
#include <limits>
#include <stdexcept>

using namespace usv_rov_sim;
void Require(bool condition, const char *message)
{ if (!condition) throw std::runtime_error(message); }
int main()
{
  const LiftParameters p;
  const auto now = std::chrono::steady_clock::time_point{} + std::chrono::seconds(10);
  LiftState state;
  auto step = [&](LiftCommand command, double q, double v, double gravity=9.81, double dt=.002)
    { return ComputeLiftForce(p, state, command, q, v, gravity, dt, now); };
  const LiftCommand down{1, now, true}, up{-1, now, true}, stop{0, now, true};
  Require(std::abs(step({}, .12, 0) + 49.05) < 1e-9, "Startup must hold current height against gravity.");
  step(down, .12, 0);
  Require(state.targetVelocity > 0 && state.targetVelocity <= .0005 + 1e-12,
    "Start motion must obey acceleration limit.");
  for (int i=0; i<300; ++i) step(down, state.targetPosition, state.targetVelocity);
  Require(state.targetVelocity <= p.maxSpeed && state.targetPosition > .15,
    "Lowering must increase joint position at bounded speed.");
  step(stop, .15, .1);
  Require(!state.moving && state.targetPosition == .15 && state.targetVelocity == 0,
    "Release must capture measured position immediately.");
  step(stop, .151, 0);
  Require(state.targetPosition == .15, "Repeated stop must retain the original hold target.");
  step(down, .15, 0);
  LiftCommand stale{1, now-std::chrono::milliseconds(401), true};
  step(stale, .16, .1);
  Require(!state.moving && state.targetPosition == .16, "Expired input must hold actual height.");
  step(down, .16, 0);
  const LiftCommand future{1, now+std::chrono::seconds(1), true};
  step(future, .165, 0);
  Require(!state.moving, "Future command timestamps must fail closed.");
  step(down, .165, 0);
  step({std::numeric_limits<double>::quiet_NaN(), now, true}, .17, 0);
  Require(!state.moving && state.targetPosition == .17, "Nonfinite commands must hold.");
  Require(step(stop, .17, 0, 0) == 0, "Gravity feedforward must follow axis projection.");
  Require(std::abs(step(stop, 1, 10)) <= p.maxForce, "Effort must be bounded.");
  Require(step(stop, std::numeric_limits<double>::quiet_NaN(), 0) == 0,
    "Invalid joint feedback must not enter physics.");
  state = {};
  for (int i=0; i<3000; ++i) step(down, state.targetPosition, state.targetVelocity);
  Require(state.targetPosition == p.travel && state.targetVelocity == 0,
    "Lower travel stop must remain bounded.");
  for (int i=0; i<3000; ++i) step(up, state.targetPosition, state.targetVelocity);
  Require(state.targetPosition == 0 && state.targetVelocity == 0,
    "Upper travel stop must remain bounded.");
  state = {};
  step(down, .1, 0);
  ComputeLiftForce(p, state, down, .11, .1, 9.81, .002, now+std::chrono::seconds(10));
  Require(!state.moving && state.targetPosition == .11,
    "Paused simulation must not preserve old commands across elapsed wall time.");
  std::cout << "PASS: gravity hold, direction, acceleration, stops, release, timeout, pause expiry and invalid data.\n";
}
