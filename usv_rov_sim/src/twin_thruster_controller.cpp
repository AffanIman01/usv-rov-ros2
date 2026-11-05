#include <usv_rov_sim/thruster_control.hpp>

#include <memory>
#include <mutex>
#include <string>

#include <gz/common/Console.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/twist.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/transport/Node.hh>

namespace usv_rov_sim
{
// Applies physical forces, not pose or velocity overrides. Buoyancy and drag
// remain the responsibility of the world's standard Gazebo systems.
class TwinThrusterController final : public gz::sim::System,
  public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate,
  public gz::sim::ISystemReset
{
public:
  void Configure(const gz::sim::Entity &entity,
    const std::shared_ptr<const sdf::Element> &sdf,
    gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &) override
  {
    const gz::sim::Model model(entity);
    if (!model.Valid(ecm))
    {
      gzerr << "TwinThrusterController must be attached to a model.\n";
      return;
    }
    const std::string linkName = sdf->Get<std::string>("link_name", "base_link").first;
    link.ResetEntity(model.LinkByName(ecm, linkName));
    if (!link.Valid(ecm))
    {
      gzerr << "TwinThrusterController cannot find link " << linkName << "\n";
      return;
    }
    portPosition = sdf->Get<gz::math::Vector3d>("port_position", portPosition).first;
    starboardPosition = sdf->Get<gz::math::Vector3d>("starboard_position", starboardPosition).first;
    parameters.maxSpeed = sdf->Get<double>("max_speed", parameters.maxSpeed).first;
    parameters.maxYawRate = sdf->Get<double>("max_yaw_rate", parameters.maxYawRate).first;
    parameters.maxThrust = sdf->Get<double>("max_thrust", parameters.maxThrust).first;
    parameters.surgeGain = sdf->Get<double>("surge_gain", parameters.surgeGain).first;
    parameters.yawGain = sdf->Get<double>("yaw_gain", parameters.yawGain).first;
    parameters.timeoutSeconds = sdf->Get<double>("command_timeout", parameters.timeoutSeconds).first;
    parameters.portY = portPosition.Y();
    parameters.starboardY = starboardPosition.Y();
    if (!parameters.Valid() || !portPosition.IsFinite() || !starboardPosition.IsFinite())
    {
      gzerr << "TwinThrusterController has invalid finite/positive control parameters or thruster positions.\n";
      return;
    }
    link.EnableVelocityChecks(ecm);
    const std::string topic = sdf->Get<std::string>("topic", "/model/me120/cmd_vel").first;
    node = std::make_unique<gz::transport::Node>();
    if (!node->Subscribe(topic, &TwinThrusterController::OnCommand, this))
    {
      gzerr << "TwinThrusterController could not subscribe to " << topic << "\n";
      node.reset();
      return;
    }
    configured = true;
    gzmsg << "TwinThrusterController: " << topic << ", max " << parameters.maxThrust
          << " N per thruster, wall-clock timeout " << parameters.timeoutSeconds
          << " s. Simulation estimates only.\n";
  }

  void Reset(const gz::sim::UpdateInfo &, gz::sim::EntityComponentManager &) override
  {
    std::lock_guard<std::mutex> lock(commandMutex);
    latestCommand = {};
  }

  void PreUpdate(const gz::sim::UpdateInfo &info,
    gz::sim::EntityComponentManager &ecm) override
  {
    if (!configured || info.paused) return;
    if (info.dt < std::chrono::steady_clock::duration::zero())
    {
      std::lock_guard<std::mutex> lock(commandMutex);
      latestCommand = {};
      return;
    }
    const auto worldLinear = link.WorldLinearVelocity(ecm);
    const auto worldAngular = link.WorldAngularVelocity(ecm);
    if (!worldLinear || !worldAngular) return;
    const auto pose = gz::sim::worldPose(link.Entity(), ecm);
    const auto linear = pose.Rot().RotateVectorReverse(*worldLinear);
    const auto angular = pose.Rot().RotateVectorReverse(*worldAngular);
    VelocityCommand command;
    {
      std::lock_guard<std::mutex> lock(commandMutex);
      command = latestCommand;
    }
    const auto thrust = ComputeThrust(parameters, command, linear.X(), angular.Z(), SteadyClock::now());
    const gz::math::Vector3d portForce(thrust.port, 0, 0);
    const gz::math::Vector3d starboardForce(thrust.starboard, 0, 0);
    const auto force = pose.Rot().RotateVector(portForce + starboardForce);
    const auto torque = pose.Rot().RotateVector(
      portPosition.Cross(portForce) + starboardPosition.Cross(starboardForce));
    if (force.IsFinite() && torque.IsFinite()) link.AddWorldWrench(ecm, force, torque);
  }

private:
  void OnCommand(const gz::msgs::Twist &message)
  {
    const double values[] = {message.linear().x(), message.linear().y(), message.linear().z(),
      message.angular().x(), message.angular().y(), message.angular().z()};
    VelocityCommand command;
    command.receivedAt = SteadyClock::now();
    command.valid = true;
    for (const double value : values)
      if (!std::isfinite(value)) command.valid = false;
    if (command.valid)
    {
      command.surge = message.linear().x();
      command.yawRate = message.angular().z();
    }
    std::lock_guard<std::mutex> lock(commandMutex);
    latestCommand = command;
  }

  bool configured{false};
  ControlParameters parameters;
  gz::math::Vector3d portPosition{-1.08, 0.55, -0.15};
  gz::math::Vector3d starboardPosition{-1.08, -0.55, -0.15};
  gz::sim::Link link;
  std::mutex commandMutex;
  VelocityCommand latestCommand;
  // Destroy transport first so callbacks finish before their state disappears.
  std::unique_ptr<gz::transport::Node> node;
};
}  // namespace usv_rov_sim

GZ_ADD_PLUGIN(usv_rov_sim::TwinThrusterController, gz::sim::System,
  gz::sim::ISystemConfigure, gz::sim::ISystemPreUpdate, gz::sim::ISystemReset)
GZ_ADD_PLUGIN_ALIAS(usv_rov_sim::TwinThrusterController, "usv_rov_sim::TwinThrusterController")
