#include <usv_rov_sim/lift_control.hpp>

#include <memory>
#include <mutex>
#include <string>
#include <gz/common/Console.hh>
#include <gz/msgs/double.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Gravity.hh>
#include <gz/transport/Node.hh>

namespace usv_rov_sim
{
class LiftController final : public gz::sim::System,
  public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate,
  public gz::sim::ISystemPostUpdate, public gz::sim::ISystemReset
{
public:
  void Configure(const gz::sim::Entity &entity,
    const std::shared_ptr<const sdf::Element> &sdf,
    gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &) override
  {
    const gz::sim::Model model(entity);
    if (!model.Valid(ecm)) return;
    joint.ResetEntity(model.JointByName(ecm, sdf->Get<std::string>("joint_name", "lift_joint").first));
    parent.ResetEntity(model.LinkByName(ecm, sdf->Get<std::string>("parent_link", "base_link").first));
    parameters.travel = sdf->Get<double>("travel", parameters.travel).first;
    parameters.maxSpeed = sdf->Get<double>("max_speed", parameters.maxSpeed).first;
    parameters.acceleration = sdf->Get<double>("acceleration", parameters.acceleration).first;
    parameters.movingMass = sdf->Get<double>("moving_mass", parameters.movingMass).first;
    parameters.positionGain = sdf->Get<double>("position_gain", parameters.positionGain).first;
    parameters.velocityGain = sdf->Get<double>("velocity_gain", parameters.velocityGain).first;
    parameters.maxForce = sdf->Get<double>("max_force", parameters.maxForce).first;
    parameters.timeoutSeconds = sdf->Get<double>("command_timeout", parameters.timeoutSeconds).first;
    if (!joint.Valid(ecm) || !parent.Valid(ecm) || !parameters.Valid())
    {
      gzerr << "LiftController requires a valid lift joint, parent link and positive finite parameters.\n";
      return;
    }
    joint.EnablePositionCheck(ecm);
    joint.EnableVelocityCheck(ecm);
    const auto topic = sdf->Get<std::string>("topic", "/model/me120/lift/velocity_cmd").first;
    const auto feedback = sdf->Get<std::string>("position_topic", "/model/me120/lift/position").first;
    node = std::make_unique<gz::transport::Node>();
    positionPublisher = std::make_unique<gz::transport::Node::Publisher>(node->Advertise<gz::msgs::Double>(feedback));
    if (!*positionPublisher || !node->Subscribe(topic, &LiftController::OnCommand, this))
    {
      gzerr << "LiftController could not create command/feedback transport.\n";
      return;
    }
    configured = true;
    gzmsg << "LiftController: force-controlled downward prismatic travel " << parameters.travel
          << " m; command wall timeout " << parameters.timeoutSeconds << " s. Prototype estimates only.\n";
  }

  void Reset(const gz::sim::UpdateInfo &, gz::sim::EntityComponentManager &) override
  {
    state = {};
    world = gz::sim::kNullEntity;
    lastPublication = {};
    std::lock_guard<std::mutex> lock(commandMutex);
    latestCommand = {};
  }

  void PreUpdate(const gz::sim::UpdateInfo &info, gz::sim::EntityComponentManager &ecm) override
  {
    if (!configured || info.paused) return;
    if (info.dt < std::chrono::steady_clock::duration::zero())
    {
      Reset(info, ecm);
      joint.SetForce(ecm, {0});
      return;
    }
    // Configure occurs before the model has its world parent in Gazebo.
    if (world == gz::sim::kNullEntity) world = gz::sim::worldEntity(parent.Entity(), ecm);
    const auto gravity = ecm.Component<gz::sim::components::Gravity>(world);
    const auto position = joint.Position(ecm);
    const auto velocity = joint.Velocity(ecm);
    if (!gravity || !position || position->empty() || !velocity || velocity->empty()) return;
    const auto axisWorld = gz::sim::worldPose(parent.Entity(), ecm).Rot().RotateVector({0, 0, -1});
    LiftCommand command;
    {
      std::lock_guard<std::mutex> lock(commandMutex);
      command = latestCommand;
    }
    joint.SetForce(ecm, {ComputeLiftForce(parameters, state, command, position->at(0),
      velocity->at(0), gravity->Data().Dot(axisWorld),
      std::chrono::duration<double>(info.dt).count(), std::chrono::steady_clock::now())});
  }

  void PostUpdate(const gz::sim::UpdateInfo &info, const gz::sim::EntityComponentManager &ecm) override
  {
    if (!configured || info.paused || info.simTime - lastPublication < std::chrono::milliseconds(50)) return;
    const auto position = joint.Position(ecm);
    if (!position || position->empty() || !std::isfinite(position->at(0))) return;
    gz::msgs::Double message;
    message.set_data(position->at(0));
    positionPublisher->Publish(message);
    lastPublication = info.simTime;
  }

private:
  void OnCommand(const gz::msgs::Double &message)
  {
    LiftCommand command{message.data(), std::chrono::steady_clock::now(), std::isfinite(message.data())};
    std::lock_guard<std::mutex> lock(commandMutex);
    latestCommand = command;
  }
  bool configured{false};
  gz::sim::Joint joint;
  gz::sim::Link parent;
  gz::sim::Entity world{gz::sim::kNullEntity};
  LiftParameters parameters;
  LiftState state;
  std::chrono::steady_clock::duration lastPublication{};
  std::mutex commandMutex;
  LiftCommand latestCommand;
  std::unique_ptr<gz::transport::Node::Publisher> positionPublisher;
  std::unique_ptr<gz::transport::Node> node;
};
}  // namespace usv_rov_sim

GZ_ADD_PLUGIN(usv_rov_sim::LiftController, gz::sim::System,
  gz::sim::ISystemConfigure, gz::sim::ISystemPreUpdate,
  gz::sim::ISystemPostUpdate, gz::sim::ISystemReset)
GZ_ADD_PLUGIN_ALIAS(usv_rov_sim::LiftController, "usv_rov_sim::LiftController")
