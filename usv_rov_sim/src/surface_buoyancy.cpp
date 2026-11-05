#include <usv_rov_sim/surface_buoyancy.hpp>

#include <memory>
#include <set>
#include <string>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Geometry.hh>
#include <gz/sim/components/Gravity.hh>
#include <sdf/Box.hh>
#include <sdf/Geometry.hh>

namespace usv_rov_sim
{
// Project-specific box-proxy buoyancy. It deliberately targets only the named
// hull collisions, not every visual or collision in the world.
class SurfaceBuoyancy final : public gz::sim::System,
  public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate
{
public:
  void Configure(const gz::sim::Entity &entity,
    const std::shared_ptr<const sdf::Element> &sdf,
    gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &) override
  {
    const gz::sim::Model model(entity);
    if (!model.Valid(ecm))
    {
      gzerr << "SurfaceBuoyancy must be attached to a model.\n";
      return;
    }
    const auto linkName = sdf->Get<std::string>("link_name", "base_link").first;
    link.ResetEntity(model.LinkByName(ecm, linkName));
    surfaceZ = sdf->Get<double>("surface_z", 0.0).first;
    waterDensity = sdf->Get<double>("water_density", 1000.0).first;
    airDensity = sdf->Get<double>("air_density", 1.225).first;
    if (!link.Valid(ecm) ||
        !std::isfinite(surfaceZ) || !std::isfinite(waterDensity) ||
        !std::isfinite(airDensity) || waterDensity <= 0 || airDensity < 0 ||
        waterDensity < airDensity)
    {
      gzerr << "SurfaceBuoyancy has invalid link or density parameters.\n";
      return;
    }
    std::set<std::string> names;
    for (auto element = sdf->FindElement("collision_name"); element;
         element = element->GetNextElement("collision_name"))
      names.insert(element->Get<std::string>());
    if (names.empty()) names = {"port_hull_buoyancy", "starboard_hull_buoyancy"};
    for (const auto &name : names)
    {
      const auto collision = link.CollisionByName(ecm, name);
      const auto geometry = ecm.Component<gz::sim::components::Geometry>(collision);
      if (!geometry || geometry->Data().Type() != sdf::GeometryType::BOX ||
          !geometry->Data().BoxShape())
      {
        gzerr << "SurfaceBuoyancy requires a box collision named " << name << "\n";
        hulls.clear();
        return;
      }
      const auto size = geometry->Data().BoxShape()->Size();
      if (!size.IsFinite() || size.X() <= 0 || size.Y() <= 0 || size.Z() <= 0)
      {
        gzerr << "SurfaceBuoyancy collision " << name << " has invalid box dimensions.\n";
        hulls.clear();
        return;
      }
      hulls.push_back({collision, size});
    }
    configured = true;
    gzmsg << "SurfaceBuoyancy: " << hulls.size() << " box proxies; water surface z="
          << surfaceZ << ", water density=" << waterDensity << " kg/m^3.\n";
  }

  void PreUpdate(const gz::sim::UpdateInfo &info,
    gz::sim::EntityComponentManager &ecm) override
  {
    if (!configured || info.paused || !link.Valid(ecm)) return;
    // Gazebo configures model plugins before SdfEntityCreator attaches the
    // model to its world. Resolve the ancestor here, after entity creation,
    // instead of permanently rejecting an otherwise valid model in Configure.
    if (world == gz::sim::kNullEntity)
      world = gz::sim::worldEntity(link.Entity(), ecm);
    if (world == gz::sim::kNullEntity) return;
    const auto gravity = ecm.Component<gz::sim::components::Gravity>(world);
    if (!gravity || !gravity->Data().IsFinite()) return;
    const auto linkOrigin = gz::sim::worldPose(link.Entity(), ecm).Pos();
    gz::math::Vector3d force{0, 0, 0}, torque{0, 0, 0};
    for (const auto &hull : hulls)
    {
      const auto wrench = ComputeBoxBuoyancy(hull.size,
        gz::sim::worldPose(hull.entity, ecm), linkOrigin, gravity->Data(),
        surfaceZ, waterDensity, airDensity);
      if (!wrench.valid) return;
      force += wrench.force;
      torque += wrench.torque;
    }
    link.AddWorldWrench(ecm, force, torque);
  }

private:
  struct HullBox { gz::sim::Entity entity; gz::math::Vector3d size; };
  bool configured{false};
  gz::sim::Link link;
  gz::sim::Entity world{gz::sim::kNullEntity};
  double surfaceZ{0}, waterDensity{1000}, airDensity{1.225};
  std::vector<HullBox> hulls;
};
}  // namespace usv_rov_sim

GZ_ADD_PLUGIN(usv_rov_sim::SurfaceBuoyancy, gz::sim::System,
  gz::sim::ISystemConfigure, gz::sim::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(usv_rov_sim::SurfaceBuoyancy, "usv_rov_sim::SurfaceBuoyancy")
