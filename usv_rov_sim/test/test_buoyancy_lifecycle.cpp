#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/Events.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/SdfEntityCreator.hh>
#include <gz/sim/System.hh>
#include <gz/sim/SystemLoader.hh>
#include <gz/sim/SystemPluginPtr.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/ExternalWorldWrenchCmd.hh>
#include <sdf/Model.hh>
#include <sdf/Root.hh>
#include <sdf/World.hh>

#include <cmath>
#include <iostream>
#include <stdexcept>

void Require(bool pass, const char *message)
{
  if (!pass) throw std::runtime_error(message);
}

int main(int argc, char **argv)
{
  Require(argc == 2, "Expected the production model path.");
  sdf::Root root;
  Require(root.Load(argv[1]).empty() && root.Model(), "Cannot parse production model.");
  sdf::World world;
  world.SetName("lifecycle_test");
  world.SetGravity({0, 0, -9.81});
  auto model = *root.Model();
  model.SetRawPose({0, 0, 0.152784564, 0, 0, 0});
  world.AddModel(model);
  sdf::Root worldRoot;
  Require(worldRoot.AddWorld(world).empty() && worldRoot.UpdateGraphs().empty(),
    "Cannot construct the world pose graph.");

  gz::sim::EntityComponentManager ecm;
  gz::sim::EventManager events;
  gz::sim::SystemLoader loader;
  gz::sim::SystemPluginPtr plugin;
  gz::sim::Entity modelEntity = gz::sim::kNullEntity;
  bool configuredBeforeWorldParent = false;
  // The server subscribes to this event and configures systems synchronously.
  // Unlike manual Configure calls after CreateEntities(world), this recreates
  // the real model-before-world-parent ordering that caused silent sinking.
  auto connection = events.Connect<gz::sim::events::LoadSdfPlugins>(
    [&](gz::sim::Entity entity, sdf::Plugins definitions)
    {
      for (const auto &definition : definitions)
      {
        if (definition.Name() != "usv_rov_sim::SurfaceBuoyancy") continue;
        modelEntity = entity;
        configuredBeforeWorldParent =
          gz::sim::worldEntity(entity, ecm) == gz::sim::kNullEntity;
        auto loaded = loader.LoadPlugin(definition);
        Require(loaded.has_value(), "Could not load production buoyancy plugin.");
        plugin = *loaded;
        auto configure = plugin->QueryInterface<gz::sim::ISystemConfigure>();
        Require(configure != nullptr, "Missing Configure interface.");
        configure->Configure(entity, definition.ToElement(), ecm, events);
      }
    });
  gz::sim::SdfEntityCreator creator(ecm, events);
  const auto worldEntity = creator.CreateEntities(worldRoot.WorldByIndex(0));
  Require(static_cast<bool>(plugin), "Model plugin event was not emitted.");
  Require(configuredBeforeWorldParent,
    "Regression fixture must reproduce Configure before the model's world parent.");
  Require(gz::sim::worldEntity(modelEntity, ecm) == worldEntity,
    "World parent must exist before the first physics iteration.");
  gz::sim::Link link(gz::sim::Model(modelEntity).LinkByName(ecm, "base_link"));
  gz::sim::UpdateInfo info;
  info.paused = false;
  info.dt = std::chrono::milliseconds(2);
  info.simTime = info.dt;
  plugin->QueryInterface<gz::sim::ISystemPreUpdate>()->PreUpdate(info, ecm);
  const auto *wrench = ecm.Component<gz::sim::components::ExternalWorldWrenchCmd>(link.Entity());
  Require(wrench != nullptr,
    "Buoyancy must produce a wrench after deferred world-parent resolution.");
  const double support = wrench->Data().force().z();
  Require(std::isfinite(support) && std::abs(support - 120.0 * 9.81) < 0.05,
    "Initial buoyancy must balance the 120 kg ME120 weight.");
  std::cout << "PASS: real Configure ordering recovered; initial upward force="
            << support << " N.\n";
}
