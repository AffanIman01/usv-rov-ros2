#include <gz/sim/System.hh>
#include <gz/sim/SystemLoader.hh>
#include <gz/sim/SystemPluginPtr.hh>
#include <gz/sim/config.hh>
#include <sdf/Model.hh>
#include <sdf/Root.hh>

#include <iostream>
#include <filesystem>
#include <set>
#include <string>

// Instantiate the two project systems using the filenames from the installed
// model. Do not Configure them: that starts transport and requires a live world.
// This catches missing libraries, ABI/dependency failures and registration errors
// before Gazebo can run a gravity-only boat with no propulsion.
int main(int argc, char **argv)
{
  if (argc > 2)
  {
    std::cerr << "Usage: check_sim_plugins [MODEL_SDF]\n";
    return 2;
  }
  std::filesystem::path modelPath;
  if (argc == 2) modelPath = argv[1];
  else
  {
    std::error_code error;
    const auto executable = std::filesystem::canonical("/proc/self/exe", error);
    if (error)
    {
      std::cerr << "Cannot locate the installed package; pass MODEL_SDF explicitly.\n";
      return 2;
    }
    modelPath = executable.parent_path().parent_path().parent_path() /
      "share/usv_rov_sim/models/me120_dynamic/model.sdf";
  }
  std::cout << "Plugin checker built for Gazebo Sim " << GZ_SIM_VERSION_FULL
            << "; model " << modelPath.string() << '\n';
  sdf::Root root;
  const auto errors = root.Load(modelPath.string());
  for (const auto &error : errors) std::cerr << error << '\n';
  const auto *model = root.Model();
  if (!errors.empty() || !model)
  {
    std::cerr << "Cannot read the dynamic ME120 model: " << modelPath.string() << '\n';
    return 2;
  }
  std::set<std::string> required{
    "usv_rov_sim::SurfaceBuoyancy", "usv_rov_sim::TwinThrusterController"};
  gz::sim::SystemLoader loader;
  for (const auto &definition : model->Plugins())
  {
    if (!required.count(definition.Name())) continue;
    const auto loaded = loader.LoadPlugin(definition);
    if (!loaded ||
        !(*loaded)->QueryInterface<gz::sim::ISystemConfigure>() ||
        !(*loaded)->QueryInterface<gz::sim::ISystemPreUpdate>())
    {
      std::cerr << "Unable to load required Gazebo 8 system " << definition.Name()
                << " from " << definition.Filename() << ".\n"
                << "Source this workspace's install/setup.bash and rebuild "
                   "usv_rov_sim for Gazebo Harmonic before launching.\n";
      return 1;
    }
    std::cout << "Loaded " << definition.Name() << " from "
              << definition.Filename() << '\n';
    required.erase(definition.Name());
  }
  if (!required.empty())
  {
    for (const auto &name : required)
      std::cerr << "Required system missing from model: " << name << '\n';
    return 1;
  }
  std::cout << "Both required Gazebo 8 systems are loadable. "
               "This checks loading, not live ROS communication or physics.\n";
  return 0;
}
