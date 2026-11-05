#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/Events.hh>
#include <gz/sim/SdfEntityCreator.hh>
#include <gz/sim/SystemLoader.hh>
#include <gz/sim/SystemPluginPtr.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/ExternalWorldWrenchCmd.hh>
#include <gz/sim/Util.hh>
#include <sdf/Root.hh>
#include <sdf/World.hh>
#include <sdf/Model.hh>
#include <sdf/Plugin.hh>
#include <sdf/ParserConfig.hh>
#include <usv_rov_sim/thruster_control.hpp>
#include <iostream>
#include <vector>
#include <filesystem>
#include <fstream>
#include <cmath>
#include <array>
struct SystemHandle {gz::sim::SystemPluginPtr plugin;gz::sim::ISystemPreUpdate *pre;gz::sim::ISystemUpdate *update;gz::sim::ISystemPostUpdate *post;};
int main(int argc,char**argv){
 if(argc<3 || argc>5){std::cerr<<"usage: vehicle_probe REPO CASE [PLUGIN_DIR] [--builtin-buoyancy]\n";return 2;}
 bool builtin=false;std::string pluginDir;for(int a=3;a<argc;a++){if(std::string(argv[a])=="--builtin-buoyancy")builtin=true;else pluginDir=argv[a];}
 const std::filesystem::path repo=std::filesystem::absolute(argv[1]);const std::string name=argv[2];
 sdf::ParserConfig parser;parser.SetFindCallback([&](const std::string &uri){if(uri=="model://me120_dynamic")return (repo/"usv_rov_sim/models/me120_dynamic").string();const std::string p="model://usv_rov_description/";if(uri.find(p)==0)return (repo/"usv_rov_description"/uri.substr(p.size())).string();return uri;});
 sdf::Root root;auto errors=root.Load((repo/"usv_rov_sim/worlds/pool.sdf").string(),parser);for(const auto&e:errors)std::cerr<<e<<"\n";if(!errors.empty())return 2;
 auto world=root.WorldByIndex(0);auto model=world->ModelByName("me120");if(!model)return 3;
 const auto initial=model->RawPose();auto start=initial;
 if(name=="roll")start.Rot()=gz::math::Quaterniond(0.08,0,0);
 if(name=="pitch")start.Rot()=gz::math::Quaterniond(0,0.08,0);
 if(name=="torque_pitch")start.Rot()=gz::math::Quaterniond(0,0.05,0);
 if(name=="torque_roll")start.Rot()=gz::math::Quaterniond(0.05,0,0);
 if(name=="heave")start.Pos().Z()+=0.1;
 model->SetRawPose(start);
 const auto graphErrors=root.UpdateGraphs();if(!graphErrors.empty())return 2;
 // Configuration comes from the production model; transport-owning plugins are
 // deliberately not instantiated in this no-network physics harness.
 auto hyd=std::find_if(model->Plugins().begin(),model->Plugins().end(),[](const auto&p){return p.Name()=="gz::sim::systems::Hydrodynamics";});if(hyd==model->Plugins().end())return 4;
 auto h=hyd->ToElement();std::array<double,6> lin{},quad{};const char*lk[]={"xU","yV","zW","kP","mQ","nR"};const char*qk[]={"xUabsU","yVabsV","zWabsW","kPabsP","mQabsQ","nRabsR"};for(int i=0;i<6;++i){lin[i]=h->Get<double>(lk[i]);quad[i]=h->Get<double>(qk[i]);}
 gz::sim::EntityComponentManager ecm;gz::sim::EventManager events;
 gz::sim::SystemLoader loader;if(!pluginDir.empty())loader.AddSystemPluginPath(pluginDir);std::vector<SystemHandle> systems;
 auto add=[&](const sdf::Plugin &p,gz::sim::Entity entity){auto loaded=loader.LoadPlugin(p);if(!loaded)throw std::runtime_error("plugin load failed: "+p.Name());auto plugin=*loaded;auto configure=plugin->QueryInterface<gz::sim::ISystemConfigure>();if(configure)configure->Configure(entity,p.ToElement(),ecm,events);systems.push_back({plugin,plugin->QueryInterface<gz::sim::ISystemPreUpdate>(),plugin->QueryInterface<gz::sim::ISystemUpdate>(),plugin->QueryInterface<gz::sim::ISystemPostUpdate>()});};
 // Match the real SimulationRunner: Configure is called synchronously from
 // LoadSdfPlugins while SdfEntityCreator is still constructing the world.
 // Calling Configure only after CreateEntities(world) hid the original bug:
 // the model has no world parent yet when its plugins are configured.
 bool configuredBeforeWorldParent=false;
 auto loadConnection=events.Connect<gz::sim::events::LoadSdfPlugins>(
  [&](gz::sim::Entity entity,sdf::Plugins plugins){
   if(builtin)return;
   for(const auto&p:plugins)if(p.Name()=="usv_rov_sim::SurfaceBuoyancy"){
    configuredBeforeWorldParent=gz::sim::worldEntity(entity,ecm)==gz::sim::kNullEntity;
    add(p,entity);
   }
  });
 gz::sim::SdfEntityCreator creator(ecm,events);const auto we=creator.CreateEntities(world);
 if(!builtin&&!configuredBeforeWorldParent)throw std::runtime_error("Expected model Configure before world-parent attachment.");
 const auto me=ecm.EntityByComponents(gz::sim::components::Name("me120"),gz::sim::components::Model());gz::sim::Model simModel(me);gz::sim::Link body(simModel.LinkByName(ecm,"base_link"));body.EnableVelocityChecks(ecm);
 for(const auto&p:world->Plugins())if(p.Name()=="gz::sim::systems::Physics")add(p,we);
 // This separate negative control is a world plugin, intentionally configured
 // after its world exists. It does not test model-plugin lifecycle ordering.
 if(builtin){sdf::Plugin b;b.SetName("gz::sim::systems::Buoyancy");b.SetFilename("gz-sim-buoyancy-system");b.InsertContent("<graded_buoyancy><default_density>1000</default_density><density_change><above_depth>0</above_depth><density>1.225</density></density_change></graded_buoyancy><enable>me120</enable>");add(b,we);}
 if(systems.size()!=2)return 6;
 usv_rov_sim::ControlParameters cp;auto ctrl=std::find_if(model->Plugins().begin(),model->Plugins().end(),[](const auto&p){return p.Name()=="usv_rov_sim::TwinThrusterController";});auto c=ctrl->ToElement();
 // Keep defaults in the shared production control implementation; current SDF
 // uses the same values. Limits and locations are read directly from the SDF.
 cp.maxSpeed=c->Get<double>("max_speed");cp.maxYawRate=c->Get<double>("max_yaw_rate");cp.maxThrust=c->Get<double>("max_thrust");cp.surgeGain=c->Get<double>("surge_gain");cp.yawGain=c->Get<double>("yaw_gain");cp.timeoutSeconds=c->Get<double>("command_timeout");
 auto port=c->Get<gz::math::Vector3d>("port_position");auto starboard=c->Get<gz::math::Vector3d>("starboard_position");cp.portY=port.Y();cp.starboardY=starboard.Y();
 const double duration=(name=="idle"?60.0:(name=="coast"?25.0:15.0));const double dt=.002;
 gz::sim::UpdateInfo info;info.paused=false;info.dt=std::chrono::milliseconds(2);
 double maxRoll=0,maxPitch=0,minZ=1e9,maxZ=-1e9,peakV=0,atRelease=0;bool finite=true;
 for(int i=1;i<=int(duration/dt);++i){info.iterations=i;info.simTime=info.dt*i;
  for(auto&s:systems)if(s.pre)s.pre->PreUpdate(info,ecm);
  if(name=="torque_pitch"||name=="torque_roll"){
   const auto *wrench=ecm.Component<gz::sim::components::ExternalWorldWrenchCmd>(body.Entity());
   if(!wrench)return 7;const auto&w=wrench->Data();const auto pose=gz::sim::worldPose(body.Entity(),ecm);const double restoring=name=="torque_pitch"?w.torque().y():w.torque().x();
   std::cout<<"RESULT {\"case\":\""<<name<<"\",\"configured_before_world_parent\":"<<(configuredBeforeWorldParent?"true":"false")<<",\"builtin_buoyancy\":"<<(builtin?"true":"false")<<",\"passed\":"<<(restoring<0?"true":"false")<<",\"initial_rpy\":["<<pose.Rot().Roll()<<","<<pose.Rot().Pitch()<<","<<pose.Rot().Yaw()<<"],\"force_z\":"<<w.force().z()<<",\"torque\":["<<w.torque().x()<<","<<w.torque().y()<<","<<w.torque().z()<<"]}"<<std::endl;return restoring<0?0:1;}
  const auto pose=body.WorldPose(ecm).value_or(start);const auto rot=pose.Rot();auto vel=rot.RotateVectorReverse(body.WorldLinearVelocity(ecm).value_or(gz::math::Vector3d::Zero));auto omega=rot.RotateVectorReverse(body.WorldAngularVelocity(ecm).value_or(gz::math::Vector3d::Zero));
  // Exact diagonal no-current/no-added-mass damping formula configured in the
  // production Hydrodynamics plugin. The plugin itself requires transport.
  std::array<double,6> v{vel.X(),vel.Y(),vel.Z(),omega.X(),omega.Y(),omega.Z()},d{};for(int j=0;j<6;j++)d[j]=(lin[j]+quad[j]*std::abs(v[j]))*v[j];
  body.AddWorldWrench(ecm,rot.RotateVector({d[0],d[1],d[2]}),rot.RotateVector({d[3],d[4],d[5]}));
  double target=0,yaw=0;const double t=i*dt;
  if(name=="forward"||(name=="coast"&&t<=5))target=.8;
  if(name=="reverse")target=-.6;
  if(name=="left")yaw=.4;
  if(name=="right")yaw=-.4;
  auto now=usv_rov_sim::SteadyClock::time_point{}+std::chrono::milliseconds(i*2);usv_rov_sim::VelocityCommand cmd{target,yaw,now,true};auto thrust=usv_rov_sim::ComputeThrust(cp,cmd,vel.X(),omega.Z(),now);
  gz::math::Vector3d fp(thrust.port,0,0),fs(thrust.starboard,0,0);body.AddWorldWrench(ecm,rot.RotateVector(fp+fs),rot.RotateVector(port.Cross(fp)+starboard.Cross(fs)));
  for(auto&s:systems)if(s.update)s.update->Update(info,ecm);for(auto&s:systems)if(s.post)s.post->PostUpdate(info,ecm);
  const auto p=body.WorldPose(ecm).value();const auto r=p.Rot().Euler();auto wv=body.WorldLinearVelocity(ecm).value();maxRoll=std::max(maxRoll,std::abs(r.X()));maxPitch=std::max(maxPitch,std::abs(r.Y()));minZ=std::min(minZ,p.Pos().Z());maxZ=std::max(maxZ,p.Pos().Z());peakV=std::max(peakV,wv.Length());if(name=="coast"&&i==2500)atRelease=wv.Length();
  finite=finite&&p.Pos().IsFinite()&&wv.IsFinite()&&r.IsFinite();if(!finite)break;
  ecm.ClearNewlyCreatedEntities();ecm.SetAllComponentsUnchanged();ecm.ProcessRemoveEntityRequests();
 }
 const auto p=body.WorldPose(ecm).value();const auto r=p.Rot().Euler();const auto v=body.WorldLinearVelocity(ecm).value();const auto w=body.WorldAngularVelocity(ecm).value();
 bool passed=finite&&std::abs(p.Pos().Z()-initial.Pos().Z())<.03&&std::abs(r.X())<.02&&std::abs(r.Y())<.02;
 if(name=="forward")passed&=p.Pos().X()>2&&v.X()>.2;
 if(name=="reverse")passed&=p.Pos().X()<-2&&v.X()<-.2;
 if(name=="left")passed&=w.Z()>.1;
 if(name=="right")passed&=w.Z()<-.1;
 if(name=="coast")passed&=atRelease>.2&&v.Length()<.01;
 if(name=="idle"||name=="roll"||name=="pitch"||name=="heave")passed&=v.Length()<.01&&w.Length()<.01;
 if(name=="roll")passed&=maxRoll>.07;
 if(name=="pitch")passed&=maxPitch>.07;
 if(name=="heave")passed&=maxZ>initial.Pos().Z()+.08;
 std::cout<<"RESULT {\"case\":\""<<name<<"\",\"configured_before_world_parent\":"<<(configuredBeforeWorldParent?"true":"false")<<",\"passed\":"<<(passed?"true":"false")<<",\"duration_s\":"<<duration<<",\"position\":["<<p.Pos().X()<<","<<p.Pos().Y()<<","<<p.Pos().Z()<<"],\"rpy\":["<<r.X()<<","<<r.Y()<<","<<r.Z()<<"],\"linear_velocity\":["<<v.X()<<","<<v.Y()<<","<<v.Z()<<"],\"yaw_rate\":"<<w.Z()<<",\"z_range\":["<<minZ<<","<<maxZ<<"],\"max_abs_roll\":"<<maxRoll<<",\"max_abs_pitch\":"<<maxPitch<<",\"peak_speed\":"<<peakV<<",\"speed_at_release\":"<<atRelease<<"}"<<std::endl;
 return passed?0:1;
}
