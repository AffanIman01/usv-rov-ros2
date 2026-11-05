#include <gz/sim/Server.hh>
#include <gz/sim/ServerConfig.hh>
#include <iostream>

int main(int argc,char **argv) {
  if(argc!=2) { std::cerr<<"usage: check_gazebo_world WORLD_SDF\n"; return 2; }
  gz::sim::ServerConfig config;
  config.SetSdfFile(argv[1]);
  gz::sim::Server server(config);
  if(!server.Run(true,20,false)) { std::cerr<<"FAIL server.Run\n"; return 1; }
  for(const auto *name:{"me120","m2_pro_max","e_reel","starlink_mini","compute_enclosure","recovery_concept"}) {
    if(!server.HasEntity(name)) { std::cerr<<"FAIL missing model "<<name<<"\n"; return 1; }
    std::cout<<"PASS model "<<name<<"\n";
  }
  const auto count=server.IterationCount();
  std::cout<<"SUMMARY iterations="<<(count?*count:0)<<" entities="<<server.EntityCount().value_or(0)<<"\n";
  return count && *count>=20 ? 0 : 1;
}
