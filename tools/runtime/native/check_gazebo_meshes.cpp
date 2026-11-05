#include <gz/common/Mesh.hh>
#include <gz/common/MeshManager.hh>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <cmath>

int main(int argc, char **argv) {
  if(argc!=2) { std::cerr<<"usage: check_gazebo_meshes EXPECTATIONS_TSV\n"; return 2; }
  unsigned int files=0; unsigned long long indices=0, expectedIndices=0;
  std::ifstream input(argv[1]);
  if(!input) { std::cerr<<"Cannot open expectations TSV\n"; return 2; }
  std::string line;
  while(std::getline(input,line)) {
    if(line.empty()) continue;
    const auto tab=line.find('\t');
    if(tab==std::string::npos) { std::cerr<<"Malformed TSV line\n"; return 2; }
    const std::filesystem::path path=line.substr(0,tab);
    std::istringstream expected(line.substr(tab+1));
    unsigned int triangles;
    double lo[3],hi[3];
    if(!(expected>>triangles>>lo[0]>>lo[1]>>lo[2]>>hi[0]>>hi[1]>>hi[2])) {
      std::cerr<<"Malformed numeric expectations\n"; return 2;
    }
    const auto *mesh=gz::common::MeshManager::Instance()->Load(path.string());
    if(!mesh || !mesh->VertexCount() || !mesh->IndexCount() || !mesh->MaterialCount()) {
      std::cerr<<"FAIL empty mesh "<<path<<"\n"; return 1;
    }
    if(mesh->IndexCount()!=triangles*3) {
      std::cerr<<"FAIL triangle count "<<path<<"\n"; return 1;
    }
    const auto actualLo=mesh->Min(),actualHi=mesh->Max();
    for(int a=0;a<3;++a) {
      if(!std::isfinite(actualLo[a]) || !std::isfinite(actualHi[a]) ||
          std::abs(actualLo[a]-lo[a])>1e-6 || std::abs(actualHi[a]-hi[a])>1e-6) {
        std::cerr<<"FAIL loaded bounds "<<path<<" axis="<<a
          <<" actual="<<actualLo[a]<<","<<actualHi[a]
          <<" expected="<<lo[a]<<","<<hi[a]<<"\n";
        return 1;
      }
    }
    ++files; indices+=mesh->IndexCount(); expectedIndices+=static_cast<unsigned long long>(triangles)*3;
    std::cout<<"PASS "<<path.filename().string()<<" vertices="<<mesh->VertexCount()
      <<" triangles="<<mesh->IndexCount()/3<<" materials="<<mesh->MaterialCount()
      <<" bounds_match=true min="<<mesh->Min()<<" max="<<mesh->Max()<<"\n";
  }
  std::cout<<"SUMMARY files="<<files<<" triangles="<<indices/3<<"\n";
  return files==39 && indices==expectedIndices ? 0 : 1;
}
