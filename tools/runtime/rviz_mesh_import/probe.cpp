#include <rviz_rendering/mesh_loader.hpp>
#include <OgreRoot.h>
#include <OgreDefaultHardwareBufferManager.h>
#include <OgreSubMesh.h>
#include <OgreMeshManager.h>
#include <OgreMaterialManager.h>
#include <OgreTechnique.h>
#include <OgrePass.h>
#include <OgreResourceGroupManager.h>
#include <iostream>
#include <array>
#include <map>
#include <vector>
#include <sstream>
#include <iomanip>
#include <filesystem>
using V=std::array<float,6>;
std::string key(V v) {std::ostringstream s;s<<std::fixed<<std::setprecision(6);for(auto n:v)s<<(std::abs(n)<0.0000005?0:n)<<",";return s.str();}
std::string canonical(std::array<std::string,3> v){return std::min(v[0]+v[1]+v[2],std::min(v[1]+v[2]+v[0],v[2]+v[0]+v[1]));}
int main(int argc,char**argv) {
 if(argc<3 || argc>4) { std::cerr<<"usage: rviz_mesh_probe FILE EXPECTED_TRIANGLES [--require-reversed-pairs]\n"; return 2; }
 const auto expected=std::stoull(argv[2]);
 const bool requirePairs=argc==4 && std::string(argv[3])=="--require-reversed-pairs";
 if(argc==4 && !requirePairs)return 2;
 bool passed=false;
 Ogre::DefaultHardwareBufferManager buffers;
 Ogre::Root root("","","");
 Ogre::ResourceGroupManager::getSingleton().createResourceGroup("rviz_rendering");
 Ogre::MaterialManager::getSingleton().initialise();
 {
  const int arg=1;
  auto mesh=rviz_rendering::loadMeshFromResource(std::string("file://")+argv[arg]);
  if(!mesh)return 3;
  unsigned long long total=0,paired=0; bool opaque=true, clockwiseCull=true;
  for(unsigned i=0;i<mesh->getNumSubMeshes();i++) {
   auto s=mesh->getSubMesh(i);total+=s->indexData->indexCount/3;
   auto mat=Ogre::MaterialManager::getSingleton().getByName(s->getMaterialName());
   auto pass=mat->getTechnique(0)->getPass(0); opaque &= pass->getDiffuse().a==1 && pass->getDepthWriteEnabled();
   clockwiseCull &= pass->getCullingMode()==Ogre::CULL_CLOCKWISE;
   auto *vd=s->vertexData;
   auto pe=vd->vertexDeclaration->findElementBySemantic(Ogre::VES_POSITION);
   auto ne=vd->vertexDeclaration->findElementBySemantic(Ogre::VES_NORMAL);
   auto vb=vd->vertexBufferBinding->getBuffer(pe->getSource());
   auto nb=vd->vertexBufferBinding->getBuffer(ne->getSource());
   auto bytes=static_cast<unsigned char*>(vb->lock(Ogre::HardwareBuffer::HBL_READ_ONLY));
   auto nbytes=nb==vb?bytes:static_cast<unsigned char*>(nb->lock(Ogre::HardwareBuffer::HBL_READ_ONLY));
   std::vector<V> verts;
   for(size_t n=0;n<vd->vertexCount;n++){
    auto p=reinterpret_cast<float*>(bytes+(n+vd->vertexStart)*vb->getVertexSize()+pe->getOffset());
    auto normal=reinterpret_cast<float*>(nbytes+(n+vd->vertexStart)*nb->getVertexSize()+ne->getOffset());
    verts.push_back({p[0],p[1],p[2],normal[0],normal[1],normal[2]});
   }
   if(nb!=vb)nb->unlock();vb->unlock();
   auto ib=s->indexData->indexBuffer;auto idata=ib->lock(Ogre::HardwareBuffer::HBL_READ_ONLY);
   std::map<std::string,unsigned> fronts;std::vector<std::string> backs;
   auto index=[&](size_t n)->unsigned{n+=s->indexData->indexStart;return ib->getType()==Ogre::HardwareIndexBuffer::IT_16BIT?static_cast<uint16_t*>(idata)[n]:static_cast<uint32_t*>(idata)[n];};
   for(size_t n=0;n<s->indexData->indexCount;n+=3){
    std::array<std::string,3> f,b;
    for(int k=0;k<3;k++){auto v=verts.at(index(n+k));f[k]=key(v);for(int j=3;j<6;j++)v[j]=-v[j];b[2-k]=key(v);}
    fronts[canonical(f)]++;backs.push_back(canonical(b));
   }
   ib->unlock();for(auto&b:backs)if(fronts.count(b))paired++;
  }
  passed=total==expected && opaque && clockwiseCull && (!requirePairs || paired==total);
  std::cout<<"RESULT {\"submeshes\":"<<mesh->getNumSubMeshes()<<",\"triangles\":"<<total<<",\"reversed_normal_pairs\":"<<paired<<",\"opaque_depthwrite\":"<<(opaque?"true":"false")<<",\"clockwise_culling\":"<<(clockwiseCull?"true":"false")<<",\"passed\":"<<(passed?"true":"false")<<"}"<<std::endl;
  mesh.reset();Ogre::MeshManager::getSingleton().removeAll();Ogre::MaterialManager::getSingleton().removeAll();
 }
 return passed?0:1;
}
