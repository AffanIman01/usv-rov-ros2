#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/RenderEngine.hh>
#include <gz/rendering/Scene.hh>
#include <gz/rendering/Visual.hh>
#include <gz/rendering/Geometry.hh>
#include <gz/rendering/Mesh.hh>
#include <gz/rendering/Material.hh>
#include <gz/rendering/Camera.hh>
#include <gz/rendering/Light.hh>
#include <gz/common/MeshManager.hh>
#include <gz/common/Image.hh>
#include <gz/rendering/Image.hh>
#include <filesystem>
#include <fstream>
#include <iostream>
int main(int argc,char **argv){
 if(argc!=3 && argc!=4)return 1;
 auto *engine=gz::rendering::engine("ogre2",{{"headless","true"}});
 if(!engine)return 2;
 auto scene=engine->CreateScene("usv_pool_visual_probe");
 scene->SetAmbientLight(.65,.65,.65);scene->SetBackgroundColor(.12,.18,.23);
 auto root=scene->RootVisual();
 std::ifstream input(argv[1]);std::string kind,path;double x,y,z,sx,sy,sz,ar,ag,ab,dr,dg,db,er,eg,eb,tr;unsigned count=0;
 while(input>>kind>>path>>x>>y>>z>>sx>>sy>>sz>>ar>>ag>>ab>>dr>>dg>>db>>er>>eg>>eb>>tr){
  gz::rendering::GeometryPtr geometry;
  if(kind=="mesh"){
   auto mesh=gz::common::MeshManager::Instance()->Load(path);if(!mesh)return 3;
   geometry=scene->CreateMesh(mesh);
  }else if(kind=="box")geometry=scene->CreateBox();
  else if(kind=="plane")geometry=scene->CreatePlane();
  else return 4;
  auto visual=scene->CreateVisual();visual->AddGeometry(geometry);
  visual->SetLocalPose(gz::math::Pose3d(x,y,z,0,0,0));visual->SetLocalScale(sx,sy,sz);
  if(kind!="mesh"){
   auto mat=scene->CreateMaterial();mat->SetAmbient(ar,ag,ab);mat->SetDiffuse(dr,dg,db);mat->SetEmissive(er,eg,eb);mat->SetSpecular(.1,.1,.1);mat->SetTransparency(tr);mat->SetCastShadows(false);visual->SetMaterial(mat);
  }
  root->AddChild(visual);count++;
 }
 std::cout<<"Loaded "<<count<<" visuals at pool/model SDF poses"<<std::endl;
 auto light=scene->CreateDirectionalLight();light->SetDiffuseColor(.9,.9,.9);light->SetSpecularColor(.1,.1,.1);light->SetDirection(-.4,.2,-.9);light->SetCastShadows(false);root->AddChild(light);
 auto camera=scene->CreateCamera();camera->SetImageWidth(1200);camera->SetImageHeight(800);camera->SetAspectRatio(1.5);camera->SetNearClipPlane(.05);camera->SetFarClipPlane(100);
 if(argc==4 && std::string(argv[3])=="overview")camera->SetLocalPose(gz::math::Pose3d(15,-16,14,0,.55,2.32));
 else if(argc==4 && std::string(argv[3])=="stern")camera->SetLocalPose(gz::math::Pose3d(-3.5,0,2.6,0,.43,0));
 else if(argc==4 && std::string(argv[3])=="mast")camera->SetLocalPose(gz::math::Pose3d(1.7,0,1.65,0,.10,3.14159265359));
 else if(argc==4 && std::string(argv[3])=="detail")camera->SetLocalPose(gz::math::Pose3d(3,-3,2.8,0,.40,2.35619449));
 else camera->SetLocalPose(gz::math::Pose3d(5,-5,4,0,.45,2.35619449));
 root->AddChild(camera);
 auto image=camera->CreateImage();camera->Capture(image);gz::common::Image out;out.SetFromData(image.Data<unsigned char>(),image.Width(),image.Height(),gz::common::Image::RGB_INT8);out.SavePNG(argv[2]);
 std::cout<<"RENDER_CAPTURE_OK "<<argv[2]<<std::endl;
 camera.reset();light.reset();root.reset();engine->DestroyScene(scene);scene.reset();gz::rendering::unloadEngine("ogre2");return std::filesystem::exists(argv[2])?0:5;
}
