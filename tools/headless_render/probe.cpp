#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/RenderEngine.hh>
#include <gz/rendering/Scene.hh>
#include <gz/rendering/Visual.hh>
#include <gz/rendering/Mesh.hh>
#include <gz/rendering/Camera.hh>
#include <gz/rendering/Light.hh>
#include <gz/common/MeshManager.hh>
#include <gz/common/Image.hh>
#include <gz/rendering/Image.hh>
#include <filesystem>
#include <fstream>
#include <vector>
#include <iostream>
int main(int argc, char **argv) {
  if (argc != 3 && argc != 4) return 1;
  auto *engine = gz::rendering::engine("ogre2", {{"headless", "true"}});
  if (!engine) {std::cerr << "FAIL: engine creation\n"; return 2;}
  std::cout << "Engine initialized: " << engine->Name() << " headless=" << engine->Headless() << std::endl;
  auto scene = engine->CreateScene("me120_headless_probe");
  if (!scene) {std::cerr << "FAIL: scene creation\n"; return 3;}
  scene->SetAmbientLight(0.7,0.7,0.7);
  scene->SetBackgroundColor(0.1,0.15,0.2);
  auto root=scene->RootVisual();
  std::ifstream parts(argv[1]);
  std::string path; double x,y,z; unsigned count=0;
  while(parts >> path >> x >> y >> z) {
    auto loadedMesh=gz::common::MeshManager::Instance()->Load(path);
    if (!loadedMesh) {std::cerr << "FAIL: mesh import " << path << "\n"; return 6;}
    auto mesh=scene->CreateMesh(loadedMesh);
    if (!mesh) {std::cerr << "FAIL: mesh creation\n"; return 4;}
    auto visual=scene->CreateVisual(); visual->AddGeometry(mesh);
    visual->SetLocalPose(gz::math::Pose3d(x,y,z,0,0,0)); root->AddChild(visual);
    count++;
  }
  if (!count) return 7;
  std::cout << "Loaded " << count << " meshes at manifest offsets\n";
  auto light=scene->CreateDirectionalLight(); light->SetDiffuseColor(0.8,0.8,0.8); light->SetDirection(-1,-1,-1); root->AddChild(light);
  auto camera=scene->CreateCamera();
  camera->SetImageWidth(960); camera->SetImageHeight(720); camera->SetAspectRatio(960.0/720);
  camera->SetNearClipPlane(0.05); camera->SetFarClipPlane(100);
  if (argc == 4 && std::string(argv[3]) == "rear")
    camera->SetLocalPose(gz::math::Pose3d(-3,3,2.5,0,0.38,-0.785398163));
  else
    camera->SetLocalPose(gz::math::Pose3d(3,-3,2.5,0,0.38,2.35619449));
  root->AddChild(camera);
  auto image=camera->CreateImage(); camera->Capture(image);
  gz::common::Image out;
  out.SetFromData(image.Data<unsigned char>(), image.Width(), image.Height(), gz::common::Image::RGB_INT8);
  out.SavePNG(argv[2]);
  if (!std::filesystem::exists(argv[2])) {std::cerr << "FAIL: frame save\n"; return 5;}
  std::cout << "RENDER_CAPTURE_OK " << argv[2] << std::endl;
  camera.reset(); light.reset(); root.reset();
  engine->DestroyScene(scene); scene.reset();
  gz::rendering::unloadEngine("ogre2");
  return 0;
}
