#include <iostream>
#include <gz/common/Image.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/RenderEngine.hh>
#include <gz/rendering/Scene.hh>
#include <gz/rendering/Material.hh>
int main(int argc, char **argv)
{
  auto engine = gz::rendering::engine("ogre");
  if (!engine || argc != 4) return 2;
  auto scene = engine->CreateScene("phase1_material_probe");
  bool passed = true;
  for (int i=1; i<4; ++i)
  {
    auto material=scene->CreateMaterial();
    material->SetTexture(argv[i]);
    material->SetAlphaFromTexture(true,.5,false);
    bool write=material->DepthWriteEnabled();
    bool check=material->DepthCheckEnabled();
    std::cout << "TEXTURE " << argv[i] << " depth_write=" << write << " depth_check=" << check << std::endl;
    passed = passed && check && (write == (i != 3));
    material->SetTransparency(.5);
    passed = passed && !material->DepthWriteEnabled();
    material->SetTransparency(0);
    passed = passed && (material->DepthWriteEnabled() == (i != 3));
    material->SetAlphaFromTexture(false,.5,false);
    passed = passed && material->DepthWriteEnabled();
    // Re-enable alpha and replace the texture: opacity must be recomputed.
    material->SetAlphaFromTexture(true,.5,false);
    material->SetTexture(argv[1]);
    passed = passed && material->DepthWriteEnabled();
    material->SetTexture(argv[3]);
    passed = passed && !material->DepthWriteEnabled() && material->DepthCheckEnabled();
    material->ClearTexture();
    passed = passed && material->DepthWriteEnabled() && material->DepthCheckEnabled();
    auto pixels = std::make_shared<gz::common::Image>();
    passed = passed && pixels->Load(argv[i]) == 0;
    material->SetTexture("phase1_memory_" + std::to_string(i), pixels);
    passed = passed && material->DepthCheckEnabled() && (material->DepthWriteEnabled() == (i != 3));
    material->SetTransparency(.5);
    passed = passed && !material->DepthWriteEnabled();
    material.reset();
  }
  engine->DestroyScene(scene); scene.reset();
  gz::rendering::unloadEngine("ogre");
  std::cout << "OPACITY_CONTRACT " << (passed?"PASS":"FAIL") << std::endl;
  return passed ? 0 : 1;
}
