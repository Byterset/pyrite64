/**
* @copyright 2025 - Max Bebök
* @license MIT
*/
#pragma once
#include "parts/assetInspector.h"
#include "parts/assetsBrowser.h"
#include "parts/logWindow.h"
#include "parts/sceneBrowser.h"
#include "parts/sceneInspector.h"
#include "parts/viewport3D.h"

namespace Editor
{
  class Scene
  {
    private:
      Viewport3D viewport3d{};
      AssetsBrowser assetsBrowser{};
      SceneBrowser sceneBrowser{};
      AssetInspector assetInspector{};
      SceneInspector sceneInspector{};
      LogWindow logWindow{};

      bool dockSpaceInit{false};

    public:
      void draw();
  };
}
