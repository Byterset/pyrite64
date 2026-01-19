/**
* @copyright 2025 - Max Bebök
* @license MIT
*/
#pragma once
#include "../../../project/project.h"
#include "../../../project/graph/graph.h"

namespace Editor
{
  class NodeEditor
  {
    private:
      Project::AssetManagerEntry *currentAsset{nullptr};
      Project::Graph::Graph graph{};

    public:
      NodeEditor();
      ~NodeEditor();
      void draw();
  };
}
