/**
* @copyright 2025 - Max Bebök
* @license MIT
*/
#include "nodeEditor.h"

#include "imgui.h"
#include "../../../context.h"
#include "../../../utils/logger.h"
#include "../../imgui/helper.h"

#include "ImNodeFlow.h"
#include "json.hpp"
#include "../../../utils/fs.h"

namespace
{

}

Editor::NodeEditor::NodeEditor()
{
  currentAsset = ctx.project->getAssets().getByName("test.p64graph");
  graph.deserialize(currentAsset
    ? Utils::FS::loadTextFile(currentAsset->path)
    : "{}"
  );
}

Editor::NodeEditor::~NodeEditor()
{
}

void Editor::NodeEditor::draw()
{
  if(!currentAsset)
  {
    ImGui::Text("No Asset loaded");
    return;
  }

  auto size = ImGui::GetContentRegionAvail();
  size.y -= 32;
  graph.graph.setSize(size);
  //mINF.getStyle().grid_subdivisions = 10.0f;
  graph.graph.update();

  if(ImGui::Button("Save"))
  {
    Utils::FS::saveTextFile(currentAsset->path, graph.serialize());
  }

  ImGui::SameLine();
  if(ImGui::Button("Build"))
  {
    graph.build().getData();
  }
}
