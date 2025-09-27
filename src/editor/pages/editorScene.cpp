/**
* @copyright 2025 - Max Bebök
* @license MIT
*/
#include "editorScene.h"

#include "imgui.h"
#include "../actions.h"
#include "../../context.h"
#include "misc/cpp/imgui_stdlib.h"
#include "../imgui/helper.h"

namespace
{
}

void Editor::Scene::draw()
{
  auto &io = ImGui::GetIO();

  float offsetY = 20;
  ImGui::SetNextWindowPos({0,offsetY}, ImGuiCond_Appearing, {0.0f, 0.0f});
  ImGui::SetNextWindowSize({400, io.DisplaySize.y-offsetY}, ImGuiCond_Always);
  ImGui::Begin("TAB_PROJECT", 0,
    ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoTitleBar
  );

  if (ImGui::CollapsingHeader("Project settings", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::InpTable::start("Project settings");
    ImGui::InpTable::addString("Name", ctx.project->conf.name);
    ImGui::InpTable::addString("ROM-Name", ctx.project->conf.romName);
    ImGui::InpTable::end();
  }
  if (ImGui::CollapsingHeader("Environment", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::InpTable::start("Environment");
    ImGui::InpTable::addPath("Emulator", ctx.project->conf.pathEmu);
    ImGui::InpTable::addPath("Libdragon", ctx.project->conf.pathLibdragon, true);
    ImGui::InpTable::addPath("N64_INST", ctx.project->conf.pathN64Inst, true);
    ImGui::InpTable::end();
  }

  ImGui::End();

  // Top bar
  ImGui::SetNextWindowPos({0,0}, ImGuiCond_Appearing, {0.0f, 0.0f});
  ImGui::Begin("TOP_BAR", 0,
    ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoTitleBar
    | ImGuiWindowFlags_MenuBar | ImGuiWindowFlags_NoBackground
  );

  if(ImGui::BeginMenuBar())
  {
    if(ImGui::BeginMenu("Project"))
    {
      if(ImGui::MenuItem("Save"))ctx.project->save();
      if(ImGui::MenuItem("Close"))Actions::call(Actions::Type::PROJECT_CLOSE);
      ImGui::EndMenu();
    }
    ImGui::EndMenuBar();
  }
  ImGui::End();

  // Bottom Status bar
  ImGui::SetNextWindowPos({0, io.DisplaySize.y - 32}, ImGuiCond_Always, {0.0f, 0.0f});
  ImGui::SetNextWindowSize({io.DisplaySize.x, 32}, ImGuiCond_Always);
  ImGui::Begin("STATUS_BAR", 0,
    ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoTitleBar
    | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse
    | ImGuiWindowFlags_NoCollapse
  );
  ImGui::Text("%.2f FPS", io.Framerate);
  ImGui::End();
}
