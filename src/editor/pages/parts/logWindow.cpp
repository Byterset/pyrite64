/**
* @copyright 2025 - Max Bebök
* @license MIT
*/
#include "logWindow.h"

#include "imgui.h"
#include "../../../context.h"
#include "../../../utils/logger.h"
#include "../../imgui/theme.h"

namespace
{
  constinit uint32_t lastLen{0};
}

void Editor::LogWindow::draw()
{
  auto log = Utils::Logger::getLog();

  ImGui::PushFont(ImGui::getFontMono());

  ImGui::PushID("LOG");
  ImGui::PushStyleColor(ImGuiCol_FrameBg, {0.05f, 0.05f, 0.05f, 1.0f});
  ImGui::InputTextMultiline("", log.data(), log.size()+1, {
    ImGui::GetWindowSize().x-18,
    ImGui::GetWindowSize().y-44
  }, ImGuiInputTextFlags_ReadOnly);

  if (lastLen != log.length()) {
    lastLen = log.length();
    ImGui::SetScrollHereY(1);
  }

  ImGui::PopStyleColor();
  ImGui::PopID();

  ImGui::PopFont();
}
