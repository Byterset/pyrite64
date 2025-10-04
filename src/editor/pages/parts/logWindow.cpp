/**
* @copyright 2025 - Max Bebök
* @license MIT
*/
#include "logWindow.h"

#include "imgui.h"
#include "../../../context.h"
#include "../../../utils/logger.h"

void Editor::LogWindow::draw()
{
  auto log = Utils::Logger::getLog();
  ImGui::Text("%s", log.c_str());
}
