/**
* @copyright 2025 - Max Bebök
* @license MIT
*/
#pragma once
#include "imgui.h"
#include "IconsFontAwesome4.h"
#include "../../utils/filePicker.h"

namespace ImGui::InpTable
{
  inline void start(const char *name)
  {
    ImGui::BeginTable(name, 2);
    ImGui::TableSetupColumn("Label", ImGuiTableColumnFlags_WidthFixed);
    ImGui::TableSetupColumn("Input", ImGuiTableColumnFlags_WidthStretch);

    ImGui::TableNextRow();
    ImGui::TableSetColumnIndex(1);
    ImGui::PushItemWidth(-FLT_MIN);
  }

  inline void end() {
    ImGui::EndTable();
  }

  inline void addString(const std::string &name, std::string &str) {
    ImGui::TableNextRow();
      ImGui::TableSetColumnIndex(0);
      ImGui::AlignTextToFramePadding();
      ImGui::Text(name.c_str());
      ImGui::TableSetColumnIndex(1);

      auto labelHidden = "##" + name;
      ImGui::InputText(labelHidden.c_str(), &str);
  }

  inline void addPath(const std::string &name, std::string &str, bool isDir = false) {

    ImGui::TableNextRow();
    ImGui::TableSetColumnIndex(0);
    ImGui::AlignTextToFramePadding();
    ImGui::Text(name.c_str());
    ImGui::TableSetColumnIndex(1);

    auto labelHidden = "##" + name;
    ImGui::PushID(labelHidden.c_str());
    if (ImGui::Button(ICON_FA_FOLDER)) {
      Utils::FilePicker::open([&str](const std::string &path) {
        str = path;
      }, isDir);
    }
    ImGui::PopID();
    ImGui::SameLine();
    ImGui::InputText(labelHidden.c_str(), &str);
  }

}
