/**
* @copyright 2026 - Max Bebök
* @license MIT
*/
#pragma once

#include "baseNode.h"
#include "../../../utils/hash.h"

namespace Project::Graph::Node
{
  class Func : public Base
  {
    private:
      std::string funcName{};
      uint32_t arg0;

    public:
      constexpr static const char* NAME = ICON_MDI_FUNCTION " Function";

      Func()
      {
        uuid = Utils::Hash::randomU64();
        setTitle(NAME);
        setStyle(std::make_shared<ImFlow::NodeStyle>(IM_COL32(90,191,93,255), ImColor(0,0,0,255), 3.5f));

        addIN<TypeLogic>("", ImFlow::ConnectionFilter::SameType(), PIN_STYLE_LOGIC);
        addOUT<TypeLogic>("", PIN_STYLE_LOGIC);
        //addOUT<TypeValue>("Res", PIN_STYLE_VALUE);
      }

      void draw() override {
        //ImGui::Text("Func:");
        //ImGui::SameLine();
        auto textWidth  = ImGui::CalcTextSize(funcName.c_str()).x + 16;
        ImGui::SetNextItemWidth(fmaxf(textWidth, 50.0f));
        ImGui::InputText("##FuncName", &funcName);
        ImGui::SetNextItemWidth(50.f);
        ImGui::InputScalar("Arg.", ImGuiDataType_U32, &arg0);

        //showIN("", 0, ImFlow::ConnectionFilter::SameType(), PIN_STYLE_LOGIC);
      }

      void serialize(nlohmann::json &j) override {
        j["funcName"] = funcName;
        j["arg0"] = arg0;
      }

      void deserialize(nlohmann::json &j) override {
        funcName = j.value("funcName", "");
        arg0 = j.value("arg0", 0);
      }

      void build(Utils::BinaryFile &f, uint32_t &memOffset) override {
        f.write<uint32_t>(Utils::Hash::crc32(funcName));
        f.write<uint32_t>(arg0);
      }
  };
}