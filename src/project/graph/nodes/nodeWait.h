/**
* @copyright 2026 - Max Bebök
* @license MIT
*/
#pragma once

#include "baseNode.h"
#include "../../../utils/hash.h"

namespace Project::Graph::Node
{
  class Wait : public Base
  {
    private:
      float time{};

    public:
      Wait()
      {
        uuid = Utils::Hash::randomU64();
        outputCount = 1;
        setTitle("Delay (sec.)");
        setStyle(std::make_shared<ImFlow::NodeStyle>(IM_COL32(90,191,93,255), ImColor(0,0,0,255), 3.5f));

        addIN<int>("In", 0, ImFlow::ConnectionFilter::SameType());
        addOUT<int>("Out", nullptr)->behaviour([this]() { return 42; });
      }

      void draw() override {
        ImGui::SetNextItemWidth(70.f);
        ImGui::InputFloat("##Time", &time);
      }

      void serialize(nlohmann::json &j) override {
        j["time"] = time;
      }

      void deserialize(nlohmann::json &j) override {
        time = j["time"];
      }

      void build(Utils::BinaryFile &f) override {
        f.write<uint16_t>(time * 1000); // milliseconds
      }
  };
}