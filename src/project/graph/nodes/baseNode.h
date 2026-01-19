/**
* @copyright 2026 - Max Bebök
* @license MIT
*/
#pragma once

#include "ImNodeFlow.h"
#include "json.hpp"

namespace Project::Graph::Node
{
  class Base : public ImFlow::BaseNode
  {
    public:
      uint64_t uuid{};
      uint32_t type{};
      uint8_t outputCount{0};

      virtual void serialize(nlohmann::json &j) = 0;
      virtual void deserialize(nlohmann::json &j) = 0;
      virtual void build(Utils::BinaryFile &f) = 0;
  };
}