/**
* @copyright 2026 - Max Bebök
* @license MIT
*/
#pragma once
#include <string>

#include "ImNodeFlow.h"
#include "../../utils/binaryFile.h"

namespace Project::Graph
{
  class Graph
  {
    public:
      ImFlow::ImNodeFlow graph{};

      bool deserialize(const std::string &jsonData);
      std::string serialize();

      Utils::BinaryFile build();
  };
}
