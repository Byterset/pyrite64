/**
* @copyright 2026 - Max Bebök
* @license MIT
*/
#include "script/nodeGraph.h"

#include <unordered_set>

#include "scene/object.h"
#include "scene/scene.h"

namespace
{
  constexpr const char* NODE_TYPE_NAMES[] {
    "START",
    "WAIT",
    "OBJ_DEL",
    "OBJ_EVENT",
    "COMPARE",
    "VALUE",
    "REPEAT",
  };

  std::unordered_map<uint32_t, P64::NodeGraph::UserFunc> userFunctionMap{};

  void dummyFunction(uint32_t arg0)
  {
    debugf("Graph called undefined function, argument: 0x%08lX\n", arg0);
  }
}

namespace P64::NodeGraph
{
  struct NodeDef
  {
    NodeType type{};
    uint8_t outCount{};
    int16_t outOffsets[];

    NodeDef *getNext(uint32_t idx) {
      if(idx >= outCount)return nullptr;
      return (NodeDef*)((uint8_t*)this + outOffsets[idx]);
    }

    uint16_t *getDataPtr() {
      return (uint16_t*)&outOffsets[outCount];
    }
  };

  struct GraphDef
  {
    uint16_t nodeCount;
    uint16_t memSize;
    NodeDef start;
  };

  inline void iterateNodes(NodeDef* node, int level, std::function<bool(NodeDef*, int)> fn)
  {
    for (uint16_t i = 0; i < node->outCount; i++) {
      auto nextNode = (NodeDef*)((uint8_t*)node + node->outOffsets[i]);
      if(!fn(nextNode, level))continue;
      iterateNodes(nextNode, level + 1, fn);
    }
  };

  void* load(const char* path)
  {
    auto data = asset_load(path, nullptr);

    std::unordered_set<NodeDef*> visitedNodes{};
    iterateNodes(&((GraphDef*)data)->start, 0, [&](NodeDef* node, int level)
    {
      if(visitedNodes.find(node) != visitedNodes.end())return false;
      visitedNodes.insert(node);

      if(node->type == NodeType::FUNC)
      {
        u_uint32_t* nodeData = (u_uint32_t*)&node->outOffsets[node->outCount];
        uint32_t funcHash = *nodeData;
        auto it = userFunctionMap.find(funcHash);
        if(it == userFunctionMap.end()) {
          *nodeData = (uint32_t)dummyFunction;
        } else {
          *nodeData = (uint32_t)it->second;
        }
      }
      return true;
    });

    return data;
  }
}

void P64::NodeGraph::Instance::update(float deltaTime) {
  if(!currNode)return;

  const uint16_t *data = currNode->getDataPtr();
  const uint8_t *dataU8 = (uint8_t*)data;
  const u_uint32_t *dataU32 = (u_uint32_t*)data;

  uint32_t outputIndex = 0;

  //printNode(currNode, 0);

  switch(currNode->type)
  {
    case NodeType::START:
      break;

    case NodeType::WAIT:
      reg += (uint16_t)(deltaTime * 1000.0f);
      if(reg < data[0])return;
      reg = 0;
      break;

    case NodeType::OBJ_DEL:
      if(object)object->remove();
      break;

    case NodeType::OBJ_EVENT:
      object->getScene().sendEvent(
        data[0] == 0 ? object->id : data[0],
        object->id,
        data[1],
        (data[2] << 16) | data[3]
      );
      break;
    case NodeType::REPEAT:
    {
      auto &count = memory[dataU8[0]];
      if(count != dataU8[1]) {
        ++count;
        outputIndex = 0;
      } else {
        outputIndex = 1;
        count = 0;
      }
    }break;

    case NodeType::FUNC:
      if(!result) {
        result = &((UserFunc)dataU32[0])(dataU32[1]);
      }
      if(!result->tryGetResult(reg))return;
      result = nullptr;
    break;

    default:
      debugf("Unhandled node type: %d\n", (uint8_t)currNode->type);
      break;
  }

  currNode = currNode->getNext(outputIndex);
}

void P64::NodeGraph::registerFunction(uint32_t strCRC32, UserFunc fn)
{
  userFunctionMap[strCRC32] = fn;
}
