/**
* @copyright 2026 - Max Bebök
* @license MIT
*/
#pragma once
#include <libdragon.h>

#include "assets/assetManager.h"

namespace P64
{
  class Object;
}

namespace P64::NodeGraph
{
  enum class NodeType : uint8_t
  {
    START = 0,
    WAIT = 1,
    OBJ_DEL = 2,
    OBJ_EVENT = 3,
    COMPARE = 4,
    VALUE = 5,
    REPEAT = 6,
    FUNC = 7,
  };

  struct GraphDef;
  struct NodeDef;

  /**
   * Result object used to communicate between a graph and external functions.
   * The expected flow is:
   * - A called user function must return a reference to a Result object
   * - The graph can poll if a value has been set, and wait for it.
   * - the user code eventually does so
   * - the graph acknowledges and used that value
   * - the user can can see this and decide to e.g. free memory if it was allocated
   *
   * A 'Result' can either be allocated on demand by user code, or a global instance returned
   * which is reset after each new call.
   */
  class Result
  {
    private:
      uint16_t value{};
      uint8_t isReady{};
      uint8_t isConsumed{};

    public:
      Result() = default;

      /**
       * Sets a value and marks it as ready to be consumed by the graph.
       * @param val
       */
      void setResult(uint16_t val) {
        value = val;
        isReady = 1;
        isConsumed = 0;
      }

      /**
       * Used by the graph internally, tries to get the value if ready.
       * @param val Reference to write the value to.
       * @return True if a value was written, false otherwise.
       */
      bool tryGetResult(uint16_t &val)
      {
        if(isReady && !isConsumed) {
          val = value;
          isConsumed = 1;
          return true;
        }
        return false;
      }

      /**
       * Can be used to check if the entire process has been completed,
       * and this object is no longer needed.
       * @return True if the value was consumed, false otherwise.
       */
      [[nodiscard]] bool isDone() const {
        return isConsumed;
      }
  };

  class Instance
  {
    private:
      GraphDef* graphDef{};

      NodeDef* currNode{};
      union
      {
        Result* result{};
        uint16_t reg;
      };
      uint8_t memory[64]{}; // @TODO: dynamic

    public:
      Object *object{};

      Instance() = default;

      explicit Instance(uint16_t assetIdx) {
        graphDef = (GraphDef*)AssetManager::getByIndex(assetIdx);
        currNode = (NodeDef*)((char*)graphDef + 4);
      }

      void update(float deltaTime);
  };

  typedef Result&(*UserFunc)(uint32_t);

  void registerFunction(uint32_t strCRC32, UserFunc fn);
}
