/**
* @copyright 2025 - Max Bebök
* @license MIT
*/
#pragma once
#include <libdragon.h>

namespace P64
{
  /**
   * Game Object:
   * This the main struct used in scenes to represent all sorts of entities.
   * Objects can have multiple components attached to them, which provide functionality
   * for running game logic and drawing things.
   * The exact makeup is set up in the editor, and loaded during a scene load.
   * Dynamic creation at runtime is only possible through prefabs. (<- @TODO)
   */
  class Object
  {
    private:

    public:
      struct CompRef
      {
        uint8_t type{};
        uint8_t flags{};
        uint16_t offset{};
      };

      ~Object();

      uint16_t id{};
      uint16_t compCount{0};

      // extra data, is overlapping with component data if unused
      fm_vec3_t pos{};
      fm_vec3_t scale{};

      // component references, this is then also followed by a buffer for the actual data
      // the object allocation logic keeps extra space to fit everything

      //CompRef compRefs[];
      //uint8_t compData[];

      /**
       * Returns pointer to the component reference table.
       * This is beyond the Object struct, but still in valid allocated memory.
       * @return pointer
       */
      [[nodiscard]] CompRef* getCompRefs() const {
        return (CompRef*)((uint8_t*)this + sizeof(Object));
      }

      /**
       * Returns pointer to the component data buffer.
       * This is beyond the Object struct, but still in valid allocated memory.
       * @return pointer
       */
      [[nodiscard]] char* getCompData() const {
        return (char*)getCompRefs() + sizeof(CompRef) * compCount;
      }

      /**
       * Removes the given object from the scene.
       * This is a shortcut for SceneManager::getCurrent().removeObject(obj);
       * Note: deletion is deferred until the end of the frame.
       */
      void remove();
  };
}