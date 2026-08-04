/**
* @copyright 2026 - Max Bebök
* @license MIT
*/
#pragma once
#include <cstdint>

#include "../../utils/prop.h"

namespace Project
{
  class Object;
  class AssetManager;
  struct SceneConf;
}

/**
 * Migration of scene/prefab files that were authored before the engine and editor
 * switched to meters.
 *
 * Version 1 stored every length in "visual units" (`visualUnitsPerMeter` of them per
 * meter) and rendered models at the size their vertices were quantized to (the asset's
 * `baseScale`). Version 2 stores meters everywhere and divides the vertex scale back
 * out at render time, so the conversion below keeps the rendered result identical.
 */
namespace Project::Migration
{
  // Current scene/prefab file version.
  constexpr int FILE_VERSION = 2;

  // Assumed visual-units-per-meter for files that carry no scene config (prefabs).
  constexpr float DEFAULT_VISUAL_UNITS_PER_METER = 100.0f;

  /**
   * Conversion state for one object while migrating a v1 file.
   * Patches a property's own value plus any prefab-instance override of it.
   */
  struct V1Context
  {
    // visual units per meter of the scene the file belongs to
    float visualUnitsPerMeter{DEFAULT_VISUAL_UNITS_PER_METER};
    // divisor for lengths the runtime multiplies by the object scale. Objects showing a
    // model absorb the model's vertex scale into their object scale, so their component
    // lengths convert by that instead.
    float relativeDiv{DEFAULT_VISUAL_UNITS_PER_METER};
    // false while walking a prefab definition reached through an instance: those values
    // belong to the prefab file and are migrated when it is loaded, only overrides
    // stored on the enclosing instances are patched here.
    bool patchValues{true};
    // number of leading PropScope layers that belong to migratable (non-definition) maps
    size_t patchableLayers{0};

    /// Lengths that are used as-is by the runtime (camera planes, light size).
    void scaleAbsolute(Property<float> &prop) const;
    void scaleAbsolute(Property<glm::vec3> &prop) const;

    /// Lengths the runtime multiplies by the object scale (collider/culling extents).
    void scaleRelative(Property<float> &prop) const;
    void scaleRelative(Property<glm::vec3> &prop) const;
  };

  /**
   * Converts an object tree, its components and prefab-instance overrides from
   * v1 visual units to meters. Call on the root of a freshly deserialized v1 file.
   *
   * @param obj root object to convert
   * @param assets asset manager, used to resolve model vertex scales and prefabs
   * @param visualUnitsPerMeter the file's v1 conversion factor
   * @param rootIsContainer true for a scene, whose root only groups the real objects.
   *                        False for a prefab, whose root is an object itself.
   */
  void migrateV1(Object &obj, AssetManager &assets, float visualUnitsPerMeter, bool rootIsContainer);

  /// Converts the length-valued fields of a scene config (fog distances) to meters.
  void migrateV1SceneConf(SceneConf &conf, float visualUnitsPerMeter);
}
