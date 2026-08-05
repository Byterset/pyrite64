/**
* @copyright 2026 - Max Bebök
* @license MIT
*/
#pragma once
#include <cstdint>
#include <string>
#include <vector>

#include "../../utils/prop.h"

namespace Project
{
  class Object;
  class Project;
  class AssetManager;
  struct SceneConf;
}

/**
 * Versioned upgrade of scene and prefab files.
 *
 * Every file carries a "version" field. A file older than FILE_VERSION is brought up to date by
 * running each step in STEPS whose target version is newer than the file's, in order, so a
 * project can skip any number of releases and still upgrade in one go.
 *
 * Adding a migration:
 *   1. write a `void stepToVn(Context&)` in migration.cpp
 *   2. add `{n, "what changes, in user terms", stepToVn}` to STEPS
 *   3. bump FILE_VERSION
 * Scanning, chaining, the confirmation dialog and saving are generic and need no changes.
 */
namespace Project::Migration
{
  /// Version written to newly saved files. Bump when appending a step.
  constexpr int FILE_VERSION = 2;

  /// Assumed visual-units-per-meter for pre-v2 files.
  constexpr float DEFAULT_VISUAL_UNITS_PER_METER = 100.0f;

  enum class DocType { SCENE, PREFAB };

  /// The document a step operates on.
  struct Context
  {
    AssetManager &assets;
    DocType docType{DocType::SCENE};
    /// Scene: the container whose children are the real objects. Prefab: the object itself.
    Object &root;
    /// Scene config, null for prefabs.
    SceneConf *conf{nullptr};
  };

  struct Step
  {
    int toVersion{};
    /// One line shown to the user before the migration runs.
    const char *summary{};
    void (*run)(Context &ctx){};
  };

  /// All known steps, ordered by target version.
  extern const std::vector<Step> STEPS;

  /**
   * Runs every step newer than `fromVersion` on the given document.
   * @return the version the document is at afterwards
   */
  int run(int fromVersion, Context &ctx);

  /// A file that is behind FILE_VERSION.
  struct PendingDoc
  {
    DocType type{DocType::SCENE};
    std::string name{};
    std::string path{};
    int version{1};
    int sceneId{0}; // scenes only
  };

  struct ScanResult
  {
    std::vector<PendingDoc> docs{};
    /// Summaries of every step that will run, deduplicated across all documents.
    std::vector<const char*> summaries{};

    bool empty() const { return docs.empty(); }
    size_t countOf(DocType type) const;
  };

  /**
   * Lists every scene and prefab of the project that is behind FILE_VERSION.
   * Reads only file versions, so it is cheap enough to call before any scene load or build.
   *
   * Always covers the whole project rather than a single scene: objects resolve against prefabs
   * and prefabs against each other, and migrating everything at once is what keeps a loaded
   * scene from sitting on converted-but-unsaved data.
   */
  ScanResult scanProject(Project &project);

  /**
   * Migrates and re-saves every document in `scan`.
   * Rewrites project files in place, so only call after the user agreed.
   */
  void apply(Project &project, const ScanResult &scan);

  // ---- helpers used by the pre-meter (v2) step, see Component::funcMigrateV1 ----

  /**
   * Conversion state for one object while migrating a pre-meter file.
   * Patches a property's own value plus any prefab-instance override of it.
   */
  struct V1Context
  {
    // visual units per meter of the file being migrated
    float visualUnitsPerMeter{DEFAULT_VISUAL_UNITS_PER_METER};
    // divisor for lengths the runtime multiplies by the object's world scale
    float relativeDiv{DEFAULT_VISUAL_UNITS_PER_METER};
    // false while walking a prefab definition reached through an instance: those values belong
    // to the prefab file and are migrated when it is loaded, only overrides stored on the
    // enclosing instances are patched here.
    bool patchValues{true};
    // number of leading PropScope layers that belong to migratable (non-definition) maps
    size_t patchableLayers{0};

    /// Lengths the runtime uses as-is (camera planes, light size).
    void scaleAbsolute(Property<float> &prop) const;
    void scaleAbsolute(Property<glm::vec3> &prop) const;

    /// Lengths the runtime multiplies by the object's world scale (collider/culling extents).
    void scaleRelative(Property<float> &prop) const;
    void scaleRelative(Property<glm::vec3> &prop) const;
  };
}
