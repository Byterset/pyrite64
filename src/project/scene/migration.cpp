/**
* @copyright 2026 - Max Bebök
* @license MIT
*/
#include "migration.h"

#include <algorithm>
#include <filesystem>

#include "object.h"
#include "prefab.h"
#include "scene.h"
#include "sceneManager.h"
#include "../project.h"
#include "../assetManager.h"
#include "../../utils/fs.h"
#include "../../utils/json.h"
#include "../../utils/logger.h"

namespace fs = std::filesystem;

namespace
{
  using namespace Project;

  // ---------------------------------------------------------------------------------------
  // v2: visual units -> meters
  //
  // Pre-v2 files stored every length in "visual units" (`visualUnitsPerMeter` of them per
  // meter) and rendered models at the size their vertices were quantized to (the asset's
  // `baseScale`). Meters are the unit now, and the vertex scale is divided out at render time.
  // ---------------------------------------------------------------------------------------

  template<typename T>
  void patchProp(Property<T> &prop, float factor, const Migration::V1Context &ctx)
  {
    if(ctx.patchValues)prop.value = prop.value * factor;

    auto patchKey = [factor](std::unordered_map<uint64_t, GenericValue> *map, uint64_t key) {
      auto it = map->find(key);
      if(it != map->end())it->second.template get<T>() = it->second.template get<T>() * factor;
    };

    size_t layerCount = std::min(ctx.patchableLayers, PropScope::stack.size());
    bool bareIsScoped = false;
    for(size_t i = 0; i < layerCount; ++i) {
      auto &layer = PropScope::stack[i];
      auto *map = const_cast<std::unordered_map<uint64_t, GenericValue>*>(layer.overrides);
      patchKey(map, PropScope::combine(layer.pathHash, prop.id));
      if(map == ctx.ownOverrides && layer.pathHash == 0)bareIsScoped = true;
    }

    // Property::resolve falls back to the bare key on the owning object's own map, which is how
    // overrides authored before keys were path-scoped still apply. They have to be converted too.
    // Object-level props are already reached that way above (their path hash is 0), so those are
    // skipped here rather than scaled twice.
    if(ctx.ownOverrides && !bareIsScoped)patchKey(ctx.ownOverrides, prop.id);
  }

  /**
   * Vertex scale of the 3D model an object shows, as it was configured before v2.
   * Model wins over an animated model, which wins over a collision mesh, so the visual size is
   * what is preserved exactly when an object mixes assets of differing scales.
   */
  float findLegacyBaseScale(Object &obj, Object &srcObj, AssetManager &assets)
  {
    float best = 0.0f;
    int bestPrio = 0;

    auto scan = [&](Object &source) {
      for(auto &entry : source.components) {
        int prio = 0;
        switch(entry.id) {
          case 1:  prio = 3; break; // Model
          case 10: prio = 2; break; // AnimModel
          case 4:  prio = 1; break; // CollMesh
          default: continue;
        }
        if(prio <= bestPrio)continue;

        const auto &info = Component::TABLE[entry.id];
        if(!info.funcGetModelUUID)continue;

        auto *asset = assets.getEntryByUUID(info.funcGetModelUUID(entry));
        if(!asset || asset->type != FileType::MODEL_3D || asset->conf.baseScale <= 0)continue;

        best = (float)asset->conf.baseScale;
        bestPrio = prio;
      }
    };

    scan(srcObj);
    if(&srcObj != &obj)scan(obj);
    return best;
  }

  /**
   * @param obj object to convert
   * @param ancestorScale product of the scale factors already applied at or above this object's
   *        parent, or 1 if this object's transform is not composed with its parent's. Where
   *        composition does happen, a child's local offset is scaled by its ancestors' world
   *        scale, so rescaling a parent shrinks every descendant's offset by the same amount
   *        and the offsets have to compensate for it.
   * @param expanding mirrors the flag of the same name in Build::writeObject: only inside a
   *        prefab definition are transforms composed down the tree. Scene objects (including
   *        children a scene adds to an instance) are written with their raw local values and an
   *        identity parent transform, so their transforms are absolute and must stay untouched
   *        by what happened to the object they hang under.
   */
  void migrateObjectToV2(Object &obj, AssetManager &assets, Migration::V1Context ctx,
                         float ancestorScale, bool expanding)
  {
    if(PropScope::stack.size() > PropScope::MAX_DEPTH)return;

    auto *srcObj = &obj;
    if(obj.isPrefabInstance()) {
      auto prefab = assets.getPrefabByUUID(obj.uuidPrefab.value);
      if(prefab)srcObj = &prefab->obj;
    }
    // A definition was actually resolved. If the prefab is missing, `obj` is treated as a plain
    // object so its own children still get converted.
    const bool hasDefinition = srcObj != &obj;

    // Mirrors Build::writeObject: this node's overrides stay active for its whole subtree.
    PropScope::PrefabLayer objLayer{obj.propOverrides};
    ctx.ownOverrides = nullptr;
    if(ctx.patchValues) {
      ctx.patchableLayers = PropScope::stack.size();
      // Build::writeObject hands this node to every component build, so its map is the one
      // un-scoped overrides resolve against, for the definition's components too.
      ctx.ownOverrides = &obj.propOverrides;
    }

    // Showing a model folds the model's vertex scale into the object scale, since the runtime
    // no longer renders vertex units. Inside a prefab the factor belongs to the whole chain down
    // to the mesh, not to each model-bearing object: `Build::writeObject` multiplies a composed
    // object's scale by all of its ancestors', so only the part they have not already
    // contributed is applied here. A model under an already-converted parent keeps its scale,
    // while an absolute object (ancestorScale 1) always takes the full factor.
    float baseScale = findLegacyBaseScale(obj, *srcObj, assets);
    float ownScale = 1.0f;
    if(baseScale > 0.0f && ancestorScale > 0.0f) {
      ownScale = (baseScale / ctx.visualUnitsPerMeter) / ancestorScale;
    }

    // Component lengths the runtime multiplies by the object's world scale have to account for
    // every factor applied at this object and above it.
    ctx.relativeDiv = ctx.visualUnitsPerMeter * ancestorScale * ownScale;

    patchProp(obj.pos, 1.0f / (ctx.visualUnitsPerMeter * ancestorScale), ctx);
    if(ownScale != 1.0f) {
      patchProp(obj.scale, ownScale, ctx);
    }

    auto migrateComps = [&](Object &source, bool patchValues) {
      Migration::V1Context compCtx = ctx;
      compCtx.patchValues = patchValues;
      for(auto &entry : source.components) {
        const auto &info = Component::TABLE[entry.id];
        if(!info.funcMigrateV1)continue;
        PropScope::Path compPath(entry.uuid);
        info.funcMigrateV1(entry, compCtx);
      }
    };
    // The definition's own values belong to the prefab file and are converted when that file is
    // migrated, only the override slots the enclosing instances hold for them are patched here.
    migrateComps(*srcObj, hasDefinition ? false : ctx.patchValues);
    if(hasDefinition)migrateComps(obj, ctx.patchValues);

    const float composedScale = ancestorScale * ownScale;

    // Resolving an instance composes its definition's subtree onto it, so from here down the
    // tree is composed even if this object itself was absolute.
    const bool childExpanding = expanding || hasDefinition;

    Migration::V1Context childCtx = ctx;
    if(hasDefinition)childCtx.patchValues = false;

    for(const auto &child : srcObj->children) {
      PropScope::Path childPath(child->uuid);
      migrateObjectToV2(*child, assets, childCtx, childExpanding ? composedScale : 1.0f,
                        childExpanding);
    }

    // Children added directly to an instance are ordinary objects of the file being migrated and
    // resolve against their own overrides only. They keep this object's `expanding`: a scene
    // hangs them off the instance without composing, a prefab composes them like any other child.
    if(hasDefinition && !obj.children.empty()) {
      PropScope::ResetScope freshScope;
      Migration::V1Context ownCtx = ctx;
      ownCtx.patchValues = true;
      ownCtx.patchableLayers = 0;
      for(const auto &child : obj.children) {
        migrateObjectToV2(*child, assets, ownCtx, expanding ? composedScale : 1.0f, expanding);
      }
    }
  }

  void stepToV2(Migration::Context &ctx)
  {
    float visualUnits = Migration::DEFAULT_VISUAL_UNITS_PER_METER;
    if(ctx.conf && ctx.conf->renderScale.value > 0.0f) {
      visualUnits = ctx.conf->renderScale.value;
    }

    // Fog distances are view-space lengths the runtime now scales itself.
    if(ctx.conf) {
      float toMeters = 1.0f / visualUnits;
      for(auto *layers : {&ctx.conf->layers3D, &ctx.conf->layersPtx, &ctx.conf->layers2D}) {
        for(auto &layer : *layers) {
          layer.fogMin.value *= toMeters;
          layer.fogMax.value *= toMeters;
        }
      }
    }

    Migration::V1Context v1{};
    v1.visualUnitsPerMeter = visualUnits;
    v1.relativeDiv = visualUnits;

    PropScope::ResetScope freshScope;
    if(ctx.docType == Migration::DocType::SCENE) {
      // the scene root only groups the real objects, whose transforms are absolute
      for(const auto &child : ctx.root.children) {
        migrateObjectToV2(*child, ctx.assets, v1, 1.0f, false);
      }
    } else {
      // a prefab is baked flat and root-relative, so its whole tree is composed
      migrateObjectToV2(ctx.root, ctx.assets, v1, 1.0f, true);
    }
  }

  /**
   * Vertex precision is computed from a model's bounds now instead of being read from the
   * asset's `baseScale`, so nearly every model quantizes to a different scale than the .t3dm
   * that is already sitting in `filesystem/`. The build only compares that file's timestamp
   * against the glTF, which a migration never touches, so it would happily ship the stale one
   * against the new scale in the asset table. Dropping the outputs makes it re-export them.
   */
  void projectStepToV2(::Project::Project &project)
  {
    auto projectPath = fs::path{project.getPath()};
    int cleared = 0;
    for(const auto &entry : project.getAssets().getTypeEntries(FileType::MODEL_3D)) {
      std::error_code ec{};
      if(fs::remove(projectPath / entry.outPath, ec))++cleared;
    }
    if(cleared > 0) {
      Utils::Logger::log("Cleared " + std::to_string(cleared) +
        " built 3D model(s), they are re-exported at the new vertex precision on the next build");
    }
  }

  // ---------------------------------------------------------------------------------------

  int readDocVersion(const fs::path &path)
  {
    auto doc = nlohmann::json::parse(Utils::FS::loadTextFile(path.string()), nullptr, false);
    if(!doc.is_object())return Migration::FILE_VERSION; // unreadable, leave it alone
    return doc.value("version", 1);
  }

  void collectSummaries(Migration::ScanResult &scan)
  {
    int oldest = Migration::FILE_VERSION;
    for(const auto &doc : scan.docs)oldest = std::min(oldest, doc.version);

    scan.summaries.clear();
    for(const auto &step : Migration::STEPS) {
      if(step.toVersion > oldest)scan.summaries.push_back(step.summary);
    }
  }
}

namespace Project::Migration
{

// Add a new step here and bump FILE_VERSION to make it run on all files with older versions.
const std::vector<Step> STEPS = {
  {2, "Positions, sizes and distances are converted from visual units to meters,"
      " 3D models are rebuilt at automatic vertex precision", stepToV2, projectStepToV2},
};

int run(int fromVersion, Context &ctx)
{
  int version = fromVersion;
  for(const auto &step : STEPS) {
    if(step.toVersion <= version)continue;
    step.run(ctx);
    version = step.toVersion;
  }
  return std::max(version, fromVersion);
}

void V1Context::scaleAbsolute(Property<float> &prop) const {
  patchProp(prop, 1.0f / visualUnitsPerMeter, *this);
}

void V1Context::scaleAbsolute(Property<glm::vec3> &prop) const {
  patchProp(prop, 1.0f / visualUnitsPerMeter, *this);
}

void V1Context::scaleRelative(Property<float> &prop) const {
  patchProp(prop, 1.0f / relativeDiv, *this);
}

void V1Context::scaleRelative(Property<glm::vec3> &prop) const {
  patchProp(prop, 1.0f / relativeDiv, *this);
}

size_t ScanResult::countOf(DocType type) const
{
  return std::count_if(docs.begin(), docs.end(),
    [type](const PendingDoc &d) { return d.type == type; });
}

ScanResult scanProject(Project &project)
{
  ScanResult scan{};

  auto scenesPath = fs::path{project.getPath()} / "data" / "scenes";
  if(fs::exists(scenesPath)) {
    for(const auto &dir : fs::directory_iterator{scenesPath}) {
      if(!dir.is_directory())continue;
      auto scenePath = dir.path() / "scene.json";
      if(!fs::exists(scenePath))continue;

      int version = readDocVersion(scenePath);
      if(version >= FILE_VERSION)continue;

      int id = 0;
      try { id = std::stoi(dir.path().filename().string()); } catch(...) { continue; }

      scan.docs.push_back({
        .type = DocType::SCENE,
        .name = "Scene " + std::to_string(id),
        .path = scenePath.string(),
        .version = version,
        .sceneId = id,
      });
    }
  }

  for(const auto &entry : project.getAssets().getTypeEntries(FileType::PREFAB)) {
    int version = entry.prefab ? entry.prefab->fileVersion : readDocVersion(entry.path);
    if(version >= FILE_VERSION)continue;

    scan.docs.push_back({
      .type = DocType::PREFAB,
      .name = entry.name,
      .path = entry.path,
      .version = version,
    });
  }

  // Prefabs first: a scene resolves its instances against them.
  std::stable_sort(scan.docs.begin(), scan.docs.end(),
    [](const PendingDoc &a, const PendingDoc &b) { return a.type > b.type; });

  collectSummaries(scan);
  return scan;
}

void apply(Project &project, const ScanResult &scan)
{
  auto &assets = project.getAssets();

  for(const auto &doc : scan.docs)
  {
    if(doc.type == DocType::PREFAB)
    {
      auto *entry = assets.getByPath(doc.path);
      if(!entry || !entry->prefab)continue;

      // AssetManager::migratePrefabs() already brought the loaded copy up to date, only need to write here.
      Context ctx{.assets = assets, .docType = DocType::PREFAB, .root = entry->prefab->obj};
      entry->prefab->memVersion = run(entry->prefab->memVersion, ctx);
      entry->prefab->fileVersion = entry->prefab->memVersion;
      entry->prefab->save(doc.path);
      Utils::Logger::log("Migrated prefab '" + doc.name + "'");
    }
    else
    {
      // Loading a scene migrates it in memory (Scene::deserialize), saving persists it.
      // The open scene is saved through its own instance so unsaved edits survive.
      auto *loaded = project.getScenes().getLoadedScene();
      if(loaded && loaded->getId() == doc.sceneId) {
        loaded->save();
      } else {
        Scene scene{doc.sceneId, project.getPath()};
        scene.save();
      }
      Utils::Logger::log("Migrated " + doc.name);
    }
  }

  // Whatever a step has to change outside the documents. Same selection the summaries use, so
  // the user is told about exactly the steps that end up running.
  int oldest = FILE_VERSION;
  for(const auto &doc : scan.docs)oldest = std::min(oldest, doc.version);
  for(const auto &step : STEPS) {
    if(step.toVersion > oldest && step.runProject)step.runProject(project);
  }
}

}
