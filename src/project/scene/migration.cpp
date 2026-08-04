/**
* @copyright 2026 - Max Bebök
* @license MIT
*/
#include "migration.h"

#include "object.h"
#include "prefab.h"
#include "scene.h"
#include "../assetManager.h"

namespace
{
  using namespace Project;

  template<typename T>
  void patchProp(Property<T> &prop, float factor, const Migration::V1Context &ctx)
  {
    if(ctx.patchValues)prop.value = prop.value * factor;

    size_t layerCount = std::min(ctx.patchableLayers, PropScope::stack.size());
    for(size_t i = 0; i < layerCount; ++i) {
      auto &layer = PropScope::stack[i];
      auto *map = const_cast<std::unordered_map<uint64_t, GenericValue>*>(layer.overrides);
      auto it = map->find(PropScope::combine(layer.pathHash, prop.id));
      if(it != map->end()) {
        it->second.template get<T>() = it->second.template get<T>() * factor;
      }
    }
  }

  /**
   * Vertex scale of the 3D model an object shows, as it was configured in v1.
   * Model wins over an animated model, which wins over a collision mesh, so the visual
   * size is what is preserved exactly when an object mixes assets of differing scales.
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

  void migrateObject(Object &obj, AssetManager &assets, Migration::V1Context ctx)
  {
    if(PropScope::stack.size() > PropScope::MAX_DEPTH)return;

    auto *srcObj = &obj;
    bool isInstance = obj.isPrefabInstance();
    if(isInstance) {
      auto prefab = assets.getPrefabByUUID(obj.uuidPrefab.value);
      if(prefab)srcObj = &prefab->obj;
    }

    // Mirrors Build::writeObject: this node's overrides stay active for its whole subtree.
    PropScope::PrefabLayer objLayer{obj.propOverrides};
    if(ctx.patchValues)ctx.patchableLayers = PropScope::stack.size();

    float baseScale = findLegacyBaseScale(obj, *srcObj, assets);
    ctx.relativeDiv = baseScale > 0.0f ? baseScale : ctx.visualUnitsPerMeter;

    // Positions become meters. Showing a model additionally folds the model's vertex
    // scale into the object scale, since the runtime no longer renders vertex units.
    patchProp(obj.pos, 1.0f / ctx.visualUnitsPerMeter, ctx);
    if(baseScale > 0.0f) {
      patchProp(obj.scale, baseScale / ctx.visualUnitsPerMeter, ctx);
    }

    auto migrateComps = [&](Object &source) {
      for(auto &entry : source.components) {
        const auto &info = Component::TABLE[entry.id];
        if(!info.funcMigrateV1)continue;
        PropScope::Path compPath(entry.uuid);
        info.funcMigrateV1(entry, ctx);
      }
    };
    migrateComps(*srcObj);
    if(srcObj != &obj)migrateComps(obj);

    // Descending into a prefab definition: its values belong to the prefab file, only the
    // override slots the enclosing instances hold for them are patched here.
    Migration::V1Context childCtx = ctx;
    if(isInstance)childCtx.patchValues = false;

    for(const auto &child : srcObj->children) {
      PropScope::Path childPath(child->uuid);
      migrateObject(*child, assets, childCtx);
    }

    // Children added directly to an instance are ordinary scene objects and resolve
    // against their own overrides only.
    if(isInstance && srcObj != &obj && !obj.children.empty()) {
      PropScope::ResetScope freshScope;
      Migration::V1Context ownCtx = ctx;
      ownCtx.patchValues = true;
      ownCtx.patchableLayers = 0;
      for(const auto &child : obj.children) {
        migrateObject(*child, assets, ownCtx);
      }
    }
  }
}

void Project::Migration::V1Context::scaleAbsolute(Property<float> &prop) const {
  patchProp(prop, 1.0f / visualUnitsPerMeter, *this);
}

void Project::Migration::V1Context::scaleAbsolute(Property<glm::vec3> &prop) const {
  patchProp(prop, 1.0f / visualUnitsPerMeter, *this);
}

void Project::Migration::V1Context::scaleRelative(Property<float> &prop) const {
  patchProp(prop, 1.0f / relativeDiv, *this);
}

void Project::Migration::V1Context::scaleRelative(Property<glm::vec3> &prop) const {
  patchProp(prop, 1.0f / relativeDiv, *this);
}

void Project::Migration::migrateV1(Object &obj, AssetManager &assets,
                                   float visualUnitsPerMeter, bool rootIsContainer)
{
  if(visualUnitsPerMeter <= 0.0f)visualUnitsPerMeter = DEFAULT_VISUAL_UNITS_PER_METER;

  V1Context ctx{};
  ctx.visualUnitsPerMeter = visualUnitsPerMeter;
  ctx.relativeDiv = visualUnitsPerMeter;

  PropScope::ResetScope freshScope;
  if(rootIsContainer) {
    for(const auto &child : obj.children) {
      migrateObject(*child, assets, ctx);
    }
  } else {
    migrateObject(obj, assets, ctx);
  }
}

void Project::Migration::migrateV1SceneConf(SceneConf &conf, float visualUnitsPerMeter)
{
  if(visualUnitsPerMeter <= 0.0f)visualUnitsPerMeter = DEFAULT_VISUAL_UNITS_PER_METER;
  float toMeters = 1.0f / visualUnitsPerMeter;

  for(auto *layers : {&conf.layers3D, &conf.layersPtx, &conf.layers2D}) {
    for(auto &layer : *layers) {
      layer.fogMin.value *= toMeters;
      layer.fogMax.value *= toMeters;
    }
  }
}
