/**
* @copyright 2026 - GitHub Copilot
* @license MIT
*/
#include "../components.h"
#include "../../../context.h"
#include "../../../editor/imgui/helper.h"
#include "../../../utils/json.h"
#include "../../../utils/jsonBuilder.h"
#include "../../../utils/binaryFile.h"
#include "../../../utils/logger.h"
#include "../../assetManager.h"
#include "../../../editor/pages/parts/viewport3D.h"
#include "../../../renderer/scene.h"
#include "../../../utils/meshGen.h"

namespace Project::Component::Audio3D
{
  struct Data
  {
    PROP_U64(audioUUID);
    PROP_FLOAT(volume);
    PROP_BOOL(loop);
    PROP_BOOL(autoPlay);
    PROP_FLOAT(radius);
    PROP_U64(listenerUUID);
  };

  std::shared_ptr<void> init(Object &obj) {
    auto data = std::make_shared<Data>();
    data->radius.value = 500.0f;
    return data;
  }

  nlohmann::json serialize(const Entry &entry) {
    Data &data = *static_cast<Data*>(entry.data.get());
    Utils::JSON::Builder builder{};
    builder.set(data.audioUUID);
    builder.set(data.volume);
    builder.set(data.loop);
    builder.set(data.autoPlay);
    builder.set(data.radius);
    builder.set(data.listenerUUID);
    return builder.doc;
  }

  std::shared_ptr<void> deserialize(nlohmann::json &doc) {
    auto data = std::make_shared<Data>();
    Utils::JSON::readProp(doc, data->audioUUID);
    Utils::JSON::readProp(doc, data->volume, 1.0f);
    Utils::JSON::readProp(doc, data->loop);
    Utils::JSON::readProp(doc, data->autoPlay);
    Utils::JSON::readProp(doc, data->radius, 500.0f);
    Utils::JSON::readProp(doc, data->listenerUUID);
    return data;
  }

  void build(Object& obj, Entry &entry, Build::SceneCtx &ctx)
  {
    Data &data = *static_cast<Data*>(entry.data.get());

    auto res = ctx.assetUUIDToIdx.find(data.audioUUID.value);
    uint16_t id = 0xDEAD;
    if (res == ctx.assetUUIDToIdx.end()) {
      Utils::Logger::log("Component Audio3D: Audio UUID not found: " + std::to_string(entry.uuid), Utils::Logger::LEVEL_ERROR);
    } else {
      id = res->second;
    }

    uint8_t flags = 0;
    if(data.loop.resolve(obj))flags |= 1 << 0;
    if(data.autoPlay.resolve(obj))flags |= 1 << 1;

    // Resolve listener object
    auto listenerObj = ctx.scene ? ctx.scene->getObjectByUUID(data.listenerUUID.value) : nullptr;
    uint16_t listenerId = listenerObj ? listenerObj->id : 0;

    // Must match InitDataPacked in runtime audio3d.cpp
    ctx.fileObj.write<uint16_t>(id);
    ctx.fileObj.write<uint16_t>((uint16_t)(data.volume.resolve(obj) * 0xFFFF));
    ctx.fileObj.write<uint16_t>(listenerId);
    ctx.fileObj.write<uint8_t>(flags);
    ctx.fileObj.write<uint8_t>(0); // padding
    ctx.fileObj.write<float>(data.radius.resolve(obj));
  }

  void draw(Object &obj, Entry &entry)
  {
    Data &data = *static_cast<Data*>(entry.data.get());

    if (ImTable::start("Comp", &obj)) {
      ImTable::add("Name", entry.name);

      auto &audioList = ctx.project->getAssets().getTypeEntries(FileType::AUDIO);
      ImTable::addVecComboBox("Audio", audioList, data.audioUUID.resolve(obj));
      ImTable::addObjProp("Volume", data.volume);
      ImTable::addObjProp("Loop", data.loop);
      ImTable::addObjProp("Auto-Play", data.autoPlay);
      ImTable::addObjProp("Radius", data.radius);

      // Listener Selector
      auto &map = ctx.project->getScenes().getLoadedScene()->objectsMap;
      std::vector<ImTable::ComboEntry> objList;
      objList.push_back({0, "<None>"});

      for (auto &[id, object] : map) {
        if(object->uuid == obj.uuid) continue;
        objList.push_back({
          .value = object->uuid,
          .name = object->name,
        });
      }
      
      // Use addObjProp with custom edit function to support prefab overrides
      ImTable::addObjProp<uint64_t>("Listener", data.listenerUUID, [&objList](uint64_t *val) -> bool {
        ImGui::VectorComboBox("Listener", objList, *val);
        return false;
      }, nullptr);

      ImTable::end();
    }
  }

  void draw3D(Object& obj, Entry &entry, Editor::Viewport3D &vp, SDL_GPUCommandBuffer* cmdBuff, SDL_GPURenderPass* pass)
  {
    Utils::Mesh::addSprite(*vp.getSprites(), obj.pos.resolve(obj), obj.uuid, 4);
  }
}
