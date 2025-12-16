/**
* @copyright 2025 - Max Bebök
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

namespace Project::Component::CollBody
{
  struct Data
  {
    glm::vec3 halfExtend{};
    glm::vec3 offset{};
    int type{};
  };

  std::shared_ptr<void> init(Object &obj) {
    auto data = std::make_shared<Data>();
    return data;
  }

  std::string serialize(Entry &entry) {
    Data &data = *static_cast<Data*>(entry.data.get());
    Utils::JSON::Builder builder{};
    builder.set("halfExtend", data.halfExtend);
    builder.set("offset", data.offset);
    builder.set("type", data.type);
    return builder.toString();
  }

  std::shared_ptr<void> deserialize(simdjson::simdjson_result<simdjson::dom::object> &doc) {
    auto data = std::make_shared<Data>();
    data->halfExtend = Utils::JSON::readVec3(doc, "halfExtend", glm::vec3{1.0f, 1.0f, 1.0f});
    data->offset = Utils::JSON::readVec3(doc, "offset");
    data->type = Utils::JSON::readInt(doc, "type");
    return data;
  }

  void build(Object&, Entry &entry, Build::SceneCtx &ctx)
  {
    Data &data = *static_cast<Data*>(entry.data.get());
    ctx.fileObj.write(data.halfExtend);
    ctx.fileObj.write(data.offset);
    ctx.fileObj.write<uint8_t>(data.type);
  }

  void draw(Object &obj, Entry &entry)
  {
    Data &data = *static_cast<Data*>(entry.data.get());

    if (ImGui::InpTable::start("Comp")) {
      ImGui::InpTable::addString("Name", entry.name);

      ImGui::InpTable::addComboBox("Type", data.type, {"Box", "Sphere", "Cylinder"});
      if(data.type == 1) {
        ImGui::InpTable::addInputFloat("Size", data.halfExtend.y);
        data.halfExtend.x = data.halfExtend.y;
        data.halfExtend.z = data.halfExtend.y;
      } else {
        ImGui::InpTable::addInputVec3("Size", data.halfExtend);
      }
      ImGui::InpTable::addInputVec3("Offset", data.offset);

      ImGui::InpTable::end();
    }
  }

  void draw3D(Object& obj, Entry &entry, Editor::Viewport3D &vp, SDL_GPUCommandBuffer* cmdBuff, SDL_GPURenderPass* pass)
  {
    Data &data = *static_cast<Data*>(entry.data.get());

    if(data.type == 0) // Box
    {
     glm::vec3 center = obj.pos + data.offset;
     glm::vec3 halfExt = data.halfExtend;

     glm::vec4 aabbCol{0.0f, 1.0f, 1.0f, 1.0f};

     Utils::Mesh::addLineBox(*vp.getLines(), center, halfExt, aabbCol);
     Utils::Mesh::addLineBox(*vp.getLines(), center, halfExt + 0.002f, aabbCol);
    } else if(data.type == 1) // Sphere
    {
      Utils::Mesh::addLineSphere(*vp.getLines(), obj.pos + data.offset, data.halfExtend, glm::vec4{0.0f, 1.0f, 1.0f, 1.0f});
    }
  }
}
