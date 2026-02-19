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
#include "../../assetManager.h"
#include "../../../editor/pages/parts/viewport3D.h"
#include "../../../renderer/scene.h"
#include "../../../utils/meshGen.h"

#include "../../../../n64/engine/include/collision/flags.h"

namespace
{
  constexpr int32_t TYPE_SPHERE   = 0;
  constexpr int32_t TYPE_BOX      = 1;
  constexpr int32_t TYPE_CYLINDER = 2;
  constexpr int32_t TYPE_CAPSULE  = 3;
}

namespace Project::Component::CollBody
{
  struct Data
  {
    PROP_VEC3(halfExtend);
    PROP_VEC3(offset);
    PROP_S32(type);
    PROP_FLOAT(friction);
    PROP_FLOAT(bounce);
    PROP_BOOL(isTrigger);
    PROP_U32(maskRead);
    PROP_U32(maskWrite);
  };

  std::shared_ptr<void> init(Object &obj) {
    auto data = std::make_shared<Data>();
    data->friction.value = 0.5f;
    data->bounce.value = 0.0f;
    return data;
  }

  nlohmann::json serialize(const Entry &entry) {
    Data &data = *static_cast<Data*>(entry.data.get());
    return Utils::JSON::Builder{}
      .set(data.halfExtend)
      .set(data.offset)
      .set(data.type)
      .set(data.friction)
      .set(data.bounce)
      .set(data.isTrigger)
      .set(data.maskRead)
      .set(data.maskWrite)
      .doc;
  }

  std::shared_ptr<void> deserialize(nlohmann::json &doc) {
    auto data = std::make_shared<Data>();
    Utils::JSON::readProp(doc, data->halfExtend, glm::vec3{1.0f, 1.0f, 1.0f});
    Utils::JSON::readProp(doc, data->offset);
    Utils::JSON::readProp(doc, data->type);
    Utils::JSON::readProp(doc, data->friction, 0.5f);
    Utils::JSON::readProp(doc, data->bounce, 0.0f);
    Utils::JSON::readProp(doc, data->isTrigger, false);
    Utils::JSON::readProp(doc, data->maskRead, 0xFFu);
    Utils::JSON::readProp(doc, data->maskWrite, 0xFFu);
    return data;
  }

  void build(Object& obj, Entry &entry, Build::SceneCtx &ctx)
  {
    Data &data = *static_cast<Data*>(entry.data.get());
    ctx.fileObj.write(data.halfExtend.resolve(obj.propOverrides));
    ctx.fileObj.write(data.offset.resolve(obj.propOverrides));

    // Map editor type to engine shape flags
    uint8_t flags = 0;
    auto type = data.type.resolve(obj.propOverrides);
    
    switch (type) {
      case TYPE_SPHERE:
        flags = P64::Coll::BCSFlags::SHAPE_SPHERE;
        break;
      case TYPE_BOX:
        flags = P64::Coll::BCSFlags::SHAPE_BOX;
        break;
      case TYPE_CYLINDER:
        flags = P64::Coll::BCSFlags::SHAPE_CYLINDER;
        break;
      case TYPE_CAPSULE:
        flags = P64::Coll::BCSFlags::SHAPE_CAPSULE;
        break;
      default:
        flags = P64::Coll::BCSFlags::SHAPE_SPHERE;
        break;
    }
    
    if(data.isTrigger.resolve(obj.propOverrides)) {
      flags |= P64::Coll::BCSFlags::TRIGGER;
    }

    ctx.fileObj.write<uint8_t>(flags);
    ctx.fileObj.write(data.friction.resolve(obj.propOverrides));
    ctx.fileObj.write(data.bounce.resolve(obj.propOverrides));
    ctx.fileObj.write<uint8_t>(data.maskRead.resolve(obj.propOverrides));
    ctx.fileObj.write<uint8_t>(data.maskWrite.resolve(obj.propOverrides));
  }

  void draw(Object &obj, Entry &entry)
  {
    Data &data = *static_cast<Data*>(entry.data.get());

    if (ImTable::start("Comp", &obj)) {
      ImTable::add("Name", entry.name);

      auto &ext = data.halfExtend.resolve(obj.propOverrides);
      auto type = data.type.resolve(obj.propOverrides);

      ImTable::addComboBox("Type", data.type.value, {"Sphere", "Box", "Cylinder", "Capsule"});
      
      // Different parameters based on shape type
      if(type == TYPE_SPHERE) {
        ImTable::add("Radius", ext.y);
        ext.x = ext.y;
        ext.z = ext.y;
      } else if(type == TYPE_BOX) {
        ImTable::addObjProp("Half Size", data.halfExtend);
      } else if(type == TYPE_CYLINDER) {
        ImTable::add("Radius", ext.x);
        ImTable::add("Half Height", ext.y);
        ext.z = ext.x;  // Keep z same as x for consistency
      } else if(type == TYPE_CAPSULE) {
        ImTable::add("Radius", ext.x);
        ImTable::add("Inner Half Height", ext.y);
        ext.z = ext.x;  // Keep z same as x for consistency
      }
      
      ImTable::addObjProp("Offset", data.offset);
      ImTable::addSeparator();
      ImTable::addObjProp("Friction", data.friction);
      ImTable::addObjProp("Bounce", data.bounce);
      ImTable::addSeparator();
      ImTable::addObjProp("Is Trigger", data.isTrigger);
      ImTable::addBitMask8("Mask Read", data.maskRead.resolve(obj.propOverrides));
      ImTable::addBitMask8("Mask Write", data.maskWrite.resolve(obj.propOverrides));

      ImTable::end();
    }
  }

  void draw3D(Object& obj, Entry &entry, Editor::Viewport3D &vp, SDL_GPUCommandBuffer* cmdBuff, SDL_GPURenderPass* pass)
  {
    Data &data = *static_cast<Data*>(entry.data.get());
    auto &objPos = obj.pos.resolve(obj.propOverrides);
    auto &objScale = obj.scale.resolve(obj.propOverrides);

    glm::vec3 halfExt = data.halfExtend.resolve(obj.propOverrides) * objScale;
    glm::vec3 center = objPos + data.offset.resolve(obj.propOverrides);
    auto type = data.type.resolve(obj.propOverrides);

    glm::vec4 shapeColor{0.0f, 1.0f, 1.0f, 1.0f};  // Cyan color for physics shapes

    if(type == TYPE_BOX) {
      Utils::Mesh::addLineBox(*vp.getLines(), center, halfExt, shapeColor);
      Utils::Mesh::addLineBox(*vp.getLines(), center, halfExt + 0.002f, shapeColor);
    } else if(type == TYPE_SPHERE) {
      Utils::Mesh::addLineSphere(*vp.getLines(), center, halfExt, shapeColor);
    } else if(type == TYPE_CYLINDER) {
      // Draw cylinder as circles at top and bottom with connecting lines
      float radius = halfExt.x;
      float halfHeight = halfExt.y;
      
      // Top and bottom circles
      glm::vec3 topCenter = center + glm::vec3(0, halfHeight, 0);
      glm::vec3 bottomCenter = center - glm::vec3(0, halfHeight, 0);
      
      // Draw circles
      constexpr int segments = 16;
      for(int i = 0; i < segments; i++) {
        float angle1 = (float)i / segments * 2.0f * 3.14159f;
        float angle2 = (float)(i + 1) / segments * 2.0f * 3.14159f;
        
        glm::vec3 p1Top = topCenter + glm::vec3(cosf(angle1) * radius, 0, sinf(angle1) * radius);
        glm::vec3 p2Top = topCenter + glm::vec3(cosf(angle2) * radius, 0, sinf(angle2) * radius);
        glm::vec3 p1Bottom = bottomCenter + glm::vec3(cosf(angle1) * radius, 0, sinf(angle1) * radius);
        glm::vec3 p2Bottom = bottomCenter + glm::vec3(cosf(angle2) * radius, 0, sinf(angle2) * radius);
        
        Utils::Mesh::addLine(*vp.getLines(), p1Top, p2Top, shapeColor);
        Utils::Mesh::addLine(*vp.getLines(), p1Bottom, p2Bottom, shapeColor);
      }
      
      // Vertical lines connecting top and bottom
      for(int i = 0; i < 4; i++) {
        float angle = (float)i / 4.0f * 2.0f * 3.14159f;
        glm::vec3 pTop = topCenter + glm::vec3(cosf(angle) * radius, 0, sinf(angle) * radius);
        glm::vec3 pBottom = bottomCenter + glm::vec3(cosf(angle) * radius, 0, sinf(angle) * radius);
        Utils::Mesh::addLine(*vp.getLines(), pTop, pBottom, shapeColor);
      }
    } else if(type == TYPE_CAPSULE) {
      // Draw capsule as cylinder with hemispheres at top and bottom
      float radius = halfExt.x;
      float innerHalfHeight = halfExt.y;
      
      glm::vec3 topCenter = center + glm::vec3(0, innerHalfHeight, 0);
      glm::vec3 bottomCenter = center - glm::vec3(0, innerHalfHeight, 0);
      
      // Draw middle cylinder part
      constexpr int segments = 16;
      for(int i = 0; i < segments; i++) {
        float angle1 = (float)i / segments * 2.0f * 3.14159f;
        float angle2 = (float)(i + 1) / segments * 2.0f * 3.14159f;
        
        glm::vec3 p1Top = topCenter + glm::vec3(cosf(angle1) * radius, 0, sinf(angle1) * radius);
        glm::vec3 p2Top = topCenter + glm::vec3(cosf(angle2) * radius, 0, sinf(angle2) * radius);
        glm::vec3 p1Bottom = bottomCenter + glm::vec3(cosf(angle1) * radius, 0, sinf(angle1) * radius);
        glm::vec3 p2Bottom = bottomCenter + glm::vec3(cosf(angle2) * radius, 0, sinf(angle2) * radius);
        
        Utils::Mesh::addLine(*vp.getLines(), p1Top, p2Top, shapeColor);
        Utils::Mesh::addLine(*vp.getLines(), p1Bottom, p2Bottom, shapeColor);
      }
      
      // Vertical lines
      for(int i = 0; i < 4; i++) {
        float angle = (float)i / 4.0f * 2.0f * 3.14159f;
        glm::vec3 pTop = topCenter + glm::vec3(cosf(angle) * radius, 0, sinf(angle) * radius);
        glm::vec3 pBottom = bottomCenter + glm::vec3(cosf(angle) * radius, 0, sinf(angle) * radius);
        Utils::Mesh::addLine(*vp.getLines(), pTop, pBottom, shapeColor);
      }
      
      // Draw hemisphere arcs at top and bottom
      constexpr int arcSegments = 8;
      for(int i = 0; i < 4; i++) {
        float angle = (float)i / 4.0f * 2.0f * 3.14159f;
        
        for(int j = 0; j < arcSegments; j++) {
          float arcAngle1 = (float)j / arcSegments * 3.14159f / 2.0f;
          float arcAngle2 = (float)(j + 1) / arcSegments * 3.14159f / 2.0f;
          
          // Top hemisphere
          glm::vec3 p1Top = topCenter + glm::vec3(
            cosf(angle) * radius * cosf(arcAngle1),
            radius * sinf(arcAngle1),
            sinf(angle) * radius * cosf(arcAngle1)
          );
          glm::vec3 p2Top = topCenter + glm::vec3(
            cosf(angle) * radius * cosf(arcAngle2),
            radius * sinf(arcAngle2),
            sinf(angle) * radius * cosf(arcAngle2)
          );
          Utils::Mesh::addLine(*vp.getLines(), p1Top, p2Top, shapeColor);
          
          // Bottom hemisphere
          glm::vec3 p1Bottom = bottomCenter - glm::vec3(
            cosf(angle) * radius * cosf(arcAngle1),
            radius * sinf(arcAngle1),
            sinf(angle) * radius * cosf(arcAngle1)
          );
          glm::vec3 p2Bottom = bottomCenter - glm::vec3(
            cosf(angle) * radius * cosf(arcAngle2),
            radius * sinf(arcAngle2),
            sinf(angle) * radius * cosf(arcAngle2)
          );
          Utils::Mesh::addLine(*vp.getLines(), p1Bottom, p2Bottom, shapeColor);
        }
      }
    }
  }
}
