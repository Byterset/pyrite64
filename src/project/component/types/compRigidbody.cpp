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

namespace Project::Component::Rigidbody
{
  struct Data
  {
    PROP_FLOAT(mass);
    PROP_BOOL(useGravity);
    PROP_FLOAT(gravityScale);
    PROP_BOOL(isKinematic);
    PROP_FLOAT(angularDamping);
    PROP_BOOL(freezePosX);
    PROP_BOOL(freezePosY);
    PROP_BOOL(freezePosZ);
    PROP_BOOL(freezeRotX);
    PROP_BOOL(freezeRotY);
    PROP_BOOL(freezeRotZ);
  };

  std::shared_ptr<void> init(Object &obj) {
    auto data = std::make_shared<Data>();
    data->mass.value = 1.0f;
    data->useGravity.value = true;
    data->gravityScale.value = 1.0f;
    data->isKinematic.value = false;
    data->angularDamping.value = 0.05f;
    data->freezePosX.value = false;
    data->freezePosY.value = false;
    data->freezePosZ.value = false;
    data->freezeRotX.value = false;
    data->freezeRotY.value = false;
    data->freezeRotZ.value = false;
    return data;
  }

  nlohmann::json serialize(const Entry &entry) {
    Data &data = *static_cast<Data*>(entry.data.get());
    return Utils::JSON::Builder{}
      .set(data.mass)
      .set(data.useGravity)
      .set(data.gravityScale)
      .set(data.isKinematic)
      .set(data.angularDamping)
      .set(data.freezePosX)
      .set(data.freezePosY)
      .set(data.freezePosZ)
      .set(data.freezeRotX)
      .set(data.freezeRotY)
      .set(data.freezeRotZ)
      .doc;
  }

  std::shared_ptr<void> deserialize(nlohmann::json &doc) {
    auto data = std::make_shared<Data>();
    Utils::JSON::readProp(doc, data->mass, 1.0f);
    Utils::JSON::readProp(doc, data->useGravity, true);
    Utils::JSON::readProp(doc, data->gravityScale, 1.0f);
    Utils::JSON::readProp(doc, data->isKinematic, false);
    Utils::JSON::readProp(doc, data->angularDamping, 0.05f);
    Utils::JSON::readProp(doc, data->freezePosX, false);
    Utils::JSON::readProp(doc, data->freezePosY, false);
    Utils::JSON::readProp(doc, data->freezePosZ, false);
    Utils::JSON::readProp(doc, data->freezeRotX, false);
    Utils::JSON::readProp(doc, data->freezeRotY, false);
    Utils::JSON::readProp(doc, data->freezeRotZ, false);
    return data;
  }

  void build(Object& obj, Entry &entry, Build::SceneCtx &ctx)
  {
    Data &data = *static_cast<Data*>(entry.data.get());
    
    // Write mass
    ctx.fileObj.write(data.mass.resolve(obj.propOverrides));
    
    // Pack gravity flags into one byte
    uint8_t gravityFlags = 0;
    if(data.useGravity.resolve(obj.propOverrides)) {
      gravityFlags |= (1 << 0);
    }
    ctx.fileObj.write<uint8_t>(gravityFlags);
    ctx.fileObj.write(data.gravityScale.resolve(obj.propOverrides));
    
    // Write kinematic flag
    ctx.fileObj.write<uint8_t>(data.isKinematic.resolve(obj.propOverrides) ? 1 : 0);
    
    // Write angular damping
    ctx.fileObj.write(data.angularDamping.resolve(obj.propOverrides));
    
    // Pack constraint flags into uint16
    uint16_t constraints = 0;
    if(data.freezePosX.resolve(obj.propOverrides)) constraints |= (1 << 0);
    if(data.freezePosY.resolve(obj.propOverrides)) constraints |= (1 << 1);
    if(data.freezePosZ.resolve(obj.propOverrides)) constraints |= (1 << 2);
    if(data.freezeRotX.resolve(obj.propOverrides)) constraints |= (1 << 3);
    if(data.freezeRotY.resolve(obj.propOverrides)) constraints |= (1 << 4);
    if(data.freezeRotZ.resolve(obj.propOverrides)) constraints |= (1 << 5);
    ctx.fileObj.write<uint16_t>(constraints);
  }

  void draw(Object &obj, Entry &entry)
  {
    Data &data = *static_cast<Data*>(entry.data.get());

    if (ImTable::start("Comp", &obj)) {
      ImTable::add("Name", entry.name);

      ImTable::addObjProp("Mass", data.mass);
      
      ImTable::addObjProp("Use Gravity", data.useGravity);
      if(data.useGravity.resolve(obj.propOverrides)) {
        ImTable::addObjProp("Gravity Scale", data.gravityScale);
      }
      
      ImTable::addObjProp("Is Kinematic", data.isKinematic);
      
      ImTable::addObjProp("Angular Damping", data.angularDamping);
      
      ImTable::add("Constraints");
      ImTable::addObjProp("Freeze Position X", data.freezePosX);
      ImTable::addObjProp("Freeze Position Y", data.freezePosY);
      ImTable::addObjProp("Freeze Position Z", data.freezePosZ);
      ImTable::addObjProp("Freeze Rotation X", data.freezeRotX);
      ImTable::addObjProp("Freeze Rotation Y", data.freezeRotY);
      ImTable::addObjProp("Freeze Rotation Z", data.freezeRotZ);

      ImTable::end();
    }
  }

  void draw3D(Object& obj, Entry &entry, Editor::Viewport3D &vp, SDL_GPUCommandBuffer* cmdBuff, SDL_GPURenderPass* pass)
  {
    // No 3D visualization for rigidbody component
  }
}
