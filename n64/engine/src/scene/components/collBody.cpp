/**
* @copyright 2025 - Max Bebök
* @license MIT
*/
#include "scene/object.h"
#include "scene/components/collBody.h"

#include "scene/scene.h"
#include "scene/sceneManager.h"

namespace
{
  struct InitData
  {
    fm_vec3_t halfExtend{};
    fm_vec3_t offset{};
    uint8_t flags{};
    uint8_t maskRead{};
    uint8_t maskWrite{};
  };
}

namespace P64::Comp
{
  void CollBody::initDelete([[maybe_unused]] Object& obj, CollBody* data, void* initData_)
  {
    InitData* initData = static_cast<InitData*>(initData_);
    auto &coll = SceneManager::getCurrent().getCollision();

    if (initData == nullptr) {
      // Clean up
      if (data->useNewPhysics && data->physicsBody) {
        // Unregister from PhysicsScene
        SceneManager::getCurrent().getPhysics().unregisterBody(data->physicsBody);
        delete data->physicsBody;
        data->physicsBody = nullptr;
      } else {
        coll.unregisterBCS(&data->bcs);
      }
      data->~CollBody();
      return;
    }

    new(data) CollBody();

    data->orgScale = initData->halfExtend;
    
    // For now, use legacy system by default for backward compatibility
    data->useNewPhysics = false;

    if (data->useNewPhysics) {
      // Initialize new physics body
      data->physicsBody = new Physics::PhysicsBody();
      data->physicsBody->init(&obj, 1.0f);
      data->physicsBody->maskRead = initData->maskRead;
      data->physicsBody->maskWrite = initData->maskWrite;
      data->physicsBody->isTrigger = (initData->flags & Coll::BCSFlags::TRIGGER);
      
      // Add default shape based on flags
      Physics::ColliderShape shape;
      shape.localOffset = initData->offset;
      
      if (initData->flags & Coll::BCSFlags::SHAPE_BOX) {
        shape.type = Physics::ShapeType::BOX;
        shape.data.box.halfSize = initData->halfExtend;
      } else {
        shape.type = Physics::ShapeType::SPHERE;
        shape.data.sphere.radius = initData->halfExtend.y;
      }
      
      data->physicsBody->addShape(shape);
      
      // Register with PhysicsScene
      SceneManager::getCurrent().getPhysics().registerBody(data->physicsBody);
    } else {
      // Legacy BCS initialization
      data->bcs = {
        .center = obj.pos + initData->offset,
        .halfExtend = data->orgScale * obj.scale,
        .parentOffset = initData->offset,
        .obj = &obj,
        .maskRead = initData->maskRead,
        .maskWrite = initData->maskWrite,
        .flags = initData->flags,
      };
      coll.registerBCS(&data->bcs);
    }
  }

  void CollBody::onEvent(Object &obj, CollBody* data, const ObjectEvent &event)
  {
    if (data->useNewPhysics) {
      // Handle enable/disable for new physics
      if(event.type == EVENT_TYPE_DISABLE) {
        obj.getScene().getPhysics().unregisterBody(data->physicsBody);
      }
      if(event.type == EVENT_TYPE_ENABLE) {
        obj.getScene().getPhysics().registerBody(data->physicsBody);
      }
    } else {
      // Legacy BCS handling
      if(event.type == EVENT_TYPE_DISABLE) {
        return obj.getScene().getCollision().unregisterBCS(&data->bcs);
      }
      if(event.type == EVENT_TYPE_ENABLE) {
        return obj.getScene().getCollision().registerBCS(&data->bcs);
      }
    }
  }

  void CollBody::update(Object &obj, CollBody* data, float deltaTime)
  {
    if (data->useNewPhysics) {
      // Update physics body shapes with scale
      if (data->physicsBody) {
        // Shapes are already scaled via object transform
        // Physics integration happens in PhysicsScene::step
      }
    } else {
      // Legacy BCS update
      data->bcs.halfExtend = data->orgScale * obj.scale;
      if(data->bcs.isTrigger()) {
        data->bcs.center = obj.pos;
      }
    }
  }
}
