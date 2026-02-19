/**
* @copyright 2025 - Max Bebök
* @license MIT
*/
#include "scene/object.h"
#include "scene/components/collBody.h"

#include "scene/scene.h"
#include "scene/sceneManager.h"
#include "collision/flags.h"
#include "lib/logger.h"

namespace
{
  struct InitData
  {
    fm_vec3_t halfExtend{};
    fm_vec3_t offset{};
    uint8_t flags{};
    float friction{};
    float bounce{};
    uint8_t maskRead{};
    uint8_t maskWrite{};
  };
}

namespace P64::Comp
{
  void CollBody::initDelete([[maybe_unused]] Object& obj, CollBody* data, void* initData_)
  {
    InitData* initData = static_cast<InitData*>(initData_);

    if (initData == nullptr) {
      // Clean up
      if (data->physicsBody) {
        SceneManager::getCurrent().getPhysics().unregisterBody(data->physicsBody);
        delete data->physicsBody;
        data->physicsBody = nullptr;
      }
      data->~CollBody();
      return;
    }

    new(data) CollBody();

    // Initialize physics body with default mass (will be overridden by Rigidbody component if present)
    data->physicsBody = new Physics::PhysicsBody();
    data->physicsBody->init(&obj, 1.0f);
    data->physicsBody->maskRead = initData->maskRead;
    data->physicsBody->maskWrite = initData->maskWrite;
    data->physicsBody->isTrigger = (initData->flags & Coll::BCSFlags::TRIGGER);
    data->physicsBody->friction = initData->friction;
    data->physicsBody->restitution = initData->bounce;
    
    // Add default shape based on flags
    Physics::ColliderShape shape;
    shape.localOffset = initData->offset;
    
    // Extract shape type from flags (bits 0-1)
    uint8_t shapeType = initData->flags & Coll::BCSFlags::SHAPE_MASK;
    
    switch (shapeType) {
      case Coll::BCSFlags::SHAPE_BOX:
        shape.type = Physics::ShapeType::BOX;
        shape.data.box.halfSize = initData->halfExtend;
        break;
        
      case Coll::BCSFlags::SHAPE_CYLINDER:
        shape.type = Physics::ShapeType::CYLINDER;
        shape.data.cylinder.radius = initData->halfExtend.x;
        shape.data.cylinder.halfHeight = initData->halfExtend.y;
        break;
        
      case Coll::BCSFlags::SHAPE_CAPSULE:
        shape.type = Physics::ShapeType::CAPSULE;
        shape.data.capsule.radius = initData->halfExtend.x;
        shape.data.capsule.innerHalfHeight = initData->halfExtend.y;
        break;
        
      case Coll::BCSFlags::SHAPE_SPHERE:
      default:
        shape.type = Physics::ShapeType::SPHERE;
        shape.data.sphere.radius = initData->halfExtend.y;
        break;
    }
    
    data->physicsBody->addShape(shape);
    
    // Register with PhysicsScene
    SceneManager::getCurrent().getPhysics().registerBody(data->physicsBody);
  }

  void CollBody::onEvent(Object &obj, CollBody* data, const ObjectEvent &event)
  {
    if(event.type == EVENT_TYPE_DISABLE) {
      if (data->physicsBody) {
        obj.getScene().getPhysics().unregisterBody(data->physicsBody);
      }
    }
    if(event.type == EVENT_TYPE_ENABLE) {
      if (data->physicsBody) {
        obj.getScene().getPhysics().registerBody(data->physicsBody);
      }
    }
  }

  void CollBody::update(Object &obj, CollBody* data, float deltaTime)
  {
    // Physics integration happens in PhysicsScene::step
    // Component update can handle per-frame logic if needed
  }
  
  // Helper methods for shape management
  void CollBody::addSphere(float radius, const fm_vec3_t& offset) {
    if (!physicsBody) return;
    
    Physics::ColliderShape shape;
    shape.type = Physics::ShapeType::SPHERE;
    shape.data.sphere.radius = radius;
    shape.localOffset = offset;
    
    physicsBody->addShape(shape);
  }
  
  void CollBody::addBox(const fm_vec3_t& halfSize, const fm_vec3_t& offset) {
    if (!physicsBody) return;
    
    Physics::ColliderShape shape;
    shape.type = Physics::ShapeType::BOX;
    shape.data.box.halfSize = halfSize;
    shape.localOffset = offset;
    
    physicsBody->addShape(shape);
  }
  
  void CollBody::addCylinder(float radius, float halfHeight, const fm_vec3_t& offset) {
    if (!physicsBody) return;
    
    Physics::ColliderShape shape;
    shape.type = Physics::ShapeType::CYLINDER;
    shape.data.cylinder.radius = radius;
    shape.data.cylinder.halfHeight = halfHeight;
    shape.localOffset = offset;
    
    physicsBody->addShape(shape);
  }
  
  void CollBody::addCapsule(float radius, float innerHalfHeight, const fm_vec3_t& offset) {
    if (!physicsBody) return;
    
    Physics::ColliderShape shape;
    shape.type = Physics::ShapeType::CAPSULE;
    shape.data.capsule.radius = radius;
    shape.data.capsule.innerHalfHeight = innerHalfHeight;
    shape.localOffset = offset;
    
    physicsBody->addShape(shape);
  }
  
  void CollBody::clearShapes() {
    if (physicsBody) {
      physicsBody->clearShapes();
    }
  }
  
  void CollBody::setFriction(float friction) {
    if (physicsBody) {
      physicsBody->friction = friction;
    }
  }
  
  void CollBody::setBounce(float bounce) {
    if (physicsBody) {
      physicsBody->restitution = bounce;
    }
  }
}
