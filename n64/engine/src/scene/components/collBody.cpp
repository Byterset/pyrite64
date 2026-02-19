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

    // Check for duplicate Rigidbody on this object
    auto compRefs = obj.getCompRefs();
    int rigidbodyCount = 0;
    for (uint32_t i = 0; i < obj.compCount; ++i) {
      if (compRefs[i].type == CollBody::ID) {
        rigidbodyCount++;
      }
    }
    
    if (rigidbodyCount > 1) {
      Log::error("Object %d already has a Rigidbody component! Only one Rigidbody per object is allowed.", obj.id);
      // Still initialize but log error
    }

    new(data) CollBody();

    // Initialize physics body
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
  
  void CollBody::setMass(float mass) {
    if (!physicsBody) return;
    physicsBody->mass = mass;
    physicsBody->invMass = (mass > 0.0f) ? (1.0f / mass) : 0.0f;
  }
  
  void CollBody::setFriction(float friction) {
    if (physicsBody) {
      physicsBody->friction = friction;
    }
  }
  
  void CollBody::setBounce(float bounce) {
    if (physicsBody) {
      physicsBody->bounce = bounce;
    }
  }
  
  void CollBody::setKinematic(bool isKinematic) {
    if (physicsBody) {
      physicsBody->isKinematic = isKinematic;
    }
  }
}
