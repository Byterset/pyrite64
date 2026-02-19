/**
 * @copyright 2026 - Max Bebök
 * @license MIT
 */
#include "scene/object.h"
#include "scene/components/rigidbody.h"
#include "scene/components/collBody.h"
#include "scene/scene.h"
#include "scene/sceneManager.h"
#include "lib/logger.h"

namespace
{
  struct InitData
  {
    float mass{};
    uint8_t gravityFlags{};
    float gravityScale{};
    uint8_t isKinematic{};
    float angularDamping{};
    uint16_t constraints{};
  };
}

namespace P64::Comp
{
  void Rigidbody::initDelete([[maybe_unused]] Object& obj, Rigidbody* data, void* initData_)
  {
    InitData* initData = static_cast<InitData*>(initData_);

    if (initData == nullptr) {
      // Clean up
      data->~Rigidbody();
      return;
    }

    // Check for duplicate Rigidbody on this object
    auto compRefs = obj.getCompRefs();
    int rigidbodyCount = 0;
    for (uint32_t i = 0; i < obj.compCount; ++i) {
      if (compRefs[i].type == Rigidbody::ID) {
        rigidbodyCount++;
      }
    }
    
    if (rigidbodyCount > 1) {
      Log::error("Object %d already has a Rigidbody component! Only one Rigidbody per object is allowed.", obj.id);
    }

    // Initialize rigidbody
    new(data) Rigidbody();
    
    data->mass = initData->mass;
    data->gravityFlags = initData->gravityFlags;
    data->gravityScale = initData->gravityScale;
    data->isKinematic = initData->isKinematic != 0;
    data->angularDamping = initData->angularDamping;
    data->constraints = initData->constraints;
    
    // Find the corresponding Collider component
    CollBody* collider = obj.getComponent<CollBody>();
    if (collider && collider->physicsBody) {
      // Apply rigidbody properties to physics body
      collider->physicsBody->setMass(data->mass);
      collider->physicsBody->setKinematic(data->isKinematic);
      collider->physicsBody->hasGravity = (data->gravityFlags & 0x01) != 0;
      collider->physicsBody->gravityScale = data->gravityScale;
      collider->physicsBody->angularDamping = data->angularDamping;
      collider->physicsBody->constraints = data->constraints;
    }
  }

  void Rigidbody::onEvent(Object &obj, Rigidbody* data, const ObjectEvent &event)
  {
    // Currently no event handling needed
    (void)obj;
    (void)data;
    (void)event;
  }
}
