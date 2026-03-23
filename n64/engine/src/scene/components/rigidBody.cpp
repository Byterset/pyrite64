/**
* @copyright 2025 - Max Bebök
* @license MIT
*/
#include "scene/object.h"
#include "scene/components/rigidBody.h"

#include "scene/scene.h"
#include "scene/sceneManager.h"
#include <cmath>

namespace
{
  struct InitData
  {
    float mass{};
    bool isKinematic{};
    bool constrainPosX{};
    bool constrainPosY{};
    bool constrainPosZ{};
    bool constrainRotX{};
    bool constrainRotY{};
    bool constrainRotZ{};
    bool hasGravity{};
    float gravityScalar{};
    float timeScalar{};
    float angularDamping{};
  };
}

namespace P64::Comp
{
  void RigidBody::initDelete([[maybe_unused]] Object& obj, RigidBody* data, void* initData_)
  {
    InitData* initData = static_cast<InitData*>(initData_);
    auto &coll = SceneManager::getCurrent().getCollisionNew();

    if (initData == nullptr) {
      coll.removeRigidBody(&data->rigid_body);
      // TODO: add collider to new collision scene
      data->~RigidBody();
      return;
    }

    new(data) RigidBody();

    data->rigid_body = {};
    //TODO: init
    data->rigid_body.init(&obj, initData->mass);
    data->rigid_body.isKinematic = initData->isKinematic;
    data->rigid_body.hasGravity = initData->hasGravity;
    data->rigid_body.gravityScalar = initData->gravityScalar;
    data->rigid_body.timeScalar = initData->timeScalar;
    data->rigid_body.angularDamping = initData->angularDamping;
    data->rigid_body.constraints = Coll::Constraint::None;
    if(initData->constrainPosX) data->rigid_body.constraints = data->rigid_body.constraints | Coll::Constraint::FreezePosX;
    if(initData->constrainPosY) data->rigid_body.constraints = data->rigid_body.constraints | Coll::Constraint::FreezePosY;
    if(initData->constrainPosZ) data->rigid_body.constraints = data->rigid_body.constraints | Coll::Constraint::FreezePosZ;
    if(initData->constrainRotX) data->rigid_body.constraints = data->rigid_body.constraints | Coll::Constraint::FreezeRotX;
    if(initData->constrainRotY) data->rigid_body.constraints = data->rigid_body.constraints | Coll::Constraint::FreezeRotY;
    if(initData->constrainRotZ) data->rigid_body.constraints = data->rigid_body.constraints | Coll::Constraint::FreezeRotZ;

    if(obj.isEnabled()) {
      coll.addRigidBody(&data->rigid_body);
    }
  }

  void RigidBody::onEvent(Object &obj, RigidBody* data, const ObjectEvent &event)
  {
    if(event.type == EVENT_TYPE_DISABLE) {
      return obj.getScene().getCollisionNew().removeRigidBody(&data->rigid_body);
    }
    if(event.type == EVENT_TYPE_ENABLE) {
      return obj.getScene().getCollisionNew().addRigidBody(&data->rigid_body);
    }
  }

  void RigidBody::update(Object &obj, RigidBody* data, float deltaTime)
  {
  }
}
