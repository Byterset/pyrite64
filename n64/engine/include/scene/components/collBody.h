/**
* @copyright 2025 - Max Bebök
* @license MIT
*/
#pragma once
#include "assets/assetManager.h"
#include "scene/object.h"
#include "assets/assetManager.h"
#include <t3d/t3dmodel.h>

#include "collision/mesh.h"
#include "physics/physicsBody.h"

namespace P64::Comp
{
  struct CollBody
  {
    static constexpr uint32_t ID = 5;

    // Legacy BCS for backward compatibility with existing collision system
    Coll::BCS bcs{};
    fm_vec3_t orgScale{};
    
    // New physics body for iterative constraint solver
    Physics::PhysicsBody* physicsBody{nullptr};
    bool useNewPhysics{false};  // Flag to enable new physics system

    static uint32_t getAllocSize([[maybe_unused]] uint16_t* initData)
    {
      return sizeof(CollBody);
    }

    static void initDelete([[maybe_unused]] Object& obj, CollBody* data, void* initData);

    static void onEvent(Object& obj, CollBody* data, const ObjectEvent& event);

    static void update(Object& obj, CollBody* data, float deltaTime);
  };
}
