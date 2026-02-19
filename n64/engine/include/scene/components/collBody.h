/**
* @copyright 2025 - Max Bebök
* @license MIT
*/
#pragma once
#include "assets/assetManager.h"
#include "scene/object.h"
#include "assets/assetManager.h"
#include <t3d/t3dmodel.h>

#include "physics/physicsBody.h"

namespace P64::Comp
{
  /**
   * Collider component - defines collision shape and properties
   * 
   * Can exist without Rigidbody for static collision.
   * When used with Rigidbody, enables physics simulation.
   * Matches physics_object_collision_data from libdragon_tiny3d_test.
   */
  struct CollBody
  {
    static constexpr uint32_t ID = 5;

    Physics::PhysicsBody* physicsBody{nullptr};

    static uint32_t getAllocSize([[maybe_unused]] uint16_t* initData)
    {
      return sizeof(CollBody);
    }

    static void initDelete([[maybe_unused]] Object& obj, CollBody* data, void* initData);

    static void onEvent(Object& obj, CollBody* data, const ObjectEvent& event);

    static void update(Object& obj, CollBody* data, float deltaTime);
    
    // Helper methods for shape management
    void addSphere(float radius, const fm_vec3_t& offset = {0,0,0});
    void addBox(const fm_vec3_t& halfSize, const fm_vec3_t& offset = {0,0,0});
    void addCylinder(float radius, float halfHeight, const fm_vec3_t& offset = {0,0,0});
    void addCapsule(float radius, float innerHalfHeight, const fm_vec3_t& offset = {0,0,0});
    void clearShapes();
    
    // Property setters
    void setFriction(float friction);
    void setBounce(float bounce);
  };
}
