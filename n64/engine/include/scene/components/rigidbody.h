/**
 * @copyright 2026 - Max Bebök
 * @license MIT
 */
#pragma once
#include <cstdint>

namespace P64
{
  class Object;
  struct ObjectEvent;
}

namespace P64::Comp
{
  /**
   * Rigidbody component - enables physics simulation for an object
   * 
   * Defines mass, gravity, kinematic state, and movement/rotation constraints.
   * Must be used together with a Collider component for collision shapes.
   * 
   * Matches physics_object structure from libdragon_tiny3d_test (excluding collision data).
   */
  struct Rigidbody
  {
    static constexpr uint32_t ID = 11;
    
    float mass{1.0f};
    float gravityScale{1.0f};
    float angularDamping{0.05f};
    
    uint16_t constraints{0}; // Freeze position/rotation flags
    uint8_t gravityFlags{0}; // Use gravity flag
    bool isKinematic{false};
    
    static void initDelete(Object& obj, Rigidbody* data, void* initData);
    static void onEvent(Object &obj, Rigidbody* data, const ObjectEvent &event);
  };
  
  // Constraint flags matching libdragon_tiny3d_test
  namespace RigidbodyConstraints {
    constexpr uint16_t NONE = 0;
    constexpr uint16_t FREEZE_POSITION_X = (1 << 0);
    constexpr uint16_t FREEZE_POSITION_Y = (1 << 1);
    constexpr uint16_t FREEZE_POSITION_Z = (1 << 2);
    constexpr uint16_t FREEZE_POSITION_ALL = (FREEZE_POSITION_X | FREEZE_POSITION_Y | FREEZE_POSITION_Z);
    constexpr uint16_t FREEZE_ROTATION_X = (1 << 3);
    constexpr uint16_t FREEZE_ROTATION_Y = (1 << 4);
    constexpr uint16_t FREEZE_ROTATION_Z = (1 << 5);
    constexpr uint16_t FREEZE_ROTATION_ALL = (FREEZE_ROTATION_X | FREEZE_ROTATION_Y | FREEZE_ROTATION_Z);
    constexpr uint16_t ALL = 0xFF;
  }
}
