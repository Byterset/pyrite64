/**
 * @copyright 2026 - Max Bebök
 * @license MIT
 */
#pragma once
#include <t3d/t3dmath.h>
#include "lib/math.h"

namespace P64::Physics
{
  /**
   * Enum of possible collision shape types
   */
  enum class ShapeType : uint8_t {
    SPHERE,
    BOX,
    CYLINDER,
    CAPSULE,
  };

  /**
   * Shape data union holding parameters for each shape type
   */
  union ShapeData {
    struct {
      float radius;
    } sphere;
    
    struct {
      fm_vec3_t halfSize;  // half extents from center
    } box;
    
    struct {
      float radius;
      float halfHeight;
    } cylinder;
    
    struct {
      float radius;
      float innerHalfHeight;  // half height of inner cylinder (not including caps)
    } capsule;
  };

  /**
   * Defines a collision shape with type, data, and local offset
   */
  struct ColliderShape {
    ShapeType type;
    ShapeData data;
    fm_vec3_t localOffset{};  // offset from body center in local space
    
    // Rotation is inherited from parent transform, not stored here
    
    /**
     * Get axis-aligned bounding box for this shape in local space
     */
    fm_vec3_t getLocalAABBMin() const;
    fm_vec3_t getLocalAABBMax() const;
    
    /**
     * GJK support function - returns furthest point in given direction
     * @param direction Direction vector (in local space of shape)
     * @return Point on shape surface in local space
     */
    fm_vec3_t support(const fm_vec3_t& direction) const;
  };
}
