/**
 * @copyright 2024 - Max Bebök
 * @license MIT
 */
#pragma once

#include "shapes.h"
#include "gjk.h"
#include "types.h"

namespace P64
{
  class Object;
}

namespace P64::CollNew {

  struct Collider {
    ShapeType type{ShapeType::Sphere};
    union {
      SphereShape sphere;
      BoxShape box;
      CapsuleShape capsule;
      CylinderShape cylinder;
      ConeShape cone;
      PyramidShape pyramid;
    };
    fm_vec3_t worldCenter{};
    fm_vec3_t parentOffset{}; // offset to apply when updating the object it's attached to
    float bounce{0.0f};
    float friction{0.8f};
    P64::Object *owner{};
    AABB worldAABB{}; // used for culling
    bool isTrigger{false}; // whether this collider is a trigger (doesn't generate contacts, only events)
    uint8_t maskRead{0x00};  // which collision layers this collider get's affected by
    uint8_t maskWrite{0x00}; // which collision layers this collider affects

    fm_vec3_t support(const fm_vec3_t &dir) const;
    AABB boundingBox(const fm_quat_t *rotation) const;
    fm_vec3_t inertiaTensor(float mass) const;
  };

  /// GJK-compatible support wrapper
  void colliderGjkSupport(const void *data, const fm_vec3_t &direction, fm_vec3_t &output);

} // namespace P64::CollNew
