/**
 * @copyright 2024 - Max Bebök
 * @license MIT
 */
#pragma once

#include "shapes.h"
#include "gjk.h"

namespace P64::CollNew {

  enum class ShapeType : uint8_t {
    Sphere,
    Capsule,
    Box,
    Cone,
    Cylinder,
    Sweep,
    Pyramid
  };

  struct Collider {
    ShapeType type{ShapeType::Sphere};
    union {
      SphereShape sphere;
      BoxShape box;
      CapsuleShape capsule;
      CylinderShape cylinder;
      ConeShape cone;
      PyramidShape pyramid;
      SweepShape sweep;
    };
    float bounce{0.0f};
    float friction{0.5f};

    fm_vec3_t support(const fm_vec3_t &dir) const;
    AABB boundingBox(const fm_quat_t *rotation) const;
    fm_vec3_t inertiaTensor(float mass) const;
  };

  /// GJK-compatible support wrapper
  void colliderGjkSupport(const void *data, const fm_vec3_t &direction, fm_vec3_t &output);

} // namespace P64::CollNew
