/**
 * @copyright 2024 - Max Bebök
 * @license MIT
 */
#pragma once

#include "gjk.h"
#include <t3d/t3dmath.h>

namespace P64::CollNew {

  /// Axis-Aligned Bounding Box using floating-point vectors
  struct AABB {
    fm_vec3_t min{};
    fm_vec3_t max{};
  };

  /// Function that calculates the bounding box for a collider shape
  using BoundingBoxCalculator = void (*)(const void *data, const fm_quat_t *rotation, AABB &box);

  /// Function that calculates the diagonal of the local inertia tensor
  using InertiaCalculator = void (*)(const void *data, float mass, fm_vec3_t &out);

  /// Collision shape type enumeration
  enum class ShapeType : uint8_t {
    Sphere,
    Capsule,
    Box,
    Cone,
    Cylinder,
    Sweep,
    Pyramid
  };

  /// Shape-specific dimensional data
  union ShapeData {
    struct { float radius; } sphere;
    struct { float radius; float innerHalfHeight; } capsule;
    struct { fm_vec3_t halfSize; } box;
    struct { float radius; float halfHeight; } cone;
    struct { float radius; float halfHeight; } cylinder;
    struct { float rangeX; float rangeY; float radius; float halfHeight; } sweep;
    struct { float baseHalfWidthX; float baseHalfWidthZ; float halfHeight; } pyramid;
  };

  /// Complete collider definition: shape data plus function pointers
  struct ColliderData {
    GjkSupportFunction gjkSupport{nullptr};
    BoundingBoxCalculator boundingBoxCalc{nullptr};
    InertiaCalculator inertiaCalc{nullptr};
    ShapeData shapeData{};
    ShapeType shapeType{ShapeType::Sphere};
    float bounce{0.0f};
    float friction{0.5f};
  };

} // namespace P64::CollNew
