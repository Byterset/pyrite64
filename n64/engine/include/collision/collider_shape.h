/**
 * @file collider_shape.h
 * @author Kevin Reier <https://github.com/Byterset>
 * @brief Defines the Basic (non-mesh) Colliders 
 */
#pragma once

#include "gjk.h"
#include "types.h"
#include "shapes.h"
#include "aabb_tree.h"

namespace P64
{
  class Object;
}

namespace P64::Coll {

  struct MeshCollider;

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
    bool isTrigger{false}; // whether this collider is a trigger (generates contacts for events, but no physical response)
    uint8_t maskRead{0x00};  // which collision layers this collider get's affected by
    uint8_t maskWrite{0x00}; // which collision layers this collider affects
    NodeProxy aabbTreeNodeId{NULL_NODE}; // Node ID in the scene's dynamic AABB tree

    fm_vec3_t support(const fm_vec3_t &dir) const;
    AABB boundingBox(const fm_quat_t *rotation) const;
    fm_vec3_t inertiaTensor(float mass) const;
    fm_vec3_t toWorldSpace(const fm_vec3_t &localPoint) const;
    fm_vec3_t toLocalSpace(const fm_vec3_t &worldPoint) const;
    fm_vec3_t rotateToWorld(const fm_vec3_t &localDir) const;
    fm_vec3_t rotateToLocal(const fm_vec3_t &worldDir) const;
    bool readsCollider(const Collider *other) const;
    bool readsMeshCollider(const MeshCollider *other) const;
  };

  /// GJK-compatible support wrapper
  void colliderGjkSupport(const void *data, const fm_vec3_t &direction, fm_vec3_t &output);

} // namespace P64::Coll
