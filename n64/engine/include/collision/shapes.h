/**
* @copyright 2023 - Max Bebök
* @license MIT
*/
#pragma once
#include <functional>
#include <t3d/t3dmath.h>
#include "lib/math.h"
#include "flags.h"

namespace P64
{
  class Object;
}

namespace P64::CollNew { struct MeshCollider; }

namespace P64::Coll
{
  struct MeshInstance;
  struct BCS;

  struct IVec3 {
    int16_t v[3]{};
  };

  struct AABB
  {
    IVec3 min{};
    IVec3 max{};

    bool vsAABB(const AABB &other) const;
    bool vsRay(const fm_vec3_t &pos, const fm_vec3_t &dir) const;
    bool vsPoint(const IVec3 &pos) const;
  };
  static_assert(sizeof(AABB) == (6 * sizeof(int16_t)));

  struct BCS
  {
    fm_vec3_t center{}; // center of shape
    fm_vec3_t halfExtend{}; // half extend from center, spheres only use X
    fm_vec3_t velocity{}; // current velocity (will be mul. by delta-time internally)
    fm_vec3_t parentOffset{}; // offset to apply when updating the object it's attached to

    P64::Object *obj{};

    uint8_t maskRead{0};
    uint8_t maskWrite{0};
    uint8_t flags{0};
    uint8_t hitTriTypes{0}; // mask of triangle types the sphere last collided with

    [[nodiscard]] constexpr float getRadius() const {
      return halfExtend.y;
    }
    [[nodiscard]] constexpr float getRadius2() const {
      return halfExtend.y * halfExtend.y;
    }

    [[nodiscard]] constexpr bool isTrigger() const {
      return (flags & BCSFlags::TRIGGER);
    }

    [[nodiscard]] constexpr bool isSolid() const {
      return !isTrigger();
    }

    [[nodiscard]] constexpr bool isFixed() const {
      return flags & BCSFlags::FIXED_XYZ;
    }

    [[nodiscard]] fm_vec3_t getMinAABB() const {
      return center - halfExtend;
    }
    [[nodiscard]] fm_vec3_t getMaxAABB() const {
      return center + halfExtend;
    }

    BCS operator*(float scale) const {
      return {
        .center = center * scale,
        .halfExtend = halfExtend * scale
      };
    }

    AABB toAABB() const {
      auto min = center - halfExtend;
      auto max = center + halfExtend;
      return {
        .min = {{ (int16_t)min.v[0], (int16_t)min.v[1], (int16_t)min.v[2] }},
        .max = {{ (int16_t)max.v[0], (int16_t)max.v[1], (int16_t)max.v[2] }}
      };
    }
  };

  struct CollInfo
  {
    fm_vec3_t penetration{};
    fm_vec3_t floorWallAngle{};
    MeshInstance* meshInstance{};
    int collCount{};
  };

  struct RaycastRes {
    fm_vec3_t hitPos{};
    fm_vec3_t normal{};
    uint32_t flags{};

    [[nodiscard]] bool hasResult() const {
      return flags != 0;
    }
  };

  struct Triangle
  {
    fm_vec3_t normal{};
    fm_vec3_t* v[3]{};
    AABB aabb{};
  };

  struct Triangle2D {
    fm_vec2_t v[3]{};
  };

  struct CollEvent
  {
    BCS* selfBCS{};
    BCS* otherBCS{};

    CollNew::MeshCollider* selfMesh{};
    CollNew::MeshCollider* otherMesh{};
  };
}
