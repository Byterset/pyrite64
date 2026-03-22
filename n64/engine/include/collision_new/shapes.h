/**
 * @copyright 2024 - Max Bebök
 * @license MIT
 */
#pragma once

#include "vec_math.h"
#include <cmath>

namespace P64::CollNew {

  struct AABB {
    fm_vec3_t min{};
    fm_vec3_t max{};
  };

  // ── Sphere ──────────────────────────────────────────────────────────

  struct SphereShape {
    float radius;

    fm_vec3_t support(const fm_vec3_t &dir) const {
      fm_vec3_t unit = vec3Normalize(dir);
      if(vec3IsZero(unit)) unit = vec3Up();
      return fm_vec3_t{{unit.x * radius, unit.y * radius, unit.z * radius}};
    }

    AABB boundingBox(const fm_quat_t * /*rotation*/) const {
      return {vec3(-radius, -radius, -radius), vec3(radius, radius, radius)};
    }

    fm_vec3_t inertiaTensor(float mass) const {
      float inertia = 0.4f * mass * radius * radius;
      return fm_vec3_t{{inertia, inertia, inertia}};
    }
  };

  // ── Box ─────────────────────────────────────────────────────────────

  struct BoxShape {
    fm_vec3_t halfSize;

    fm_vec3_t support(const fm_vec3_t &dir) const {
      return fm_vec3_t{{
        copysignf(halfSize.x, dir.x),
        copysignf(halfSize.y, dir.y),
        copysignf(halfSize.z, dir.z)
      }};
    }

    AABB boundingBox(const fm_quat_t *q) const;
    fm_vec3_t inertiaTensor(float mass) const;
  };

  // ── Capsule ─────────────────────────────────────────────────────────

  struct CapsuleShape {
    float radius;
    float innerHalfHeight;

    fm_vec3_t support(const fm_vec3_t &dir) const {
      fm_vec3_t unit = vec3Normalize(dir);
      if(vec3IsZero(unit)) unit = vec3Up();

      float y = copysignf(innerHalfHeight, unit.y);
      return fm_vec3_t{{unit.x * radius, unit.y * radius + y, unit.z * radius}};
    }

    AABB boundingBox(const fm_quat_t *q) const;
    fm_vec3_t inertiaTensor(float mass) const;
  };

  // ── Cylinder ────────────────────────────────────────────────────────

  struct CylinderShape {
    float radius;
    float halfHeight;

    fm_vec3_t support(const fm_vec3_t &dir) const;
    AABB boundingBox(const fm_quat_t *q) const;
    fm_vec3_t inertiaTensor(float mass) const;
  };

  // ── Cone ────────────────────────────────────────────────────────────

  struct ConeShape {
    float radius;
    float halfHeight;

    fm_vec3_t support(const fm_vec3_t &dir) const;
    AABB boundingBox(const fm_quat_t *q) const;
    fm_vec3_t inertiaTensor(float mass) const;
  };

  // ── Pyramid ─────────────────────────────────────────────────────────

  struct PyramidShape {
    float baseHalfWidthX;
    float baseHalfWidthZ;
    float halfHeight;

    fm_vec3_t support(const fm_vec3_t &dir) const;
    AABB boundingBox(const fm_quat_t *q) const;
    fm_vec3_t inertiaTensor(float mass) const;
  };

} // namespace P64::CollNew
