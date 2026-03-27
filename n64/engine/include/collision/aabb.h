/**
 * @file aabb.h
 * @author Kevin Reier <https://github.com/Byterset>
 * @brief Defines the Axis Aligned Bounding Box and helper functions for it
 */
#pragma once

#include "vec_math.h"
#include <cmath>

namespace P64::Coll {

  /// @brief Struct for the Axis Aligned Bounding Box consisting of its min and max point 
  struct AABB {
    fm_vec3_t min{};
    fm_vec3_t max{};
  };

  // ── AABB utility functions ────────────────────────────────────────

  /// @brief Determines if two AABBs overlap
  /// @param a 
  /// @param b 
  /// @return true if they do, false if not
  inline bool aabbOverlap(const AABB &a, const AABB &b)
  {
      return (a.max.x >= b.min.x) && (a.min.x <= b.max.x) && (a.max.y >= b.min.y) && (a.min.y <= b.max.y) && (a.max.z >= b.min.z) && (a.min.z <= b.max.z);
  }

  /// @brief Determines if an AABB fully contains another
  /// @param outer 
  /// @param inner 
  /// @return true if it does, fals if not
  inline bool aabbContains(const AABB &outer, const AABB &inner)
  {
      return (outer.min.x <= inner.min.x) && (outer.max.x >= inner.max.x) && (outer.min.y <= inner.min.y) && (outer.max.y >= inner.max.y) && (outer.min.z <= inner.min.z) && (outer.max.z >= inner.max.z);
  }

  /// @brief Determines if an AABB contains a Point in 3D Space
  /// @param box 
  /// @param p 
  /// @return 
  inline bool aabbContainsPoint(const AABB &box, const fm_vec3_t &p)
  {
      return (p.x >= box.min.x) && (p.x <= box.max.x) && (p.y >= box.min.y) && (p.y <= box.max.y) && (p.z >= box.min.z) && (p.z <= box.max.z);
  }

  /// @brief Returns the union of two AABBs that contains them both
  /// @param a 
  /// @param b 
  /// @return 
  inline AABB aabbUnion(const AABB &a, const AABB &b)
  {
      return {vec3Min(a.min, b.min), vec3Max(a.max, b.max)};
  }

  /// @brief Calculates the Area of an AABB 
  ///
  /// Used for efficient Leaf insertion in AABB Tree
  /// @param box 
  /// @return 
  inline float aabbArea(const AABB &box)
  {
      float dx = box.max.x - box.min.x;
      float dy = box.max.y - box.min.y;
      float dz = box.max.z - box.min.z;
      return 2.0f * (dx * dy + dy * dz + dz * dx);
  }


  /// @brief Extends an AABB in a given direction by the magnitude of the input direction
  /// @param in 
  /// @param dir 
  /// @param out 
  inline void aabbExtendDirection(const AABB &in, const fm_vec3_t &dir, AABB &out)
  {
      out = in;
      if (dir.x > 0.0f)
          out.max.x += dir.x;
      else
          out.min.x += dir.x;
      if (dir.y > 0.0f)
          out.max.y += dir.y;
      else
          out.min.y += dir.y;
      if (dir.z > 0.0f)
          out.max.z += dir.z;
      else
          out.min.z += dir.z;
  }

  /// @brief Determines if a Ray intersects an AABB
  /// @param box 
  /// @param origin 
  /// @param invDir 
  /// @param maxDist 
  /// @return 
  inline bool aabbIntersectsRay(const AABB &box, const fm_vec3_t &origin,
                                const fm_vec3_t &invDir, float maxDist)
  {
      float t1x = (box.min.x - origin.x) * invDir.x;
      float t2x = (box.max.x - origin.x) * invDir.x;
      float t1y = (box.min.y - origin.y) * invDir.y;
      float t2y = (box.max.y - origin.y) * invDir.y;
      float t1z = (box.min.z - origin.z) * invDir.z;
      float t2z = (box.max.z - origin.z) * invDir.z;

      float tmin = fminf(t1x, t2x);
      float tmax = fmaxf(t1x, t2x);
      tmin = fmaxf(tmin, fminf(t1y, t2y));
      tmax = fminf(tmax, fmaxf(t1y, t2y));
      tmin = fmaxf(tmin, fminf(t1z, t2z));
      tmax = fminf(tmax, fmaxf(t1z, t2z));

      return tmax >= fmaxf(0.0f, tmin) && tmin < maxDist;
  }
}