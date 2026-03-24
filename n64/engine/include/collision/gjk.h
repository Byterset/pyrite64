/**
 * @copyright 2024 - Max Bebök
 * @license MIT
 */
#pragma once

#include "vec_math.h"

namespace P64::Coll {

  /// GJK support function: returns the furthest point on a convex shape in a given direction
  using GjkSupportFunction = void (*)(const void *data, const fm_vec3_t &direction, fm_vec3_t &output);

  constexpr int GJK_MAX_SIMPLEX_SIZE = 4;

  /// Simplex (tetrahedron) used during GJK and as input to EPA
  struct Simplex {
    fm_vec3_t points[GJK_MAX_SIMPLEX_SIZE]{};
    fm_vec3_t rigidBodyAPoint[GJK_MAX_SIMPLEX_SIZE]{};
    short nPoints{0};
  };

  /// Adds a new support point (Minkowski difference) to the simplex
  /// @return pointer to the new point in the simplex, or nullptr if full
  fm_vec3_t *simplexAddPoint(Simplex &simplex, const fm_vec3_t &aPoint, const fm_vec3_t &bPoint);

  /// Checks whether the simplex encloses the origin and updates the search direction
  bool simplexCheck(Simplex &simplex, fm_vec3_t &nextDirection);

  /// Performs GJK overlap test between two convex rigidBodys
  bool gjkCheckForOverlap(
    Simplex &simplex,
    const void *rigidBodyA, GjkSupportFunction rigidBodyASupport,
    const void *rigidBodyB, GjkSupportFunction rigidBodyBSupport,
    const fm_vec3_t &firstDirection
  );

} // namespace P64::Coll
