/**
 * @copyright 2024 - Max Bebök
 * @license MIT
 */
#pragma once

#include "gjk.h"

namespace P64::Coll {

  /// Result of EPA solving: contact information for overlapping rigidBodys
  struct EpaResult {
    fm_vec3_t contactA{}; ///< Point on A's surface furthest inside B
    fm_vec3_t contactB{}; ///< Point on B's surface furthest inside A
    fm_vec3_t normal{};   ///< Contact normal pointing from B to A
    float penetration{0.0f}; ///< Overlap depth
  };

  /// Solves EPA to find penetration depth and contact information for overlapping rigidBodys
  bool epaSolve(
    Simplex &startingSimplex,
    const void *rigidBodyA, GjkSupportFunction rigidBodyASupport,
    const void *rigidBodyB, GjkSupportFunction rigidBodyBSupport,
    EpaResult &result
  );

  /// Swept EPA for continuous collision detection (time-of-impact calculation)
  bool epaSolveSwept(
    Simplex &startingSimplex,
    const void *rigidBodyA, GjkSupportFunction rigidBodyASupport,
    const void *rigidBodyB, GjkSupportFunction rigidBodyBSupport,
    fm_vec3_t &bStart, fm_vec3_t &bEnd,
    EpaResult &result
  );

} // namespace P64::Coll
