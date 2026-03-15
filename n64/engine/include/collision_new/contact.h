/**
 * @copyright 2024 - Max Bebök
 * @license MIT
 */
#pragma once

#include "vec_math.h"
#include <cstdint>

namespace P64::CollNew {

  constexpr int MAX_CONTACT_POINTS_PER_PAIR = 4;

  /// Single contact point within a contact constraint
  struct ContactPoint {
    fm_vec3_t point{};            ///< 3D world-space contact position
    fm_vec3_t contactA{};         ///< Contact point on surface A (world space)
    fm_vec3_t contactB{};         ///< Contact point on surface B (world space)
    fm_vec3_t localPointA{};      ///< Contact point on A (local space)
    fm_vec3_t localPointB{};      ///< Contact point on B (local space)
    fm_vec3_t aToContact{};       ///< Contact relative to A's center of mass
    fm_vec3_t bToContact{};       ///< Contact relative to B's center of mass
    float penetration{0.0f};      ///< Depth of penetration for this point

    // Cached solver data for warm starting and iterative solving
    float accumulatedNormalImpulse{0.0f};
    float accumulatedTangentImpulseU{0.0f};
    float accumulatedTangentImpulseV{0.0f};
    float normalMass{0.0f};
    float tangentMassU{0.0f};
    float tangentMassV{0.0f};
    float velocityBias{0.0f};

    bool active{false};
  };

  /// Contact constraint representing a pair of colliding objects
  struct ContactConstraint {
    void *objectA{nullptr};
    void *objectB{nullptr};

    fm_vec3_t normal{};     ///< Collision normal (B toward A)
    fm_vec3_t tangentU{};   ///< First tangent direction for friction
    fm_vec3_t tangentV{};   ///< Second tangent direction for friction

    float combinedFriction{0.0f};
    float combinedBounce{0.0f};

    uint32_t pairId{0};
    int nextSamePidIndex{-1};
    int pointCount{0};

    bool isActive{false};
    bool isTrigger{false};

    ContactPoint points[MAX_CONTACT_POINTS_PER_PAIR]{};
  };

} // namespace P64::CollNew
