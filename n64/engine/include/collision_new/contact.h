/**
 * @copyright 2024 - Max Bebök
 * @license MIT
 */
#pragma once

#include "vec_math.h"
#include <cstdint>

namespace P64::CollNew {

  constexpr int MAX_CONTACT_POINTS_PER_PAIR = 4;

  struct PhysicsObject; // forward declare

  using ContactPairId = uint32_t;

  /// Linked-list node for tracking contacts on a physics object
  struct Contact {
    Contact *next{nullptr};
    struct ContactConstraint *constraint{nullptr};
    PhysicsObject *otherObject{nullptr};
  };

  /// Single contact point within a contact constraint
  struct ContactPoint {
    fm_vec3_t point{};
    fm_vec3_t contactA{};
    fm_vec3_t contactB{};
    fm_vec3_t localPointA{};
    fm_vec3_t localPointB{};
    fm_vec3_t aToContact{};
    fm_vec3_t bToContact{};
    float penetration{0.0f};

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
    PhysicsObject *objectA{nullptr};
    PhysicsObject *objectB{nullptr};

    fm_vec3_t normal{};
    fm_vec3_t tangentU{};
    fm_vec3_t tangentV{};

    float combinedFriction{0.0f};
    float combinedBounce{0.0f};

    ContactPairId pid{0};
    int nextSamePidIndex{-1};
    int pointCount{0};

    bool isActive{false};
    bool isTrigger{false};

    ContactPoint points[MAX_CONTACT_POINTS_PER_PAIR]{};
  };

  inline ContactPairId makeContactPairId(uint16_t a, uint16_t b) {
    if(a < b) return (static_cast<uint32_t>(a) << 16) | b;
    return (static_cast<uint32_t>(b) << 16) | a;
  }

} // namespace P64::CollNew
