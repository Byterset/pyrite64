/**
 * @copyright 2024 - Max Bebök
 * @license MIT
 */
#pragma once

#include "vec_math.h"
#include <cstdint>

namespace P64 { class Object; }

namespace P64::CollNew {

  constexpr int MAX_CONTACT_POINTS_PER_PAIR = 4;

  struct RigidBody; // forward declare
  struct Collider;  // forward declare
  struct MeshCollider;  // forward declare

  /// Linked-list node for tracking contacts on a physics object
  struct Contact {
    struct ContactConstraint *constraint{nullptr};
    RigidBody *otherBody{nullptr};
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

  /// Contact constraint representing a pair of colliding rigid bodies
  struct ContactConstraint {
    RigidBody *rigidBodyA{nullptr};
    Collider *colliderA{nullptr};
    Object *objectA{nullptr};
    RigidBody *rigidBodyB{nullptr};
    Collider *colliderB{nullptr};
    Object *objectB{nullptr};

    fm_vec3_t normal{};
    fm_vec3_t tangentU{};
    fm_vec3_t tangentV{};

    float combinedFriction{0.0f};
    float combinedBounce{0.0f};

    int pointCount{0};

    bool isActive{false};
    bool isTrigger{false};

    ContactPoint points[MAX_CONTACT_POINTS_PER_PAIR]{};
  };

} // namespace P64::CollNew
