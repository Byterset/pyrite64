/**
 * @copyright 2024 - Max Bebök
 * @license MIT
 */
#pragma once

#include "vec_math.h"
#include "collider_shape.h"
#include "aabb_tree.h"
#include "contact.h"
#include <cstdint>

namespace P64::CollNew {

  // Constants
  constexpr float GRAVITY_CONSTANT = -9.8f * 1.5f;
  constexpr float TERMINAL_SPEED = 90.0f;
  constexpr float TERMINAL_ANGULAR_SPEED = 50.0f;
  constexpr float TERMINAL_ANGULAR_SPEED_SQ = TERMINAL_ANGULAR_SPEED * TERMINAL_ANGULAR_SPEED;
  constexpr float POS_SLEEP_THRESHOLD = 0.015f;
  constexpr float POS_SLEEP_THRESHOLD_SQ = POS_SLEEP_THRESHOLD * POS_SLEEP_THRESHOLD;
  constexpr float SPEED_SLEEP_THRESHOLD = 0.65f;
  constexpr float SPEED_SLEEP_THRESHOLD_SQ = SPEED_SLEEP_THRESHOLD * SPEED_SLEEP_THRESHOLD;
  constexpr float ROT_SIMILARITY_SLEEP_THRESHOLD = 0.9999988f;
  constexpr float ANGULAR_SLEEP_THRESHOLD = 0.12f;
  constexpr float ANGULAR_SLEEP_THRESHOLD_SQ = ANGULAR_SLEEP_THRESHOLD * ANGULAR_SLEEP_THRESHOLD;
  constexpr float AMPLIFY_ANG_DAMPING_THRESHOLD = 0.1f;
  constexpr float AMPLIFY_ANG_DAMPING_THRESHOLD_SQ = AMPLIFY_ANG_DAMPING_THRESHOLD * AMPLIFY_ANG_DAMPING_THRESHOLD;
  constexpr float AMPLIFY_ANG_DAMPING_THRESHOLD_SQ_INV = 1.0f / AMPLIFY_ANG_DAMPING_THRESHOLD_SQ;
  constexpr int SLEEP_STEPS = 60;
  constexpr float FIXED_DT = 1.0f / 30.0f;

  enum class CollisionLayer : uint16_t {
    None         = 0,
    Tangible     = (1 << 0),
    Player       = (1 << 1),
    DamageEnemy  = (1 << 2),
    Collectables = (1 << 3),
    TerrainLike  = (1 << 4),
    All          = 0xFF
  };

  enum class CollisionGroup : uint16_t {
    None = 0, Player = 1, Collectable = 2, All = 0xFF
  };

  enum class Constraint : uint16_t {
    None = 0,
    FreezePosX   = (1 << 0),
    FreezePosY   = (1 << 1),
    FreezePosZ   = (1 << 2),
    FreezePosAll = (1 << 0) | (1 << 1) | (1 << 2),
    FreezeRotX   = (1 << 3),
    FreezeRotY   = (1 << 4),
    FreezeRotZ   = (1 << 5),
    FreezeRotAll = (1 << 3) | (1 << 4) | (1 << 5),
    All          = 0xFF
  };

  inline Constraint operator|(Constraint a, Constraint b) {
    return static_cast<Constraint>(static_cast<uint16_t>(a) | static_cast<uint16_t>(b));
  }
  inline Constraint operator&(Constraint a, Constraint b) {
    return static_cast<Constraint>(static_cast<uint16_t>(a) & static_cast<uint16_t>(b));
  }
  inline bool hasFlag(Constraint c, Constraint flag) {
    return (c & flag) == flag;
  }

  /// Simple 3x3 matrix for inertia tensor
  struct Matrix3x3 {
    float m[3][3]{};

    static Matrix3x3 identity() {
      Matrix3x3 r{};
      r.m[0][0] = r.m[1][1] = r.m[2][2] = 1.0f;
      return r;
    }
  };

  fm_vec3_t matrix3Vec3Mul(const Matrix3x3 &mat, const fm_vec3_t &v);
  Matrix3x3 matrix3Mul(const Matrix3x3 &a, const Matrix3x3 &b);
  Matrix3x3 matrix3Transpose(const Matrix3x3 &m);
  Matrix3x3 quatToMatrix3(const fm_quat_t &q);

  using EntityId = uint16_t;

  struct PhysicsObject {
    // Hot data
    fm_vec3_t *position{nullptr};
    fm_quat_t *rotation{nullptr};
    fm_vec3_t velocity{};
    fm_vec3_t angularVelocity{};

    float invMass{1.0f};
    float timeScalar{1.0f};
    float gravityScalar{1.0f};
    float angularDamping{0.03f * (60.0f / 30.0f)};

    // Collision
    AABB boundingBox{};
    fm_vec3_t centerOffset{};
    Collider *collider{nullptr};
    Contact *activeContacts{nullptr};

    // Cached transforms
    Matrix3x3 invWorldInertiaTensor{};
    Matrix3x3 rotationMatrix{};
    fm_vec3_t worldCenterOfMass{};

    // State
    fm_vec3_t acceleration{};
    fm_vec3_t torqueAccumulator{};
    fm_vec3_t prevStepPos{};
    fm_quat_t prevStepRot{};
    fm_vec3_t localInertiaTensor{};
    fm_vec3_t invLocalInertiaTensor{};

    float mass{1.0f};
    EntityId entityId{0};
    NodeProxy aabbTreeNodeId{NULL_NODE};
    Constraint constraints{Constraint::None};
    uint16_t sleepCounter{0};
    uint16_t collisionLayers{0};
    uint16_t collisionGroup{0};

    bool hasGravity{true};
    bool isTrigger{false};
    bool isKinematic{false};
    bool isGrounded{false};
    bool isSleeping{false};

    // Methods
    void init(EntityId id, Collider *coll, uint16_t layers,
              fm_vec3_t *pos, fm_quat_t *rot, fm_vec3_t offset, float m);

    void setMass(float newMass);

    void integrateVelocity();
    void integrateAngularVelocity();
    void integratePosition();
    void integrateRotation();

    void accelerate(const fm_vec3_t &accel);
    void setVelocity(const fm_vec3_t &vel);
    void applyLinearImpulse(const fm_vec3_t &impulse);
    void applyTorque(const fm_vec3_t &torque);
    void applyAngularImpulse(const fm_vec3_t &angImpulse);
    void setAngularVelocity(const fm_vec3_t &angVel);
    void applyForceAtPoint(const fm_vec3_t &force, const fm_vec3_t &worldPoint);

    void recalculateAABB();
    void updateWorldInertia();
    void applyPositionConstraints();

    void wake() { isSleeping = false; sleepCounter = 0; }
    void sleep() { isSleeping = true; velocity = vec3Zero(); angularVelocity = vec3Zero(); }

    fm_vec3_t applyWorldInertia(const fm_vec3_t &in) const {
      return matrix3Vec3Mul(invWorldInertiaTensor, in);
    }

    void gjkSupport(const fm_vec3_t &direction, fm_vec3_t &output) const;

    Contact *nearestContact() const;
    bool isTouching(EntityId id) const;
  };

  /// GJK-compatible free function wrapper
  void physicsObjectGjkSupport(const void *data, const fm_vec3_t &direction, fm_vec3_t &output);

} // namespace P64::CollNew
