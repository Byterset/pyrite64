/**
 * @file rigid_body.h
 * @author Kevin Reier <https://github.com/Byterset>
 * @brief Contains the rigidBody definition, constants and related functions
 */
#pragma once

#include "vec_math.h"
#include "collider_shape.h"
#include "aabb_tree.h"
#include "contact.h"
#include <cstdint>
#include <vector>
#include "scene/object.h"

namespace P64::Coll {

  // Constants
  constexpr float TERMINAL_SPEED = 1000.0f;
  constexpr float TERMINAL_ANGULAR_SPEED = 400.0f;
  constexpr float TERMINAL_ANGULAR_SPEED_SQ = TERMINAL_ANGULAR_SPEED * TERMINAL_ANGULAR_SPEED;
  constexpr float POS_SLEEP_THRESHOLD = 0.01f * 16.0f; // convert to Scaled Units
  constexpr float POS_SLEEP_THRESHOLD_SQ = POS_SLEEP_THRESHOLD * POS_SLEEP_THRESHOLD;
  constexpr float SPEED_SLEEP_THRESHOLD = 0.65f * 16.0f; // convert to Scaled Units
  constexpr float SPEED_SLEEP_THRESHOLD_SQ = SPEED_SLEEP_THRESHOLD * SPEED_SLEEP_THRESHOLD;
  constexpr float ROT_SIMILARITY_SLEEP_THRESHOLD = 0.9999988f;
  constexpr float ANGULAR_SLEEP_THRESHOLD = 0.12f; // Radians per second
  constexpr float ANGULAR_SLEEP_THRESHOLD_SQ = ANGULAR_SLEEP_THRESHOLD * ANGULAR_SLEEP_THRESHOLD;
  constexpr float AMPLIFY_ANG_DAMPING_THRESHOLD = 0.015f; // Radians per second, below this angular velocity, amplification is applied to damping
  constexpr float AMPLIFY_ANG_DAMPING_THRESHOLD_SQ = AMPLIFY_ANG_DAMPING_THRESHOLD * AMPLIFY_ANG_DAMPING_THRESHOLD;
  constexpr float AMPLIFY_ANG_DAMPING_THRESHOLD_SQ_INV = 1.0f / AMPLIFY_ANG_DAMPING_THRESHOLD_SQ;
  constexpr int SLEEP_STEPS = 120;

  /// @brief Defines the different positional and rotational constraints that can be imposed on a rigidbody
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

  struct RigidBody {
    // Hot data
    fm_vec3_t *position{nullptr};
    fm_quat_t *rotation{nullptr};
    fm_vec3_t velocity{};
    fm_vec3_t angularVelocity{};

    float invMass{1.0f};
    float timeScalar{1.0f};
    float gravityScalar{1.0f};
    float angularDamping{0.03f};

    // Collision
    AABB boundingBox{};
    fm_vec3_t centerOffset{};

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
    fm_vec3_t compoundScale{};

    P64::Object *owner{};
    NodeProxy aabbTreeNodeId{NULL_NODE};
    Constraint constraints{Constraint::None};
    uint16_t sleepCounter{0};
    uint16_t collisionLayers{0};
    uint16_t collisionGroup{0};

    bool hasGravity{true};
    bool isKinematic{false};
    bool isSleeping{false};
    bool compoundPropertiesDirty{true};

    // Methods
    void init(P64::Object *object, float m);

    float getMass() const { return mass_; }
    void setMass(float newMass);

    void integrateVelocity(float fixedDt, const fm_vec3_t &gravity);
    void integrateAngularVelocity(float fixedDt);
    void integratePosition(float fixedDt);
    void integrateRotation(float fixedDt);

    void accelerate(const fm_vec3_t &accel);
    void setVelocity(const fm_vec3_t &vel);
    void applyLinearImpulse(const fm_vec3_t &impulse);
    void applyTorque(const fm_vec3_t &torque);
    void applyAngularImpulse(const fm_vec3_t &angImpulse);
    void setAngularVelocity(const fm_vec3_t &angVel);
    void applyForceAtPoint(const fm_vec3_t &force, const fm_vec3_t &worldPoint);

    void updateWorldInertia();
    void applyPositionConstraints();

    void wake() { isSleeping = false; sleepCounter = 0; }
    void sleep() { isSleeping = true; velocity = VEC3_ZERO; angularVelocity = VEC3_ZERO; }

    fm_vec3_t applyWorldInertia(const fm_vec3_t &in) const {
      return matrix3Vec3Mul(invWorldInertiaTensor, in);
    }

  private:
    float mass_{1.0f};

  };

} // namespace P64::Coll
