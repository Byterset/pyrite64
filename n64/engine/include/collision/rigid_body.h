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
  constexpr float TERMINAL_SPEED = 100.0f; // Units per second, scaled by physicsScale when applied
  constexpr float TERMINAL_ANGULAR_SPEED = 50.0f; // Radians per second
  constexpr float TERMINAL_ANGULAR_SPEED_SQ = TERMINAL_ANGULAR_SPEED * TERMINAL_ANGULAR_SPEED;
  constexpr float POS_SLEEP_THRESHOLD = 0.01f; // Units moved, scaled by physicsScale when used
  constexpr float POS_SLEEP_THRESHOLD_SQ = POS_SLEEP_THRESHOLD * POS_SLEEP_THRESHOLD;
  constexpr float SPEED_SLEEP_THRESHOLD = 0.65f; // Units per second,scaled by physicsScale when used
  constexpr float SPEED_SLEEP_THRESHOLD_SQ = SPEED_SLEEP_THRESHOLD * SPEED_SLEEP_THRESHOLD;
  constexpr float ROT_SIMILARITY_SLEEP_THRESHOLD = 0.9999988f;
  constexpr float ANGULAR_SLEEP_THRESHOLD = 0.12f; // Radians per second, no need to scale with physicsScale since it's an angular velocity
  constexpr float ANGULAR_SLEEP_THRESHOLD_SQ = ANGULAR_SLEEP_THRESHOLD * ANGULAR_SLEEP_THRESHOLD;
  constexpr float AMPLIFY_ANG_DAMPING_THRESHOLD = 0.015f; // Radians per second, below this angular velocity, amplification is applied to damping
  constexpr float AMPLIFY_ANG_DAMPING_THRESHOLD_SQ = AMPLIFY_ANG_DAMPING_THRESHOLD * AMPLIFY_ANG_DAMPING_THRESHOLD;
  constexpr float AMPLIFY_ANG_DAMPING_THRESHOLD_SQ_INV = 1.0f / AMPLIFY_ANG_DAMPING_THRESHOLD_SQ;
  constexpr int SLEEP_STEPS = 120; // Number of consecutive steps an object must be below the sleep thresholds before it goes to sleep

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
    AABB worldAABB{};

    // Cached transforms
    Matrix3x3 invWorldInertiaTensor{};
    Matrix3x3 rotationMatrix{};
    fm_vec3_t worldCenterOfMass{};

    // State
    fm_vec3_t acceleration{};
    fm_vec3_t torqueAccumulator{};
    fm_vec3_t prevStepPos{};
    fm_quat_t prevStepRot{};
    fm_vec3_t prevStepScale{};

    P64::Object *owner{};
    NodeProxy aabbTreeNodeId{NULL_NODE};
    uint16_t sleepCounter{0};
    uint16_t collisionLayers{0};
    uint16_t collisionGroup{0};

    bool hasGravity{true};
    bool isKinematic{false};
    bool isSleeping{false};

    // Methods
    void init(P64::Object *object, float m);

    float getMass() const { return mass_; }
    void setMass(float newMass);
    Constraint getConstraints() const { return constraints_; }
    void setConstraints(Constraint newConstraints);

    bool hasLinearConstraints() const { return hasLinearConstraints_; }
    bool hasAngularConstraints() const { return hasAngularConstraints_; }
    bool canApplyAngularResponse() const { return !isKinematic && rotation && !hasFlag(constraints_, Constraint::FreezeRotAll); }

    bool compoundPropertiesDirty() const { return compoundPropertiesDirty_; }
    const fm_vec3_t &getCenterOffset() const { return centerOffset_; }
    const fm_vec3_t &getLocalInertiaTensor() const { return localInertiaTensor_; }
    const fm_vec3_t &getDefaultLocalInertiaTensor() const { return defaultLocalInertiaTensor_; }
    const fm_vec3_t &getCompoundScale() const { return compoundScale_; }
    void markCompoundPropertiesDirty() { compoundPropertiesDirty_ = true; }
    void applyCompoundProperties(const fm_vec3_t &centerOffset, const fm_vec3_t &localInertiaTensor, const fm_vec3_t &compoundScale);
    void setKinematic(bool newIsKinematic) { isKinematic = newIsKinematic; }

    fm_vec3_t constrainLinearWorld(const fm_vec3_t &worldLinear) const;
    fm_vec3_t constrainAngularWorld(const fm_vec3_t &worldAngular) const;
    void applyConstrainedLinearVelocityDelta(const fm_vec3_t &deltaLinearVelocity);
    void applyConstrainedImpulseAtContact(const fm_vec3_t &impulse, const fm_vec3_t &toContact);
    float constrainedLinearInvMassAlong(const fm_vec3_t &direction) const;

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
    fm_vec3_t getVelocityAtPoint(const fm_vec3_t &worldPoint) const;

    void updateWorldInertia();
    void applyPositionConstraints();
    fm_vec3_t toWorldSpace(const fm_vec3_t &localPoint) const;
    fm_vec3_t toLocalSpace(const fm_vec3_t &worldPoint) const;
    fm_vec3_t rotateToWorld(const fm_vec3_t &localDir) const;
    fm_vec3_t rotateToLocal(const fm_vec3_t &worldDir) const;

    void wake() { isSleeping = false; sleepCounter = 0; }
    void sleep() { isSleeping = true; velocity = VEC3_ZERO; angularVelocity = VEC3_ZERO; }

    fm_vec3_t applyWorldInertia(const fm_vec3_t &in) const {
      return matrix3Vec3Mul(invWorldInertiaTensor, in);
    }

    fm_vec3_t applyConstrainedWorldInertia(const fm_vec3_t &in) const {
      if(!hasAngularConstraints_) return applyWorldInertia(in);
      return matrix3Vec3Mul(constrainedInvWorldInertiaTensor_, in);
    }

  private:
    float mass_{1.0f};
    Constraint constraints_{Constraint::None};
    fm_vec3_t centerOffset_{};
    fm_vec3_t localInertiaTensor_{};
    fm_vec3_t invLocalInertiaTensor_{};
    fm_vec3_t defaultLocalInertiaTensor_{};
    fm_vec3_t compoundScale_{};
    fm_vec3_t linearConstraintScale_ = fm_vec3_t{{1.0f, 1.0f, 1.0f}};
    fm_vec3_t linearInvMassScale_{};
    Matrix3x3 angularConstraintProjection_ = Matrix3x3::identity();
    Matrix3x3 constrainedInvWorldInertiaTensor_{};
    bool hasLinearConstraints_{false};
    bool hasAngularConstraints_{false};
    bool compoundPropertiesDirty_{true};

    void refreshConstraintCaches();
    void refreshAngularConstraintProjection();
    void refreshConstrainedInertiaTensor();

  };

} // namespace P64::Coll
