/**
 * @file rigid_body.cpp
 * @author Kevin Reier <https://github.com/Byterset>
 * @brief Contains the rigidBody definition, constants and related functions (see rigid_body.h)
 */
#include "collision/rigid_body.h"
#include "collision/collision_scene.h"
#include <cassert>
#include <cmath>

namespace P64::Coll {

  static fm_vec3_t scaleVec3Components(const fm_vec3_t &lhs, const fm_vec3_t &rhs) {
    return fm_vec3_t{{
      lhs.x * rhs.x,
      lhs.y * rhs.y,
      lhs.z * rhs.z
    }};
  }

  static fm_vec3_t diagonalInverse(const fm_vec3_t &v) {
    return fm_vec3_t{{
      v.x > FM_EPSILON ? 1.0f / v.x : 0.0f,
      v.y > FM_EPSILON ? 1.0f / v.y : 0.0f,
      v.z > FM_EPSILON ? 1.0f / v.z : 0.0f
    }};
  }

  static Matrix3x3 diagonalMatrix(const fm_vec3_t &diag) {
    Matrix3x3 result{};
    result.m[0][0] = diag.x;
    result.m[1][1] = diag.y;
    result.m[2][2] = diag.z;
    return result;
  }

  static fm_vec3_t linearConstraintScale(Constraint constraints) {
    return fm_vec3_t{{
      hasFlag(constraints, Constraint::FreezePosX) ? 0.0f : 1.0f,
      hasFlag(constraints, Constraint::FreezePosY) ? 0.0f : 1.0f,
      hasFlag(constraints, Constraint::FreezePosZ) ? 0.0f : 1.0f
    }};
  }

  static fm_vec3_t angularConstraintScale(Constraint constraints) {
    return fm_vec3_t{{
      hasFlag(constraints, Constraint::FreezeRotX) ? 0.0f : 1.0f,
      hasFlag(constraints, Constraint::FreezeRotY) ? 0.0f : 1.0f,
      hasFlag(constraints, Constraint::FreezeRotZ) ? 0.0f : 1.0f
    }};
  }

  // ── Matrix utilities ──────────────────────────────────────────────

  fm_vec3_t matrix3Vec3Mul(const Matrix3x3 &mat, const fm_vec3_t &v) {
    return fm_vec3_t{{
      mat.m[0][0] * v.x + mat.m[0][1] * v.y + mat.m[0][2] * v.z,
      mat.m[1][0] * v.x + mat.m[1][1] * v.y + mat.m[1][2] * v.z,
      mat.m[2][0] * v.x + mat.m[2][1] * v.y + mat.m[2][2] * v.z
    }};
  }

  Matrix3x3 matrix3Mul(const Matrix3x3 &a, const Matrix3x3 &b) {
    Matrix3x3 r{};
    for(int i = 0; i < 3; ++i) {
      for(int j = 0; j < 3; ++j) {
        r.m[i][j] = a.m[i][0] * b.m[0][j]
                   + a.m[i][1] * b.m[1][j]
                   + a.m[i][2] * b.m[2][j];
      }
    }
    return r;
  }

  Matrix3x3 matrix3Transpose(const Matrix3x3 &m) {
    Matrix3x3 r{};
    for(int i = 0; i < 3; ++i) {
      for(int j = 0; j < 3; ++j) {
        r.m[i][j] = m.m[j][i];
      }
    }
    return r;
  }

  Matrix3x3 quatToMatrix3(const fm_quat_t &q) {
    float xx = q.x * q.x, yy = q.y * q.y, zz = q.z * q.z;
    float xy = q.x * q.y, xz = q.x * q.z, yz = q.y * q.z;
    float wx = q.w * q.x, wy = q.w * q.y, wz = q.w * q.z;

    Matrix3x3 r{};
    r.m[0][0] = 1.0f - 2.0f * (yy + zz);
    r.m[0][1] = 2.0f * (xy - wz);
    r.m[0][2] = 2.0f * (xz + wy);

    r.m[1][0] = 2.0f * (xy + wz);
    r.m[1][1] = 1.0f - 2.0f * (xx + zz);
    r.m[1][2] = 2.0f * (yz - wx);

    r.m[2][0] = 2.0f * (xz - wy);
    r.m[2][1] = 2.0f * (yz + wx);
    r.m[2][2] = 1.0f - 2.0f * (xx + yy);
    return r;
  }

  // ── RigidBody ─────────────────────────────────────────────────

  void RigidBody::init(P64::Object *object, float m) {
    assertf(m > 0.0f, "Mass must be greater than zero");
    assertf(object, "RigidBody must be initialized with a valid owner object");

    owner = object;
    position = &object->pos;
    rotation = &object->rot;
    collisionGroup = 0;
    aabbTreeNodeId = NULL_NODE;
    constraints_ = Constraint::None;
    sleepCounter = 0;
    isSleeping = false;
    isKinematic = false;
    hasGravity = true;

    velocity = VEC3_ZERO;
    angularVelocity = VEC3_ZERO;
    acceleration = VEC3_ZERO;
    torqueAccumulator = VEC3_ZERO;
    centerOffset_ = VEC3_ZERO;
    compoundScale_ = object->scale;
    compoundPropertiesDirty_ = true;

    timeScalar = 1.0f;
    gravityScalar = 1.0f;
    angularDamping = 0.03f;

    setMass(m);

    if(rotation) {
      prevStepRot = *rotation;
    }
    if(position) {
      prevStepPos = *position;
    }
    prevStepScale = object->scale;
  }

  void RigidBody::setMass(float newMass) {
    assert(newMass > 0.0f);
    mass_ = newMass;
    invMass = 1.0f / newMass;
    compoundPropertiesDirty_ = true;

    // Initialize inertia tensor for a solid sphere as a simple default.
    // This can be overridden later by the colliders associated with the owner object.
    float inertia = 0.4f * mass_;
    defaultLocalInertiaTensor_ = fm_vec3_t{{inertia, inertia, inertia}};
    localInertiaTensor_ = defaultLocalInertiaTensor_;
    invLocalInertiaTensor_ = diagonalInverse(localInertiaTensor_);
    refreshConstraintCaches();
    updateWorldInertia();
  }

  void RigidBody::setConstraints(Constraint newConstraints) {
    constraints_ = newConstraints;
    refreshConstraintCaches();
    velocity = constrainLinearWorld(velocity);
    angularVelocity = constrainAngularWorld(angularVelocity);
  }

  void RigidBody::applyCompoundProperties(const fm_vec3_t &centerOffset, const fm_vec3_t &localInertiaTensor, const fm_vec3_t &compoundScale) {
    centerOffset_ = centerOffset;
    localInertiaTensor_ = localInertiaTensor;
    invLocalInertiaTensor_ = diagonalInverse(localInertiaTensor_);
    compoundScale_ = compoundScale;
    compoundPropertiesDirty_ = false;
    updateWorldInertia();
  }

  fm_vec3_t RigidBody::constrainLinearWorld(const fm_vec3_t &worldLinear) const {
    if(!hasLinearConstraints_) return worldLinear;
    return scaleVec3Components(worldLinear, linearConstraintScale_);
  }

  fm_vec3_t RigidBody::constrainAngularWorld(const fm_vec3_t &worldAngular) const {
    if(!hasAngularConstraints_) return worldAngular;
    return matrix3Vec3Mul(angularConstraintProjection_, worldAngular);
  }

  void RigidBody::applyConstrainedLinearVelocityDelta(const fm_vec3_t &deltaLinearVelocity) {
    velocity = velocity + constrainLinearWorld(deltaLinearVelocity);
  }

  void RigidBody::applyConstrainedImpulseAtContact(const fm_vec3_t &impulse, const fm_vec3_t &toContact) {
    if(isKinematic) return;

    applyConstrainedLinearVelocityDelta(impulse * invMass);
    if(!canApplyAngularResponse()) return;

    fm_vec3_t cross;
    fm_vec3_cross(&cross, &toContact, &impulse);
    angularVelocity = angularVelocity + applyConstrainedWorldInertia(cross);
  }

  float RigidBody::constrainedLinearInvMassAlong(const fm_vec3_t &direction) const {
    if(isKinematic) return 0.0f;
    if(invMass <= FM_EPSILON) return 0.0f;
    if(!hasLinearConstraints_) return invMass;

    return direction.x * direction.x * linearInvMassScale_.x +
           direction.y * direction.y * linearInvMassScale_.y +
           direction.z * direction.z * linearInvMassScale_.z;
  }

  void RigidBody::refreshConstraintCaches() {
    hasLinearConstraints_ = (constraints_ & Constraint::FreezePosAll) != Constraint::None;
    hasAngularConstraints_ = (constraints_ & Constraint::FreezeRotAll) != Constraint::None;
    linearConstraintScale_ = linearConstraintScale(constraints_);
    linearInvMassScale_ = linearConstraintScale_ * invMass;
    refreshAngularConstraintProjection();
    refreshConstrainedInertiaTensor();
  }

  void RigidBody::refreshAngularConstraintProjection() {
    if(!hasAngularConstraints_) {
      angularConstraintProjection_ = Matrix3x3::identity();
      return;
    }

    const Matrix3x3 localProjection = diagonalMatrix(angularConstraintScale(constraints_));
    const Matrix3x3 rotationTranspose = matrix3Transpose(rotationMatrix);
    angularConstraintProjection_ = matrix3Mul(matrix3Mul(rotationMatrix, localProjection), rotationTranspose);
  }

  void RigidBody::refreshConstrainedInertiaTensor() {
    if(!hasAngularConstraints_) {
      constrainedInvWorldInertiaTensor_ = invWorldInertiaTensor;
      return;
    }

    constrainedInvWorldInertiaTensor_ = matrix3Mul(angularConstraintProjection_, invWorldInertiaTensor);
  }

  void RigidBody::integrateVelocity(float fixedDt, const fm_vec3_t &gravity) {
    if(isKinematic || isSleeping) return;

    if(hasFlag(constraints_, Constraint::FreezePosAll)) {
      velocity = VEC3_ZERO;
      acceleration = VEC3_ZERO;
      return;
    }

    // Apply gravity
    if(hasGravity) {
      acceleration = acceleration + (gravity * gravityScalar);
    }

    float dt = fixedDt * timeScalar;

    // Add user acceleration
    velocity = velocity + (acceleration * dt);
    acceleration = VEC3_ZERO;

    // Apply position constraints
    velocity = constrainLinearWorld(velocity);

    CollisionScene *scene = collisionSceneGetInstance();
    float physicsScale = scene->getPhysicsScale();

    // Clamp to terminal speed
    float speedSq = fm_vec3_len2(&velocity);
    if(speedSq > TERMINAL_SPEED * TERMINAL_SPEED * (physicsScale * physicsScale)) {
      velocity = velocity * ((TERMINAL_SPEED * physicsScale) / sqrtf(speedSq));
    }
  }

  void RigidBody::integrateAngularVelocity(float fixedDt) {
    if(isKinematic || isSleeping) return;
    if(invMass < FM_EPSILON) return;

    if(hasFlag(constraints_, Constraint::FreezeRotAll)) {
      angularVelocity = VEC3_ZERO;
      torqueAccumulator = VEC3_ZERO;
      return;
    }

    float dt = fixedDt * timeScalar;
    const bool hadExternalTorque = !vec3IsZero(torqueAccumulator);

    // Apply torque accumulator
    if(hadExternalTorque) {
      fm_vec3_t angAccel = applyWorldInertia(torqueAccumulator);
      angularVelocity = angularVelocity + (angAccel * dt * timeScalar);
      torqueAccumulator = VEC3_ZERO;
    }

    // Clamp angular speed
    float angSpeedSq = fm_vec3_len2(&angularVelocity);
    if(angSpeedSq > TERMINAL_ANGULAR_SPEED_SQ) {
      angularVelocity = angularVelocity * (TERMINAL_ANGULAR_SPEED / sqrtf(angSpeedSq));
      angSpeedSq = TERMINAL_ANGULAR_SPEED_SQ;
    }

    // Apply rotation constraints in local space
    if(hasAngularConstraints_) {
      angularVelocity = constrainAngularWorld(angularVelocity);
    }

    // Angular damping — amplify near rest
    float dampFactor = angularDamping;
    if(!hadExternalTorque && angSpeedSq < AMPLIFY_ANG_DAMPING_THRESHOLD_SQ && angSpeedSq > FM_EPSILON) {
      float ratio = angSpeedSq * AMPLIFY_ANG_DAMPING_THRESHOLD_SQ_INV;
      dampFactor = angularDamping + (1.0f - angularDamping) * (1.0f - ratio);
    }
    angularVelocity = angularVelocity * (1.0f - dampFactor);
  }

  void RigidBody::integratePosition(float fixedDt) {
    if(isKinematic || isSleeping) return;
    if(!position) return;

    float dt = fixedDt * timeScalar;
    prevStepPos = *position;
    *position = *position + (velocity * dt);
  }

  void RigidBody::integrateRotation(float fixedDt) {
    if(isKinematic || isSleeping) return;
    if(!rotation) return;
    if(vec3IsZero(angularVelocity)) return;

    float dt = fixedDt * timeScalar;

    prevStepRot = *rotation;

    // Store old world center of mass for offset correction
    fm_vec3_t oldWorldCOM = *position + (*rotation * centerOffset_);

    *rotation = quatApplyAngularVelocity(*rotation, angularVelocity, dt);

    // Correct position so that the center of mass stays in place
    if(!vec3IsZero(centerOffset_)) {
      fm_vec3_t newWorldCOM = *position + (*rotation * centerOffset_);
      fm_vec3_t correction = oldWorldCOM - newWorldCOM;
      *position = *position + correction;
    }
  }

  void RigidBody::accelerate(const fm_vec3_t &accel) {
    acceleration = acceleration + accel;
    if(isSleeping) wake();
  }

  void RigidBody::setVelocity(const fm_vec3_t &vel) {
    velocity = constrainLinearWorld(vel);
    if(isSleeping) wake();
  }

  void RigidBody::applyLinearImpulse(const fm_vec3_t &impulse) {
    if(isKinematic) return;
    fm_vec3_t deltaV = constrainLinearWorld(impulse * invMass);
    velocity = velocity + deltaV;
    if(isSleeping) wake();
  }

  void RigidBody::applyTorque(const fm_vec3_t &torque) {
    if(isKinematic) return;
    torqueAccumulator = torqueAccumulator + torque;
    if(isSleeping) wake();
  }

  void RigidBody::applyAngularImpulse(const fm_vec3_t &angImpulse) {
    if(isKinematic) return;
    angularVelocity = angularVelocity + applyConstrainedWorldInertia(angImpulse);
    if(isSleeping) wake();
  }

  void RigidBody::setAngularVelocity(const fm_vec3_t &angVel) {
    angularVelocity = constrainAngularWorld(angVel);
    if(isSleeping) wake();
  }

  fm_vec3_t RigidBody::getVelocityAtPoint(const fm_vec3_t &worldPoint) const {
    fm_vec3_t pointVelocity = velocity;

    if(vec3IsZero(angularVelocity)) {
      return pointVelocity;
    }

    fm_vec3_t offset = worldPoint - worldCenterOfMass;
    fm_vec3_t angularPointVelocity;
    fm_vec3_cross(&angularPointVelocity, &angularVelocity, &offset);
    return pointVelocity + angularPointVelocity;
  }

  void RigidBody::applyForceAtPoint(const fm_vec3_t &force, const fm_vec3_t &worldPoint) {
    if(isKinematic) return;
    applyLinearImpulse(force);
    fm_vec3_t r = worldPoint - worldCenterOfMass;
    fm_vec3_t torque;
    fm_vec3_cross(&torque, &r, &force);
    applyAngularImpulse(torque);
  }

  void RigidBody::updateWorldInertia() {
    if(rotation) {
      rotationMatrix = quatToMatrix3(*rotation);
    } else {
      rotationMatrix = Matrix3x3::identity();
    }

    const Matrix3x3 localInv = diagonalMatrix(invLocalInertiaTensor_);
    const Matrix3x3 rTranspose = matrix3Transpose(rotationMatrix);
    invWorldInertiaTensor = matrix3Mul(matrix3Mul(rotationMatrix, localInv), rTranspose);
    refreshAngularConstraintProjection();
    refreshConstrainedInertiaTensor();

    const fm_vec3_t worldOffset = rotateToWorld(centerOffset_);
    worldCenterOfMass = position ? *position + worldOffset : worldOffset;
  }

  fm_vec3_t RigidBody::toWorldSpace(const fm_vec3_t &localPoint) const {
    if(!position) return localPoint;
    return *position + rotateToWorld(localPoint);
  }

  fm_vec3_t RigidBody::toLocalSpace(const fm_vec3_t &worldPoint) const {
    if(!position) return worldPoint;
    return rotateToLocal(worldPoint - *position);
  }

  fm_vec3_t RigidBody::rotateToWorld(const fm_vec3_t &localDir) const {
    if(!rotation) return localDir;
    return *rotation * localDir;
  }

  fm_vec3_t RigidBody::rotateToLocal(const fm_vec3_t &worldDir) const {
    if(!rotation) return worldDir;
    return quatConjugate(*rotation) * worldDir;
  }

  void RigidBody::applyPositionConstraints() {
    if(!position) return;
    if(hasFlag(constraints_, Constraint::FreezePosX)) position->x = prevStepPos.x;
    if(hasFlag(constraints_, Constraint::FreezePosY)) position->y = prevStepPos.y;
    if(hasFlag(constraints_, Constraint::FreezePosZ)) position->z = prevStepPos.z;
  }

} // namespace P64::Coll
