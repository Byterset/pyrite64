/**
 * @copyright 2024 - Max Bebök
 * @license MIT
 */
#include "collision_new/rigid_body.h"
#include <cassert>
#include <cmath>

namespace P64::CollNew {

  static fm_vec3_t constrainLinearByFlags(Constraint constraints, const fm_vec3_t &v) {
    fm_vec3_t out = v;
    if(hasFlag(constraints, Constraint::FreezePosX)) out.x = 0.0f;
    if(hasFlag(constraints, Constraint::FreezePosY)) out.y = 0.0f;
    if(hasFlag(constraints, Constraint::FreezePosZ)) out.z = 0.0f;
    return out;
  }

  // ── Matrix utilities ──────────────────────────────────────────────

  fm_vec3_t matrix3Vec3Mul(const Matrix3x3 &mat, const fm_vec3_t &v) {
    return vec3(
      mat.m[0][0] * v.x + mat.m[0][1] * v.y + mat.m[0][2] * v.z,
      mat.m[1][0] * v.x + mat.m[1][1] * v.y + mat.m[1][2] * v.z,
      mat.m[2][0] * v.x + mat.m[2][1] * v.y + mat.m[2][2] * v.z
    );
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
    constraints = Constraint::None;
    sleepCounter = 0;
    isSleeping = false;
    isTrigger = false;
    isKinematic = false;
    hasGravity = true;

    velocity = vec3Zero();
    angularVelocity = vec3Zero();
    acceleration = vec3Zero();
    torqueAccumulator = vec3Zero();

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

    updateWorldInertia();
  }

  void RigidBody::setMass(float newMass) {
    assert(newMass > 0.0f);
    mass_ = newMass;
    invMass = 1.0f / newMass;

    // Initialize inertia tensor for a solid sphere as a simple default.
    // This can be overridden later by the colliders associated with the owner object.
    float inertia = 0.4f * mass_;
    localInertiaTensor = vec3(inertia, inertia, inertia);

    invLocalInertiaTensor = vec3(
      localInertiaTensor.x > EPSILON ? 1.0f / localInertiaTensor.x : 0.0f,
      localInertiaTensor.y > EPSILON ? 1.0f / localInertiaTensor.y : 0.0f,
      localInertiaTensor.z > EPSILON ? 1.0f / localInertiaTensor.z : 0.0f
    );
  }

  void RigidBody::integrateVelocity(float fixedDt, const fm_vec3_t &gravity) {
    if(isTrigger || isKinematic || isSleeping) return;

    if(hasFlag(constraints, Constraint::FreezePosAll)) {
      velocity = vec3Zero();
      acceleration = vec3Zero();
      return;
    }

    // Apply gravity
    if(hasGravity) {
      acceleration = acceleration + (gravity * gravityScalar);
    }

    float dt = fixedDt * timeScalar;

    // Add user acceleration
    velocity = velocity + (acceleration * dt);
    acceleration = vec3Zero();

    // Apply position constraints
    velocity = constrainLinearByFlags(constraints, velocity);

    // Clamp to terminal speed
    float speedSq = vec3MagSqrd(velocity);
    if(speedSq > TERMINAL_SPEED * TERMINAL_SPEED) {
      velocity = velocity * (TERMINAL_SPEED / sqrtf(speedSq));
    }
  }

  void RigidBody::integrateAngularVelocity(float fixedDt) {
    if(isTrigger || isKinematic || isSleeping) return;
    if(invMass < EPSILON) return;

    if(hasFlag(constraints, Constraint::FreezeRotAll)) {
      angularVelocity = vec3Zero();
      torqueAccumulator = vec3Zero();
      return;
    }

    float dt = fixedDt * timeScalar;

    // Apply torque accumulator
    if(!vec3IsZero(torqueAccumulator)) {
      fm_vec3_t angAccel = applyWorldInertia(torqueAccumulator);
      angularVelocity = angularVelocity + (angAccel * dt * timeScalar);
      torqueAccumulator = vec3Zero();
    }

    // Clamp angular speed
    float angSpeedSq = vec3MagSqrd(angularVelocity);
    if(angSpeedSq > TERMINAL_ANGULAR_SPEED_SQ) {
      angularVelocity = angularVelocity * (TERMINAL_ANGULAR_SPEED / sqrtf(angSpeedSq));
      angSpeedSq = TERMINAL_ANGULAR_SPEED_SQ;
    }

    // Apply rotation constraints in local space
    if(rotation && (constraints & Constraint::FreezeRotAll) != Constraint::None) {
      fm_vec3_t localAV = quatRotateVec(quatConjugate(*rotation), angularVelocity);
      if(hasFlag(constraints, Constraint::FreezeRotX)) localAV.x = 0.0f;
      if(hasFlag(constraints, Constraint::FreezeRotY)) localAV.y = 0.0f;
      if(hasFlag(constraints, Constraint::FreezeRotZ)) localAV.z = 0.0f;
      angularVelocity = quatRotateVec(*rotation, localAV);
    }

    // Angular damping — amplify near rest
    float dampFactor = angularDamping;
    if(angSpeedSq < AMPLIFY_ANG_DAMPING_THRESHOLD_SQ && angSpeedSq > EPSILON) {
      float ratio = angSpeedSq * AMPLIFY_ANG_DAMPING_THRESHOLD_SQ_INV;
      dampFactor = angularDamping + (1.0f - angularDamping) * (1.0f - ratio);
    }
    angularVelocity = vec3Scale(angularVelocity, 1.0f - dampFactor);
  }

  void RigidBody::integratePosition(float fixedDt) {
    if(isTrigger || isKinematic || isSleeping) return;
    if(!position) return;

    float dt = fixedDt * timeScalar;
    prevStepPos = *position;
    *position = vec3Add(*position, vec3Scale(velocity, dt));
  }

  void RigidBody::integrateRotation(float fixedDt) {
    if(isTrigger || isKinematic || isSleeping) return;
    if(!rotation) return;
    if(vec3IsZero(angularVelocity)) return;

    float dt = fixedDt * timeScalar;

    prevStepRot = *rotation;

    // Store old world center of mass for offset correction
    fm_vec3_t oldWorldCOM = vec3Add(*position, quatRotateVec(*rotation, centerOffset));

    *rotation = quatApplyAngularVelocity(*rotation, angularVelocity, dt);

    // Correct position so that the center of mass stays in place
    if(!vec3IsZero(centerOffset)) {
      fm_vec3_t newWorldCOM = vec3Add(*position, quatRotateVec(*rotation, centerOffset));
      fm_vec3_t correction = vec3Sub(oldWorldCOM, newWorldCOM);
      *position = vec3Add(*position, correction);
    }
  }

  void RigidBody::accelerate(const fm_vec3_t &accel) {
    acceleration = vec3Add(acceleration, accel);
    if(isSleeping) wake();
  }

  void RigidBody::setVelocity(const fm_vec3_t &vel) {
    velocity = constrainLinearByFlags(constraints, vel);
    if(isSleeping) wake();
  }

  void RigidBody::applyLinearImpulse(const fm_vec3_t &impulse) {
    if(isKinematic || isTrigger) return;
    fm_vec3_t deltaV = constrainLinearByFlags(constraints, vec3Scale(impulse, invMass));
    velocity = vec3Add(velocity, deltaV);
    if(isSleeping) wake();
  }

  void RigidBody::applyTorque(const fm_vec3_t &torque) {
    if(isKinematic || isTrigger) return;
    torqueAccumulator = vec3Add(torqueAccumulator, torque);
    if(isSleeping) wake();
  }

  void RigidBody::applyAngularImpulse(const fm_vec3_t &angImpulse) {
    if(isKinematic || isTrigger) return;
    angularVelocity = vec3Add(angularVelocity, applyWorldInertia(angImpulse));
    if(isSleeping) wake();
  }

  void RigidBody::setAngularVelocity(const fm_vec3_t &angVel) {
    angularVelocity = angVel;
    if(isSleeping) wake();
  }

  void RigidBody::applyForceAtPoint(const fm_vec3_t &force, const fm_vec3_t &worldPoint) {
    if(isKinematic || isTrigger) return;
    applyLinearImpulse(force);
    fm_vec3_t r = vec3Sub(worldPoint, worldCenterOfMass);
    applyAngularImpulse(vec3Cross(r, force));
  }

  void RigidBody::updateWorldInertia() {
    if(!rotation) {
      // Diagonal inertia with no rotation
      invWorldInertiaTensor = Matrix3x3{};
      invWorldInertiaTensor.m[0][0] = invLocalInertiaTensor.x;
      invWorldInertiaTensor.m[1][1] = invLocalInertiaTensor.y;
      invWorldInertiaTensor.m[2][2] = invLocalInertiaTensor.z;
      rotationMatrix = Matrix3x3::identity();
      worldCenterOfMass = position ? *position : vec3Zero();
      return;
    }

    rotationMatrix = quatToMatrix3(*rotation);

    // I_world_inv = R * I_local_inv * R^T
    Matrix3x3 localInv{};
    localInv.m[0][0] = invLocalInertiaTensor.x;
    localInv.m[1][1] = invLocalInertiaTensor.y;
    localInv.m[2][2] = invLocalInertiaTensor.z;

    Matrix3x3 rTranspose = matrix3Transpose(rotationMatrix);
    invWorldInertiaTensor = matrix3Mul(matrix3Mul(rotationMatrix, localInv), rTranspose);

    if(position) {
      worldCenterOfMass = vec3Add(*position, quatRotateVec(*rotation, centerOffset));
    }
  }

  void RigidBody::applyPositionConstraints() {
    if(!position) return;
    if(hasFlag(constraints, Constraint::FreezePosX)) position->x = prevStepPos.x;
    if(hasFlag(constraints, Constraint::FreezePosY)) position->y = prevStepPos.y;
    if(hasFlag(constraints, Constraint::FreezePosZ)) position->z = prevStepPos.z;
  }

} // namespace P64::CollNew
