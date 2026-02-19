/**
 * @copyright 2026 - Max Bebök
 * @license MIT
 */
#include "physics/physicsBody.h"
#include "scene/object.h"

namespace P64::Physics
{
  void PhysicsBody::init(Object* obj, float bodyMass) {
    object = obj;
    mass = bodyMass;
    invMass = (mass > 0.0f) ? (1.0f / mass) : 0.0f;
    velocity = fm_vec3_t{0, 0, 0};
    angularVelocity = fm_vec3_t{0, 0, 0};
    acceleration = fm_vec3_t{0, 0, 0};
  }
  
  void PhysicsBody::addShape(const ColliderShape& shape) {
    shapes.push_back(shape);
  }
  
  void PhysicsBody::clearShapes() {
    shapes.clear();
  }
  
  void PhysicsBody::getWorldAABB(fm_vec3_t& outMin, fm_vec3_t& outMax) const {
    if (shapes.empty()) {
      outMin = object->pos;
      outMax = object->pos;
      return;
    }
    
    // Initialize with first shape
    fm_vec3_t localMin = shapes[0].getLocalAABBMin();
    fm_vec3_t localMax = shapes[0].getLocalAABBMax();
    
    // Transform to world space (simplified - assumes axis-aligned for now)
    outMin = object->pos + (object->rot * localMin) * object->scale;
    outMax = object->pos + (object->rot * localMax) * object->scale;
    
    // Expand for remaining shapes
    for (size_t i = 1; i < shapes.size(); i++) {
      localMin = shapes[i].getLocalAABBMin();
      localMax = shapes[i].getLocalAABBMax();
      
      fm_vec3_t worldMin = object->pos + (object->rot * localMin) * object->scale;
      fm_vec3_t worldMax = object->pos + (object->rot * localMax) * object->scale;
      
      outMin.x = fminf(outMin.x, worldMin.x);
      outMin.y = fminf(outMin.y, worldMin.y);
      outMin.z = fminf(outMin.z, worldMin.z);
      
      outMax.x = fmaxf(outMax.x, worldMax.x);
      outMax.y = fmaxf(outMax.y, worldMax.y);
      outMax.z = fmaxf(outMax.z, worldMax.z);
    }
  }
  
  void PhysicsBody::applyLinearImpulse(const fm_vec3_t& impulse) {
    if (isKinematic || invMass == 0.0f) return;
    velocity = velocity + impulse * invMass;
  }
  
  void PhysicsBody::applyAngularImpulse(const fm_vec3_t& impulse) {
    if (isKinematic) return;
    // Simplified: assume identity inertia tensor for now
    angularVelocity = angularVelocity + impulse;
  }
  
  void PhysicsBody::integrateVelocity(float deltaTime) {
    if (isKinematic || invMass == 0.0f) return;
    
    // Semi-implicit Euler: velocity first
    velocity = velocity + acceleration * deltaTime;
    
    // Apply damping
    velocity = velocity * linearDamping;
    angularVelocity = angularVelocity * angularDamping;
    
    // Clear acceleration for next frame
    acceleration = fm_vec3_t{0, 0, 0};
  }
  
  void PhysicsBody::integratePosition(float deltaTime) {
    if (isKinematic) return;
    
    // Position from velocity
    object->pos = object->pos + velocity * deltaTime;
    
    // Rotation from angular velocity (simplified)
    if (fm_vec3_len2(&angularVelocity) > 0.0001f) {
      float angle = fm_vec3_len(&angularVelocity) * deltaTime;
      fm_vec3_t axis = angularVelocity / fm_vec3_len(&angularVelocity);
      
      fm_quat_t deltaRot;
      fm_quat_from_axis_angle(&deltaRot, &axis, angle);
      fm_quat_mul(&object->rot, &deltaRot, &object->rot);
      fm_quat_norm(&object->rot);
    }
  }
}
