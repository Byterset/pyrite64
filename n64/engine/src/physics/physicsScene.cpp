/**
 * @copyright 2026 - Max Bebök
 * @license MIT
 */
#include "physics/physicsScene.h"
#include "physics/gjk.h"
#include "physics/epa.h"
#include "scene/object.h"
#include <algorithm>
#include <cstring>

namespace P64::Physics
{
  PhysicsScene::PhysicsScene() {
    cachedConstraintCount = 0;
    std::memset(cachedConstraints, 0, sizeof(cachedConstraints));
  }
  
  PhysicsScene::~PhysicsScene() {
    bodies.clear();
  }
  
  void PhysicsScene::registerBody(PhysicsBody* body) {
    if (body && std::find(bodies.begin(), bodies.end(), body) == bodies.end()) {
      bodies.push_back(body);
    }
  }
  
  void PhysicsScene::unregisterBody(PhysicsBody* body) {
    auto it = std::find(bodies.begin(), bodies.end(), body);
    if (it != bodies.end()) {
      bodies.erase(it);
    }
  }
  
  void PhysicsScene::step(float deltaTime) {
    // Phase 1: Apply gravity and integrate velocities
    applyGravityAndIntegrateVelocity(deltaTime);
    
    // Phase 2: Detect all contacts
    detectAllContacts();
    
    // Phase 3: Pre-solve contacts
    preSolveContacts();
    
    // Phase 4: Warm start
    warmStart();
    
    // Phase 5: Solve velocity constraints
    for (int i = 0; i < VELOCITY_SOLVER_ITERATIONS; i++) {
      solveVelocityConstraints();
    }
    
    // Phase 6: Integrate positions
    integratePositions(deltaTime);
    
    // Phase 7: Solve position constraints
    for (int i = 0; i < POSITION_SOLVER_ITERATIONS; i++) {
      solvePositionConstraints();
    }
    
    // Phase 8: Update sleep states (placeholder)
    updateSleepStates();
  }
  
  void PhysicsScene::applyGravityAndIntegrateVelocity(float deltaTime) {
    for (auto* body : bodies) {
      if (!body->isKinematic && body->hasGravity) {
        body->acceleration.y += GRAVITY;
      }
      
      body->integrateVelocity(deltaTime);
    }
  }
  
  void PhysicsScene::detectAllContacts() {
    // Mark all cached constraints as inactive
    for (int i = 0; i < cachedConstraintCount; i++) {
      cachedConstraints[i].isActive = false;
      for (int p = 0; p < cachedConstraints[i].pointCount; p++) {
        cachedConstraints[i].points[p].active = false;
      }
    }
    
    // Broadphase: check all pairs (O(n²) - could optimize with BVH)
    for (size_t i = 0; i < bodies.size(); i++) {
      for (size_t j = i + 1; j < bodies.size(); j++) {
        auto* bodyA = bodies[i];
        auto* bodyB = bodies[j];
        
        // Check collision masks
        if ((bodyA->maskRead & bodyB->maskWrite) == 0 &&
            (bodyB->maskRead & bodyA->maskWrite) == 0) {
          continue;
        }
        
        detectBodyVsBody(bodyA, bodyB);
      }
    }
    
    // Remove inactive constraints (compact array)
    int writeIdx = 0;
    for (int readIdx = 0; readIdx < cachedConstraintCount; readIdx++) {
      if (cachedConstraints[readIdx].isActive) {
        if (writeIdx != readIdx) {
          cachedConstraints[writeIdx] = cachedConstraints[readIdx];
        }
        writeIdx++;
      }
    }
    cachedConstraintCount = writeIdx;
    
    // Rebuild contact map
    contactMap.clear();
    for (int i = 0; i < cachedConstraintCount; i++) {
      contactMap[cachedConstraints[i].pairId] = i;
    }
  }
  
  void PhysicsScene::detectBodyVsBody(PhysicsBody* bodyA, PhysicsBody* bodyB) {
    // Check each shape pair
    for (const auto& shapeA : bodyA->shapes) {
      for (const auto& shapeB : bodyB->shapes) {
        // Transform shapes to world space
        fm_vec3_t posA = bodyA->object->pos + (bodyA->object->rot * shapeA.localOffset);
        fm_vec3_t posB = bodyB->object->pos + (bodyB->object->rot * shapeB.localOffset);
        
        fm_vec3_t normal;
        float penetration;
        fm_vec3_t contactA, contactB;
        
        if (gjkCheckOverlap(shapeA, posA, bodyA->object->rot,
                           shapeB, posB, bodyB->object->rot,
                           normal, penetration, contactA, contactB)) {
          // Cache contact constraint
          auto* constraint = findOrCreateContactConstraint(bodyA->object, bodyB->object);
          if (constraint) {
            constraint->isActive = true;
            constraint->normal = normal;
            constraint->combinedFriction = sqrtf(bodyA->friction * bodyB->friction);
            constraint->combinedBounce = fmaxf(bodyA->bounce, bodyB->bounce);
            constraint->isTrigger = bodyA->isTrigger || bodyB->isTrigger;
            
            // Add contact point (simplified - single point for now)
            if (constraint->pointCount < MAX_CONTACT_POINTS_PER_PAIR) {
              auto& point = constraint->points[constraint->pointCount];
              point.contactA = contactA;
              point.contactB = contactB;
              point.point = (contactA + contactB) * 0.5f;
              point.penetration = penetration;
              point.active = true;
              
              point.aToContact = contactA - bodyA->object->pos;
              point.bToContact = contactB - bodyB->object->pos;
              
              constraint->pointCount++;
            }
          }
        }
      }
    }
  }
  
  ContactConstraint* PhysicsScene::findOrCreateContactConstraint(Object* objA, Object* objB) {
    ContactPairId pairId = makeContactPairId(objA->id, objB->id);
    
    auto it = contactMap.find(pairId);
    if (it != contactMap.end()) {
      return &cachedConstraints[it->second];
    }
    
    // Create new constraint
    if (cachedConstraintCount >= MAX_CACHED_CONTACTS) {
      return nullptr;  // Out of space
    }
    
    int idx = cachedConstraintCount++;
    auto* constraint = &cachedConstraints[idx];
    
    // Ensure consistent ordering
    if (objA->id < objB->id) {
      constraint->objectA = objA;
      constraint->objectB = objB;
    } else {
      constraint->objectA = objB;
      constraint->objectB = objA;
    }
    
    constraint->pairId = pairId;
    constraint->pointCount = 0;
    constraint->isActive = true;
    
    contactMap[pairId] = idx;
    
    return constraint;
  }
  
  bool PhysicsScene::gjkCheckOverlap(const ColliderShape& shapeA, const fm_vec3_t& posA, const fm_quat_t& rotA,
                                     const ColliderShape& shapeB, const fm_vec3_t& posB, const fm_quat_t& rotB,
                                     fm_vec3_t& outNormal, float& outPenetration,
                                     fm_vec3_t& outContactA, fm_vec3_t& outContactB) {
    // Create support function wrappers
    struct ShapeData {
      const ColliderShape* shape;
      fm_vec3_t pos;
      fm_quat_t rot;
    };
    
    ShapeData dataA = {&shapeA, posA, rotA};
    ShapeData dataB = {&shapeB, posB, rotB};
    
    auto supportFunc = [](const void* data, const fm_vec3_t& dir) -> fm_vec3_t {
      const ShapeData* sd = static_cast<const ShapeData*>(data);
      // Transform direction to local space
      fm_quat_t invRot;
      fm_quat_inverse(&invRot, &sd->rot);
      fm_vec3_t localDir = invRot * dir;
      
      // Get support point in local space
      fm_vec3_t localSupport = sd->shape->support(localDir);
      
      // Transform back to world space
      return sd->pos + (sd->rot * localSupport);
    };
    
    // Initial direction
    fm_vec3_t initialDir = posB - posA;
    if (fm_vec3_len2(&initialDir) < 0.0001f) {
      initialDir = fm_vec3_t{1, 0, 0};
    }
    
    Simplex simplex;
    if (!gjkCheckOverlap(simplex, &dataA, supportFunc, &dataB, supportFunc, initialDir)) {
      return false;  // No collision
    }
    
    // Run EPA to get penetration info
    EpaResult epaResult;
    if (!epaSolve(simplex, &dataA, supportFunc, &dataB, supportFunc, epaResult)) {
      return false;
    }
    
    outNormal = epaResult.normal;
    outPenetration = epaResult.penetration;
    outContactA = epaResult.contactA;
    outContactB = epaResult.contactB;
    
    return true;
  }
  
  void PhysicsScene::preSolveContacts() {
    // Calculate effective masses and prepare constraints
    for (int i = 0; i < cachedConstraintCount; i++) {
      auto& constraint = cachedConstraints[i];
      if (!constraint.isActive || constraint.isTrigger) continue;
      
      // Get physics bodies (placeholder - would need lookup)
      // This would require storing PhysicsBody* in Object or component
      
      // Calculate tangent vectors
      fm_vec3_t tangentU, tangentV;
      if (fabsf(constraint.normal.x) < 0.9f) {
        tangentU = fm_vec3_t{1, 0, 0};
      } else {
        tangentU = fm_vec3_t{0, 1, 0};
      }
      t3d_vec3_cross(&tangentU, &constraint.normal, &tangentU);
      fm_vec3_norm(&tangentU);
      t3d_vec3_cross(&tangentV, &constraint.normal, &tangentU);
      
      constraint.tangentU = tangentU;
      constraint.tangentV = tangentV;
      
      // Calculate effective masses for each contact point
      for (int p = 0; p < constraint.pointCount; p++) {
        auto& point = constraint.points[p];
        if (!point.active) continue;
        
        // Simplified effective mass calculation
        // Full implementation would include inertia tensor
        point.normalMass = 1.0f;  // Placeholder
        point.tangentMassU = 1.0f;
        point.tangentMassV = 1.0f;
        point.velocityBias = 0.0f;  // Would calculate from relative velocity
      }
    }
  }
  
  void PhysicsScene::warmStart() {
    // Apply cached impulses from previous frame
    // This helps convergence by starting with last frame's solution
    // Implementation would multiply accumulated impulses by a warm-start factor
  }
  
  void PhysicsScene::solveVelocityConstraints() {
    // Iteratively solve velocity constraints
    // This would apply impulses to correct relative velocities at contact points
    // Full implementation requires access to PhysicsBody from Object
  }
  
  void PhysicsScene::integratePositions(float deltaTime) {
    for (auto* body : bodies) {
      body->integratePosition(deltaTime);
    }
  }
  
  void PhysicsScene::solvePositionConstraints() {
    // Iteratively solve position constraints
    // This directly adjusts positions to resolve penetration
    for (int i = 0; i < cachedConstraintCount; i++) {
      auto& constraint = cachedConstraints[i];
      if (!constraint.isActive || constraint.isTrigger) continue;
      
      // Would apply position corrections here
      // Similar to velocity solver but modifies positions directly
    }
  }
  
  void PhysicsScene::updateSleepStates() {
    // Update sleep/wake states based on velocity and motion
    // Objects at rest can be put to sleep to save computation
  }
  
  void PhysicsScene::updateWorldInertia() {
    // Update world-space inertia tensors from rotation
    // Needed for accurate torque/angular impulse calculations
  }
}
