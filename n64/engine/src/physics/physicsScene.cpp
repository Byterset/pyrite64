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
    constexpr float EPSILON = 0.0001f;
    
    // Calculate effective masses and prepare constraints
    for (int i = 0; i < cachedConstraintCount; i++) {
      auto& constraint = cachedConstraints[i];
      if (!constraint.isActive || constraint.isTrigger) continue;
      
      // Find physics bodies from objects
      PhysicsBody* bodyA = nullptr;
      PhysicsBody* bodyB = nullptr;
      
      for (auto* body : bodies) {
        if (body->object == constraint.objectA) bodyA = body;
        if (body->object == constraint.objectB) bodyB = body;
      }
      
      if (!bodyA && !bodyB) continue;
      
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
        
        // Update contact point positions relative to centers of mass
        if (bodyA) {
          point.aToContact = point.contactA - bodyA->object->pos;
        }
        if (bodyB) {
          point.bToContact = point.contactB - bodyB->object->pos;
        }
        
        float invMassA = (bodyA && !bodyA->isKinematic) ? bodyA->invMass : 0.0f;
        float invMassB = (bodyB && !bodyB->isKinematic) ? bodyB->invMass : 0.0f;
        
        // Effective mass for normal direction
        float denominator = invMassA + invMassB;
        
        // Add rotational inertia contribution (simplified - using unit inertia)
        if (bodyA && !bodyA->isKinematic) {
          fm_vec3_t rCrossN;
          t3d_vec3_cross(&rCrossN, &point.aToContact, &constraint.normal);
          denominator += t3d_vec3_dot(rCrossN, rCrossN);  // Simplified inertia
        }
        if (bodyB && !bodyB->isKinematic) {
          fm_vec3_t rCrossN;
          t3d_vec3_cross(&rCrossN, &point.bToContact, &constraint.normal);
          denominator += t3d_vec3_dot(rCrossN, rCrossN);
        }
        
        point.normalMass = (denominator > EPSILON) ? (1.0f / denominator) : 0.0f;
        
        // Effective mass for tangent U
        float denomU = invMassA + invMassB;
        if (bodyA && !bodyA->isKinematic) {
          fm_vec3_t rCrossT;
          t3d_vec3_cross(&rCrossT, &point.aToContact, &tangentU);
          denomU += t3d_vec3_dot(rCrossT, rCrossT);
        }
        if (bodyB && !bodyB->isKinematic) {
          fm_vec3_t rCrossT;
          t3d_vec3_cross(&rCrossT, &point.bToContact, &tangentU);
          denomU += t3d_vec3_dot(rCrossT, rCrossT);
        }
        point.tangentMassU = (denomU > EPSILON) ? (1.0f / denomU) : 0.0f;
        
        // Effective mass for tangent V
        float denomV = invMassA + invMassB;
        if (bodyA && !bodyA->isKinematic) {
          fm_vec3_t rCrossT;
          t3d_vec3_cross(&rCrossT, &point.aToContact, &tangentV);
          denomV += t3d_vec3_dot(rCrossT, rCrossT);
        }
        if (bodyB && !bodyB->isKinematic) {
          fm_vec3_t rCrossT;
          t3d_vec3_cross(&rCrossT, &point.bToContact, &tangentV);
          denomV += t3d_vec3_dot(rCrossT, rCrossT);
        }
        point.tangentMassV = (denomV > EPSILON) ? (1.0f / denomV) : 0.0f;
        
        // Calculate velocity bias for restitution
        fm_vec3_t relVel{0, 0, 0};
        if (bodyA && !bodyA->isKinematic) {
          relVel = bodyA->velocity;
          fm_vec3_t angularContrib;
          t3d_vec3_cross(&angularContrib, &bodyA->angularVelocity, &point.aToContact);
          relVel = relVel + angularContrib;
        }
        if (bodyB && !bodyB->isKinematic) {
          fm_vec3_t velB = bodyB->velocity;
          fm_vec3_t angularContrib;
          t3d_vec3_cross(&angularContrib, &bodyB->angularVelocity, &point.bToContact);
          velB = velB + angularContrib;
          relVel = relVel - velB;
        }
        
        float normalVel = t3d_vec3_dot(relVel, constraint.normal);
        point.velocityBias = 0.0f;
        
        // Apply restitution if separating
        if (normalVel < -1.0f) {  // Threshold for bouncing
          point.velocityBias = -constraint.combinedBounce * normalVel;
        }
      }
    }
  }
  
  void PhysicsScene::warmStart() {
    // Apply cached impulses from previous frame
    constexpr float WARM_START_FACTOR = 1.0f;  // Could reduce for more stability
    
    for (int i = 0; i < cachedConstraintCount; i++) {
      auto& constraint = cachedConstraints[i];
      if (!constraint.isActive || constraint.isTrigger) continue;
      
      // Find physics bodies
      PhysicsBody* bodyA = nullptr;
      PhysicsBody* bodyB = nullptr;
      
      for (auto* body : bodies) {
        if (body->object == constraint.objectA) bodyA = body;
        if (body->object == constraint.objectB) bodyB = body;
      }
      
      if (!bodyA && !bodyB) continue;
      
      for (int p = 0; p < constraint.pointCount; p++) {
        auto& point = constraint.points[p];
        if (!point.active) continue;
        
        // Apply cached normal impulse
        fm_vec3_t impulse = constraint.normal * (point.accumulatedNormalImpulse * WARM_START_FACTOR);
        
        if (bodyA && !bodyA->isKinematic) {
          bodyA->velocity = bodyA->velocity + impulse * bodyA->invMass;
          fm_vec3_t angImp;
          t3d_vec3_cross(&angImp, &point.aToContact, &impulse);
          bodyA->angularVelocity = bodyA->angularVelocity + angImp;
        }
        
        if (bodyB && !bodyB->isKinematic) {
          bodyB->velocity = bodyB->velocity - impulse * bodyB->invMass;
          fm_vec3_t angImp;
          t3d_vec3_cross(&angImp, &point.bToContact, &impulse);
          bodyB->angularVelocity = bodyB->angularVelocity - angImp;
        }
        
        // Apply cached tangent impulses
        fm_vec3_t tangentImpulseU = constraint.tangentU * (point.accumulatedTangentImpulseU * WARM_START_FACTOR);
        fm_vec3_t tangentImpulseV = constraint.tangentV * (point.accumulatedTangentImpulseV * WARM_START_FACTOR);
        
        if (bodyA && !bodyA->isKinematic) {
          bodyA->velocity = bodyA->velocity + (tangentImpulseU + tangentImpulseV) * bodyA->invMass;
        }
        
        if (bodyB && !bodyB->isKinematic) {
          bodyB->velocity = bodyB->velocity - (tangentImpulseU + tangentImpulseV) * bodyB->invMass;
        }
      }
    }
  }
  
  void PhysicsScene::solveVelocityConstraints() {
    constexpr float EPSILON = 0.0001f;
    
    for (int i = 0; i < cachedConstraintCount; i++) {
      auto& constraint = cachedConstraints[i];
      if (!constraint.isActive || constraint.isTrigger) continue;
      
      // Find physics bodies
      PhysicsBody* bodyA = nullptr;
      PhysicsBody* bodyB = nullptr;
      
      for (auto* body : bodies) {
        if (body->object == constraint.objectA) bodyA = body;
        if (body->object == constraint.objectB) bodyB = body;
      }
      
      if (!bodyA && !bodyB) continue;
      
      // Process each contact point
      for (int p = 0; p < constraint.pointCount; p++) {
        auto& point = constraint.points[p];
        if (!point.active) continue;
        
        // Calculate contact velocities
        fm_vec3_t contactVelA{0, 0, 0};
        fm_vec3_t contactVelB{0, 0, 0};
        
        if (bodyA && !bodyA->isKinematic) {
          contactVelA = bodyA->velocity;
          fm_vec3_t angularContrib;
          t3d_vec3_cross(&angularContrib, &bodyA->angularVelocity, &point.aToContact);
          contactVelA = contactVelA + angularContrib;
        }
        
        if (bodyB && !bodyB->isKinematic) {
          contactVelB = bodyB->velocity;
          fm_vec3_t angularContrib;
          t3d_vec3_cross(&angularContrib, &bodyB->angularVelocity, &point.bToContact);
          contactVelB = contactVelB + angularContrib;
        }
        
        // Calculate relative velocity
        fm_vec3_t relVel = contactVelA - contactVelB;
        float normalVelocity = t3d_vec3_dot(relVel, constraint.normal);
        
        // Calculate lambda (impulse change)
        float lambda = -(normalVelocity + point.velocityBias) * point.normalMass;
        
        // Clamp accumulated impulse (non-penetration constraint)
        float oldImpulse = point.accumulatedNormalImpulse;
        point.accumulatedNormalImpulse = fmaxf(oldImpulse + lambda, 0.0f);
        lambda = point.accumulatedNormalImpulse - oldImpulse;
        
        if (fabsf(lambda) < EPSILON) continue;
        
        // Apply impulse
        fm_vec3_t impulse = constraint.normal * lambda;
        
        // Apply to object A
        if (bodyA && !bodyA->isKinematic) {
          bodyA->velocity = bodyA->velocity + impulse * bodyA->invMass;
          
          fm_vec3_t angularImpulse;
          t3d_vec3_cross(&angularImpulse, &point.aToContact, &impulse);
          bodyA->angularVelocity = bodyA->angularVelocity + angularImpulse;  // Simplified
        }
        
        // Apply to object B
        if (bodyB && !bodyB->isKinematic) {
          bodyB->velocity = bodyB->velocity - impulse * bodyB->invMass;
          
          fm_vec3_t angularImpulse;
          t3d_vec3_cross(&angularImpulse, &point.bToContact, &impulse);
          bodyB->angularVelocity = bodyB->angularVelocity - angularImpulse;  // Simplified
        }
        
        // Handle friction
        if (constraint.combinedFriction > 0.0f) {
          // Recalculate relative velocity after normal impulse
          contactVelA = fm_vec3_t{0, 0, 0};
          contactVelB = fm_vec3_t{0, 0, 0};
          
          if (bodyA && !bodyA->isKinematic) {
            contactVelA = bodyA->velocity;
            fm_vec3_t angularContrib;
            t3d_vec3_cross(&angularContrib, &bodyA->angularVelocity, &point.aToContact);
            contactVelA = contactVelA + angularContrib;
          }
          
          if (bodyB && !bodyB->isKinematic) {
            contactVelB = bodyB->velocity;
            fm_vec3_t angularContrib;
            t3d_vec3_cross(&angularContrib, &bodyB->angularVelocity, &point.bToContact);
            contactVelB = contactVelB + angularContrib;
          }
          
          relVel = contactVelA - contactVelB;
          
          // Calculate tangential velocities
          float vTangentU = t3d_vec3_dot(relVel, constraint.tangentU);
          float vTangentV = t3d_vec3_dot(relVel, constraint.tangentV);
          
          // Calculate friction impulse changes
          float lambdaU = -vTangentU * point.tangentMassU;
          float lambdaV = -vTangentV * point.tangentMassV;
          
          // Calculate new accumulated tangent impulses
          float newAccumU = point.accumulatedTangentImpulseU + lambdaU;
          float newAccumV = point.accumulatedTangentImpulseV + lambdaV;
          
          // Clamp to friction cone
          float maxFriction = constraint.combinedFriction * point.accumulatedNormalImpulse;
          float tangentMagnitude = sqrtf(newAccumU * newAccumU + newAccumV * newAccumV);
          
          if (tangentMagnitude > maxFriction) {
            float scale = maxFriction / tangentMagnitude;
            newAccumU *= scale;
            newAccumV *= scale;
          }
          
          // Calculate actual impulse deltas
          lambdaU = newAccumU - point.accumulatedTangentImpulseU;
          lambdaV = newAccumV - point.accumulatedTangentImpulseV;
          
          point.accumulatedTangentImpulseU = newAccumU;
          point.accumulatedTangentImpulseV = newAccumV;
          
          // Apply tangent impulses
          if (fabsf(lambdaU) > EPSILON) {
            fm_vec3_t tangentImpulse = constraint.tangentU * lambdaU;
            
            if (bodyA && !bodyA->isKinematic) {
              bodyA->velocity = bodyA->velocity + tangentImpulse * bodyA->invMass;
              fm_vec3_t angImp;
              t3d_vec3_cross(&angImp, &point.aToContact, &tangentImpulse);
              bodyA->angularVelocity = bodyA->angularVelocity + angImp;
            }
            
            if (bodyB && !bodyB->isKinematic) {
              bodyB->velocity = bodyB->velocity - tangentImpulse * bodyB->invMass;
              fm_vec3_t angImp;
              t3d_vec3_cross(&angImp, &point.bToContact, &tangentImpulse);
              bodyB->angularVelocity = bodyB->angularVelocity - angImp;
            }
          }
          
          if (fabsf(lambdaV) > EPSILON) {
            fm_vec3_t tangentImpulse = constraint.tangentV * lambdaV;
            
            if (bodyA && !bodyA->isKinematic) {
              bodyA->velocity = bodyA->velocity + tangentImpulse * bodyA->invMass;
              fm_vec3_t angImp;
              t3d_vec3_cross(&angImp, &point.aToContact, &tangentImpulse);
              bodyA->angularVelocity = bodyA->angularVelocity + angImp;
            }
            
            if (bodyB && !bodyB->isKinematic) {
              bodyB->velocity = bodyB->velocity - tangentImpulse * bodyB->invMass;
              fm_vec3_t angImp;
              t3d_vec3_cross(&angImp, &point.bToContact, &tangentImpulse);
              bodyB->angularVelocity = bodyB->angularVelocity - angImp;
            }
          }
        }
      }
    }
  }
  
  void PhysicsScene::integratePositions(float deltaTime) {
    for (auto* body : bodies) {
      body->integratePosition(deltaTime);
    }
  }
  
  void PhysicsScene::solvePositionConstraints() {
    constexpr float SLOP = 0.01f;  // Allow small penetration
    constexpr float BAUMGARTE = 0.2f;  // Position correction factor
    constexpr float EPSILON = 0.0001f;
    
    // Iteratively solve position constraints
    for (int i = 0; i < cachedConstraintCount; i++) {
      auto& constraint = cachedConstraints[i];
      if (!constraint.isActive || constraint.isTrigger) continue;
      
      // Find physics bodies
      PhysicsBody* bodyA = nullptr;
      PhysicsBody* bodyB = nullptr;
      
      for (auto* body : bodies) {
        if (body->object == constraint.objectA) bodyA = body;
        if (body->object == constraint.objectB) bodyB = body;
      }
      
      if (!bodyA && !bodyB) continue;
      
      for (int p = 0; p < constraint.pointCount; p++) {
        auto& point = constraint.points[p];
        if (!point.active) continue;
        
        // Update contact positions
        if (bodyA) {
          point.aToContact = point.contactA - bodyA->object->pos;
        }
        if (bodyB) {
          point.bToContact = point.contactB - bodyB->object->pos;
        }
        
        // Calculate penetration
        float penetration = point.penetration;
        if (penetration <= SLOP) continue;
        
        // Calculate correction
        float correction = -(penetration - SLOP) * BAUMGARTE;
        
        // Calculate effective mass
        float invMassA = (bodyA && !bodyA->isKinematic) ? bodyA->invMass : 0.0f;
        float invMassB = (bodyB && !bodyB->isKinematic) ? bodyB->invMass : 0.0f;
        
        float invMassSum = invMassA + invMassB;
        
        // Add rotational inertia (simplified)
        if (bodyA && !bodyA->isKinematic) {
          fm_vec3_t rCrossN;
          t3d_vec3_cross(&rCrossN, &point.aToContact, &constraint.normal);
          invMassSum += t3d_vec3_dot(rCrossN, rCrossN);
        }
        if (bodyB && !bodyB->isKinematic) {
          fm_vec3_t rCrossN;
          t3d_vec3_cross(&rCrossN, &point.bToContact, &constraint.normal);
          invMassSum += t3d_vec3_dot(rCrossN, rCrossN);
        }
        
        if (invMassSum < EPSILON) continue;
        
        float correctionMag = correction / invMassSum;
        fm_vec3_t correctionImpulse = constraint.normal * correctionMag;
        
        // Apply position correction
        if (bodyA && !bodyA->isKinematic) {
          bodyA->object->pos = bodyA->object->pos + correctionImpulse * invMassA;
          
          // Apply angular correction (simplified)
          fm_vec3_t angularImpulse;
          t3d_vec3_cross(&angularImpulse, &point.aToContact, &correctionImpulse);
          
          float angle = fm_vec3_len(&angularImpulse) * invMassA;
          if (angle > EPSILON) {
            fm_vec3_t axis = angularImpulse / fm_vec3_len(&angularImpulse);
            fm_quat_t deltaRot;
            fm_quat_from_axis_angle(&deltaRot, &axis, angle);
            fm_quat_mul(&bodyA->object->rot, &deltaRot, &bodyA->object->rot);
            fm_quat_norm(&bodyA->object->rot);
          }
        }
        
        if (bodyB && !bodyB->isKinematic) {
          bodyB->object->pos = bodyB->object->pos - correctionImpulse * invMassB;
          
          // Apply angular correction (simplified)
          fm_vec3_t angularImpulse;
          t3d_vec3_cross(&angularImpulse, &point.bToContact, &correctionImpulse);
          
          float angle = fm_vec3_len(&angularImpulse) * invMassB;
          if (angle > EPSILON) {
            fm_vec3_t axis = angularImpulse / fm_vec3_len(&angularImpulse);
            fm_quat_t deltaRot;
            fm_quat_from_axis_angle(&deltaRot, &axis, -angle);
            fm_quat_mul(&bodyB->object->rot, &deltaRot, &bodyB->object->rot);
            fm_quat_norm(&bodyB->object->rot);
          }
        }
        
        // Reduce penetration
        point.penetration -= fabsf(correction);
      }
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
