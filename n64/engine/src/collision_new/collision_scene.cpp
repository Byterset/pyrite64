/**
 * @copyright 2024 - Max Bebök
 * @license MIT
 */
#include "collision_new/collision_scene.h"
#include "collision_new/collide.h"
#include "collision_new/gjk.h"

#include <cmath>
#include <cassert>
#include <functional>
#include <algorithm>
#include <utility>

#include "debug/debugDraw.h"

namespace P64::CollNew {

  static CollisionScene g_scene;

  std::pair<Collider *, Collider *> CollisionScene::makeColliderPairKey(Collider *a, Collider *b) {
    return (a < b) ? std::make_pair(a, b)
                   : std::make_pair(b, a);
  }

  void CollisionScene::rebuildCachedConstraintPairs() {
    cachedConstraintPairs_.clear();
    for(int i = 0; i < cachedConstraintCount_; ++i) {
      ContactConstraint &cc = cachedConstraints_[i];
      std::pair<Collider *, Collider *> key = makeColliderPairKey(cc.colliderA, cc.colliderB);
      cachedConstraintPairs_[key].push_back(i);
    }
  }

  CollisionScene *collisionSceneGetInstance() {
    return &g_scene;
  }

  // ── Reset / Init ──────────────────────────────────────────────────

  void CollisionScene::reset() {
    rigidBodyAABBTree.destroy();

    rigidBodies_.clear();
    ownerRigidBodies_.clear();
    colliders_.clear();
    ownerColliders_.clear();
    meshColliders_.clear();
    cachedConstraintCount_ = 0;
    cachedConstraints_.clear();
    cachedConstraintPairs_.clear();

    rigidBodyAABBTree.init(32); // Initial capacity (will grow as needed)
  }

  /// @brief Updates the world state of a collider.
  /// Recalculates the world center and world AABB of the collider based on its owner's transform.
  /// @param collider The collider to update.
  void CollisionScene::updateColliderWorldState(Collider *collider) const {
    if(!collider || !collider->owner) return;
    const Object *owner = collider->owner;
    collider->worldCenter = owner->pos + (owner->rot * (collider->parentOffset * owner->scale));

    const AABB local = collider->boundingBox(&owner->rot);
    collider->worldAABB.min = vec3Add(local.min, collider->worldCenter);
    collider->worldAABB.max = vec3Add(local.max, collider->worldCenter);
  }

  RigidBody *CollisionScene::findRigidBodyByOwner(const Object *owner) const {
    if(!owner) return nullptr;
    auto it = ownerRigidBodies_.find(owner);
    return (it != ownerRigidBodies_.end()) ? it->second : nullptr;
  }

  const std::vector<Collider *> *CollisionScene::findCollidersForOwner(const Object *owner) const {
    if(!owner) return nullptr;
    auto it = ownerColliders_.find(owner);
    if(it == ownerColliders_.end()) return nullptr;
    return &it->second;
  }

  void CollisionScene::updateCompoundProperties(RigidBody *rigidBody) const {
    if(!rigidBody || !rigidBody->owner || !rigidBody->position) return;

    const std::vector<Collider *> *ownerColliders = findCollidersForOwner(rigidBody->owner);
    if(!ownerColliders || ownerColliders->empty()) {
      rigidBody->centerOffset = vec3Zero();
      return;
    }

    int count = 0;
    fm_vec3_t worldCenterSum = vec3Zero();
    for(Collider *collider : *ownerColliders) {
      if(!collider) continue;
      worldCenterSum = vec3Add(worldCenterSum, collider->worldCenter);
      ++count;
    }

    if(count <= 0) {
      rigidBody->centerOffset = vec3Zero();
      return;
    }

    const float invCount = 1.0f / static_cast<float>(count);
    const fm_vec3_t worldCenter = vec3Scale(worldCenterSum, invCount);

    fm_vec3_t localCenterOffset = vec3Sub(worldCenter, *rigidBody->position);
    if(rigidBody->rotation) {
      localCenterOffset = quatRotateVec(quatConjugate(*rigidBody->rotation), localCenterOffset);
    }
    rigidBody->centerOffset = localCenterOffset;

    if(rigidBody->mass <= EPSILON) {
      return;
    }

    const float massPerCollider = rigidBody->mass * invCount;
    fm_vec3_t compoundInertia = vec3Zero();

    for(Collider *collider : *ownerColliders) {
      if(!collider) continue;

      fm_vec3_t colliderInertia = collider->inertiaTensor(massPerCollider);
      fm_vec3_t r = vec3Sub(collider->worldCenter, worldCenter);
      if(rigidBody->rotation) {
        r = quatRotateVec(quatConjugate(*rigidBody->rotation), r);
      }

      const float x2 = r.x * r.x;
      const float y2 = r.y * r.y;
      const float z2 = r.z * r.z;

      colliderInertia.x += massPerCollider * (y2 + z2);
      colliderInertia.y += massPerCollider * (x2 + z2);
      colliderInertia.z += massPerCollider * (x2 + y2);

      compoundInertia = vec3Add(compoundInertia, colliderInertia);
    }

    rigidBody->localInertiaTensor = compoundInertia;
    rigidBody->invLocalInertiaTensor = vec3(
      compoundInertia.x > EPSILON ? 1.0f / compoundInertia.x : 0.0f,
      compoundInertia.y > EPSILON ? 1.0f / compoundInertia.y : 0.0f,
      compoundInertia.z > EPSILON ? 1.0f / compoundInertia.z : 0.0f
    );
  }

  // ── Object management ─────────────────────────────────────────────

  void CollisionScene::addRigidBody(RigidBody *rigidBody) {
    if(!rigidBody || !rigidBody->owner) return;
    rigidBodies_.push_back(rigidBody);
    ownerRigidBodies_[rigidBody->owner] = rigidBody;
    

    updateCompoundProperties(rigidBody);
    rigidBody->updateWorldInertia();
    const fm_vec3_t worldPos = rigidBody->position ? *rigidBody->position : vec3Zero();
    rigidBody->boundingBox = AABB{worldPos, worldPos};
    rigidBody->aabbTreeNodeId = rigidBodyAABBTree.createNode(rigidBody->boundingBox, rigidBody);
  }

  void CollisionScene::removeRigidBody(RigidBody *rigidBody) {
    if(!rigidBody) return;

    if(rigidBody->owner) {
      auto ownerIt = ownerRigidBodies_.find(rigidBody->owner);
      if(ownerIt != ownerRigidBodies_.end() && ownerIt->second == rigidBody) {
        ownerRigidBodies_.erase(ownerIt);
      }
    }

    // Release contacts
    releaseObjectContacts(rigidBody);

    // Remove from AABB tree
    if(rigidBody->aabbTreeNodeId != NULL_NODE) {
      rigidBodyAABBTree.removeLeaf(rigidBody->aabbTreeNodeId, true);
      rigidBody->aabbTreeNodeId = NULL_NODE;
    }

    // Remove cached constraints referencing this rigidBody
    bool removedAny = false;
    for(int i = 0; i < cachedConstraintCount_;) {
      ContactConstraint &cc = cachedConstraints_[i];
      if(cc.rigidBodyA == rigidBody || cc.rigidBodyB == rigidBody) {
        int lastIndex = cachedConstraintCount_ - 1;
        if(i != lastIndex) {
          cachedConstraints_[i] = cachedConstraints_[lastIndex];
        }
        cachedConstraints_.pop_back();
        cachedConstraintCount_--;
        removedAny = true;
      } else {
        ++i;
      }
    }
    if(removedAny) {
      rebuildCachedConstraintPairs();
    }

    rigidBodies_.erase(std::remove(rigidBodies_.begin(), rigidBodies_.end(), rigidBody), rigidBodies_.end());
  }

  RigidBody *CollisionScene::findRigidBodyByObjectId(uint16_t id) const
  {
    for (RigidBody *body : rigidBodies_)
    {
      if (body && body->owner && body->owner->id == id)
      {
        return body;
      }
    }
    return nullptr;
  }

  void CollisionScene::addCollider(Collider *collider) {
    if(!collider || !collider->owner) return;
    colliders_.push_back(collider);
    ownerColliders_[collider->owner].push_back(collider);
    updateColliderWorldState(collider);

    RigidBody *rigidBody = findRigidBodyByOwner(collider->owner);
    if (rigidBody)
    {
      updateCompoundProperties(rigidBody);
      rigidBody->updateWorldInertia();
    }
  }

  void CollisionScene::removeCollider(Collider *collider) {
    if(!collider) return;
    Object *owner = collider->owner;

    bool removedAny = false;
    for(int i = 0; i < cachedConstraintCount_;) {
      ContactConstraint &cc = cachedConstraints_[i];
      if(cc.colliderA == collider || cc.colliderB == collider) {
        int lastIndex = cachedConstraintCount_ - 1;
        if(i != lastIndex) {
          cachedConstraints_[i] = cachedConstraints_[lastIndex];
        }
        cachedConstraints_.pop_back();
        cachedConstraintCount_--;
        removedAny = true;
      } else {
        ++i;
      }
    }
    if(removedAny) {
      rebuildCachedConstraintPairs();
    }

    if(owner) {
      auto it = ownerColliders_.find(owner);
      if(it != ownerColliders_.end()) {
        std::vector<Collider *> &ownerList = it->second;
        ownerList.erase(std::remove(ownerList.begin(), ownerList.end(), collider), ownerList.end());
        if(ownerList.empty()) {
          ownerColliders_.erase(it);
        }
      }
    }

    colliders_.erase(std::remove(colliders_.begin(), colliders_.end(), collider), colliders_.end());

    if(owner) {
      RigidBody *rigidBody = findRigidBodyByOwner(owner);
      if(rigidBody) {
        updateCompoundProperties(rigidBody);
        rigidBody->updateWorldInertia();
      }
    }
  }

  void CollisionScene::addMeshCollider(MeshCollider *mesh) {
    if(!mesh) return;

    mesh->computeLocalRootAABB();
    mesh->recalculateWorldAABB();

    meshColliders_.push_back(mesh);
  }

  void CollisionScene::removeMeshCollider(MeshCollider *mesh) {
    if(!mesh) return;

    // Remove cached constraints referencing this mesh collider
    bool removedAny = false;
    for(int i = 0; i < cachedConstraintCount_;) {
      ContactConstraint &cc = cachedConstraints_[i];
      if(cc.objectB == mesh->owner) {
        // Release contacts from the rigid body's linked list for this constraint
        // (activeContacts may reference this constraint)
        int lastIndex = cachedConstraintCount_ - 1;
        if(i != lastIndex) {
          cachedConstraints_[i] = cachedConstraints_[lastIndex];
        }
        cachedConstraints_.pop_back();
        cachedConstraintCount_--;
        removedAny = true;
      } else {
        ++i;
      }
    }
    if(removedAny) {
      rebuildCachedConstraintPairs();
    }

    for(std::size_t i = 0; i < meshColliders_.size(); ++i) {
      if(meshColliders_[i] == mesh) {
        meshColliders_[i] = meshColliders_.back();
        meshColliders_.pop_back();
        break;
      }
    }
  }

  void CollisionScene::configureSimulation(float fixedDt, const fm_vec3_t &gravity, uint8_t velocityIterations, uint8_t positionIterations) {
    fixedDt_ = fixedDt > 0.0f ? fixedDt : DEFAULT_FIXED_DT;
    gravity_ = gravity;
    velocitySolverIterations_ = std::max<uint8_t>(1, velocityIterations);
    positionSolverIterations_ = std::max<uint8_t>(1, positionIterations);
  }

  int CollisionScene::getCachedConstraintCount() const {
    return cachedConstraintCount_;
  }

  ContactConstraint &CollisionScene::getCachedConstraint(int index) {
    assert(index >= 0 && index < cachedConstraintCount_);
    return cachedConstraints_[index];
  }

  const ContactConstraint &CollisionScene::getCachedConstraint(int index) const {
    assert(index >= 0 && index < cachedConstraintCount_);
    return cachedConstraints_[index];
  }

  ContactConstraint *CollisionScene::createCachedConstraint(
    RigidBody *rigidBodyA, Collider *colliderA, Object *objectA,
    RigidBody *rigidBodyB, Collider *colliderB, Object *objectB) {
    cachedConstraints_.push_back(ContactConstraint{});
    cachedConstraintCount_ = static_cast<int>(cachedConstraints_.size());
    ContactConstraint &cc = cachedConstraints_.back();
    cc.rigidBodyA = rigidBodyA;
    cc.colliderA = colliderA;
    cc.objectA = objectA;
    cc.rigidBodyB = rigidBodyB;
    cc.colliderB = colliderB;
    cc.objectB = objectB;

    std::pair<Collider *, Collider *> key = makeColliderPairKey(colliderA, colliderB);
    cachedConstraintPairs_[key].push_back(cachedConstraintCount_ - 1);
    return &cc;
  }

  ContactConstraint *CollisionScene::findCachedConstraintByPair(
    Collider *colliderA, Collider *colliderB,
    const fm_vec3_t &normal, float minNormalDot) {

    std::pair<Collider *, Collider *> key = makeColliderPairKey(colliderA, colliderB);
    auto it = cachedConstraintPairs_.find(key);
    if(it == cachedConstraintPairs_.end()) return nullptr;

    for(int idx : it->second) {
      ContactConstraint &cc = cachedConstraints_[idx];
      if(vec3Dot(cc.normal, normal) > minNormalDot) {
        return &cc;
      }
    }

    return nullptr;
  }

  void CollisionScene::releaseObjectContacts(RigidBody *rigidBody) {
    if(!rigidBody) return;
    for(Contact &c : rigidBody->activeContacts) {
      c.constraint = nullptr;
      c.otherBody = nullptr;
    }
    rigidBody->activeContacts.clear();
  }

  // ── Wake island ───────────────────────────────────────────────────

  void CollisionScene::wakeIsland(RigidBody *rigidBody) {
    if(!rigidBody || !rigidBody->isSleeping) return;
    rigidBody->wake();

    // Wake connected rigidBodies through contacts
    for(const Contact &c : rigidBody->activeContacts) {
      if(c.otherBody && c.otherBody->isSleeping) {
        wakeIsland(c.otherBody);
      }
    }
  }

  // ── Contact refresh ───────────────────────────────────────────────

  void CollisionScene::refreshContacts() {
    for(int i = 0; i < cachedConstraintCount_; ++i) {
      ContactConstraint &cc = cachedConstraints_[i];
      if(!cc.isActive) continue;

      RigidBody *a = cc.rigidBodyA;
      RigidBody *b = cc.rigidBodyB;

      for(int j = 0; j < cc.pointCount; ++j) {
        ContactPoint &cp = cc.points[j];
        if(!cp.active) continue;

        // Recompute world-space contact points from local coordinates
        if(a) {
          fm_vec3_t rA = cp.localPointA;
          if(a->rotation) {
            rA = quatRotateVec(*a->rotation, rA);
          }
          cp.contactA = vec3Add(*a->position, rA);
        }
        if(b) {
          fm_vec3_t rB = cp.localPointB;
          if(b->rotation) {
            rB = quatRotateVec(*b->rotation, rB);
          }
          cp.contactB = vec3Add(*b->position, rB);
        }

        // Update penetration: pen = -dot(A - B, normal) = dot(B - A, normal)
        fm_vec3_t diff = vec3Sub(cp.contactA, cp.contactB);
        cp.penetration = -vec3Dot(diff, cc.normal);

        // Deactivate if too separated
        if(cp.penetration < -0.1f) {
          cp.active = false;
        }
      }

      // Compact: prune inactive points to free slots for new contacts
      int writeIdx = 0;
      for(int j = 0; j < cc.pointCount; ++j) {
        if(cc.points[j].active) {
          if(writeIdx != j) {
            cc.points[writeIdx] = cc.points[j];
          }
          writeIdx++;
        }
      }
      cc.pointCount = writeIdx;
    }
  }

  void CollisionScene::removeInactiveContacts() {
    bool removedAny = false;
    for(int i = 0; i < cachedConstraintCount_;) {
      ContactConstraint &cc = cachedConstraints_[i];
      if(!cc.isActive) {
        int lastIndex = cachedConstraintCount_ - 1;
        if(i != lastIndex) {
          cachedConstraints_[i] = cachedConstraints_[lastIndex];
        }
        cachedConstraints_.pop_back();
        cachedConstraintCount_--;
        removedAny = true;
      } else {
        ++i;
      }
    }
    if(removedAny) {
      rebuildCachedConstraintPairs();
    }
  }

  // ── Contact detection ─────────────────────────────────────────────

  void CollisionScene::detectAllContacts() {
    // Mark all constraints as inactive; detection will re-activate them
    for(int i = 0; i < cachedConstraintCount_; ++i) {
      cachedConstraints_[i].isActive = false;
    }

    // Collider-to-collider broad phase using collider world AABBs.
    for(std::size_t i = 0; i < colliders_.size(); ++i) {
      Collider *a = colliders_[i];
      if(!a || !a->owner) continue;

      updateColliderWorldState(a);
      RigidBody *rbA = findRigidBodyByOwner(a->owner);
      if(rbA && rbA->isSleeping && !rbA->isTrigger) continue;

      for(std::size_t j = i + 1; j < colliders_.size(); ++j) {
        Collider *b = colliders_[j];
        if(!b || !b->owner) continue;
        if(a->owner == b->owner) continue;

        updateColliderWorldState(b);
        if(!aabbOverlap(a->worldAABB, b->worldAABB)) continue;

        RigidBody *rbB = findRigidBodyByOwner(b->owner);
        if(rbA && rbB && rbA->isSleeping && rbB->isSleeping) continue;

        collideDetectObjectToObject(a, rbA, b, rbB);
      }
    }

    // Collider-to-mesh tests.
    for(std::size_t m = 0; m < meshColliders_.size(); ++m) {
      MeshCollider *mesh = meshColliders_[m];
      if(!mesh || mesh->triangleCount == 0) continue;

      for(Collider *collider : colliders_) {
        if(!collider || !collider->owner) continue;

        updateColliderWorldState(collider);

        RigidBody *rigidBody = findRigidBodyByOwner(collider->owner);
        if(rigidBody && (rigidBody->isTrigger || rigidBody->isSleeping)) continue;

        if(!aabbOverlap(collider->worldAABB, mesh->worldBoundingBox)) continue;

        collideDetectObjectToMesh(collider, rigidBody, *mesh);
      }
    }

    //TODO: possibly offer mesh-mesh collision detection in the future, but not needed for current use cases

    removeInactiveContacts();
  }

  // ── Pre-solve ─────────────────────────────────────────────────────

  void CollisionScene::preSolveContacts() {
    constexpr float RESTITUTION_SLOP = 0.5f;

    for(int i = 0; i < cachedConstraintCount_; ++i) {
      ContactConstraint &cc = cachedConstraints_[i];
      if(!cc.isActive || cc.isTrigger) continue;

      RigidBody *a = cc.rigidBodyA;
      RigidBody *b = cc.rigidBodyB;

      float invMassA = (a && !a->isKinematic) ? a->invMass : 0.0f;
      float invMassB = (b && !b->isKinematic) ? b->invMass : 0.0f;
      float totalInvMass = invMassA + invMassB;
      if(totalInvMass < EPSILON) continue;

      for(int j = 0; j < cc.pointCount; ++j) {
        ContactPoint &cp = cc.points[j];
        if(!cp.active) continue;

        // Relative vectors from centers of mass
        cp.aToContact = a ? vec3Sub(cp.contactA, a->worldCenterOfMass) : vec3Zero();
        cp.bToContact = b ? vec3Sub(cp.contactB, b->worldCenterOfMass) : vec3Zero();

        // Normal effective mass: 1 / (invMassA + invMassB + (rA×n)·I_A^-1·(rA×n) + ...)
        fm_vec3_t raCrossN = vec3Cross(cp.aToContact, cc.normal);
        fm_vec3_t rbCrossN = vec3Cross(cp.bToContact, cc.normal);

        float angularA = a ? vec3Dot(raCrossN, a->applyWorldInertia(raCrossN)) : 0.0f;
        float angularB = b ? vec3Dot(rbCrossN, b->applyWorldInertia(rbCrossN)) : 0.0f;

        float denomN = totalInvMass + angularA + angularB;
        if(denomN < EPSILON) denomN = EPSILON;
        cp.normalMass = 1.0f / denomN;

        // Tangent effective masses
        {
          fm_vec3_t raCrossU = vec3Cross(cp.aToContact, cc.tangentU);
          fm_vec3_t rbCrossU = vec3Cross(cp.bToContact, cc.tangentU);
          float angU_A = a ? vec3Dot(raCrossU, a->applyWorldInertia(raCrossU)) : 0.0f;
          float angU_B = b ? vec3Dot(rbCrossU, b->applyWorldInertia(rbCrossU)) : 0.0f;
          float denomU = totalInvMass + angU_A + angU_B;
          if(denomU < EPSILON) denomU = EPSILON;
          cp.tangentMassU = 1.0f / denomU;
        }
        {
          fm_vec3_t raCrossV = vec3Cross(cp.aToContact, cc.tangentV);
          fm_vec3_t rbCrossV = vec3Cross(cp.bToContact, cc.tangentV);
          float angV_A = a ? vec3Dot(raCrossV, a->applyWorldInertia(raCrossV)) : 0.0f;
          float angV_B = b ? vec3Dot(rbCrossV, b->applyWorldInertia(rbCrossV)) : 0.0f;
          float denomV = totalInvMass + angV_A + angV_B;
          if(denomV < EPSILON) denomV = EPSILON;
          cp.tangentMassV = 1.0f / denomV;
        }

        // Velocity bias (restitution only; Baumgarte is handled in position solver)
        cp.velocityBias = 0.0f;

        // Restitution bias
        fm_vec3_t relVel = vec3Zero();
        if(a) {
          relVel = vec3Add(a->velocity, vec3Cross(a->angularVelocity, cp.aToContact));
        }
        if(b) {
          relVel = vec3Sub(relVel, vec3Add(b->velocity, vec3Cross(b->angularVelocity, cp.bToContact)));
        }
        float relVelN = vec3Dot(relVel, cc.normal);
        if(relVelN < -RESTITUTION_SLOP) {
          cp.velocityBias += cc.combinedBounce * relVelN;
        }
      }
    }
  }

  // ── Warm start ────────────────────────────────────────────────────

  void CollisionScene::warmStart() {
    for(int i = 0; i < cachedConstraintCount_; ++i) {
      ContactConstraint &cc = cachedConstraints_[i];
      if(!cc.isActive || cc.isTrigger) continue;

      RigidBody *a = cc.rigidBodyA;
      RigidBody *b = cc.rigidBodyB;

      for(int j = 0; j < cc.pointCount; ++j) {
        ContactPoint &cp = cc.points[j];
        if(!cp.active) continue;

        fm_vec3_t impulse = vec3Scale(cc.normal, cp.accumulatedNormalImpulse);
        impulse = vec3Add(impulse, vec3Scale(cc.tangentU, cp.accumulatedTangentImpulseU));
        impulse = vec3Add(impulse, vec3Scale(cc.tangentV, cp.accumulatedTangentImpulseV));

        if(a && !a->isKinematic) {
          a->velocity = vec3Add(a->velocity, vec3Scale(impulse, a->invMass));
          a->angularVelocity = vec3Add(a->angularVelocity,
            a->applyWorldInertia(vec3Cross(cp.aToContact, impulse)));
        }
        if(b && !b->isKinematic) {
          b->velocity = vec3Sub(b->velocity, vec3Scale(impulse, b->invMass));
          b->angularVelocity = vec3Sub(b->angularVelocity,
            b->applyWorldInertia(vec3Cross(cp.bToContact, impulse)));
        }
      }
    }
  }

  // ── Velocity constraint solver ────────────────────────────────────

  void CollisionScene::solveVelocityConstraints() {
    for(uint8_t iter = 0; iter < velocitySolverIterations_; ++iter) {
      for(int i = 0; i < cachedConstraintCount_; ++i) {
        ContactConstraint &cc = cachedConstraints_[i];
        if(!cc.isActive || cc.isTrigger) continue;

        RigidBody *a = cc.rigidBodyA;
        RigidBody *b = cc.rigidBodyB;

        for(int j = 0; j < cc.pointCount; ++j) {
          ContactPoint &cp = cc.points[j];
          if(!cp.active) continue;

          // Compute relative velocity at contact
          fm_vec3_t relVel = vec3Zero();
          if(a) {
            relVel = vec3Add(a->velocity, vec3Cross(a->angularVelocity, cp.aToContact));
          }
          if(b) {
            relVel = vec3Sub(relVel, vec3Add(b->velocity, vec3Cross(b->angularVelocity, cp.bToContact)));
          }

          // Normal impulse
          float relVelN = vec3Dot(relVel, cc.normal);
          float dImpulseN = cp.normalMass * (-(relVelN + cp.velocityBias));

          // Clamp accumulated impulse (normal must be non-negative)
          float oldAccum = cp.accumulatedNormalImpulse;
          cp.accumulatedNormalImpulse = fmaxf(oldAccum + dImpulseN, 0.0f);
          dImpulseN = cp.accumulatedNormalImpulse - oldAccum;

          fm_vec3_t impulseN = vec3Scale(cc.normal, dImpulseN);

          if(a && !a->isKinematic) {
            a->velocity = vec3Add(a->velocity, vec3Scale(impulseN, a->invMass));
            a->angularVelocity = vec3Add(a->angularVelocity,
              a->applyWorldInertia(vec3Cross(cp.aToContact, impulseN)));
          }
          if(b && !b->isKinematic) {
            b->velocity = vec3Sub(b->velocity, vec3Scale(impulseN, b->invMass));
            b->angularVelocity = vec3Sub(b->angularVelocity,
              b->applyWorldInertia(vec3Cross(cp.bToContact, impulseN)));
          }

          // Friction with proper accumulation and Coulomb cone clamping.
          if(cc.combinedFriction > 0.0f) {
            // Recompute relative velocity after normal impulse.
            fm_vec3_t contactVelA = vec3Zero();
            fm_vec3_t contactVelB = vec3Zero();
            if(a && !a->isKinematic) {
              contactVelA = a->velocity;
              if(a->rotation) {
                contactVelA = vec3Add(contactVelA, vec3Cross(a->angularVelocity, cp.aToContact));
              }
            }
            if(b && !b->isKinematic) {
              contactVelB = b->velocity;
              if(b->rotation) {
                contactVelB = vec3Add(contactVelB, vec3Cross(b->angularVelocity, cp.bToContact));
              }
            }

            fm_vec3_t relVelF = vec3Sub(contactVelA, contactVelB);
            float vTangentU = vec3Dot(relVelF, cc.tangentU);
            float vTangentV = vec3Dot(relVelF, cc.tangentV);

            float lambdaU = -vTangentU * cp.tangentMassU;
            float lambdaV = -vTangentV * cp.tangentMassV;

            float newAccumU = cp.accumulatedTangentImpulseU + lambdaU;
            float newAccumV = cp.accumulatedTangentImpulseV + lambdaV;

            float maxFriction = cc.combinedFriction * cp.accumulatedNormalImpulse;
            float tangentMagnitude = sqrtf(newAccumU * newAccumU + newAccumV * newAccumV);
            if(tangentMagnitude > maxFriction && tangentMagnitude > EPSILON) {
              float scale = maxFriction / tangentMagnitude;
              newAccumU *= scale;
              newAccumV *= scale;
            }

            lambdaU = newAccumU - cp.accumulatedTangentImpulseU;
            lambdaV = newAccumV - cp.accumulatedTangentImpulseV;

            cp.accumulatedTangentImpulseU = newAccumU;
            cp.accumulatedTangentImpulseV = newAccumV;

            if(fabsf(lambdaU) > EPSILON) {
              fm_vec3_t impulseU = vec3Scale(cc.tangentU, lambdaU);
              if(a && !a->isKinematic) {
                fm_vec3_t linearImpulseA = vec3Scale(impulseU, a->invMass);
                if(!hasFlag(a->constraints, Constraint::FreezePosX)) a->velocity.x += linearImpulseA.x;
                if(!hasFlag(a->constraints, Constraint::FreezePosY)) a->velocity.y += linearImpulseA.y;
                if(!hasFlag(a->constraints, Constraint::FreezePosZ)) a->velocity.z += linearImpulseA.z;
                if(a->rotation) {
                  a->angularVelocity = vec3Add(a->angularVelocity,
                    a->applyWorldInertia(vec3Cross(cp.aToContact, impulseU)));
                }
              }
              if(b && !b->isKinematic) {
                fm_vec3_t linearImpulseB = vec3Scale(impulseU, -b->invMass);
                if(!hasFlag(b->constraints, Constraint::FreezePosX)) b->velocity.x += linearImpulseB.x;
                if(!hasFlag(b->constraints, Constraint::FreezePosY)) b->velocity.y += linearImpulseB.y;
                if(!hasFlag(b->constraints, Constraint::FreezePosZ)) b->velocity.z += linearImpulseB.z;
                if(b->rotation) {
                  b->angularVelocity = vec3Sub(b->angularVelocity,
                    b->applyWorldInertia(vec3Cross(cp.bToContact, impulseU)));
                }
              }
            }

            if(fabsf(lambdaV) > EPSILON) {
              fm_vec3_t impulseV = vec3Scale(cc.tangentV, lambdaV);
              if(a && !a->isKinematic) {
                fm_vec3_t linearImpulseA = vec3Scale(impulseV, a->invMass);
                if(!hasFlag(a->constraints, Constraint::FreezePosX)) a->velocity.x += linearImpulseA.x;
                if(!hasFlag(a->constraints, Constraint::FreezePosY)) a->velocity.y += linearImpulseA.y;
                if(!hasFlag(a->constraints, Constraint::FreezePosZ)) a->velocity.z += linearImpulseA.z;
                if(a->rotation) {
                  a->angularVelocity = vec3Add(a->angularVelocity,
                    a->applyWorldInertia(vec3Cross(cp.aToContact, impulseV)));
                }
              }
              if(b && !b->isKinematic) {
                fm_vec3_t linearImpulseB = vec3Scale(impulseV, -b->invMass);
                if(!hasFlag(b->constraints, Constraint::FreezePosX)) b->velocity.x += linearImpulseB.x;
                if(!hasFlag(b->constraints, Constraint::FreezePosY)) b->velocity.y += linearImpulseB.y;
                if(!hasFlag(b->constraints, Constraint::FreezePosZ)) b->velocity.z += linearImpulseB.z;
                if(b->rotation) {
                  b->angularVelocity = vec3Sub(b->angularVelocity,
                    b->applyWorldInertia(vec3Cross(cp.bToContact, impulseV)));
                }
              }
            }
          }
        }
      }
    }
  }

  // ── Position constraint solver ────────────────────────────────────

  void CollisionScene::solvePositionConstraints() {
    constexpr float SLOP = 0.01f;
    constexpr float STEERING = 0.3f;
    constexpr float MAX_CORRECTION = 0.04f;

    for(int i = 0; i < cachedConstraintCount_; ++i) {
      ContactConstraint &cc = cachedConstraints_[i];
      if(!cc.isActive || cc.isTrigger) continue;

      RigidBody *a = cc.rigidBodyA;
      RigidBody *b = cc.rigidBodyB;

      for(int j = 0; j < cc.pointCount; ++j) {
        ContactPoint &cp = cc.points[j];

        // Refresh world-space contacts from local anchors with current transforms
        if(a && a->position) {
          fm_vec3_t rA = cp.localPointA;
          if(a->rotation) rA = quatRotateVec(*a->rotation, rA);
          cp.contactA = vec3Add(*a->position, rA);
          cp.aToContact = vec3Sub(cp.contactA, a->worldCenterOfMass);
        } else {
          cp.contactA = cp.localPointA;
          cp.aToContact = vec3Zero();
        }

        if(b && b->position) {
          fm_vec3_t rB = cp.localPointB;
          if(b->rotation) rB = quatRotateVec(*b->rotation, rB);
          cp.contactB = vec3Add(*b->position, rB);
          cp.bToContact = vec3Sub(cp.contactB, b->worldCenterOfMass);
        } else {
          cp.contactB = cp.localPointB;
          cp.bToContact = vec3Zero();
        }

        // Recompute penetration from refreshed contacts
        fm_vec3_t diff = vec3Sub(cp.contactA, cp.contactB);
        cp.penetration = -vec3Dot(diff, cc.normal);

        if(cp.penetration < SLOP) continue;

        float steeringForce = fminf(STEERING * (cp.penetration - SLOP), MAX_CORRECTION);
        if(steeringForce <= 0.0f) continue;

        float invMassA = (a && !a->isKinematic) ? a->invMass : 0.0f;
        float invMassB = (b && !b->isKinematic) ? b->invMass : 0.0f;
        float invMassSum = invMassA + invMassB;

        // Add rotational inertia terms
        if(a && a->rotation && !a->isKinematic) {
          fm_vec3_t rCrossN = vec3Cross(cp.aToContact, cc.normal);
          invMassSum += vec3Dot(rCrossN, a->applyWorldInertia(rCrossN));
        }
        if(b && b->rotation && !b->isKinematic) {
          fm_vec3_t rCrossN = vec3Cross(cp.bToContact, cc.normal);
          invMassSum += vec3Dot(rCrossN, b->applyWorldInertia(rCrossN));
        }

        if(invMassSum < EPSILON) continue;

        float correctionMag = steeringForce / invMassSum;
        fm_vec3_t impulse = vec3Scale(cc.normal, correctionMag);

        // Apply linear + angular corrections to A
        if(a && !a->isKinematic && a->position) {
          if(invMassA > 0.0f) {
            *a->position = vec3Add(*a->position, vec3Scale(cc.normal, correctionMag * invMassA));
          }
          if(a->rotation) {
            fm_vec3_t angImpulse = vec3Cross(cp.aToContact, impulse);
            fm_vec3_t rotChange = a->applyWorldInertia(angImpulse);
            float angle = vec3Mag(rotChange);
            if(angle > EPSILON) {
              fm_vec3_t axis = vec3Scale(rotChange, 1.0f / angle);
              fm_quat_t dq = quatFromAxisAngle(axis, angle);
              *a->rotation = quatNormalize(quatMultiply(dq, *a->rotation));
            }
          }
        }

        // Apply linear + angular corrections to B
        if(b && !b->isKinematic && b->position) {
          if(invMassB > 0.0f) {
            *b->position = vec3Sub(*b->position, vec3Scale(cc.normal, correctionMag * invMassB));
          }
          if(b->rotation) {
            fm_vec3_t angImpulse = vec3Negate(vec3Cross(cp.bToContact, impulse));
            fm_vec3_t rotChange = b->applyWorldInertia(angImpulse);
            float angle = vec3Mag(rotChange);
            if(angle > EPSILON) {
              fm_vec3_t axis = vec3Scale(rotChange, 1.0f / angle);
              fm_quat_t dq = quatFromAxisAngle(axis, angle);
              *b->rotation = quatNormalize(quatMultiply(dq, *b->rotation));
            }
          }
        }
      }
    }
  }

  /// @brief Recalculate the world-space AABBs of all Mesh Colliders in the Collision Scene.
  void CollisionScene::updateMeshColliderWorldAABBs() {
    for(std::size_t i = 0; i < meshColliders_.size(); ++i) {
      MeshCollider *mesh = meshColliders_[i];
      if(!mesh) continue;
      mesh->recalculateWorldAABB();
    }
  }

  // ── Raycast ───────────────────────────────────────────────────────

  static bool rayTriangleIntersect(const fm_vec3_t &origin, const fm_vec3_t &dir,
                                   const fm_vec3_t &v0, const fm_vec3_t &v1, const fm_vec3_t &v2,
                                   float &outDist, fm_vec3_t &outNormal) {
    fm_vec3_t e1 = vec3Sub(v1, v0);
    fm_vec3_t e2 = vec3Sub(v2, v0);
    fm_vec3_t h = vec3Cross(dir, e2);
    float a = vec3Dot(e1, h);
    if(fabsf(a) < EPSILON) return false;

    float f = 1.0f / a;
    fm_vec3_t s = vec3Sub(origin, v0);
    float u = f * vec3Dot(s, h);
    if(u < 0.0f || u > 1.0f) return false;

    fm_vec3_t q = vec3Cross(s, e1);
    float v = f * vec3Dot(dir, q);
    if(v < 0.0f || u + v > 1.0f) return false;

    float t = f * vec3Dot(e2, q);
    if(t < EPSILON) return false;

    outDist = t;
    outNormal = vec3Normalize(vec3Cross(e1, e2));
    return true;
  }

  bool CollisionScene::raycast(Raycast &ray, RaycastHit &hit) const {
    hit = RaycastHit{};

    // Test mesh colliders
    if(hasFlag(ray.mask, RaycastMask::MESH_COLLIDERS)) {
      for(std::size_t m = 0; m < meshColliders_.size(); ++m) {
        const MeshCollider *mesh = meshColliders_[m];
        if(!mesh || mesh->triangleCount == 0) continue;

        // Transform the ray into the mesh's local space for AABB tree query
        fm_vec3_t localOrigin = mesh->hasTransform() ? mesh->toLocalSpace(ray.origin) : ray.origin;
        fm_vec3_t localDir = mesh->hasTransform() ? mesh->rotateToLocal(ray.dir) : ray.dir;
        fm_vec3_t localInvDir = vec3(
          fabsf(localDir.x) > EPSILON ? 1.0f / localDir.x : 1e30f,
          fabsf(localDir.y) > EPSILON ? 1.0f / localDir.y : 1e30f,
          fabsf(localDir.z) > EPSILON ? 1.0f / localDir.z : 1e30f
        );

        NodeProxy triCandidates[64];
        int triCount = mesh->aabbTree.queryRay(
          localOrigin, localInvDir, ray.maxDistance, triCandidates, 64);

        int tested = 0;
        for(int i = 0; i < triCount && tested < RAYCAST_MAX_TRIANGLE_TESTS; ++i) {
          void *data = mesh->aabbTree.getNodeData(triCandidates[i]);
          if(!data) continue;
          int triIdx = static_cast<int>(reinterpret_cast<intptr_t>(data)) - 1; // stored as index+1
          if(triIdx < 0 || triIdx >= mesh->triangleCount) continue;

          const MeshTriangleIndices &tri = mesh->triangles[triIdx];
          // Get vertices in world space
          fm_vec3_t v0 = mesh->toWorldSpace(mesh->vertices[tri.indices[0]]);
          fm_vec3_t v1 = mesh->toWorldSpace(mesh->vertices[tri.indices[1]]);
          fm_vec3_t v2 = mesh->toWorldSpace(mesh->vertices[tri.indices[2]]);

          float dist;
          fm_vec3_t normal;
          if(rayTriangleIntersect(ray.origin, ray.dir, v0, v1, v2, dist, normal)) {
            if(dist < hit.distance && dist <= ray.maxDistance) {
              hit.distance = dist;
              hit.point = vec3Add(ray.origin, vec3Scale(ray.dir, dist));
              hit.normal = normal;
              hit.hitId = 0;
              hit.didHit = true;
            }
          }
          tested++;
        }
      }
    }

    // Test physics objects
    if(hasFlag(ray.mask, RaycastMask::COLLIDER_BODIES)) {
      NodeProxy objCandidates[MAX_OBJ_COLLISION_CANDIDATES];
      int objCount = rigidBodyAABBTree.queryRay(
        ray.origin, ray.invDir, ray.maxDistance, objCandidates, MAX_OBJ_COLLISION_CANDIDATES);

      int tested = 0;
      for(int i = 0; i < objCount && tested < RAYCAST_MAX_OBJECT_TESTS; ++i) {
        void *data = rigidBodyAABBTree.getNodeData(objCandidates[i]);
        if(!data) continue;
        auto *obj = static_cast<RigidBody *>(data);

        if(!obj->collider) continue;
        if((obj->collisionLayers & ray.collisionLayers) == 0) continue;
        if((obj->collisionLayers & ray.ignoreLayers) != 0) continue;
        if(obj->isTrigger && !ray.interactTrigger) continue;

        // Simple sphere approximation for ray-rigidBody test
        fm_vec3_t toObj = vec3Sub(obj->worldCenterOfMass, ray.origin);
        float projLen = vec3Dot(toObj, ray.dir);
        if(projLen < 0.0f) continue;

        fm_vec3_t closestOnRay = vec3Add(ray.origin, vec3Scale(ray.dir, projLen));
        float distSq = vec3DistSqrd(closestOnRay, obj->worldCenterOfMass);

        // Approximate radius from AABB
        fm_vec3_t halfExtent = vec3Scale(vec3Sub(obj->boundingBox.max, obj->boundingBox.min), 0.5f);
        float approxRadius = vec3Mag(halfExtent);

        if(distSq > approxRadius * approxRadius) continue;

        // Refine: for spheres, do exact intersection
        if(obj->collider->type == ShapeType::Sphere) {
          float r = obj->collider->sphere.radius;
          float disc = projLen * projLen - vec3MagSqrd(toObj) + r * r;
          if(disc < 0.0f) continue;

          float dist = projLen - sqrtf(disc);
          if(dist < EPSILON || dist > ray.maxDistance) continue;
          if(dist >= hit.distance) continue;

          hit.distance = dist;
          hit.point = vec3Add(ray.origin, vec3Scale(ray.dir, dist));
          hit.normal = vec3Normalize(vec3Sub(hit.point, obj->worldCenterOfMass));
          hit.hitId = obj->owner ? obj->owner->id : 0;
          hit.didHit = true;
        } else {
          // Use approximate distance for non-sphere shapes
          float dist = projLen - approxRadius;
          if(dist < EPSILON) dist = projLen;
          if(dist > ray.maxDistance || dist >= hit.distance) continue;

          hit.distance = dist;
          hit.point = vec3Add(ray.origin, vec3Scale(ray.dir, dist));
          hit.normal = vec3Normalize(vec3Sub(hit.point, obj->worldCenterOfMass));
          hit.hitId = obj->owner ? obj->owner->id : 0;
          hit.didHit = true;
        }
        tested++;
      }
    }

    return hit.didHit;
  }

  // ── Main step ─────────────────────────────────────────────────────

  void CollisionScene::step() {
    // 0. Update mesh collider world AABBs (in case transforms changed)
    // TODO: maybe only update if transform is dirty?
    updateMeshColliderWorldAABBs();

    // 1. Refresh collider world state
    for(Collider *collider : colliders_) {
      updateColliderWorldState(collider);
    }

    // 2. Update compound COM/inertia and world inertia tensors.
    for (RigidBody *body : rigidBodies_){
      RigidBody *obj = body;
      if(!obj || obj->isSleeping) continue;
      updateCompoundProperties(obj);
      obj->updateWorldInertia();
    }

    // 3. Integrate velocities, release old contacts
    for(RigidBody *body : rigidBodies_) {
      RigidBody *obj = body;
      if(!obj) continue;

      releaseObjectContacts(obj);

      if(!obj->isSleeping) {
        obj->integrateVelocity(fixedDt_, gravity_);
        obj->integrateAngularVelocity(fixedDt_);
      }
    }

    // 4. Detect all contacts (broad + narrow phase)
    detectAllContacts();

    // Refresh anchors from local-space points before solving.
    refreshContacts();

    // 5. Pre-solve contacts (compute effective masses)
    preSolveContacts();

    // 6. Warm start
    warmStart();

    // 7. Velocity constraint solver
    solveVelocityConstraints();

    // 8. Integrate positions and update AABBs
    for(RigidBody *body : rigidBodies_) {
      RigidBody *obj = body;
      if(!obj || obj->isSleeping) continue;

      obj->integratePosition(fixedDt_);
      obj->integrateRotation(fixedDt_);
      AABB bounds{};
      bool hasBounds = false;
      if(const std::vector<Collider *> *ownerColliders = findCollidersForOwner(obj->owner)) {
        for(Collider *collider : *ownerColliders) {
          if(!collider) continue;
          updateColliderWorldState(collider);
          if(!hasBounds) {
            bounds = collider->worldAABB;
            hasBounds = true;
          } else {
            bounds.min = vec3Min(bounds.min, collider->worldAABB.min);
            bounds.max = vec3Max(bounds.max, collider->worldAABB.max);
          }
        }
      }

      if(hasBounds) {
        obj->boundingBox = bounds;
      } else {
        obj->recalculateAABB();
      }

      if(obj->aabbTreeNodeId != NULL_NODE) {
        fm_vec3_t displacement = vec3Scale(obj->velocity, fixedDt_);
        rigidBodyAABBTree.moveNode(obj->aabbTreeNodeId, obj->boundingBox, displacement);
      }
    }

    // 9. Position constraint solver
    for(uint8_t iter = 0; iter < positionSolverIterations_; ++iter) {
      solvePositionConstraints();
    }

    // 10. Apply position constraints and update sleep
    for(RigidBody *body : rigidBodies_) {
      RigidBody *obj = body;
      if(!obj) continue;

      obj->applyPositionConstraints();
      obj->updateWorldInertia();
      AABB bounds{};
      bool hasBounds = false;
      if(const std::vector<Collider *> *ownerColliders = findCollidersForOwner(obj->owner)) {
        for(Collider *collider : *ownerColliders) {
          if(!collider) continue;
          updateColliderWorldState(collider);
          if(!hasBounds) {
            bounds = collider->worldAABB;
            hasBounds = true;
          } else {
            bounds.min = vec3Min(bounds.min, collider->worldAABB.min);
            bounds.max = vec3Max(bounds.max, collider->worldAABB.max);
          }
        }
      }

      if(hasBounds) {
        obj->boundingBox = bounds;
      } else {
        obj->recalculateAABB();
      }

      // Keep broadphase in sync with post-solve corrected transforms.
      if(!obj->isSleeping && obj->aabbTreeNodeId != NULL_NODE) {
        fm_vec3_t displacement = vec3Sub(*obj->position, obj->prevStepPos);
        rigidBodyAABBTree.moveNode(obj->aabbTreeNodeId, obj->boundingBox, displacement);
      }

      // Sleep detection
      if(obj->isTrigger || obj->isKinematic) continue;
      if(obj->isSleeping) continue;

      bool canSleep = true;
      float speedSq = vec3MagSqrd(obj->velocity);
      float angSpeedSq = vec3MagSqrd(obj->angularVelocity);
      float posDeltaSq = vec3DistSqrd(*obj->position, obj->prevStepPos);

      if(speedSq > SPEED_SLEEP_THRESHOLD_SQ) canSleep = false;
      if(angSpeedSq > ANGULAR_SLEEP_THRESHOLD_SQ) canSleep = false;
      if(posDeltaSq > POS_SLEEP_THRESHOLD_SQ) canSleep = false;
      if(obj->rotation) {
        float rotSim = fabsf(quatDot(*obj->rotation, obj->prevStepRot));
        if(rotSim < ROT_SIMILARITY_SLEEP_THRESHOLD) canSleep = false;
      }

      if(canSleep) {
        obj->sleepCounter++;
        if(obj->sleepCounter >= SLEEP_STEPS) {
          obj->sleep();
        }
      } else {
        obj->sleepCounter = 0;
      }
    }
  }

  /// @brief Draws debug visuals for the collision scene.
  /// Draws on the CPU which may cause significant slowdown
  /// @param showMeshColliders Whether to draw mesh colliders.
  /// @param showRigidBodies Whether to draw rigid bodies.
  void P64::CollNew::CollisionScene::debugDraw(bool showMeshColliders, bool showRigidBodies)
  {
    if (showMeshColliders)
    {
      for (std::size_t meshIdx = 0; meshIdx < meshColliders_.size(); ++meshIdx)
      {
        const auto *meshCollider = meshColliders_[meshIdx];
        if (!meshCollider || !meshCollider->vertices || !meshCollider->triangles || meshCollider->triangleCount == 0)
        {
          continue;
        }

        color_t color = Debug::paletteColor(static_cast<uint32_t>(meshIdx));

        const bool useRotation = meshCollider->hasRotation();

        const fm_quat_t rot = meshCollider->owner->rot;
        const fm_vec3_t pos = meshCollider->owner->pos;
        const fm_vec3_t scale = meshCollider->owner->scale;

        auto toWorldMesh = [&](const fm_vec3_t &local)
        {
          fm_vec3_t scaled = vec3(local.x * scale.x, local.y * scale.y, local.z * scale.z);
          if (useRotation)
          {
            scaled = quatRotateVec(rot, scaled);
          }
          return vec3Add(scaled, pos);
        };

        for (uint16_t t = 0; t < meshCollider->triangleCount; ++t)
        {
          int idxA = meshCollider->triangles[t].indices[0];
          int idxB = meshCollider->triangles[t].indices[1];
          int idxC = meshCollider->triangles[t].indices[2];

          fm_vec3_t v0 = toWorldMesh(meshCollider->vertices[idxA]);
          fm_vec3_t v1 = toWorldMesh(meshCollider->vertices[idxB]);
          fm_vec3_t v2 = toWorldMesh(meshCollider->vertices[idxC]);

          Debug::drawLine(v0, v1, color);
          Debug::drawLine(v1, v2, color);
          Debug::drawLine(v2, v0, color);
        }
      }
    }

    if (showRigidBodies)
    {
      for (const auto &collider : colliders_)
      {

        color_t col{0xFF, 0xFF, 0x00, 0xFF};
        if (collider)
        {
          switch (collider->type)
          {
          case ShapeType::Sphere:
            col = color_t{0xFF, 0x00, 0x00, 0xFF};
            Debug::drawSphere(collider->worldCenter, collider->sphere.radius, col);
            break;
          case ShapeType::Box:
            col = color_t{0x00, 0xFF, 0xFF, 0xFF};
            Debug::drawOBB(collider->worldCenter, collider->box.halfSize, collider->owner->rot, col);
            break;
          case ShapeType::Capsule:
            col = color_t{0x00, 0x80, 0xFF, 0xFF};
            Debug::drawCapsule(
                collider->worldCenter,
                collider->capsule.radius,
                collider->capsule.innerHalfHeight,
                collider->owner->rot,
                col);
            break;
          case ShapeType::Cylinder:
            col = color_t{0xFF, 0x80, 0x00, 0xFF};
            Debug::drawCylinder(
                collider->worldCenter,
                collider->cylinder.radius,
                collider->cylinder.halfHeight,
                collider->owner->rot,
                col);
            break;
          case ShapeType::Cone:
            col = color_t{0xFF, 0x40, 0xA0, 0xFF};
            Debug::drawCone(
                collider->worldCenter,
                collider->cone.radius,
                collider->cone.halfHeight,
                collider->owner->rot,
                col);
            break;
          case ShapeType::Pyramid:
            col = color_t{0xB0, 0xFF, 0x40, 0xFF};
            Debug::drawPyramid(
                collider->worldCenter,
                collider->pyramid.baseHalfWidthX,
                collider->pyramid.baseHalfWidthZ,
                collider->pyramid.halfHeight,
                collider->owner->rot,
                col);
            break;
          default:
            break;
          }
        }
      }
    }
  }

} // namespace P64::CollNew
