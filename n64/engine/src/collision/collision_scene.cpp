/**
 * @file collision_scene.cpp
 * @author Kevin Reier <https://github.com/Byterset>
 * @brief Defines the Collision Scene which keeps track of physics participants and updates them (see collision_scene.h)
 */
#include "collision/collision_scene.h"
#include "collision/collide.h"
#include "collision/gjk.h"
#include "scene/scene.h"

#include <cmath>
#include <cassert>
#include <cinttypes>
#include <functional>
#include <algorithm>
#include <limits>
#include <utility>
#include <unordered_set>

#include "debug/debugDraw.h"

namespace P64::Coll {

  static CollisionScene g_scene;

  static double ticksToMs(uint64_t ticks) {
    return static_cast<double>(TICKS_TO_US(ticks)) / 1000.0;
  }

  static bool aabbChanged(const AABB &lhs, const AABB &rhs) {
    return fm_vec3_distance2(&lhs.min, &rhs.min) > FM_EPSILON * FM_EPSILON ||
           fm_vec3_distance2(&lhs.max, &rhs.max) > FM_EPSILON * FM_EPSILON;
  }

  static AABB mergeAABBs(const AABB &lhs, const AABB &rhs) {
    return AABB{
      vec3Min(lhs.min, rhs.min),
      vec3Max(lhs.max, rhs.max)
    };
  }

  static bool hasLinearConstraints(const RigidBody *body) {
    return body && (body->constraints & Constraint::FreezePosAll) != Constraint::None;
  }

  static bool hasAngularConstraints(const RigidBody *body) {
    return body && (body->constraints & Constraint::FreezeRotAll) != Constraint::None;
  }

  static bool canApplyAngularResponse(const RigidBody *body) {
    return body && !body->isKinematic && body->rotation && !hasFlag(body->constraints, Constraint::FreezeRotAll);
  }

  static bool colliderReadsCollider(const Collider *reader, const Collider *writer) {
    return reader && writer && ((reader->maskRead & writer->maskWrite) != 0);
  }

  static bool collidersShouldGenerateContact(const Collider *colliderA, const Collider *colliderB) {
    return colliderReadsCollider(colliderA, colliderB) || colliderReadsCollider(colliderB, colliderA);
  }

  static bool colliderShouldTestMesh(const Collider *collider) {
    return collider && (collider->maskWrite != 0 || collider->maskRead != 0);
  }

  static fm_vec3_t constrainAngularWorld(const RigidBody *body, const fm_vec3_t &worldAngular) {
    if(!body) return worldAngular;
    if(!hasAngularConstraints(body)) return worldAngular;
    if(hasFlag(body->constraints, Constraint::FreezeRotAll)) return VEC3_ZERO;
    if(!body->rotation) return worldAngular;

    fm_vec3_t local = quatRotateVec(quatConjugate(*body->rotation), worldAngular);
    if(hasFlag(body->constraints, Constraint::FreezeRotX)) local.x = 0.0f;
    if(hasFlag(body->constraints, Constraint::FreezeRotY)) local.y = 0.0f;
    if(hasFlag(body->constraints, Constraint::FreezeRotZ)) local.z = 0.0f;
    return quatRotateVec(*body->rotation, local);
  }

  static fm_vec3_t constrainLinearWorld(const RigidBody *body, const fm_vec3_t &worldLinear) {
    if(!body) return worldLinear;
    if(!hasLinearConstraints(body)) return worldLinear;
    fm_vec3_t out = worldLinear;
    if(hasFlag(body->constraints, Constraint::FreezePosX)) out.x = 0.0f;
    if(hasFlag(body->constraints, Constraint::FreezePosY)) out.y = 0.0f;
    if(hasFlag(body->constraints, Constraint::FreezePosZ)) out.z = 0.0f;
    return out;
  }

  static void applyConstrainedLinearVelocityDelta(RigidBody *body, const fm_vec3_t &deltaLinearVelocity) {
    if(!body) return;
    body->velocity = body->velocity + (hasLinearConstraints(body) ? constrainLinearWorld(body, deltaLinearVelocity) : deltaLinearVelocity);
  }

  static void applyConstrainedImpulseAtContact(RigidBody *body, const fm_vec3_t &impulse, const fm_vec3_t &toContact) {
    if(!body || body->isKinematic) return;

    applyConstrainedLinearVelocityDelta(body, impulse * body->invMass);
    if(!canApplyAngularResponse(body)) return;
    fm_vec3_t cross;
    fm_vec3_cross(&cross, &toContact, &impulse);
    fm_vec3_t angDelta = body->applyWorldInertia(cross);
    if(hasAngularConstraints(body)) {
      angDelta = constrainAngularWorld(body, angDelta);
    }
    body->angularVelocity = body->angularVelocity + angDelta;
  }

  static float constrainedLinearInvMassAlong(const RigidBody *body, const fm_vec3_t &direction) {
    if(!body || body->isKinematic) return 0.0f;
    if(body->invMass <= FM_EPSILON) return 0.0f;
    if(!hasLinearConstraints(body)) return body->invMass;

    fm_vec3_t constrainedDir = constrainLinearWorld(body, direction);
    float dirFactor = fm_vec3_dot(&direction, &constrainedDir);
    if(dirFactor <= FM_EPSILON) return 0.0f;
    return body->invMass * dirFactor;
  }

  bool CollisionScene::shouldTrackSleepState(const RigidBody *rigidBody) {
    return rigidBody && !rigidBody->isKinematic && rigidBody->position;
  }

  bool CollisionScene::rigidBodyVelocitiesExceededSleepThreshold(const RigidBody *rigidBody) {
    if(!shouldTrackSleepState(rigidBody)) return false;

    const float speedSq = fm_vec3_len2(&rigidBody->velocity);
    if(speedSq > SPEED_SLEEP_THRESHOLD_SQ) return true;

    const float angSpeedSq = fm_vec3_len2(&rigidBody->angularVelocity);
    return angSpeedSq > ANGULAR_SLEEP_THRESHOLD_SQ;
  }

  bool CollisionScene::rigidBodyTransformExceededSleepThreshold(const RigidBody *rigidBody) {
    if(!shouldTrackSleepState(rigidBody)) return false;

    const float posDeltaSq = fm_vec3_distance2(rigidBody->position, &rigidBody->prevStepPos);
    if(posDeltaSq > POS_SLEEP_THRESHOLD_SQ) return true;

    if(rigidBody->rotation) {
      const float rotSim = fabsf(quatDot(*rigidBody->rotation, rigidBody->prevStepRot));
      if(rotSim < ROT_SIMILARITY_SLEEP_THRESHOLD) return true;
    }

    return false;
  }

  bool CollisionScene::rigidBodyCompoundPropertiesNeedUpdate(const RigidBody *rigidBody) {
    if(!rigidBody || !rigidBody->owner) return false;
    if(rigidBody->compoundPropertiesDirty) return true;
    return fm_vec3_distance2(&rigidBody->compoundScale, &rigidBody->owner->scale) > FM_EPSILON * FM_EPSILON;
  }

  void CollisionScene::rebuildCachedConstraintLookup() {
    cachedConstraintLookup_.clear();
    for(int i = 0; i < cachedConstraintCount_; ++i) {
      ContactConstraint &cc = cachedConstraints_[i];
      cachedConstraintLookup_[cc.key] = i;
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
    cachedConstraintLookup_.clear();
    solverConstraints_.clear();
    ticksWakePrep = 0;
    ticksWorldUpdate = 0;
    ticksIntegrateVel = 0;
    ticksDetect = 0;
    ticksDetectDeactivate = 0;
    ticksDetectBuildOrder = 0;
    ticksDetectBodyPairs = 0;
    ticksDetectDetachedPairs = 0;
    ticksDetectDetachedBodyPairs = 0;
    ticksDetectDetachedDetachedPairs = 0;
    ticksDetectMeshPairs = 0;
    ticksDetectCleanup = 0;
    ticksRefreshCallbacks = 0;
    ticksPreSolve = 0;
    ticksWarmStart = 0;
    ticksVelocitySolve = 0;
    ticksIntegratePos = 0;
    ticksPositionSolve = 0;
    ticksFinalize = 0;
    ticksTotal = 0;
    detectOrderedColliderCount = 0;
    detectTriggerColliderCount = 0;
    detectOrderedBodyCount = 0;
    detectDetachedColliderCount = 0;
    detectBodyCandidateCount = 0;
    detectObjectPairCount = 0;
    detectDetachedPairCount = 0;
    detectMeshPairCount = 0;
    detectDebugPrintCounter_ = 0;

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
    collider->worldAABB.min = local.min + collider->worldCenter;
    collider->worldAABB.max = local.max + collider->worldCenter;
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
      rigidBody->centerOffset = VEC3_ZERO;
      return;
    }

    int count = 0;
    fm_vec3_t worldCenterSum = VEC3_ZERO;
    for(Collider *collider : *ownerColliders) {
      if(!collider) continue;
      worldCenterSum = worldCenterSum + collider->worldCenter;
      ++count;
    }

    if(count <= 0) {
      rigidBody->centerOffset = VEC3_ZERO;
      return;
    }

    const float invCount = 1.0f / static_cast<float>(count);
    const fm_vec3_t worldCenter = worldCenterSum * invCount;

    fm_vec3_t localCenterOffset = worldCenter - *rigidBody->position;
    if(rigidBody->rotation) {
      localCenterOffset = quatRotateVec(quatConjugate(*rigidBody->rotation), localCenterOffset);
    }
    rigidBody->centerOffset = localCenterOffset;

    if(rigidBody->getMass() <= FM_EPSILON) {
      return;
    }

    const float massPerCollider = rigidBody->getMass() * invCount;
    fm_vec3_t compoundInertia = VEC3_ZERO;

    for(Collider *collider : *ownerColliders) {
      if(!collider) continue;

      fm_vec3_t colliderInertia = collider->inertiaTensor(massPerCollider);
      fm_vec3_t r = collider->worldCenter - worldCenter;
      if(rigidBody->rotation) {
        r = quatRotateVec(quatConjugate(*rigidBody->rotation), r);
      }

      const float x2 = r.x * r.x;
      const float y2 = r.y * r.y;
      const float z2 = r.z * r.z;

      colliderInertia.x += massPerCollider * (y2 + z2);
      colliderInertia.y += massPerCollider * (x2 + z2);
      colliderInertia.z += massPerCollider * (x2 + y2);

      compoundInertia = compoundInertia + colliderInertia;
    }

    rigidBody->localInertiaTensor = compoundInertia;
    rigidBody->invLocalInertiaTensor = fm_vec3_t{{
      compoundInertia.x > FM_EPSILON ? 1.0f / compoundInertia.x : 0.0f,
      compoundInertia.y > FM_EPSILON ? 1.0f / compoundInertia.y : 0.0f,
      compoundInertia.z > FM_EPSILON ? 1.0f / compoundInertia.z : 0.0f
    }};
  }

  void CollisionScene::syncCompoundProperties(RigidBody *rigidBody) const {
    if(!rigidBody || !rigidBody->owner) return;
    if(!rigidBodyCompoundPropertiesNeedUpdate(rigidBody)) return;

    updateCompoundProperties(rigidBody);
    rigidBody->compoundScale = rigidBody->owner->scale;
    rigidBody->compoundPropertiesDirty = false;
  }

  // ── Object management ─────────────────────────────────────────────

  void CollisionScene::addRigidBody(RigidBody *rigidBody) {
    if(!rigidBody || !rigidBody->owner) return;
    rigidBodies_.push_back(rigidBody);
    ownerRigidBodies_[rigidBody->owner] = rigidBody;
    

    rigidBody->compoundPropertiesDirty = true;
    syncCompoundProperties(rigidBody);
    rigidBody->updateWorldInertia();
    const fm_vec3_t worldPos = rigidBody->position ? *rigidBody->position : VEC3_ZERO;
    rigidBody->boundingBox = AABB{worldPos, worldPos};
    rigidBody->aabbTreeNodeId = rigidBodyAABBTree.createNode(rigidBody->boundingBox, rigidBody);
  }

  void CollisionScene::removeRigidBody(RigidBody *rigidBody) {
    if(!rigidBody) return;

    std::vector<RigidBody *> wakeCandidates;

    if(rigidBody->owner) {
      auto ownerIt = ownerRigidBodies_.find(rigidBody->owner);
      if(ownerIt != ownerRigidBodies_.end() && ownerIt->second == rigidBody) {
        ownerRigidBodies_.erase(ownerIt);
      }
    }

    // Remove from AABB tree
    if(rigidBody->aabbTreeNodeId != NULL_NODE) {
      rigidBodyAABBTree.removeLeaf(rigidBody->aabbTreeNodeId, true);
      rigidBody->aabbTreeNodeId = NULL_NODE;
    }

    removeCachedConstraints([rigidBody](const ContactConstraint &cc) {
      return cc.rigidBodyA == rigidBody || cc.rigidBodyB == rigidBody;
    }, wakeCandidates, rigidBody);

    // Sleeping rigidBodies overlapping the removed body are likely support-dependent and should re-evaluate.
    const AABB removedBounds = rigidBody->boundingBox;
    for(RigidBody *body : rigidBodies_) {
      if(!body || body == rigidBody) continue;
      if(aabbOverlap(body->boundingBox, removedBounds)) {
        addWakeCandidate(wakeCandidates, body, rigidBody);
      }
    }

    wakeCandidateIslands(wakeCandidates);

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
      rigidBody->compoundPropertiesDirty = true;
      syncCompoundProperties(rigidBody);
      rigidBody->updateWorldInertia();
    }
  }

  void CollisionScene::removeCollider(Collider *collider) {
    if(!collider) return;
    Object *owner = collider->owner;

    std::vector<RigidBody *> wakeCandidates;
    removeCachedConstraints([collider](const ContactConstraint &cc) {
      return cc.colliderA == collider || cc.colliderB == collider;
    }, wakeCandidates);

    wakeCandidateIslands(wakeCandidates);

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
        rigidBody->compoundPropertiesDirty = true;
        syncCompoundProperties(rigidBody);
        rigidBody->updateWorldInertia();
      }
    }
  }

  void CollisionScene::addMeshCollider(MeshCollider *mesh) {
    if(!mesh) return;

    mesh->computeLocalRootAABB();
    mesh->recalculateWorldAABB();
    mesh->syncOwnerTransform();

    meshColliders_.push_back(mesh);
  }

  void CollisionScene::removeMeshCollider(MeshCollider *mesh) {
    if(!mesh) return;

    std::vector<RigidBody *> wakeCandidates;
    removeCachedConstraints([mesh](const ContactConstraint &cc) {
      return cc.meshColliderA == mesh || cc.meshColliderB == mesh;
    }, wakeCandidates);

    wakeCandidateIslands(wakeCandidates);

    for(std::size_t i = 0; i < meshColliders_.size(); ++i) {
      if(meshColliders_[i] == mesh) {
        meshColliders_[i] = meshColliders_.back();
        meshColliders_.pop_back();
        break;
      }
    }
  }

  void CollisionScene::configureSimulation(float fixedDt, const fm_vec3_t &gravity, uint8_t velocityIterations, uint8_t positionIterations, float physicsScale) {
    fixedDt_ = fixedDt > 0.0f ? fixedDt : DEFAULT_FIXED_DT;
    physicsScale_ = physicsScale > FM_EPSILON ? physicsScale : DEFAULT_PHYSICS_SCALE;
    gravity_ = gravity * physicsScale;
    velocitySolverIterations_ = std::max<uint8_t>(1, velocityIterations);
    positionSolverIterations_ = std::max<uint8_t>(1, positionIterations);
  }

  void CollisionScene::wakeRigidBodyIsland(RigidBody *rigidBody) {
    wakeIsland(rigidBody);
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
    const ContactConstraintKey &key,
    RigidBody *rigidBodyA, Collider *colliderA, MeshCollider *meshColliderA, Object *objectA,
    RigidBody *rigidBodyB, Collider *colliderB, MeshCollider *meshColliderB, Object *objectB) {
    if(ContactConstraint *existing = findCachedConstraint(key)) {
      return existing;
    }

    cachedConstraints_.push_back(ContactConstraint{});
    cachedConstraintCount_ = static_cast<int>(cachedConstraints_.size());
    ContactConstraint &cc = cachedConstraints_.back();
    cc.key = key;
    cc.rigidBodyA = rigidBodyA;
    cc.colliderA = colliderA;
    cc.meshColliderA = meshColliderA;
    cc.objectA = objectA;
    cc.rigidBodyB = rigidBodyB;
    cc.colliderB = colliderB;
    cc.meshColliderB = meshColliderB;
    cc.objectB = objectB;

    cachedConstraintLookup_[key] = cachedConstraintCount_ - 1;
    return &cc;
  }

  ContactConstraint *CollisionScene::findCachedConstraint(const ContactConstraintKey &key) {
    auto it = cachedConstraintLookup_.find(key);
    if(it == cachedConstraintLookup_.end()) return nullptr;
    return &cachedConstraints_[it->second];
  }

  void CollisionScene::collectConnectedIsland(RigidBody *seed, std::vector<RigidBody *> &island, std::unordered_set<RigidBody *> &visited) const {
    if(!shouldTrackSleepState(seed)) return;

    std::vector<RigidBody *> stack;
    stack.push_back(seed);

    while(!stack.empty()) {
      RigidBody *current = stack.back();
      stack.pop_back();

      if(!shouldTrackSleepState(current)) continue;
      if(visited.find(current) != visited.end()) continue;
      visited.insert(current);
      island.push_back(current);

      for(int i = 0; i < cachedConstraintCount_; ++i) {
        const ContactConstraint &cc = cachedConstraints_[i];
        if(!cc.isActive || cc.isTrigger) continue;

        RigidBody *other = nullptr;
        if(cc.rigidBodyA == current) {
          other = cc.rigidBodyB;
        } else if(cc.rigidBodyB == current) {
          other = cc.rigidBodyA;
        }

        if(!shouldTrackSleepState(other)) continue;
        if(visited.find(other) == visited.end()) {
          stack.push_back(other);
        }
      }
    }
  }

  void CollisionScene::addWakeCandidate(std::vector<RigidBody *> &wakeCandidates, RigidBody *candidate, RigidBody *ignoredCandidate) {
    if(!candidate || candidate == ignoredCandidate) return;
    if(std::find(wakeCandidates.begin(), wakeCandidates.end(), candidate) == wakeCandidates.end()) {
      wakeCandidates.push_back(candidate);
    }
  }

  void CollisionScene::wakeCandidateIslands(const std::vector<RigidBody *> &wakeCandidates) {
    for(RigidBody *candidate : wakeCandidates) {
      wakeIsland(candidate);
    }
  }

  void CollisionScene::removeCachedConstraints(
    const std::function<bool(const ContactConstraint &)> &shouldRemove,
    std::vector<RigidBody *> &wakeCandidates,
    RigidBody *ignoredCandidate) {
    for(int i = 0; i < cachedConstraintCount_;) {
      ContactConstraint &cc = cachedConstraints_[i];
      if(shouldRemove(cc)) {
        addWakeCandidate(wakeCandidates, cc.rigidBodyA, ignoredCandidate);
        addWakeCandidate(wakeCandidates, cc.rigidBodyB, ignoredCandidate);

        removeCachedConstraintAt(i);
      } else {
        ++i;
      }
    }
  }

  void CollisionScene::removeCachedConstraintAt(int index) {
    assert(index >= 0 && index < cachedConstraintCount_);

    const int lastIndex = cachedConstraintCount_ - 1;
    cachedConstraintLookup_.erase(cachedConstraints_[index].key);

    if(index != lastIndex) {
      cachedConstraints_[index] = cachedConstraints_[lastIndex];
      cachedConstraintLookup_[cachedConstraints_[index].key] = index;
    }

    cachedConstraints_.pop_back();
    cachedConstraintCount_ = static_cast<int>(cachedConstraints_.size());
  }

  CollEvent CollisionScene::makeCollisionEvent(const ContactConstraint &constraint) const {
    CollEvent event{};
    event.selfCollider = constraint.colliderA;
    event.hitCollider = constraint.colliderB;
    event.selfMeshCollider = constraint.meshColliderA;
    event.hitMeshCollider = constraint.meshColliderB;
    event.selfRigidBody = constraint.rigidBodyA;
    event.hitRigidBody = constraint.rigidBodyB;
    event.otherObject = constraint.objectB;
    event.contactCount = static_cast<uint16_t>(std::min(constraint.pointCount, MAX_CONTACT_POINTS_PER_PAIR));

    for(uint16_t i = 0; i < event.contactCount; ++i) {
      event.contacts[i] = constraint.points[i];
    }

    return event;
  }

  void CollisionScene::dispatchCollisionCallbacks() const {
    for(int i = 0; i < cachedConstraintCount_; ++i) {
      const ContactConstraint &constraint = cachedConstraints_[i];
      if(!constraint.isActive || constraint.pointCount <= 0) continue;
      if(!constraint.objectA || !constraint.objectB) continue;

      SceneManager::getCurrent().onObjectCollision(makeCollisionEvent(constraint));
    }
  }


  // ── Wake island ───────────────────────────────────────────────────

  void CollisionScene::wakeIsland(RigidBody *rigidBody) {
    if(!rigidBody) return;

    std::vector<RigidBody *> island;
    std::unordered_set<RigidBody *> visited;
    collectConnectedIsland(rigidBody, island, visited);

    if(island.empty() && shouldTrackSleepState(rigidBody)) {
      island.push_back(rigidBody);
    }

    for(RigidBody *body : island) {
      if(!body) continue;
      if(body->isSleeping) {
        body->wake();
      } else {
        body->sleepCounter = 0;
      }
    }
  }

  void CollisionScene::wakeBodiesMovedExternally() {
    std::vector<RigidBody *> wakeCandidates;

    for(RigidBody *body : rigidBodies_) {
      if(!body || !body->isSleeping) continue;
      if(!rigidBodyTransformExceededSleepThreshold(body)) continue;
      wakeCandidates.push_back(body);
    }

    for(RigidBody *body : wakeCandidates) {
      wakeIsland(body);
    }
  }

  void CollisionScene::updateSleepStates() {
    std::unordered_set<RigidBody *> visited;

    for(RigidBody *body : rigidBodies_) {
      if(!shouldTrackSleepState(body) || body->isSleeping) continue;
      if(visited.find(body) != visited.end()) continue;

      std::vector<RigidBody *> island;
      collectConnectedIsland(body, island, visited);
      if(island.empty()) continue;

      bool islandCanSleep = true;
      for(RigidBody *islandBody : island) {
        if(islandBody->isSleeping) {
          islandBody->wake();
        }

        const bool transformChangedTooMuch = rigidBodyTransformExceededSleepThreshold(islandBody);
        const bool velocitiesTooHigh = rigidBodyVelocitiesExceededSleepThreshold(islandBody);
        if(transformChangedTooMuch || velocitiesTooHigh) {
          islandCanSleep = false;
        }
      }

      if(!islandCanSleep) {
        for(RigidBody *islandBody : island) {
          islandBody->sleepCounter = 0;
        }
        continue;
      }

      bool shouldSleepIsland = true;
      for(RigidBody *islandBody : island) {
        if(islandBody->sleepCounter < std::numeric_limits<uint16_t>::max()) {
          islandBody->sleepCounter++;
        }
        if(islandBody->sleepCounter < SLEEP_STEPS) {
          shouldSleepIsland = false;
        }
      }

      if(shouldSleepIsland) {
        for(RigidBody *islandBody : island) {
          islandBody->sleep();
        }
      }
    }
  }

  // ── Contact refresh ───────────────────────────────────────────────

  void CollisionScene::refreshContacts() {
    for(int i = 0; i < cachedConstraintCount_; ++i) {
      ContactConstraint &cc = cachedConstraints_[i];
      if(!cc.isActive) continue;

      if(cc.isTrigger) {
        continue;
      }

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
          cp.contactA = *a->position + rA;
        } else if(cc.meshColliderA) {
          cp.contactA = cc.meshColliderA->toWorldSpace(cp.localPointA);
        } else {
          cp.contactA = cp.localPointA;
        }
        if(b) {
          fm_vec3_t rB = cp.localPointB;
          if(b->rotation) {
            rB = quatRotateVec(*b->rotation, rB);
          }
          cp.contactB = *b->position + rB;
        } else if(cc.meshColliderB) {
          cp.contactB = cc.meshColliderB->toWorldSpace(cp.localPointB);
        } else {
          cp.contactB = cp.localPointB;
        }

        cp.point = (cp.contactA + cp.contactB) * 0.5f;

        // Update penetration: pen = -dot(A - B, normal) = dot(B - A, normal)
        fm_vec3_t diff = cp.contactA - cp.contactB;
        cp.penetration = -fm_vec3_dot(&diff, &cc.normal);

        // Deactivate if too separated
        if(cp.penetration < -(0.1f * physicsScale_)) {
          debugf("Deactivating contact point due to separation: pen=%.4f\n", (double)cp.penetration);
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
    for(int i = 0; i < cachedConstraintCount_;) {
      ContactConstraint &cc = cachedConstraints_[i];
      if(!cc.isActive) {
        removeCachedConstraintAt(i);
      } else {
        ++i;
      }
    }
  }

  void CollisionScene::rebuildSolverConstraints() {
    solverConstraints_.clear();
    solverConstraints_.reserve(cachedConstraintCount_);

    for(int i = 0; i < cachedConstraintCount_; ++i) {
      ContactConstraint &cc = cachedConstraints_[i];
      if(!cc.isActive || cc.isTrigger || cc.pointCount <= 0) continue;
      solverConstraints_.push_back(&cc);
    }
  }

  // ── Contact detection ─────────────────────────────────────────────

  void CollisionScene::detectAllContacts() {
    detectOrderedColliderCount = 0;
    detectTriggerColliderCount = 0;
    detectOrderedBodyCount = 0;
    detectDetachedColliderCount = 0;
    detectBodyCandidateCount = 0;
    detectObjectPairCount = 0;
    detectDetachedPairCount = 0;
    detectMeshPairCount = 0;

    uint64_t stageStart = get_ticks();
    // Mark all constraints as inactive; detection will re-activate them
    for(int i = 0; i < cachedConstraintCount_; ++i) {
      cachedConstraints_[i].isActive = false;
    }
    ticksDetectDeactivate = get_ticks() - stageStart;

    struct OrderedColliderEntry {
      Collider *collider{nullptr};
      RigidBody *rigidBody{nullptr};
      bool isSleepingSolid{false};
    };

    struct OrderedBodyEntry {
      RigidBody *rigidBody{nullptr};
      const std::vector<Collider *> *colliders{nullptr};
      bool isSleepingSolid{false};
      int sortKey{0};
    };

    auto ownerHasTriggerCollider = [](const std::vector<Collider *> *ownerColliders) {
      if(!ownerColliders) return false;
      for(Collider *collider : *ownerColliders) {
        if(collider && collider->isTrigger) {
          return true;
        }
      }
      return false;
    };

    // Build deterministic processing lists so broadphase traversal does not depend on
    // scene graph order or AABB tree query result order.
    stageStart = get_ticks();
    std::vector<OrderedColliderEntry> orderedColliders;
    orderedColliders.reserve(colliders_.size());
    for(Collider *collider : colliders_) {
      if(!collider || !collider->owner) continue;

      RigidBody *rigidBody = findRigidBodyByOwner(collider->owner);
      if(collider->isTrigger) {
        ++detectTriggerColliderCount;
      }
      orderedColliders.push_back(OrderedColliderEntry{
        collider,
        rigidBody,
        rigidBody && rigidBody->isSleeping && !collider->isTrigger
      });
    }

    std::sort(orderedColliders.begin(), orderedColliders.end(), [](const OrderedColliderEntry &lhs, const OrderedColliderEntry &rhs) {
      if(lhs.isSleepingSolid != rhs.isSleepingSolid) return !lhs.isSleepingSolid;
      return lhs.collider < rhs.collider;
    });

    std::unordered_map<const Collider *, int> colliderOrder;
    colliderOrder.reserve(orderedColliders.size());
    std::vector<OrderedColliderEntry> detachedColliders;
    detachedColliders.reserve(orderedColliders.size());
    for(std::size_t i = 0; i < orderedColliders.size(); ++i) {
      const OrderedColliderEntry &entry = orderedColliders[i];
      colliderOrder[entry.collider] = static_cast<int>(i);
      if(!entry.rigidBody) {
        detachedColliders.push_back(entry);
      }
    }

    std::vector<OrderedBodyEntry> orderedBodies;
    orderedBodies.reserve(rigidBodies_.size());
    for(RigidBody *rigidBody : rigidBodies_) {
      if(!rigidBody || !rigidBody->owner) continue;

      const std::vector<Collider *> *ownerColliders = findCollidersForOwner(rigidBody->owner);
      if(!ownerColliders || ownerColliders->empty()) continue;

      int sortKey = std::numeric_limits<int>::max();
      for(Collider *collider : *ownerColliders) {
        if(!collider) continue;
        auto orderIt = colliderOrder.find(collider);
        if(orderIt == colliderOrder.end()) continue;
        sortKey = std::min(sortKey, orderIt->second);
      }
      if(sortKey == std::numeric_limits<int>::max()) continue;

      orderedBodies.push_back(OrderedBodyEntry{
        rigidBody,
        ownerColliders,
        rigidBody->isSleeping && !ownerHasTriggerCollider(ownerColliders),
        sortKey
      });
    }

    std::sort(orderedBodies.begin(), orderedBodies.end(), [](const OrderedBodyEntry &lhs, const OrderedBodyEntry &rhs) {
      if(lhs.isSleepingSolid != rhs.isSleepingSolid) return !lhs.isSleepingSolid;
      if(lhs.sortKey != rhs.sortKey) return lhs.sortKey < rhs.sortKey;
      return lhs.rigidBody < rhs.rigidBody;
    });

    std::unordered_map<const RigidBody *, int> bodyOrder;
    bodyOrder.reserve(orderedBodies.size());
    for(std::size_t i = 0; i < orderedBodies.size(); ++i) {
      bodyOrder[orderedBodies[i].rigidBody] = static_cast<int>(i);
    }

    detectOrderedColliderCount = static_cast<uint32_t>(orderedColliders.size());
    detectDetachedColliderCount = static_cast<uint32_t>(detachedColliders.size());
    detectOrderedBodyCount = static_cast<uint32_t>(orderedBodies.size());
    ticksDetectBuildOrder = get_ticks() - stageStart;

    // Collider-to-collider broad phase using the rigidbody AABB tree.
    stageStart = get_ticks();
    std::vector<NodeProxy> candidateBodies;
    candidateBodies.resize(rigidBodies_.size());
    for(const OrderedBodyEntry &entryA : orderedBodies) {
      RigidBody *rbA = entryA.rigidBody;
      if(!rbA) continue;
      if(rbA->isSleeping && !ownerHasTriggerCollider(entryA.colliders)) continue;
      if(candidateBodies.empty()) continue;

      auto orderAIt = bodyOrder.find(rbA);
      if(orderAIt == bodyOrder.end()) continue;
      const int orderA = orderAIt->second;

      const int candidateCount = rigidBodyAABBTree.queryBounds(
        rbA->boundingBox,
        candidateBodies.data(),
        static_cast<int>(candidateBodies.size())
      );

      std::vector<RigidBody *> orderedCandidates;
      orderedCandidates.reserve(candidateCount);
      for(int candidateIdx = 0; candidateIdx < candidateCount; ++candidateIdx) {
        void *data = rigidBodyAABBTree.getNodeData(candidateBodies[candidateIdx]);
        if(!data) continue;

        RigidBody *rbB = static_cast<RigidBody *>(data);
        if(!rbB || rbB == rbA || !rbB->owner) continue;
        if(rbA->owner == rbB->owner) continue;

        auto orderBIt = bodyOrder.find(rbB);
        if(orderBIt == bodyOrder.end() || orderBIt->second <= orderA) continue;

        orderedCandidates.push_back(rbB);
      }

      detectBodyCandidateCount += static_cast<uint32_t>(orderedCandidates.size());

      std::sort(orderedCandidates.begin(), orderedCandidates.end(), [&bodyOrder](const RigidBody *lhs, const RigidBody *rhs) {
        const int lhsOrder = bodyOrder.at(lhs);
        const int rhsOrder = bodyOrder.at(rhs);
        if(lhsOrder != rhsOrder) return lhsOrder < rhsOrder;
        return lhs < rhs;
      });

      for(RigidBody *rbB : orderedCandidates) {
        const std::vector<Collider *> *ownerCollidersB = findCollidersForOwner(rbB->owner);
        if(!ownerCollidersB || ownerCollidersB->empty()) continue;

        for(Collider *colliderA : *entryA.colliders) {
          if(!colliderA) continue;

          for(Collider *colliderB : *ownerCollidersB) {
            if(!colliderB) continue;
            if(!collidersShouldGenerateContact(colliderA, colliderB)) continue;
            if(!aabbOverlap(colliderA->worldAABB, colliderB->worldAABB)) continue;
            if(rbA->isSleeping && rbB->isSleeping && !colliderA->isTrigger && !colliderB->isTrigger) continue;

            ++detectObjectPairCount;
            collideDetectObjectToObject(colliderA, rbA, colliderB, rbB);
          }
        }
      }
    }
    ticksDetectBodyPairs = get_ticks() - stageStart;

    // Fallback for colliders without a rigidbody owner.
    stageStart = get_ticks();
    std::vector<NodeProxy> detachedCandidateBodies;
    detachedCandidateBodies.resize(rigidBodies_.size());

    for(const OrderedColliderEntry &entryA : detachedColliders) {
      Collider *colliderA = entryA.collider;
      if(!colliderA) continue;

      if(!detachedCandidateBodies.empty()) {
        const int candidateCount = rigidBodyAABBTree.queryBounds(
          colliderA->worldAABB,
          detachedCandidateBodies.data(),
          static_cast<int>(detachedCandidateBodies.size())
        );

        std::vector<RigidBody *> orderedCandidates;
        orderedCandidates.reserve(candidateCount);
        for(int candidateIdx = 0; candidateIdx < candidateCount; ++candidateIdx) {
          void *data = rigidBodyAABBTree.getNodeData(detachedCandidateBodies[candidateIdx]);
          if(!data) continue;

          RigidBody *rbB = static_cast<RigidBody *>(data);
          if(!rbB || !rbB->owner) continue;

          auto orderBIt = bodyOrder.find(rbB);
          if(orderBIt == bodyOrder.end()) continue;
          orderedCandidates.push_back(rbB);
        }

        std::sort(orderedCandidates.begin(), orderedCandidates.end(), [&bodyOrder](const RigidBody *lhs, const RigidBody *rhs) {
          const int lhsOrder = bodyOrder.at(lhs);
          const int rhsOrder = bodyOrder.at(rhs);
          if(lhsOrder != rhsOrder) return lhsOrder < rhsOrder;
          return lhs < rhs;
        });

        for(RigidBody *rbB : orderedCandidates) {
          const std::vector<Collider *> *ownerCollidersB = findCollidersForOwner(rbB->owner);
          if(!ownerCollidersB || ownerCollidersB->empty()) continue;

          for(Collider *colliderB : *ownerCollidersB) {
            if(!colliderB) continue;
            if(!collidersShouldGenerateContact(colliderA, colliderB)) continue;
            if(!aabbOverlap(colliderA->worldAABB, colliderB->worldAABB)) continue;

            const bool sleepingBodyNeedsWakeCheck = colliderReadsCollider(colliderB, colliderA);
            if(rbB->isSleeping && !colliderA->isTrigger && !colliderB->isTrigger && !sleepingBodyNeedsWakeCheck) continue;

            ++detectDetachedPairCount;
            collideDetectObjectToObject(colliderA, nullptr, colliderB, rbB);
          }
        }
      }
    }
    ticksDetectDetachedBodyPairs = get_ticks() - stageStart;

    stageStart = get_ticks();
    std::vector<const OrderedColliderEntry *> detachedSweep;
    detachedSweep.reserve(detachedColliders.size());
    for(const OrderedColliderEntry &entry : detachedColliders) {
      if(entry.collider) {
        detachedSweep.push_back(&entry);
      }
    }

    std::sort(detachedSweep.begin(), detachedSweep.end(), [](const OrderedColliderEntry *lhs, const OrderedColliderEntry *rhs) {
      if(lhs->collider->worldAABB.min.x != rhs->collider->worldAABB.min.x) {
        return lhs->collider->worldAABB.min.x < rhs->collider->worldAABB.min.x;
      }
      return lhs->collider < rhs->collider;
    });

    for(std::size_t i = 0; i < detachedSweep.size(); ++i) {
      Collider *colliderA = detachedSweep[i]->collider;
      if(!colliderA) continue;

      const float maxX = colliderA->worldAABB.max.x;
      for(std::size_t j = i + 1; j < detachedSweep.size(); ++j) {
        Collider *colliderB = detachedSweep[j]->collider;
        if(!colliderB) continue;
        if(colliderB->worldAABB.min.x > maxX) break;
        if(colliderA->owner == colliderB->owner) continue;
        if(!collidersShouldGenerateContact(colliderA, colliderB)) continue;
        if(!aabbOverlap(colliderA->worldAABB, colliderB->worldAABB)) continue;

        ++detectDetachedPairCount;
        collideDetectObjectToObject(colliderA, nullptr, colliderB, nullptr);
      }
    }
    ticksDetectDetachedDetachedPairs = get_ticks() - stageStart;
    ticksDetectDetachedPairs = ticksDetectDetachedBodyPairs + ticksDetectDetachedDetachedPairs;

    // Collider-to-mesh tests.
    stageStart = get_ticks();
    for(std::size_t m = 0; m < meshColliders_.size(); ++m) {
      MeshCollider *mesh = meshColliders_[m];
      if(!mesh || mesh->triangleCount <= 0) continue;

      for(const OrderedColliderEntry &entry : orderedColliders) {
        Collider *collider = entry.collider;
        RigidBody *rigidBody = entry.rigidBody;
        if(!collider || !colliderShouldTestMesh(collider))
          continue;
        if(rigidBody && rigidBody->isSleeping && !collider->isTrigger)
          continue;

        if(!aabbOverlap(collider->worldAABB, mesh->worldBoundingBox))
          continue;

        ++detectMeshPairCount;
        collideDetectObjectToMesh(collider, rigidBody, *mesh);
      }
    }
    ticksDetectMeshPairs = get_ticks() - stageStart;

    //TODO: possibly offer mesh-mesh collision detection in the future, but not needed for current use cases

    stageStart = get_ticks();
    removeInactiveContacts();
    ticksDetectCleanup = get_ticks() - stageStart;
  }

  // ── Pre-solve ─────────────────────────────────────────────────────

  void CollisionScene::preSolveContacts() {
    const float restitutionSlop = 0.5f * physicsScale_;

    for(ContactConstraint *constraint : solverConstraints_) {
      ContactConstraint &cc = *constraint;

      RigidBody *a = cc.rigidBodyA;
      RigidBody *b = cc.rigidBodyB;
      const bool aHasMotionAngular = canApplyAngularResponse(a);
      const bool bHasMotionAngular = canApplyAngularResponse(b);
      const bool aCanRotate = cc.respondsA && aHasMotionAngular;
      const bool bCanRotate = cc.respondsB && bHasMotionAngular;
      const bool aHasAngularConstraints = hasAngularConstraints(a);
      const bool bHasAngularConstraints = hasAngularConstraints(b);

      float invMassA = cc.respondsA ? constrainedLinearInvMassAlong(a, cc.normal) : 0.0f;
      float invMassB = cc.respondsB ? constrainedLinearInvMassAlong(b, cc.normal) : 0.0f;
      float totalInvMass = invMassA + invMassB;
      const float linearU = (cc.respondsA ? constrainedLinearInvMassAlong(a, cc.tangentU) : 0.0f) +
                (cc.respondsB ? constrainedLinearInvMassAlong(b, cc.tangentU) : 0.0f);
      const float linearV = (cc.respondsA ? constrainedLinearInvMassAlong(a, cc.tangentV) : 0.0f) +
                (cc.respondsB ? constrainedLinearInvMassAlong(b, cc.tangentV) : 0.0f);
      if(totalInvMass < FM_EPSILON) continue;

      for(int j = 0; j < cc.pointCount; ++j) {
        ContactPoint &cp = cc.points[j];
        if(!cp.active) continue;

        // Relative vectors from centers of mass
        cp.aToContact = a ? cp.contactA - a->worldCenterOfMass : VEC3_ZERO;
        cp.bToContact = b ? cp.contactB - b->worldCenterOfMass : VEC3_ZERO;

        // Normal effective mass: 1 / (invMassA + invMassB + (rA×n)·I_A^-1·(rA×n) + ...)
        fm_vec3_t raCrossN;
        fm_vec3_cross(&raCrossN, &cp.aToContact, &cc.normal);
        fm_vec3_t rbCrossN;
        fm_vec3_cross(&rbCrossN, &cp.bToContact, &cc.normal);

        float angularA = 0.0f;
        if(aCanRotate) {
          fm_vec3_t inertia = a->applyWorldInertia(raCrossN);
          if(aHasAngularConstraints) inertia = constrainAngularWorld(a, inertia);
          angularA = fm_vec3_dot(&raCrossN, &inertia);
        }

        float angularB = 0.0f;
        if(bCanRotate) {
          fm_vec3_t inertia = b->applyWorldInertia(rbCrossN);
          if(bHasAngularConstraints) inertia = constrainAngularWorld(b, inertia);
          angularB = fm_vec3_dot(&rbCrossN, &inertia);
        }

        float denomN = totalInvMass + angularA + angularB;
        if(denomN < FM_EPSILON) denomN = FM_EPSILON;
        cp.normalMass = 1.0f / denomN;

        // Tangent effective masses
        {
          fm_vec3_t raCrossU;
          fm_vec3_cross(&raCrossU, &cp.aToContact, &cc.tangentU);
          fm_vec3_t rbCrossU;
          fm_vec3_cross(&rbCrossU, &cp.bToContact, &cc.tangentU);
          float angU_A = 0.0f;
          if(aCanRotate) {
            fm_vec3_t inertia = a->applyWorldInertia(raCrossU);
            if(aHasAngularConstraints) inertia = constrainAngularWorld(a, inertia);
            angU_A = fm_vec3_dot(&raCrossU, &inertia);
          }
          float angU_B = 0.0f;
          if(bCanRotate) {
            fm_vec3_t inertia = b->applyWorldInertia(rbCrossU);
            if(bHasAngularConstraints) inertia = constrainAngularWorld(b, inertia);
            angU_B = fm_vec3_dot(&rbCrossU, &inertia);
          }
          float denomU = linearU + angU_A + angU_B;
          if(denomU < FM_EPSILON) denomU = FM_EPSILON;
          cp.tangentMassU = 1.0f / denomU;
        }
        {
          fm_vec3_t raCrossV;
          fm_vec3_cross(&raCrossV, &cp.aToContact, &cc.tangentV);
          fm_vec3_t rbCrossV;
          fm_vec3_cross(&rbCrossV, &cp.bToContact, &cc.tangentV);
          float angV_A = 0.0f;
          if(aCanRotate) {
            fm_vec3_t inertia = a->applyWorldInertia(raCrossV);
            if(aHasAngularConstraints) inertia = constrainAngularWorld(a, inertia);
            angV_A = fm_vec3_dot(&raCrossV, &inertia);
          }
          float angV_B = 0.0f;
          if(bCanRotate) {
            fm_vec3_t inertia = b->applyWorldInertia(rbCrossV);
            if(bHasAngularConstraints) inertia = constrainAngularWorld(b, inertia);
            angV_B = fm_vec3_dot(&rbCrossV, &inertia);
          }
          float denomV = linearV + angV_A + angV_B;
          if(denomV < FM_EPSILON) denomV = FM_EPSILON;
          cp.tangentMassV = 1.0f / denomV;
        }

        // Velocity bias (restitution only; Baumgarte is handled in position solver)
        cp.velocityBias = 0.0f;

        // Restitution bias
        fm_vec3_t relVel = VEC3_ZERO;
        if(a) {
          fm_vec3_t aCross;
          fm_vec3_cross(&aCross, &a->angularVelocity, &cp.aToContact);
          relVel = a->velocity + aCross;
        }
        if(b) {
          fm_vec3_t bCross;
          fm_vec3_cross(&bCross, &b->angularVelocity, &cp.bToContact);
          relVel -= (b->velocity + bCross);
        }
        float relVelN = fm_vec3_dot(&relVel, &cc.normal);
        if(relVelN < -restitutionSlop) {
          cp.velocityBias += cc.combinedBounce * relVelN;
        }
      }
    }
  }

  // ── Warm start ────────────────────────────────────────────────────

  void CollisionScene::warmStart() {
    for(ContactConstraint *constraint : solverConstraints_) {
      ContactConstraint &cc = *constraint;

      RigidBody *a = cc.rigidBodyA;
      RigidBody *b = cc.rigidBodyB;

      for(int j = 0; j < cc.pointCount; ++j) {
        ContactPoint &cp = cc.points[j];
        if(!cp.active) continue;

        fm_vec3_t impulse = cc.normal * cp.accumulatedNormalImpulse;
        impulse += cc.tangentU * cp.accumulatedTangentImpulseU;
        impulse += cc.tangentV * cp.accumulatedTangentImpulseV;

        if(cc.respondsA) applyConstrainedImpulseAtContact(a, impulse, cp.aToContact);
        if(cc.respondsB) applyConstrainedImpulseAtContact(b, -impulse, cp.bToContact);
      }
    }
  }

  // ── Velocity constraint solver ────────────────────────────────────

  void CollisionScene::solveVelocityConstraints() {
    for(uint8_t iter = 0; iter < velocitySolverIterations_; ++iter) {
      for(ContactConstraint *constraint : solverConstraints_) {
        ContactConstraint &cc = *constraint;

        RigidBody *a = cc.rigidBodyA;
        RigidBody *b = cc.rigidBodyB;
        const bool aHasMotionAngular = canApplyAngularResponse(a);
        const bool bHasMotionAngular = canApplyAngularResponse(b);
        const bool aCanRotate = cc.respondsA && aHasMotionAngular;
        const bool bCanRotate = cc.respondsB && bHasMotionAngular;
        const bool hasFriction = cc.combinedFriction > 0.0f;

        for(int j = 0; j < cc.pointCount; ++j) {
          ContactPoint &cp = cc.points[j];
          if(!cp.active) continue;

          // Compute relative velocity at contact
          fm_vec3_t relVel = VEC3_ZERO;
          if(a) {
            relVel = a->velocity;
            if(aHasMotionAngular) {
              fm_vec3_t aCross;
              fm_vec3_cross(&aCross, &a->angularVelocity, &cp.aToContact);
              relVel += aCross;
            }
          }
          if(b) {
            fm_vec3_t velB = b->velocity;
            if(bHasMotionAngular) {
              fm_vec3_t bCross;
              fm_vec3_cross(&bCross, &b->angularVelocity, &cp.bToContact);
              velB += bCross;
            }
            relVel -= velB;
          }

          // Normal impulse
          float relVelN = fm_vec3_dot(&relVel, &cc.normal);
          float dImpulseN = cp.normalMass * (-(relVelN + cp.velocityBias));

          // Clamp accumulated impulse (normal must be non-negative)
          float oldAccum = cp.accumulatedNormalImpulse;
          cp.accumulatedNormalImpulse = fmaxf(oldAccum + dImpulseN, 0.0f);
          dImpulseN = cp.accumulatedNormalImpulse - oldAccum;

          fm_vec3_t impulseN = cc.normal * dImpulseN;

          if(cc.respondsA) applyConstrainedImpulseAtContact(a, impulseN, cp.aToContact);
          if(cc.respondsB) applyConstrainedImpulseAtContact(b, -impulseN, cp.bToContact);

          // Friction with proper accumulation and Coulomb cone clamping.
          if(hasFriction) {
            // Recompute relative velocity after normal impulse.
            fm_vec3_t contactVelA = VEC3_ZERO;
            fm_vec3_t contactVelB = VEC3_ZERO;
            if(a && !a->isKinematic) {
              contactVelA = a->velocity;
              if(aHasMotionAngular) {
                fm_vec3_t aCross;
                fm_vec3_cross(&aCross, &a->angularVelocity, &cp.aToContact);
                contactVelA += aCross;
              }
            }
            if(b && !b->isKinematic) {
              contactVelB = b->velocity;
              if(bHasMotionAngular) {
                fm_vec3_t bCross;
                fm_vec3_cross(&bCross, &b->angularVelocity, &cp.bToContact);
                contactVelB += bCross;
              }
            }

            fm_vec3_t relVelF = contactVelA - contactVelB;
            float vTangentU = fm_vec3_dot(&relVelF, &cc.tangentU);
            float vTangentV = fm_vec3_dot(&relVelF, &cc.tangentV);

            float lambdaU = -vTangentU * cp.tangentMassU;
            float lambdaV = -vTangentV * cp.tangentMassV;

            float newAccumU = cp.accumulatedTangentImpulseU + lambdaU;
            float newAccumV = cp.accumulatedTangentImpulseV + lambdaV;

            float maxFriction = cc.combinedFriction * cp.accumulatedNormalImpulse;
            float tangentMagnitude = sqrtf(newAccumU * newAccumU + newAccumV * newAccumV);
            if(tangentMagnitude > maxFriction && tangentMagnitude > FM_EPSILON) {
              float scale = maxFriction / tangentMagnitude;
              newAccumU *= scale;
              newAccumV *= scale;
            }

            lambdaU = newAccumU - cp.accumulatedTangentImpulseU;
            lambdaV = newAccumV - cp.accumulatedTangentImpulseV;

            cp.accumulatedTangentImpulseU = newAccumU;
            cp.accumulatedTangentImpulseV = newAccumV;

            fm_vec3_t tangentImpulse = cc.tangentU * lambdaU + cc.tangentV * lambdaV;

            if(fm_vec3_len2(&tangentImpulse) > FM_EPSILON * FM_EPSILON) {
              if(cc.respondsA) applyConstrainedImpulseAtContact(a, tangentImpulse, cp.aToContact);
              if(cc.respondsB) applyConstrainedImpulseAtContact(b, -tangentImpulse, cp.bToContact);
            }
          }
        }
      }
    }
  }

  // ── Position constraint solver ────────────────────────────────────

  bool CollisionScene::solvePositionConstraints() {
    const float slop = 0.01f * physicsScale_;
    const float steering = 0.3f * physicsScale_;
    const float maxCorrection = 0.04f * physicsScale_;
    bool appliedCorrection = false;

    for(ContactConstraint *constraint : solverConstraints_) {
      ContactConstraint &cc = *constraint;

      RigidBody *a = cc.rigidBodyA;
      RigidBody *b = cc.rigidBodyB;
      const bool aCanRotate = cc.respondsA && canApplyAngularResponse(a);
      const bool bCanRotate = cc.respondsB && canApplyAngularResponse(b);
      const bool aHasAngularConstraints = hasAngularConstraints(a);
      const bool bHasAngularConstraints = hasAngularConstraints(b);
      const float invMassA = cc.respondsA ? constrainedLinearInvMassAlong(a, cc.normal) : 0.0f;
      const float invMassB = cc.respondsB ? constrainedLinearInvMassAlong(b, cc.normal) : 0.0f;

      for(int j = 0; j < cc.pointCount; ++j) {
        ContactPoint &cp = cc.points[j];
        if(!cp.active) continue;

        // Refresh world-space contacts from local anchors with current transforms
        if(a && a->position) {
          fm_vec3_t rA = cp.localPointA;
          if(a->rotation) rA = quatRotateVec(*a->rotation, rA);
          cp.contactA = *a->position + rA;
          cp.aToContact = cp.contactA - a->worldCenterOfMass;
        } else if (cc.meshColliderA) {
          cp.contactA = cc.meshColliderA->toWorldSpace(cp.localPointA);
          cp.aToContact = cp.contactA - (cc.meshColliderA->owner ? cc.meshColliderA->owner->pos : VEC3_ZERO);
        } else {
          cp.contactA = cp.localPointA;
          cp.aToContact = VEC3_ZERO;
        }

        if(b && b->position) {
          fm_vec3_t rB = cp.localPointB;
          if(b->rotation) rB = quatRotateVec(*b->rotation, rB);
          cp.contactB = *b->position + rB;
          cp.bToContact = cp.contactB - b->worldCenterOfMass;
        } else if (cc.meshColliderB) {
          cp.contactB = cc.meshColliderB->toWorldSpace(cp.localPointB);
          cp.bToContact = cp.contactB - (cc.meshColliderB->owner ? cc.meshColliderB->owner->pos : VEC3_ZERO);
        } else {
          cp.contactB = cp.localPointB;
          cp.bToContact = VEC3_ZERO;
        }

        cp.point = (cp.contactA + cp.contactB) * 0.5f;

        // Recompute penetration from refreshed contacts
        fm_vec3_t diff = cp.contactA - cp.contactB;
        cp.penetration = -fm_vec3_dot(&diff, &cc.normal);

        if(cp.penetration < slop) continue;

        float steeringForce = fminf(steering * (cp.penetration - slop), maxCorrection);
        if(steeringForce <= 0.0f) continue;

        float invMassSum = invMassA + invMassB;

        // Add rotational inertia terms
        if(aCanRotate) {
          fm_vec3_t rCrossN;
          fm_vec3_cross(&rCrossN, &cp.aToContact, &cc.normal);
          fm_vec3_t inertia = a->applyWorldInertia(rCrossN);
          if(aHasAngularConstraints) inertia = constrainAngularWorld(a, inertia);
          invMassSum += fm_vec3_dot(&rCrossN, &inertia);
        }
        if(bCanRotate) {
          fm_vec3_t rCrossN;
          fm_vec3_cross(&rCrossN, &cp.bToContact, &cc.normal);
          fm_vec3_t inertia = b->applyWorldInertia(rCrossN);
          if(bHasAngularConstraints) inertia = constrainAngularWorld(b, inertia);
          invMassSum += fm_vec3_dot(&rCrossN, &inertia);
        }

        if(invMassSum < FM_EPSILON) continue;

        float correctionMag = steeringForce / invMassSum;
        fm_vec3_t impulse = cc.normal * correctionMag;
        appliedCorrection = true;

        // Apply linear + angular corrections to A
        if(a && !a->isKinematic && a->position) {
          if(invMassA > 0.0f) {
            fm_vec3_t corrA = constrainLinearWorld(a, cc.normal * (correctionMag * invMassA));
            *a->position = *a->position + corrA;
          }
          if(aCanRotate) {
            fm_vec3_t angImpulse;
            fm_vec3_cross(&angImpulse, &cp.aToContact, &impulse);
            fm_vec3_t rotChange = a->applyWorldInertia(angImpulse);
            if(aHasAngularConstraints) rotChange = constrainAngularWorld(a, rotChange);
            float angle = fm_vec3_len(&rotChange);
            if(angle > FM_EPSILON) {
              fm_vec3_t axis = rotChange / angle;
              fm_quat_t dq;
              fm_quat_from_axis_angle(&dq, &axis, angle);
              *a->rotation = dq * *a->rotation;
              fm_quat_norm(a->rotation, a->rotation);
            }
          }
        }

        // Apply linear + angular corrections to B
        if(b && !b->isKinematic && b->position) {
          if(invMassB > 0.0f) {
            fm_vec3_t corrB = constrainLinearWorld(b, cc.normal * (correctionMag * invMassB));
            *b->position = *b->position - corrB;
          }
          if(bCanRotate) {
            fm_vec3_t angImpulse;
            fm_vec3_cross(&angImpulse, &cp.bToContact, &impulse);
            angImpulse = -angImpulse;
            fm_vec3_t rotChange = b->applyWorldInertia(angImpulse);
            if(bHasAngularConstraints) rotChange = constrainAngularWorld(b, rotChange);
            float angle = fm_vec3_len(&rotChange);
            if(angle > FM_EPSILON) {
              fm_vec3_t axis = rotChange / angle;
              fm_quat_t dq;
              fm_quat_from_axis_angle(&dq, &axis, angle);
              *b->rotation = dq * *b->rotation;
              fm_quat_norm(b->rotation, b->rotation);
            }
          }
        }
      }
    }

    return appliedCorrection;
  }

  /// @brief Recalculate the world-space AABBs of all Mesh Colliders in the Collision Scene.
  void CollisionScene::updateMeshColliderWorldAABBs() {
    std::vector<RigidBody *> wakeCandidates;
    auto addWakeCandidate = [&](RigidBody *candidate) {
      if(!candidate || !candidate->isSleeping) return;
      if(std::find(wakeCandidates.begin(), wakeCandidates.end(), candidate) == wakeCandidates.end()) {
        wakeCandidates.push_back(candidate);
      }
    };

    std::vector<NodeProxy> queryResults;
    queryResults.resize(rigidBodies_.size());

    for(std::size_t i = 0; i < meshColliders_.size(); ++i) {
      MeshCollider *mesh = meshColliders_[i];
      if(!mesh) continue;

      const bool transformChanged = mesh->ownerTransformChanged();
      if(!transformChanged && mesh->hasCachedOwnerTransform) continue;

      const AABB previousBounds = mesh->worldBoundingBox;
      mesh->recalculateWorldAABB();
      mesh->syncOwnerTransform();

      if(!transformChanged && !aabbChanged(previousBounds, mesh->worldBoundingBox)) continue;

      const AABB affectedBounds = mergeAABBs(previousBounds, mesh->worldBoundingBox);
      if(queryResults.empty()) continue;

      const int candidateCount = rigidBodyAABBTree.queryBounds(
        affectedBounds,
        queryResults.data(),
        static_cast<int>(queryResults.size())
      );

      for(int resultIdx = 0; resultIdx < candidateCount; ++resultIdx) {
        void *data = rigidBodyAABBTree.getNodeData(queryResults[resultIdx]);
        if(!data) continue;

        RigidBody *body = static_cast<RigidBody *>(data);
        if(!body || !body->isSleeping) continue;
        addWakeCandidate(body);
        
      }
    }

    for(RigidBody *candidate : wakeCandidates) {
      wakeIsland(candidate);
    }
  }

  // ── Raycast ───────────────────────────────────────────────────────

  static bool rayTriangleIntersect(const fm_vec3_t &origin, const fm_vec3_t &dir,
                                   const fm_vec3_t &v0, const fm_vec3_t &v1, const fm_vec3_t &v2,
                                   float &outDist, fm_vec3_t &outNormal) {
    fm_vec3_t e1 = v1 - v0;
    fm_vec3_t e2 = v2 - v0;
    fm_vec3_t h;
    fm_vec3_cross(&h, &dir, &e2);
    float a = fm_vec3_dot(&e1, &h);
    if(fabsf(a) < FM_EPSILON) return false;

    float f = 1.0f / a;
    fm_vec3_t s = origin - v0;
    float u = f * fm_vec3_dot(&s, &h);
    if(u < 0.0f || u > 1.0f) return false;

    fm_vec3_t q;
    fm_vec3_cross(&q, &s, &e1);
    float v = f * fm_vec3_dot(&dir, &q);
    if(v < 0.0f || u + v > 1.0f) return false;

    float t = f * fm_vec3_dot(&e2, &q);
    if(t < FM_EPSILON) return false;

    outDist = t;
    fm_vec3_t cross;
    fm_vec3_cross(&cross, &e1, &e2);
    fm_vec3_norm(&outNormal, &cross);
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
        fm_vec3_t localInvDir = fm_vec3_t{{
          fabsf(localDir.x) > FM_EPSILON ? 1.0f / localDir.x : 1e30f,
          fabsf(localDir.y) > FM_EPSILON ? 1.0f / localDir.y : 1e30f,
          fabsf(localDir.z) > FM_EPSILON ? 1.0f / localDir.z : 1e30f
        }};

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
              hit.point = ray.origin + ray.dir * dist;
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

        // if(!obj->collider) continue;
        if((obj->collisionLayers & ray.collisionLayers) == 0) continue;
        if((obj->collisionLayers & ray.ignoreLayers) != 0) continue;
        if(!ray.interactTrigger) {
          const std::vector<Collider *> *ownerColliders = findCollidersForOwner(obj->owner);
          if(ownerColliders && !ownerColliders->empty()) {
            bool hasSolidCollider = false;
            for(const Collider *ownerCollider : *ownerColliders) {
              if(ownerCollider && !ownerCollider->isTrigger) {
                hasSolidCollider = true;
                break;
              }
            }
            if(!hasSolidCollider) continue;
          }
        }

        // Simple sphere approximation for ray-rigidBody test
        fm_vec3_t toObj = obj->worldCenterOfMass - ray.origin;
        float projLen = fm_vec3_dot(&toObj, &ray.dir);
        if(projLen < 0.0f) continue;

        fm_vec3_t closestOnRay = ray.origin + ray.dir * projLen;
        float distSq = fm_vec3_distance2(&closestOnRay, &obj->worldCenterOfMass);

        // Approximate radius from AABB
        fm_vec3_t halfExtent = (obj->boundingBox.max - obj->boundingBox.min) * 0.5f;
        float approxRadius = fm_vec3_len2(&halfExtent);

        if(distSq > approxRadius) continue;

        // Refine: for spheres, do exact intersection
        //TODO: fix this
        // if(obj->collider->type == ShapeType::Sphere) {
        //   float r = obj->collider->sphere.radius;
        //   float disc = projLen * projLen - vec3MagSqrd(toObj) + r * r;
        //   if(disc < 0.0f) continue;

        //   float dist = projLen - sqrtf(disc);
        //   if(dist < FM_EPSILON || dist > ray.maxDistance) continue;
        //   if(dist >= hit.distance) continue;

        //   hit.distance = dist;
        //   hit.point = vec3Add(ray.origin, vec3Scale(ray.dir, dist));
        //   hit.normal = vec3Normalize(vec3Sub(hit.point, obj->worldCenterOfMass));
        //   hit.hitId = obj->owner ? obj->owner->id : 0;
        //   hit.didHit = true;
        // } else {
        //   // Use approximate distance for non-sphere shapes
        //   float dist = projLen - approxRadius;
        //   if(dist < FM_EPSILON) dist = projLen;
        //   if(dist > ray.maxDistance || dist >= hit.distance) continue;

        //   hit.distance = dist;
        //   hit.point = vec3Add(ray.origin, vec3Scale(ray.dir, dist));
        //   hit.normal = vec3Normalize(vec3Sub(hit.point, obj->worldCenterOfMass));
        //   hit.hitId = obj->owner ? obj->owner->id : 0;
        //   hit.didHit = true;
        // }
        tested++;
      }
    }

    return hit.didHit;
  }

  // ── Main step ─────────────────────────────────────────────────────

  void CollisionScene::step() {
    const uint64_t totalStart = get_ticks();

    uint64_t stageStart = get_ticks();

    // 0. Update mesh collider world AABBs (in case transforms changed)
    // TODO: maybe only update if transform is dirty?
    updateMeshColliderWorldAABBs();

    // 0.5 Wake sleeping dynamic bodies that were moved or rotated externally.
    wakeBodiesMovedExternally();
    ticksWakePrep = get_ticks() - stageStart;

    stageStart = get_ticks();
    // 1. Refresh collider world state
    for(Collider *collider : colliders_) {
      updateColliderWorldState(collider);
    }

    // 2. Update compound COM/inertia on demand and refresh world inertia tensors.
    for (RigidBody *body : rigidBodies_){
      if(!body) continue;
      syncCompoundProperties(body);
      if(body->isSleeping) continue;
      body->updateWorldInertia();
    }
    ticksWorldUpdate = get_ticks() - stageStart;

    stageStart = get_ticks();
    // 3. Integrate velocities
    for(RigidBody *body : rigidBodies_) {
      if(!body) continue;

      if(!body->isSleeping) {
        body->integrateVelocity(fixedDt_, gravity_);
        body->integrateAngularVelocity(fixedDt_);
      }
    }
    ticksIntegrateVel = get_ticks() - stageStart;

    // 4. Detect all contacts (broad + narrow phase)
    const uint64_t detectStart = get_ticks();
    detectAllContacts();
    ticksDetect = get_ticks() - detectStart;

    stageStart = get_ticks();
    // Refresh anchors from local-space points before solving.
    refreshContacts();
    rebuildSolverConstraints();

    dispatchCollisionCallbacks();
    ticksRefreshCallbacks = get_ticks() - stageStart;

    // 5. Pre-solve contacts (compute effective masses)
    stageStart = get_ticks();
    preSolveContacts();
    ticksPreSolve = get_ticks() - stageStart;

    // 6. Warm start
    stageStart = get_ticks();
    warmStart();
    ticksWarmStart = get_ticks() - stageStart;

    // 7. Velocity constraint solver
    stageStart = get_ticks();
    solveVelocityConstraints();
    ticksVelocitySolve = get_ticks() - stageStart;

    // 8. Integrate positions and update AABBs
    stageStart = get_ticks();
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
      }

      if(obj->aabbTreeNodeId != NULL_NODE) {
        fm_vec3_t displacement = obj->velocity * fixedDt_;
        rigidBodyAABBTree.moveNode(obj->aabbTreeNodeId, obj->boundingBox, displacement);
      }
    }
    ticksIntegratePos = get_ticks() - stageStart;

    // 9. Position constraint solver
    stageStart = get_ticks();
    for(uint8_t iter = 0; iter < positionSolverIterations_; ++iter) {
      if(!solvePositionConstraints()) {
        break;
      }
    }
    ticksPositionSolve = get_ticks() - stageStart;

    // 10. Apply position constraints and update sleep
    stageStart = get_ticks();
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
      }

      // Keep broadphase in sync with post-solve corrected transforms.
      if(!obj->isSleeping && obj->aabbTreeNodeId != NULL_NODE) {
        fm_vec3_t displacement = *obj->position - obj->prevStepPos;
        rigidBodyAABBTree.moveNode(obj->aabbTreeNodeId, obj->boundingBox, displacement);
      }
    }

    updateSleepStates();
    ticksFinalize = get_ticks() - stageStart;

    ticksTotal = get_ticks() - totalStart;
  }

  /// @brief Draws debug visuals for the collision scene.
  /// Draws on the CPU which may cause significant slowdown
  /// @param showMeshColliders Whether to draw mesh colliders.
  /// @param showRigidBodies Whether to draw rigid bodies.
  void P64::Coll::CollisionScene::debugDraw(bool showMeshColliders, bool showRigidBodies)
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
          fm_vec3_t scaled = fm_vec3_t{{local.x * scale.x, local.y * scale.y, local.z * scale.z}};
          if (useRotation)
          {
            scaled = quatRotateVec(rot, scaled);
          }
          return scaled + pos;
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
          const RigidBody *rigidBody = findRigidBodyByOwner(collider->owner);
          const bool isSleepingBody = rigidBody && rigidBody->isSleeping;

          if (isSleepingBody)
          {
            col = color_t{0x80, 0x80, 0x80, 0xFF};
          }

          switch (collider->type)
          {
          case ShapeType::Sphere:
            if (!isSleepingBody) col = color_t{0xFF, 0x00, 0x00, 0xFF};
            Debug::drawSphere(collider->worldCenter, collider->sphere.radius, col);
            break;
          case ShapeType::Box:
            if (!isSleepingBody) col = color_t{0x00, 0xFF, 0xFF, 0xFF};
            Debug::drawOBB(collider->worldCenter, collider->box.halfSize, collider->owner->rot, col);
            break;
          case ShapeType::Capsule:
            if (!isSleepingBody) col = color_t{0x00, 0x80, 0xFF, 0xFF};
            Debug::drawCapsule(
                collider->worldCenter,
                collider->capsule.radius,
                collider->capsule.innerHalfHeight,
                collider->owner->rot,
                col);
            break;
          case ShapeType::Cylinder:
            if (!isSleepingBody) col = color_t{0xFF, 0x80, 0x00, 0xFF};
            Debug::drawCylinder(
                collider->worldCenter,
                collider->cylinder.radius,
                collider->cylinder.halfHeight,
                collider->owner->rot,
                col);
            break;
          case ShapeType::Cone:
            if (!isSleepingBody) col = color_t{0xFF, 0x40, 0xA0, 0xFF};
            Debug::drawCone(
                collider->worldCenter,
                collider->cone.radius,
                collider->cone.halfHeight,
                collider->owner->rot,
                col);
            break;
          case ShapeType::Pyramid:
            if (!isSleepingBody) col = color_t{0xB0, 0xFF, 0x40, 0xFF};
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

} // namespace P64::Coll
