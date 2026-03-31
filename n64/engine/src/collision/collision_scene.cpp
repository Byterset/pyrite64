/**
 * @file collision_scene.cpp
 * @author Kevin Reier <https://github.com/Byterset>
 * @brief Defines the Collision Scene which keeps track of physics participants and updates them (see collision_scene.h)
 */
#include "collision/collision_scene.h"
#include "collision/collide.h"
#include "collision/contact_utils.h"
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
    return collider && (collider->maskWrite != 0);
  }

  static fm_vec3_t constrainAngularWorld(const RigidBody *body, const fm_vec3_t &worldAngular) {
    if(!body) return worldAngular;
    if(!hasAngularConstraints(body)) return worldAngular;
    if(hasFlag(body->constraints, Constraint::FreezeRotAll)) return VEC3_ZERO;
    if(!body->rotation) return worldAngular;

    fm_vec3_t local = quatConjugate(*body->rotation) * worldAngular;
    if(hasFlag(body->constraints, Constraint::FreezeRotX)) local.x = 0.0f;
    if(hasFlag(body->constraints, Constraint::FreezeRotY)) local.y = 0.0f;
    if(hasFlag(body->constraints, Constraint::FreezeRotZ)) local.z = 0.0f;
    return *body->rotation * local;
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
    if(speedSq > SPEED_SLEEP_THRESHOLD_SQ * (g_scene.physicsScale_ * g_scene.physicsScale_)) return true;

    const float angSpeedSq = fm_vec3_len2(&rigidBody->angularVelocity);
    return angSpeedSq > ANGULAR_SLEEP_THRESHOLD_SQ;
  }

  bool CollisionScene::rigidBodyTransformExceededSleepThreshold(const RigidBody *rigidBody) {
    if(!shouldTrackSleepState(rigidBody)) return false;

    const float posDeltaSq = fm_vec3_distance2(rigidBody->position, &rigidBody->prevStepPos);
    if(posDeltaSq > POS_SLEEP_THRESHOLD_SQ * (g_scene.physicsScale_ * g_scene.physicsScale_)) return true;

    if(rigidBody->rotation) {
      const float rotSim = fabsf(quatDot(*rigidBody->rotation, rigidBody->prevStepRot));
      if(rotSim < ROT_SIMILARITY_SLEEP_THRESHOLD) return true;
    }

    if(rigidBody->owner){
      const float scaleDeltaSq = fm_vec3_distance2(&rigidBody->owner->scale, &rigidBody->prevStepScale);
      if(scaleDeltaSq > POS_SLEEP_THRESHOLD_SQ * (g_scene.physicsScale_ * g_scene.physicsScale_)) return true;
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
    colliderAABBTree.destroy();

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
    ticksDetectBodyPairs = 0;
    ticksDetectMeshPairs = 0;
    ticksRefreshCallbacks = 0;
    ticksPreSolve = 0;
    ticksWarmStart = 0;
    ticksVelocitySolve = 0;
    ticksIntegration = 0;
    ticksPositionSolve = 0;
    ticksFinalize = 0;
    ticksTotal = 0;

    colliderAABBTree.init(32); // Initial capacity (will grow as needed)
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
      localCenterOffset = quatConjugate(*rigidBody->rotation) * localCenterOffset;
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
        r = quatConjugate(*rigidBody->rotation) * r;
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
    rigidBody->worldAABB = AABB{worldPos, worldPos};
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

    removeCachedConstraints([rigidBody](const ContactConstraint &cc) {
      return cc.rigidBodyA == rigidBody || cc.rigidBodyB == rigidBody;
    }, wakeCandidates, rigidBody);

    // Sleeping rigidBodies overlapping the removed body are likely support-dependent and should re-evaluate.
    const AABB removedBounds = rigidBody->worldAABB;
    
    for(RigidBody *body : rigidBodies_) {
      if(!body || body == rigidBody) continue;
      if(aabbOverlap(body->worldAABB, removedBounds)) {
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
    collider->aabbTreeNodeId = colliderAABBTree.createNode(collider->worldAABB, collider);
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

    if(collider->aabbTreeNodeId != NULL_NODE) {
      colliderAABBTree.removeLeaf(collider->aabbTreeNodeId, true);
      collider->aabbTreeNodeId = NULL_NODE;
    }

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

    struct ObjectPairHash
    {
      size_t operator()(std::pair<const Object *, const Object *> p) const
      {
        uintptr_t a = (uintptr_t)p.first;
        uintptr_t b = (uintptr_t)p.second;
        // Canonical order so (a,b) == (b,a)
        if (a > b)
          std::swap(a, b);
        // Combine with a good mixer
        return a * 2654435761ULL ^ (b * 2246822519ULL);
      }
    };
    std::vector<CollEvent> pendingEvents;
    pendingEvents.reserve(cachedConstraintCount_);
    std::unordered_set<std::pair<const Object *, const Object *>, ObjectPairHash> processedPairs;

    for(int i = 0; i < cachedConstraintCount_; ++i) {
      const ContactConstraint &constraint = cachedConstraints_[i];
      if(!constraint.isActive || constraint.pointCount <= 0) continue;
      if(!constraint.objectA || !constraint.objectB) continue;
      if(!constraint.objectA->isEnabled() || !constraint.objectB->isEnabled()) continue;

      auto key = std::make_pair(constraint.objectA, constraint.objectB);
      // make sure we only send one event per object pair, even if they have multiple contact constraints
      if (processedPairs.insert(key).second)
      {
        // first time seeing this pair
        pendingEvents.push_back(makeCollisionEvent(constraint));
      }
      
    }

    for(const CollEvent &event : pendingEvents) {
      SceneManager::getCurrent().onObjectCollision(event);
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

  void CollisionScene::wakeBodiesTransformedExternally() {
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

      for(int j = 0; j < cc.pointCount; ++j) {
        ContactPoint &cp = cc.points[j];
        if(!cp.active) continue;

        refreshContactPointWorldState(cp, cc);

        // Deactivate if too separated
        if(cp.penetration < -(0.1f * physicsScale_)) {
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

    uint64_t stageStart = get_ticks();
    // Mark all constraints as inactive; detection will re-activate them
    for(int i = 0; i < cachedConstraintCount_; ++i) {
      cachedConstraints_[i].isActive = false;
    }

    //map of unique collider pairs that have already been tested this step to avoid duplication
    std::unordered_set<int32_t> tested_pairs;

    //list of candidate colliders for broad phase query results
    std::vector<NodeProxy> candidateColliders;

    
    
    candidateColliders.resize(colliders_.size());
    for (Collider *collider : colliders_)
    {
      if (!collider || !collider->owner)
        continue;
      if (collider->isTrigger)
        continue;

      RigidBody *rbA = findRigidBodyByOwner(collider->owner);

      const int candidateCount = colliderAABBTree.queryBounds(
          collider->worldAABB,
          candidateColliders.data(),
          static_cast<int>(candidateColliders.size()));
      for (int candidateIdx = 0; candidateIdx < candidateCount; ++candidateIdx)
      {
        void *data = colliderAABBTree.getNodeData(candidateColliders[candidateIdx]);
        if (!data)
          continue;
        
        Collider *collB = static_cast<Collider *>(data);

        // When you get a candidate pair:
        auto key = AABBTree::makeNodePairKey(collider->aabbTreeNodeId, collB->aabbTreeNodeId);
        if (tested_pairs.insert(key).second)
        {
          // Was not present -> test this pair
          // don't let collider collide with itself or colliders of the same object
          if (!collB || collB == collider || !collB->owner)
            continue;
          if (collider->owner == collB->owner)
            continue;

          if (!collidersShouldGenerateContact(collider, collB))
            continue;
          RigidBody *rbB = findRigidBodyByOwner(collB->owner);
          if((rbA && rbA->isSleeping) && (rbB && rbB->isSleeping)) {
            // Allow sleeping objects to generate contacts with triggers, but skip if both are sleeping non-triggers to save performance
            if(!collider->isTrigger && !collB->isTrigger) {
              continue;
            }
          }
          collideDetectObjectToObject(collider, rbA, collB, rbB);
        }
      }
    }
    for (std::size_t m = 0; m < meshColliders_.size(); ++m)
    {

      MeshCollider *mesh = meshColliders_[m];

      if (!mesh || mesh->triangleCount <= 0)
        continue;

      const int candidateCount = colliderAABBTree.queryBounds(
          mesh->worldBoundingBox,
          candidateColliders.data(),
          static_cast<int>(candidateColliders.size()));

      for (int candidateIdx = 0; candidateIdx < candidateCount; ++candidateIdx)
      {
        void *data = colliderAABBTree.getNodeData(candidateColliders[candidateIdx]);
        if (!data)
          continue;

        Collider *collA = static_cast<Collider *>(data);
        if(!collA || !collA->owner)
          continue;

        RigidBody *rigidBodyA = findRigidBodyByOwner(collA->owner);
        if (!colliderShouldTestMesh(collA))
          continue;

        //prevent perpetural collision checks of sleeping objects with meshes
        if(!mesh->transformChanged && rigidBodyA && rigidBodyA->isSleeping && !collA->isTrigger)
          continue;
        collideDetectObjectToMesh(collA, rigidBodyA, *mesh);
      }

    }
    //TODO: possibly offer mesh-mesh collision detection in the future, but not needed for current use cases

    removeInactiveContacts();
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

        refreshContactPointWorldState(cp, cc, true);

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
  void CollisionScene::updateMeshColliderWorldStates() {
    for(std::size_t i = 0; i < meshColliders_.size(); ++i) {
      MeshCollider *mesh = meshColliders_[i];
      if(!mesh) continue;

      mesh->transformChanged = mesh->ownerTransformChanged();
      if(!mesh->transformChanged && mesh->hasCachedOwnerTransform) continue;

      const AABB previousBounds = mesh->worldBoundingBox;
      mesh->recalculateWorldAABB();
      mesh->syncOwnerTransform();
    }
  }

  // ── Raycast ───────────────────────────────────────────────────────


  bool CollisionScene::raycast(Raycast &ray, RaycastHit &hit) const {
    RaycastHit currentHit = {};
    hit.didHit = false;
    hit.distance = std::numeric_limits<float>::max();
    currentHit.distance = std::numeric_limits<float>::max();

    // Test mesh colliders
    if(hasFlag(ray.collTypes, RaycastColliderTypeFlags::MESH_COLLIDERS)) {
      for(std::size_t m = 0; m < meshColliders_.size(); ++m) {
        const MeshCollider *mesh = meshColliders_[m];
        if(!mesh || mesh->triangleCount == 0 || !mesh->owner) continue;
        Raycast localRay = ray;
        localRay.origin = mesh->hasTransform() ? mesh->toLocalSpace(ray.origin) : ray.origin;
        localRay.dir = mesh->hasTransform() ? mesh->rotateToLocal(ray.dir) : ray.dir;
        localRay.invDir = fm_vec3_t{{
          fabsf(localRay.dir.x) > FM_EPSILON ? 1.0f / localRay.dir.x : FM_EPSILON,
          fabsf(localRay.dir.y) > FM_EPSILON ? 1.0f / localRay.dir.y : FM_EPSILON,
          fabsf(localRay.dir.z) > FM_EPSILON ? 1.0f / localRay.dir.z : FM_EPSILON
        }};


        NodeProxy triCandidates[RAYCAST_MAX_TRIANGLE_TESTS];
        int triCount = mesh->aabbTree.queryRay(localRay, triCandidates, RAYCAST_MAX_TRIANGLE_TESTS);
        for(int i = 0; i < triCount; ++i) {
          void *data = mesh->aabbTree.getNodeData(triCandidates[i]);
          if(!data) continue;
          int triIdx = static_cast<int>(reinterpret_cast<intptr_t>(data)) - 1; // stored as index+1
          if(triIdx < 0 || triIdx >= mesh->triangleCount) continue;

          const MeshTriangleIndices &tri = mesh->triangles[triIdx];

          // Get vertices in world space
          fm_vec3_t v0 = mesh->vertices[tri.indices[0]];
          fm_vec3_t v1 = mesh->vertices[tri.indices[1]];
          fm_vec3_t v2 = mesh->vertices[tri.indices[2]];

          currentHit.distance = std::numeric_limits<float>::max();
          currentHit.didHit = false;
          hit.didHit = hit.didHit | ray_triangle_intersection(localRay, v0, v1, v2, mesh->normals[triIdx], currentHit);
          if(currentHit.didHit && currentHit.distance < hit.distance && currentHit.distance <= ray.maxDistance) {
            // Transform hit point and normal back to world space
            hit.point = mesh->hasTransform() ? mesh->toWorldSpace(currentHit.point) : currentHit.point;
            hit.normal = mesh->hasTransform() ? mesh->rotateToWorld(currentHit.normal) : currentHit.normal;
            hit.distance = currentHit.distance;
            hit.hitObjectId = mesh->owner->id;
          }
        }
      }
    }

    // Test physics objects
    if(hasFlag(ray.collTypes, RaycastColliderTypeFlags::COLLIDER_BODIES)) {
      NodeProxy collCandidates[RAYCAST_MAX_COLLIDER_TESTS];
      int candidate_count = colliderAABBTree.queryRay(ray, collCandidates, RAYCAST_MAX_COLLIDER_TESTS);

      for(int i = 0; i < candidate_count; ++i) {
        void *data = colliderAABBTree.getNodeData(collCandidates[i]);
        if(!data) continue;
        auto *coll = static_cast<Collider *>(data);
        
        if(!coll->owner) continue;
        if(!ray.interactTrigger && coll->isTrigger) continue;
        if((coll->maskWrite & ray.readMask) == 0) continue;
        currentHit.didHit = false;
        currentHit.distance = std::numeric_limits<float>::max();
        hit.didHit = hit.didHit | ray_collider_intersection(ray, coll, currentHit);
        if(currentHit.didHit && currentHit.distance < hit.distance && currentHit.distance <= ray.maxDistance) {
          hit = currentHit;
        }
      }
    }

    return hit.didHit;
  }

  // ── Main step ─────────────────────────────────────────────────────

  void CollisionScene::step() {
    const uint64_t totalStart = get_ticks();

    uint64_t stageStart = get_ticks();

    // 0. Update mesh collider world states (in case transforms changed)
    // recalculates world AABBs and marks if transform changed for potential broadphase optimization
    updateMeshColliderWorldStates();

    // 0.5 Wake sleeping dynamic bodies that were moved or rotated externally.
    wakeBodiesTransformedExternally();
    ticksWakePrep = get_ticks() - stageStart;

    stageStart = get_ticks();
    // 1. Refresh collider world state
    for(Collider *collider : colliders_) {
      updateColliderWorldState(collider);
    }

    // 2. Update compound CoM/inertia on demand and refresh world inertia tensors.
    for (RigidBody *body : rigidBodies_){
      syncCompoundProperties(body);
      if(body->isSleeping) continue;
      body->updateWorldInertia();
    }
    ticksWorldUpdate = get_ticks() - stageStart;

    stageStart = get_ticks();
    // 3. Integrate velocities
    for(RigidBody *body : rigidBodies_) {
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

    // 8. Integrate positions and rotations
    stageStart = get_ticks();
    for(RigidBody *body : rigidBodies_) {
      if(body->isSleeping) continue;

      body->integratePosition(fixedDt_);
      body->integrateRotation(fixedDt_);
    }
    ticksIntegration = get_ticks() - stageStart;

    // 9. Position constraint solver
    stageStart = get_ticks();
    for(uint8_t iter = 0; iter < positionSolverIterations_; ++iter) {
      if(!solvePositionConstraints()) {
        break;
      }
    }
    ticksPositionSolve = get_ticks() - stageStart;

    // 10. Apply position constraints, inertia and world state of rigidbodies and colliders
    stageStart = get_ticks();
    for(RigidBody *body : rigidBodies_) {

      if(!body) continue;

      body->applyPositionConstraints();
      body->updateWorldInertia();
    }
    for(Collider *collider : colliders_) {
      if(!collider) continue;
      RigidBody *body = findRigidBodyByOwner(collider->owner);
      updateColliderWorldState(collider);
      fm_vec3_t displacement = body ? (*body->position - body->prevStepPos) : VEC3_ZERO;
      if(body){
        body->worldAABB.min = vec3Min(body->worldAABB.min, collider->worldAABB.min);
        body->worldAABB.max = vec3Max(body->worldAABB.max, collider->worldAABB.max);
      }
      if(collider->aabbTreeNodeId != NULL_NODE)
      {
        colliderAABBTree.moveNode(collider->aabbTreeNodeId, collider->worldAABB, displacement);
      }
    }

    // 11. Update RigidBody Sleep
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
            scaled = rot * scaled;
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
