/**
 * @copyright 2024 - Max Bebök
 * @license MIT
 */
#pragma once

#include "rigid_body.h"
#include "collider_shape.h"
#include "mesh_collider.h"
#include "contact.h"
#include "aabb_tree.h"
#include "raycast.h"
#include <array>
#include <deque>
#include <functional>
#include <map>
#include <cstddef>
#include <unordered_set>
#include <unordered_map>
#include <utility>
#include <vector>

namespace P64::Coll {
  constexpr int MAX_OBJ_COLLISION_CANDIDATES = 15;
  constexpr float DEFAULT_FIXED_DT = 1.0f / 50.0f;
  constexpr fm_vec3_t DEFAULT_GRAVITY = {0.0f, -9.8f * 16.0f, 0.0f}; //scaled with Pyrites default scale for assets
  constexpr uint8_t DEFAULT_VELOCITY_SOLVER_ITERATIONS = 7;
  constexpr uint8_t DEFAULT_POSITION_SOLVER_ITERATIONS = 6;

  struct CollEvent
  {
    Collider *selfCollider{};
    Collider *hitCollider{};
    MeshCollider *selfMeshCollider{};
    MeshCollider *hitMeshCollider{};
    RigidBody *selfRigidBody{};
    RigidBody *hitRigidBody{};
    uint16_t contactCount{0};
    std::array<ContactPoint, MAX_CONTACT_POINTS_PER_PAIR> contacts{};
    Object *otherObject{};
  };

  struct ConstraintCacheKeyPart {
    const void *identity{nullptr};
    uint8_t kind{0};

    bool operator<(const ConstraintCacheKeyPart &other) const {
      if(identity != other.identity) {
        return std::less<const void *>{}(identity, other.identity);
      }
      return kind < other.kind;
    }
  };

  using ConstraintCacheKey = std::pair<ConstraintCacheKeyPart, ConstraintCacheKeyPart>;

  class CollisionScene {
  public:
    uint64_t ticksWakePrep{0};
    uint64_t ticksWorldUpdate{0};
    uint64_t ticksIntegrateVel{0};
    uint64_t ticksDetect{0};
    uint64_t ticksRefreshCallbacks{0};
    uint64_t ticksPreSolve{0};
    uint64_t ticksWarmStart{0};
    uint64_t ticksVelocitySolve{0};
    uint64_t ticksIntegratePos{0};
    uint64_t ticksPositionSolve{0};
    uint64_t ticksFinalize{0};
    uint64_t ticksTotal{0};
    uint64_t raycastCount{0};
    void debugDraw(bool showMeshColliders, bool showRigidBodies);
    void reset();

    void addRigidBody(RigidBody *rigidBody);
    void removeRigidBody(RigidBody *rigidBody);
    RigidBody *findRigidBodyByObjectId(uint16_t id) const;
    const std::vector<RigidBody *> &getRigidBodies() const { return rigidBodies_; }

    void addCollider(Collider *collider);
    void removeCollider(Collider *collider);
    const std::vector<Collider *> &getColliders() const { return colliders_; }

    void addMeshCollider(MeshCollider *mesh);
    void removeMeshCollider(MeshCollider *mesh);

    void configureSimulation(float fixedDt, const fm_vec3_t &gravity, uint8_t velocityIterations, uint8_t positionIterations);
    void wakeRigidBodyIsland(RigidBody *rigidBody);

    void step();

    int getCachedConstraintCount() const;
    ContactConstraint &getCachedConstraint(int index);
    const ContactConstraint &getCachedConstraint(int index) const;
    ContactConstraint *createCachedConstraint(
      RigidBody *rigidBodyA, Collider *colliderA, MeshCollider *meshColliderA, Object *objectA,
      RigidBody *rigidBodyB, Collider *colliderB, MeshCollider *meshColliderB, Object *objectB);
    ContactConstraint *findCachedConstraintByPair(
      Collider *colliderA, Object *objectA,
      Collider *colliderB, Object *objectB,
      const fm_vec3_t &normal, float minNormalDot);

    bool raycast(Raycast &ray, RaycastHit &hit) const;

  private:

    std::vector<RigidBody *> rigidBodies_{};
    std::unordered_map<const Object *, RigidBody *> ownerRigidBodies_{};
    std::vector<Collider *> colliders_{};
    std::unordered_map<const Object *, std::vector<Collider *>> ownerColliders_{};
    std::deque<ContactConstraint> cachedConstraints_{};
    std::map<ConstraintCacheKey, std::vector<int>> cachedConstraintPairs_{};

    AABBTree rigidBodyAABBTree;

    // Multiple mesh colliders
    std::vector<MeshCollider *> meshColliders_{};

    float fixedDt_{DEFAULT_FIXED_DT};
    fm_vec3_t gravity_{DEFAULT_GRAVITY};
    uint8_t velocitySolverIterations_{DEFAULT_VELOCITY_SOLVER_ITERATIONS};
    uint8_t positionSolverIterations_{DEFAULT_POSITION_SOLVER_ITERATIONS};

    int cachedConstraintCount_{0};

    static ConstraintCacheKeyPart makeConstraintCacheKeyPart(Collider *collider, Object *object);
    static ConstraintCacheKey makeConstraintPairKey(Collider *colliderA, Object *objectA, Collider *colliderB, Object *objectB);
    static bool shouldTrackSleepState(const RigidBody *rigidBody);
    static bool rigidBodyTransformExceededSleepThreshold(const RigidBody *rigidBody);
    static bool rigidBodyVelocitiesExceededSleepThreshold(const RigidBody *rigidBody);
    RigidBody *findRigidBodyByOwner(const Object *owner) const;
    const std::vector<Collider *> *findCollidersForOwner(const Object *owner) const;
    void updateColliderWorldState(Collider *collider) const;
    void updateCompoundProperties(RigidBody *rigidBody) const;
    void collectConnectedIsland(RigidBody *seed, std::vector<RigidBody *> &island, std::unordered_set<RigidBody *> &visited) const;
    static void addWakeCandidate(std::vector<RigidBody *> &wakeCandidates, RigidBody *candidate, RigidBody *ignoredCandidate = nullptr);
    void wakeCandidateIslands(const std::vector<RigidBody *> &wakeCandidates);
    void removeCachedConstraints(
      const std::function<bool(const ContactConstraint &)> &shouldRemove,
      std::vector<RigidBody *> &wakeCandidates,
      RigidBody *ignoredCandidate = nullptr);
    CollEvent makeCollisionEvent(const ContactConstraint &constraint) const;
    void dispatchCollisionCallbacks() const;

    void rebuildCachedConstraintPairs();
    void wakeIsland(RigidBody *rigidBody);
    void wakeBodiesMovedExternally();
    void updateSleepStates();
    void refreshContacts();
    void removeInactiveContacts();
    void detectAllContacts();
    void preSolveContacts();
    void warmStart();
    void solveVelocityConstraints();
    void solvePositionConstraints();
    void fixSweptCollisions();
    void updateMeshColliderWorldAABBs();
  };

  CollisionScene *collisionSceneGetInstance();

} // namespace P64::Coll
