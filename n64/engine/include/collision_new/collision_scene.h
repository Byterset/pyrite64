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
#include <map>
#include <cstddef>
#include <vector>

namespace P64::CollNew {
  constexpr int MAX_OBJ_COLLISION_CANDIDATES = 15;
  constexpr int MAX_ACTIVE_CONTACTS = 128;
  constexpr float DEFAULT_FIXED_DT = 1.0f / 50.0f;
  constexpr fm_vec3_t DEFAULT_GRAVITY = {0.0f, -9.8f * 16.0f, 0.0f}; //scaled with Pyrites default scale for assets
  constexpr uint8_t DEFAULT_VELOCITY_SOLVER_ITERATIONS = 7;
  constexpr uint8_t DEFAULT_POSITION_SOLVER_ITERATIONS = 6;

  class CollisionScene {
  public:
    uint64_t ticks{0};
    uint64_t ticksBVH{0};
    uint64_t raycastCount{0};
    void debugDraw(bool showMeshColliders, bool showRigidBodies);
    void reset();

    void addRigidBody(RigidBody *rigidBody);
    void removeRigidBody(RigidBody *rigidBody);
    RigidBody *findRigidBody(uint16_t id) const;
    const std::vector<RigidBody *> &getRigidBodies() const { return rigidBodies_; }

    void addCollider(Collider *collider);
    void removeCollider(Collider *collider);
    Collider *findCollider(uint16_t id) const;
    const std::vector<Collider *> &getColliders() const { return colliders_; }

    void addMeshCollider(MeshCollider *mesh);
    void removeMeshCollider(MeshCollider *mesh);

    void configureSimulation(float fixedDt, const fm_vec3_t &gravity, uint8_t velocityIterations, uint8_t positionIterations);

    void step();

    Contact *allocateContact();
    int getCachedConstraintCount() const;
    ContactConstraint &getCachedConstraint(int index);
    const ContactConstraint &getCachedConstraint(int index) const;
    ContactConstraint *createCachedConstraint(
      RigidBody *rigidBodyA, Collider *colliderA, Object *objectA,
      RigidBody *rigidBodyB, Collider *colliderB, Object *objectB);
    ContactConstraint *findCachedConstraintByPair(
      Collider *colliderA, Collider *colliderB,
      const fm_vec3_t &normal, float minNormalDot);

    bool raycast(Raycast &ray, RaycastHit &hit) const;

  private:

    std::vector<RigidBody *> rigidBodies_{};
    std::vector<Collider *> colliders_{};
    std::array<Contact, MAX_ACTIVE_CONTACTS> contacts_{};
    std::deque<ContactConstraint> cachedConstraints_{};
    std::map<std::pair<Collider *, Collider *>, std::vector<int>> cachedConstraintPairs_{};

    Contact *nextFreeContact_{nullptr};
    AABBTree rigidBodyAABBTree;

    // Multiple mesh colliders
    std::vector<MeshCollider *> meshColliders_{};

    float fixedDt_{DEFAULT_FIXED_DT};
    fm_vec3_t gravity_{DEFAULT_GRAVITY};
    uint8_t velocitySolverIterations_{DEFAULT_VELOCITY_SOLVER_ITERATIONS};
    uint8_t positionSolverIterations_{DEFAULT_POSITION_SOLVER_ITERATIONS};

    int cachedConstraintCount_{0};

    static std::pair<Collider *, Collider *> makeColliderPairKey(Collider *a, Collider *b);
    RigidBody *findRigidBodyForObject(const Object *owner) const;
    void updateColliderWorldState(Collider *collider) const;
    void updateCompoundProperties(RigidBody *rigidBody) const;

    void releaseObjectContacts(RigidBody *rigidBody);
    void rebuildCachedConstraintPairs();
    void wakeIsland(RigidBody *rigidBody);
    void refreshContacts();
    void removeInactiveContacts();
    void detectAllContacts();
    void preSolveContacts();
    void warmStart();
    void solveVelocityConstraints();
    void solvePositionConstraints();
    void fixSweptCollisions();
    void updateMeshColliderAABBs();
  };

  CollisionScene *collisionSceneGetInstance();

} // namespace P64::CollNew
