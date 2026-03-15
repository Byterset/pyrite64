/**
 * @copyright 2024 - Max Bebök
 * @license MIT
 */
#pragma once

#include "physics_object.h"
#include "mesh_collider.h"
#include "contact.h"
#include "aabb_tree.h"
#include "raycast.h"

namespace P64::CollNew {

  constexpr int MAX_PHYSICS_OBJECTS = 64;
  constexpr int MAX_ACTIVE_CONTACTS = 128;
  constexpr int MAX_CACHED_CONTACTS = 256;
  constexpr int VELOCITY_SOLVER_ITERATIONS = 5;
  constexpr int POSITION_SOLVER_ITERATIONS = 4;

  struct CollisionSceneElement {
    PhysicsObject *object{nullptr};
  };

  class CollisionScene {
  public:
    void reset();

    void addObject(PhysicsObject *object);
    void removeObject(PhysicsObject *object);
    PhysicsObject *findObject(EntityId id) const;

    void setMeshCollider(MeshCollider *mesh);
    void removeMeshCollider();

    void step();

    Contact *allocateContact();

    bool raycast(Raycast &ray, RaycastHit &hit) const;

    // Public state for access by collide functions
    CollisionSceneElement *elements{nullptr};
    Contact *nextFreeContact{nullptr};
    Contact *allContacts{nullptr};
    uint16_t objectCount{0};
    uint16_t capacity{0};
    AABBTree objectAABBTree;
    MeshCollider *meshCollider{nullptr};

    ContactConstraint *cachedConstraints{nullptr};
    int cachedConstraintCount{0};

  private:
    void releaseObjectContacts(PhysicsObject *object);
    void wakeIsland(PhysicsObject *obj);
    void refreshContacts();
    void removeInactiveContacts();
    void detectAllContacts();
    void preSolveContacts();
    void warmStart();
    void solveVelocityConstraints();
    void solvePositionConstraints();
    void fixSweptCollisions();
  };

  CollisionScene *collisionSceneGetInstance();

} // namespace P64::CollNew
