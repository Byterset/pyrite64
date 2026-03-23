/**
 * @copyright 2024 - Max Bebök
 * @license MIT
 */
#pragma once

#include "rigid_body.h"
#include "mesh_collider.h"
#include "epa.h"

namespace P64::CollNew {

  struct CollisionScene; // forward declare

  void collideAddContact(RigidBody *rigidBody, ContactConstraint *constraint, RigidBody *other);

  void collideDetectObjectToObject(Collider *colliderA, RigidBody *rigidBodyA, Collider *colliderB, RigidBody *rigidBodyB);
  void collideDetectObjectToMesh(Collider *collider, RigidBody *rigidBody, const MeshCollider &mesh);
  bool collideDetectObjectToTriangle(Collider *collider, RigidBody *rigidBody, const MeshCollider &mesh, int triangleIndex);

  ContactConstraint *collideCacheContactConstraint(
    RigidBody *rigidBodyA, Collider *colliderA, Object *objectA,
    RigidBody *rigidBodyB, Collider *colliderB, Object *objectB, const EpaResult &result,
    float combinedFriction, float combinedBounce, bool isTrigger);

} // namespace P64::CollNew
