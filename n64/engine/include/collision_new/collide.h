/**
 * @copyright 2024 - Max Bebök
 * @license MIT
 */
#pragma once

#include "physics_object.h"
#include "mesh_collider.h"
#include "epa.h"

namespace P64::CollNew {

  struct CollisionScene; // forward declare

  void collideAddContact(PhysicsObject *object, ContactConstraint *constraint, PhysicsObject *other);
  void collideCorrectVelocity(PhysicsObject *b, const EpaResult &result, float friction, float bounce);

  void collideDetectObjectToObject(PhysicsObject *a, PhysicsObject *b);
  void collideDetectObjectToMesh(PhysicsObject *object, const MeshCollider &mesh);
  bool collideDetectObjectToTriangle(PhysicsObject *object, const MeshCollider &mesh, int triangleIndex);

  ContactConstraint *collideCacheContactConstraint(
    PhysicsObject *objectA, PhysicsObject *objectB, const EpaResult &result,
    float combinedFriction, float combinedBounce, bool isTrigger);

} // namespace P64::CollNew
