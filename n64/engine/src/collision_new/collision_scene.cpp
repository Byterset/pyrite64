/**
 * @copyright 2024 - Max Bebök
 * @license MIT
 */
#include "collision_new/collision_scene.h"
#include "collision_new/collide.h"
#include "collision_new/collide_swept.h"
#include "collision_new/gjk.h"

#include <cstdlib>
#include <cstring>
#include <cmath>
#include <cassert>

namespace P64::CollNew {

  static CollisionScene g_scene;

  CollisionScene *collisionSceneGetInstance() {
    return &g_scene;
  }

  // ── Reset / Init ──────────────────────────────────────────────────

  void CollisionScene::reset() {
    // Free previous allocations
    if(elements) { free(elements); elements = nullptr; }
    if(allContacts) { free(allContacts); allContacts = nullptr; }
    if(cachedConstraints) { free(cachedConstraints); cachedConstraints = nullptr; }
    objectAABBTree.destroy();

    capacity = MAX_PHYSICS_OBJECTS;
    objectCount = 0;
    meshCollider = nullptr;
    cachedConstraintCount = 0;

    elements = static_cast<CollisionSceneElement *>(
      malloc(sizeof(CollisionSceneElement) * capacity));
    for(int i = 0; i < capacity; ++i) {
      elements[i] = CollisionSceneElement{};
    }

    // Allocate contact pool as a free list
    allContacts = static_cast<Contact *>(
      malloc(sizeof(Contact) * MAX_ACTIVE_CONTACTS));
    for(int i = 0; i < MAX_ACTIVE_CONTACTS; ++i) {
      allContacts[i] = Contact{};
      allContacts[i].next = (i + 1 < MAX_ACTIVE_CONTACTS) ? &allContacts[i + 1] : nullptr;
    }
    nextFreeContact = &allContacts[0];

    // Allocate constraint cache
    cachedConstraints = static_cast<ContactConstraint *>(
      malloc(sizeof(ContactConstraint) * MAX_CACHED_CONTACTS));
    for(int i = 0; i < MAX_CACHED_CONTACTS; ++i) {
      cachedConstraints[i] = ContactConstraint{};
    }

    objectAABBTree.init(capacity);
  }

  // ── Object management ─────────────────────────────────────────────

  void CollisionScene::addObject(PhysicsObject *object) {
    if(!object || objectCount >= capacity) return;

    elements[objectCount].object = object;
    objectCount++;

    // Create AABB tree node
    object->recalculateAABB();
    object->aabbTreeNodeId = objectAABBTree.createNode(object->boundingBox, object);
  }

  void CollisionScene::removeObject(PhysicsObject *object) {
    if(!object) return;

    // Release contacts
    releaseObjectContacts(object);

    // Remove from AABB tree
    if(object->aabbTreeNodeId != NULL_NODE) {
      objectAABBTree.removeLeaf(object->aabbTreeNodeId, true);
      object->aabbTreeNodeId = NULL_NODE;
    }

    // Remove cached constraints referencing this object
    for(int i = cachedConstraintCount - 1; i >= 0; --i) {
      ContactConstraint &cc = cachedConstraints[i];
      if(cc.objectA == object || cc.objectB == object) {
        // Swap with last
        if(i < cachedConstraintCount - 1) {
          cachedConstraints[i] = cachedConstraints[cachedConstraintCount - 1];
        }
        cachedConstraintCount--;
      }
    }

    // Remove from elements array
    for(int i = 0; i < objectCount; ++i) {
      if(elements[i].object == object) {
        elements[i] = elements[objectCount - 1];
        elements[objectCount - 1].object = nullptr;
        objectCount--;
        break;
      }
    }
  }

  PhysicsObject *CollisionScene::findObject(EntityId id) const {
    for(int i = 0; i < objectCount; ++i) {
      if(elements[i].object && elements[i].object->entityId == id) {
        return elements[i].object;
      }
    }
    return nullptr;
  }

  void CollisionScene::setMeshCollider(MeshCollider *mesh) {
    meshCollider = mesh;
  }

  void CollisionScene::removeMeshCollider() {
    meshCollider = nullptr;
  }

  // ── Contact pool management ───────────────────────────────────────

  Contact *CollisionScene::allocateContact() {
    if(!nextFreeContact) return nullptr;
    Contact *c = nextFreeContact;
    nextFreeContact = c->next;
    *c = Contact{};
    return c;
  }

  void CollisionScene::releaseObjectContacts(PhysicsObject *object) {
    Contact *c = object->activeContacts;
    while(c) {
      Contact *next = c->next;
      // Return to free list
      c->constraint = nullptr;
      c->otherObject = nullptr;
      c->next = nextFreeContact;
      nextFreeContact = c;
      c = next;
    }
    object->activeContacts = nullptr;
  }

  // ── Wake island ───────────────────────────────────────────────────

  void CollisionScene::wakeIsland(PhysicsObject *obj) {
    if(!obj || !obj->isSleeping) return;
    obj->wake();

    // Wake connected objects through contacts
    for(Contact *c = obj->activeContacts; c; c = c->next) {
      if(c->otherObject && c->otherObject->isSleeping) {
        wakeIsland(c->otherObject);
      }
    }
  }

  // ── Contact refresh ───────────────────────────────────────────────

  void CollisionScene::refreshContacts() {
    for(int i = 0; i < cachedConstraintCount; ++i) {
      ContactConstraint &cc = cachedConstraints[i];
      if(!cc.isActive) continue;

      PhysicsObject *a = cc.objectA;
      PhysicsObject *b = cc.objectB;

      for(int j = 0; j < cc.pointCount; ++j) {
        ContactPoint &cp = cc.points[j];
        if(!cp.active) continue;

        // Recompute world-space contact points from local coordinates
        if(a && a->rotation) {
          cp.contactA = vec3Add(a->worldCenterOfMass, quatRotateVec(*a->rotation, cp.localPointA));
        }
        if(b && b->rotation) {
          cp.contactB = vec3Add(b->worldCenterOfMass, quatRotateVec(*b->rotation, cp.localPointB));
        }

        // Update penetration
        fm_vec3_t diff = vec3Sub(cp.contactB, cp.contactA);
        cp.penetration = vec3Dot(diff, cc.normal);

        // Deactivate if too separated
        if(cp.penetration < -0.1f) {
          cp.active = false;
        }
      }
    }
  }

  void CollisionScene::removeInactiveContacts() {
    for(int i = cachedConstraintCount - 1; i >= 0; --i) {
      ContactConstraint &cc = cachedConstraints[i];
      if(!cc.isActive) {
        if(i < cachedConstraintCount - 1) {
          cachedConstraints[i] = cachedConstraints[cachedConstraintCount - 1];
        }
        cachedConstraintCount--;
      }
    }
  }

  // ── Contact detection ─────────────────────────────────────────────

  void CollisionScene::detectAllContacts() {
    // Mark all constraints as inactive; detection will re-activate them
    for(int i = 0; i < cachedConstraintCount; ++i) {
      cachedConstraints[i].isActive = false;
    }

    // Object-to-object broad phase using AABB tree
    for(int i = 0; i < objectCount; ++i) {
      PhysicsObject *a = elements[i].object;
      if(!a || !a->collider) continue;
      if(a->isSleeping && !a->isTrigger) continue;

      // Query AABB tree for potential collisions
      NodeProxy candidates[MAX_PHYSICS_OBJECTS];
      int count = objectAABBTree.queryBounds(a->boundingBox, candidates, MAX_PHYSICS_OBJECTS);

      for(int j = 0; j < count; ++j) {
        void *data = objectAABBTree.getNodeData(candidates[j]);
        if(!data) continue;
        auto *b = static_cast<PhysicsObject *>(data);
        if(b == a) continue;
        // Avoid duplicate pairs
        if(b->entityId <= a->entityId) continue;

        collideDetectObjectToObject(a, b);
      }
    }

    // Object-to-mesh
    if(meshCollider) {
      for(int i = 0; i < objectCount; ++i) {
        PhysicsObject *obj = elements[i].object;
        if(!obj || !obj->collider) continue;
        if(obj->isTrigger) continue;

        collideDetectObjectToMesh(obj, *meshCollider);
      }
    }

    removeInactiveContacts();
  }

  // ── Pre-solve ─────────────────────────────────────────────────────

  void CollisionScene::preSolveContacts() {
    constexpr float BAUMGARTE = 0.2f;
    constexpr float SLOP = 0.005f;
    constexpr float RESTITUTION_SLOP = 1.0f;

    for(int i = 0; i < cachedConstraintCount; ++i) {
      ContactConstraint &cc = cachedConstraints[i];
      if(!cc.isActive || cc.isTrigger) continue;

      PhysicsObject *a = cc.objectA;
      PhysicsObject *b = cc.objectB;

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

        cp.normalMass = 1.0f / (totalInvMass + angularA + angularB);

        // Tangent effective masses
        {
          fm_vec3_t raCrossU = vec3Cross(cp.aToContact, cc.tangentU);
          fm_vec3_t rbCrossU = vec3Cross(cp.bToContact, cc.tangentU);
          float angU_A = a ? vec3Dot(raCrossU, a->applyWorldInertia(raCrossU)) : 0.0f;
          float angU_B = b ? vec3Dot(rbCrossU, b->applyWorldInertia(rbCrossU)) : 0.0f;
          cp.tangentMassU = 1.0f / (totalInvMass + angU_A + angU_B);
        }
        {
          fm_vec3_t raCrossV = vec3Cross(cp.aToContact, cc.tangentV);
          fm_vec3_t rbCrossV = vec3Cross(cp.bToContact, cc.tangentV);
          float angV_A = a ? vec3Dot(raCrossV, a->applyWorldInertia(raCrossV)) : 0.0f;
          float angV_B = b ? vec3Dot(rbCrossV, b->applyWorldInertia(rbCrossV)) : 0.0f;
          cp.tangentMassV = 1.0f / (totalInvMass + angV_A + angV_B);
        }

        // Velocity bias for Baumgarte stabilization
        float penetrationError = fminf(cp.penetration - SLOP, 0.0f);
        cp.velocityBias = -BAUMGARTE * (1.0f / FIXED_DT) * penetrationError;

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
    for(int i = 0; i < cachedConstraintCount; ++i) {
      ContactConstraint &cc = cachedConstraints[i];
      if(!cc.isActive || cc.isTrigger) continue;

      PhysicsObject *a = cc.objectA;
      PhysicsObject *b = cc.objectB;

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
    for(int iter = 0; iter < VELOCITY_SOLVER_ITERATIONS; ++iter) {
      for(int i = 0; i < cachedConstraintCount; ++i) {
        ContactConstraint &cc = cachedConstraints[i];
        if(!cc.isActive || cc.isTrigger) continue;

        PhysicsObject *a = cc.objectA;
        PhysicsObject *b = cc.objectB;

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
          float dImpulseN = cp.normalMass * (-relVelN + cp.velocityBias);

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

          // Friction (tangent U)
          float maxFriction = cc.combinedFriction * cp.accumulatedNormalImpulse;
          {
            // Recompute relative velocity after normal impulse
            fm_vec3_t relVelF = vec3Zero();
            if(a) relVelF = vec3Add(a->velocity, vec3Cross(a->angularVelocity, cp.aToContact));
            if(b) relVelF = vec3Sub(relVelF, vec3Add(b->velocity, vec3Cross(b->angularVelocity, cp.bToContact)));

            float relVelU = vec3Dot(relVelF, cc.tangentU);
            float dImpulseU = cp.tangentMassU * (-relVelU);

            float oldAccumU = cp.accumulatedTangentImpulseU;
            cp.accumulatedTangentImpulseU = fmaxf(-maxFriction, fminf(oldAccumU + dImpulseU, maxFriction));
            dImpulseU = cp.accumulatedTangentImpulseU - oldAccumU;

            fm_vec3_t impulseU = vec3Scale(cc.tangentU, dImpulseU);
            if(a && !a->isKinematic) {
              a->velocity = vec3Add(a->velocity, vec3Scale(impulseU, a->invMass));
              a->angularVelocity = vec3Add(a->angularVelocity,
                a->applyWorldInertia(vec3Cross(cp.aToContact, impulseU)));
            }
            if(b && !b->isKinematic) {
              b->velocity = vec3Sub(b->velocity, vec3Scale(impulseU, b->invMass));
              b->angularVelocity = vec3Sub(b->angularVelocity,
                b->applyWorldInertia(vec3Cross(cp.bToContact, impulseU)));
            }
          }

          // Friction (tangent V)
          {
            fm_vec3_t relVelF = vec3Zero();
            if(a) relVelF = vec3Add(a->velocity, vec3Cross(a->angularVelocity, cp.aToContact));
            if(b) relVelF = vec3Sub(relVelF, vec3Add(b->velocity, vec3Cross(b->angularVelocity, cp.bToContact)));

            float relVelV = vec3Dot(relVelF, cc.tangentV);
            float dImpulseV = cp.tangentMassV * (-relVelV);

            float oldAccumV = cp.accumulatedTangentImpulseV;
            cp.accumulatedTangentImpulseV = fmaxf(-maxFriction, fminf(oldAccumV + dImpulseV, maxFriction));
            dImpulseV = cp.accumulatedTangentImpulseV - oldAccumV;

            fm_vec3_t impulseV = vec3Scale(cc.tangentV, dImpulseV);
            if(a && !a->isKinematic) {
              a->velocity = vec3Add(a->velocity, vec3Scale(impulseV, a->invMass));
              a->angularVelocity = vec3Add(a->angularVelocity,
                a->applyWorldInertia(vec3Cross(cp.aToContact, impulseV)));
            }
            if(b && !b->isKinematic) {
              b->velocity = vec3Sub(b->velocity, vec3Scale(impulseV, b->invMass));
              b->angularVelocity = vec3Sub(b->angularVelocity,
                b->applyWorldInertia(vec3Cross(cp.bToContact, impulseV)));
            }
          }
        }
      }
    }
  }

  // ── Position constraint solver ────────────────────────────────────

  void CollisionScene::solvePositionConstraints() {
    constexpr float BAUMGARTE_POS = 0.2f;
    constexpr float MAX_CORRECTION = 0.2f;
    constexpr float SLOP = 0.005f;

    for(int iter = 0; iter < POSITION_SOLVER_ITERATIONS; ++iter) {
      for(int i = 0; i < cachedConstraintCount; ++i) {
        ContactConstraint &cc = cachedConstraints[i];
        if(!cc.isActive || cc.isTrigger) continue;

        PhysicsObject *a = cc.objectA;
        PhysicsObject *b = cc.objectB;

        float invMassA = (a && !a->isKinematic) ? a->invMass : 0.0f;
        float invMassB = (b && !b->isKinematic) ? b->invMass : 0.0f;
        float totalInvMass = invMassA + invMassB;
        if(totalInvMass < EPSILON) continue;

        for(int j = 0; j < cc.pointCount; ++j) {
          ContactPoint &cp = cc.points[j];
          if(!cp.active) continue;

          float separation = cp.penetration - SLOP;
          if(separation >= 0.0f) continue;

          float correction = fminf(-BAUMGARTE_POS * separation / totalInvMass, MAX_CORRECTION);

          if(a && !a->isKinematic && a->position) {
            *a->position = vec3Add(*a->position, vec3Scale(cc.normal, correction * invMassA));
          }
          if(b && !b->isKinematic && b->position) {
            *b->position = vec3Sub(*b->position, vec3Scale(cc.normal, correction * invMassB));
          }
        }
      }
    }
  }

  // ── Swept collision fix ───────────────────────────────────────────

  void CollisionScene::fixSweptCollisions() {
    if(!meshCollider) return;

    for(int i = 0; i < objectCount; ++i) {
      PhysicsObject *obj = elements[i].object;
      if(!obj || !obj->collider || !obj->position) continue;
      if(obj->isTrigger || obj->isKinematic || obj->isSleeping) continue;

      // Only do swept check if the object moved a significant distance
      float moveDist = vec3DistSqrd(obj->prevStepPos, *obj->position);
      if(moveDist < 0.01f) continue;

      collideObjectToMeshSwept(obj, meshCollider, &obj->prevStepPos);
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

    // Test mesh collider
    if(meshCollider && hasFlag(ray.mask, RaycastMask::StaticCollision)) {
      NodeProxy triCandidates[64];
      int triCount = meshCollider->aabbTree.queryRay(
        ray.origin, ray.invDir, ray.maxDistance, triCandidates, 64);

      int tested = 0;
      for(int i = 0; i < triCount && tested < RAYCAST_MAX_TRIANGLE_TESTS; ++i) {
        void *data = meshCollider->aabbTree.getNodeData(triCandidates[i]);
        if(!data) continue;
        int triIdx = static_cast<int>(reinterpret_cast<intptr_t>(data));
        if(triIdx < 0 || triIdx >= meshCollider->triangleCount) continue;

        const MeshTriangleIndices &tri = meshCollider->triangles[triIdx];
        const fm_vec3_t &v0 = meshCollider->vertices[tri.indices[0]];
        const fm_vec3_t &v1 = meshCollider->vertices[tri.indices[1]];
        const fm_vec3_t &v2 = meshCollider->vertices[tri.indices[2]];

        float dist;
        fm_vec3_t normal;
        if(rayTriangleIntersect(ray.origin, ray.dir, v0, v1, v2, dist, normal)) {
          if(dist < hit.distance && dist <= ray.maxDistance) {
            hit.distance = dist;
            hit.point = vec3Add(ray.origin, vec3Scale(ray.dir, dist));
            hit.normal = normal;
            hit.hitEntityId = 0;
            hit.didHit = true;
          }
        }
        tested++;
      }
    }

    // Test physics objects
    if(hasFlag(ray.mask, RaycastMask::PhysicsObjects)) {
      NodeProxy objCandidates[MAX_PHYSICS_OBJECTS];
      int objCount = objectAABBTree.queryRay(
        ray.origin, ray.invDir, ray.maxDistance, objCandidates, MAX_PHYSICS_OBJECTS);

      int tested = 0;
      for(int i = 0; i < objCount && tested < RAYCAST_MAX_OBJECT_TESTS; ++i) {
        void *data = objectAABBTree.getNodeData(objCandidates[i]);
        if(!data) continue;
        auto *obj = static_cast<PhysicsObject *>(data);

        if(!obj->collider) continue;
        if((obj->collisionLayers & ray.collisionLayers) == 0) continue;
        if((obj->collisionLayers & ray.ignoreLayers) != 0) continue;
        if(obj->isTrigger && !ray.interactTrigger) continue;

        // Simple sphere approximation for ray-object test
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
          hit.hitEntityId = obj->entityId;
          hit.didHit = true;
        } else {
          // Use approximate distance for non-sphere shapes
          float dist = projLen - approxRadius;
          if(dist < EPSILON) dist = projLen;
          if(dist > ray.maxDistance || dist >= hit.distance) continue;

          hit.distance = dist;
          hit.point = vec3Add(ray.origin, vec3Scale(ray.dir, dist));
          hit.normal = vec3Normalize(vec3Sub(hit.point, obj->worldCenterOfMass));
          hit.hitEntityId = obj->entityId;
          hit.didHit = true;
        }
        tested++;
      }
    }

    return hit.didHit;
  }

  // ── Main step ─────────────────────────────────────────────────────

  void CollisionScene::step() {
    // 1. Update world inertia tensors and world center of mass
    for(int i = 0; i < objectCount; ++i) {
      PhysicsObject *obj = elements[i].object;
      if(!obj || obj->isSleeping) continue;
      obj->updateWorldInertia();
    }

    // 2. Integrate velocities, release old contacts
    for(int i = 0; i < objectCount; ++i) {
      PhysicsObject *obj = elements[i].object;
      if(!obj) continue;

      obj->isGrounded = false;
      releaseObjectContacts(obj);

      if(!obj->isSleeping) {
        obj->integrateVelocity();
        obj->integrateAngularVelocity();
      }
    }

    // 3. Detect all contacts (broad + narrow phase)
    detectAllContacts();

    // 4. Pre-solve contacts (compute effective masses)
    preSolveContacts();

    // 5. Warm start
    warmStart();

    // 6. Velocity constraint solver
    solveVelocityConstraints();

    // 7. Integrate positions and update AABBs
    for(int i = 0; i < objectCount; ++i) {
      PhysicsObject *obj = elements[i].object;
      if(!obj || obj->isSleeping) continue;

      obj->integratePosition();
      obj->integrateRotation();
      obj->recalculateAABB();

      if(obj->aabbTreeNodeId != NULL_NODE) {
        fm_vec3_t displacement = vec3Scale(obj->velocity, FIXED_DT);
        objectAABBTree.moveNode(obj->aabbTreeNodeId, obj->boundingBox, displacement);
      }
    }

    // 8. Position constraint solver
    solvePositionConstraints();

    // 9. Fix swept collisions (tunneling prevention)
    fixSweptCollisions();

    // 10. Apply position constraints and update sleep
    for(int i = 0; i < objectCount; ++i) {
      PhysicsObject *obj = elements[i].object;
      if(!obj) continue;

      obj->applyPositionConstraints();
      obj->updateWorldInertia();
      obj->recalculateAABB();

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

} // namespace P64::CollNew
