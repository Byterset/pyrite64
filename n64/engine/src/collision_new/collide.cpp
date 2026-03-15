/**
 * @copyright 2024 - Max Bebök
 * @license MIT
 */
#include "collision_new/collide.h"
#include "collision_new/collision_scene.h"
#include "collision_new/gjk.h"

#include <cmath>
#include <cstdlib>

namespace P64::CollNew {

  // ── Analytical collision helpers ──────────────────────────────────

  static bool analyticalSphereSphere(PhysicsObject *a, PhysicsObject *b, EpaResult &result) {
    if(a->collider->type != ShapeType::Sphere || b->collider->type != ShapeType::Sphere) return false;

    float rA = a->collider->sphere.radius;
    float rB = b->collider->sphere.radius;
    float combinedRadius = rA + rB;

    fm_vec3_t diff = vec3Sub(a->worldCenterOfMass, b->worldCenterOfMass);
    float distSq = vec3MagSqrd(diff);

    if(distSq >= combinedRadius * combinedRadius) return false;

    float dist = sqrtf(distSq);
    if(dist < EPSILON) {
      result.normal = vec3Up();
      result.penetration = combinedRadius;
      result.contactA = vec3Sub(a->worldCenterOfMass, vec3Scale(result.normal, rA));
      result.contactB = vec3Add(b->worldCenterOfMass, vec3Scale(result.normal, rB));
      return true;
    }

    result.normal = vec3Scale(diff, 1.0f / dist);
    result.penetration = combinedRadius - dist;
    result.contactA = vec3Sub(a->worldCenterOfMass, vec3Scale(result.normal, rA));
    result.contactB = vec3Add(b->worldCenterOfMass, vec3Scale(result.normal, rB));
    return true;
  }

  static bool analyticalSphereBox(PhysicsObject *sphere, PhysicsObject *box, EpaResult &result) {
    if(sphere->collider->type != ShapeType::Sphere || box->collider->type != ShapeType::Box) return false;

    float radius = sphere->collider->sphere.radius;
    fm_vec3_t halfSize = box->collider->box.halfSize;

    // Transform sphere center to box local space
    fm_vec3_t localCenter = vec3Sub(sphere->worldCenterOfMass, box->worldCenterOfMass);
    Matrix3x3 boxRotT = matrix3Transpose(box->rotationMatrix);
    fm_vec3_t localSpherePos = matrix3Vec3Mul(boxRotT, localCenter);

    // Clamp to box surface
    fm_vec3_t closest = vec3(
      fmaxf(-halfSize.x, fminf(localSpherePos.x, halfSize.x)),
      fmaxf(-halfSize.y, fminf(localSpherePos.y, halfSize.y)),
      fmaxf(-halfSize.z, fminf(localSpherePos.z, halfSize.z))
    );

    fm_vec3_t diff = vec3Sub(localSpherePos, closest);
    float distSq = vec3MagSqrd(diff);

    if(distSq >= radius * radius) return false;

    float dist = sqrtf(distSq);
    fm_vec3_t localNormal;
    if(dist < EPSILON) {
      // Sphere center is inside box — find shortest escape axis
      float dx = halfSize.x - fabsf(localSpherePos.x);
      float dy = halfSize.y - fabsf(localSpherePos.y);
      float dz = halfSize.z - fabsf(localSpherePos.z);
      if(dx <= dy && dx <= dz) {
        localNormal = vec3(copysignf(1.0f, localSpherePos.x), 0.0f, 0.0f);
        result.penetration = dx + radius;
      } else if(dy <= dz) {
        localNormal = vec3(0.0f, copysignf(1.0f, localSpherePos.y), 0.0f);
        result.penetration = dy + radius;
      } else {
        localNormal = vec3(0.0f, 0.0f, copysignf(1.0f, localSpherePos.z));
        result.penetration = dz + radius;
      }
    } else {
      localNormal = vec3Scale(diff, 1.0f / dist);
      result.penetration = radius - dist;
    }

    result.normal = matrix3Vec3Mul(box->rotationMatrix, localNormal);
    fm_vec3_t worldClosest = vec3Add(matrix3Vec3Mul(box->rotationMatrix, closest), box->worldCenterOfMass);
    result.contactB = worldClosest;
    result.contactA = vec3Sub(sphere->worldCenterOfMass, vec3Scale(result.normal, radius));
    return true;
  }

  static bool analyticalSphereCapsule(PhysicsObject *sphere, PhysicsObject *capsule, EpaResult &result) {
    if(sphere->collider->type != ShapeType::Sphere || capsule->collider->type != ShapeType::Capsule) return false;

    float rS = sphere->collider->sphere.radius;
    float rC = capsule->collider->capsule.radius;
    float hh = capsule->collider->capsule.innerHalfHeight;

    // Capsule axis endpoints in world space
    fm_vec3_t localUp = vec3(0.0f, hh, 0.0f);
    fm_vec3_t capTop = vec3Add(capsule->worldCenterOfMass, quatRotateVec(*capsule->rotation, localUp));
    fm_vec3_t capBot = vec3Sub(capsule->worldCenterOfMass, quatRotateVec(*capsule->rotation, localUp));

    // Closest point on capsule segment to sphere center
    fm_vec3_t seg = vec3Sub(capTop, capBot);
    float segLenSq = vec3MagSqrd(seg);
    float t = 0.5f;
    if(segLenSq > EPSILON) {
      t = vec3Dot(vec3Sub(sphere->worldCenterOfMass, capBot), seg) / segLenSq;
      if(t < 0.0f) t = 0.0f;
      if(t > 1.0f) t = 1.0f;
    }
    fm_vec3_t closestOnSeg = vec3Add(capBot, vec3Scale(seg, t));

    float combinedRadius = rS + rC;
    fm_vec3_t diff = vec3Sub(sphere->worldCenterOfMass, closestOnSeg);
    float distSq = vec3MagSqrd(diff);

    if(distSq >= combinedRadius * combinedRadius) return false;

    float dist = sqrtf(distSq);
    if(dist < EPSILON) {
      result.normal = vec3Up();
    } else {
      result.normal = vec3Scale(diff, 1.0f / dist);
    }

    result.penetration = combinedRadius - dist;
    result.contactA = vec3Sub(sphere->worldCenterOfMass, vec3Scale(result.normal, rS));
    result.contactB = vec3Add(closestOnSeg, vec3Scale(result.normal, rC));
    return true;
  }

  // ── Contact management ────────────────────────────────────────────

  void collideAddContact(PhysicsObject *object, ContactConstraint *constraint, PhysicsObject *other) {
    CollisionScene *scene = collisionSceneGetInstance();
    Contact *contact = scene->allocateContact();
    if(!contact) return;

    contact->constraint = constraint;
    contact->otherObject = other;
    contact->next = object->activeContacts;
    object->activeContacts = contact;
  }

  void collideCorrectVelocity(PhysicsObject *b, const EpaResult &result, float friction, float bounce) {
    if(b->isKinematic || b->isTrigger) return;

    float relVelN = vec3Dot(b->velocity, result.normal);

    // Only correct if moving into the surface
    if(relVelN >= 0.0f) return;

    // Normal impulse
    float jN = -(1.0f + bounce) * relVelN;
    fm_vec3_t normalImpulse = vec3Scale(result.normal, jN);
    b->velocity = vec3Add(b->velocity, vec3Scale(normalImpulse, b->invMass));

    // Friction
    fm_vec3_t tangentVel = vec3Sub(b->velocity, vec3Scale(result.normal, vec3Dot(b->velocity, result.normal)));
    float tangentSpeed = vec3Mag(tangentVel);
    if(tangentSpeed > EPSILON) {
      fm_vec3_t tangentDir = vec3Scale(tangentVel, 1.0f / tangentSpeed);
      float jT = fminf(friction * jN, tangentSpeed / b->invMass);
      fm_vec3_t frictionImpulse = vec3Scale(tangentDir, -jT);
      b->velocity = vec3Add(b->velocity, vec3Scale(frictionImpulse, b->invMass));
    }

    // Apply constraints
    if(hasFlag(b->constraints, Constraint::FreezePosX)) b->velocity.x = 0.0f;
    if(hasFlag(b->constraints, Constraint::FreezePosY)) b->velocity.y = 0.0f;
    if(hasFlag(b->constraints, Constraint::FreezePosZ)) b->velocity.z = 0.0f;
  }

  // ── Contact constraint caching ────────────────────────────────────

  ContactConstraint *collideCacheContactConstraint(
    PhysicsObject *objectA, PhysicsObject *objectB, const EpaResult &result,
    float combinedFriction, float combinedBounce, bool isTrigger) {

    CollisionScene *scene = collisionSceneGetInstance();
    uint16_t idB = objectB ? objectB->entityId : 0xFFFF;
    ContactPairId pid = makeContactPairId(objectA->entityId, idB);

    // Search for existing constraint with matching PID and similar normal
    ContactConstraint *existing = nullptr;
    for(int i = 0; i < scene->cachedConstraintCount; ++i) {
      ContactConstraint &cc = scene->cachedConstraints[i];
      if(cc.pid == pid && vec3Dot(cc.normal, result.normal) > 0.9f) {
        existing = &cc;
        break;
      }
    }

    if(existing) {
      // Update existing constraint
      existing->isActive = true;
      existing->normal = result.normal;
      vec3CalculateTangents(result.normal, existing->tangentU, existing->tangentV);

      // Add contact point if room
      if(existing->pointCount < MAX_CONTACT_POINTS_PER_PAIR) {
        ContactPoint &cp = existing->points[existing->pointCount];
        cp = ContactPoint{};
        cp.contactA = result.contactA;
        cp.contactB = result.contactB;
        cp.point = vec3Scale(vec3Add(result.contactA, result.contactB), 0.5f);
        cp.penetration = result.penetration;
        cp.active = true;

        // Compute local points for warm starting persistence
        if(objectA->rotation) {
          fm_vec3_t relA = vec3Sub(cp.contactA, objectA->worldCenterOfMass);
          cp.localPointA = quatRotateVec(quatConjugate(*objectA->rotation), relA);
        }
        if(objectB && objectB->rotation) {
          fm_vec3_t relB = vec3Sub(cp.contactB, objectB->worldCenterOfMass);
          cp.localPointB = quatRotateVec(quatConjugate(*objectB->rotation), relB);
        }

        existing->pointCount++;
      }
      return existing;
    }

    // Create new constraint
    if(scene->cachedConstraintCount >= MAX_CACHED_CONTACTS) return nullptr;

    ContactConstraint *cc = &scene->cachedConstraints[scene->cachedConstraintCount++];
    *cc = ContactConstraint{};
    cc->objectA = objectA;
    cc->objectB = objectB;
    cc->pid = pid;
    cc->normal = result.normal;
    vec3CalculateTangents(result.normal, cc->tangentU, cc->tangentV);
    cc->combinedFriction = combinedFriction;
    cc->combinedBounce = combinedBounce;
    cc->isActive = true;
    cc->isTrigger = isTrigger;
    cc->pointCount = 1;

    ContactPoint &cp = cc->points[0];
    cp = ContactPoint{};
    cp.contactA = result.contactA;
    cp.contactB = result.contactB;
    cp.point = vec3Scale(vec3Add(result.contactA, result.contactB), 0.5f);
    cp.penetration = result.penetration;
    cp.active = true;

    if(objectA->rotation) {
      fm_vec3_t relA = vec3Sub(cp.contactA, objectA->worldCenterOfMass);
      cp.localPointA = quatRotateVec(quatConjugate(*objectA->rotation), relA);
    }
    if(objectB && objectB->rotation) {
      fm_vec3_t relB = vec3Sub(cp.contactB, objectB->worldCenterOfMass);
      cp.localPointB = quatRotateVec(quatConjugate(*objectB->rotation), relB);
    }

    // Link contacts to objects
    collideAddContact(objectA, cc, objectB);
    if(objectB) {
      collideAddContact(objectB, cc, objectA);
    }

    return cc;
  }

  // ── Object-to-triangle ────────────────────────────────────────────

  bool collideDetectObjectToTriangle(PhysicsObject *object, const MeshCollider &mesh, int triangleIndex) {
    if(triangleIndex < 0 || triangleIndex >= mesh.triangleCount) return false;

    MeshTriangle tri;
    tri.vertices = mesh.vertices;
    tri.tri = mesh.triangles[triangleIndex];
    tri.normal = mesh.normals[triangleIndex];

    // Quick face-culling: skip if object is far behind the triangle
    float cullDist = 2.0f;
    if(object->collider->type == ShapeType::Sphere) {
      cullDist = object->collider->sphere.radius * 2.0f;
    }
    if(tri.comparePoint(object->worldCenterOfMass) < -cullDist) {
      return false;
    }

    // GJK overlap test
    fm_vec3_t firstDir = vec3Sub(object->worldCenterOfMass, tri.vertices[tri.tri.indices[0]]);
    if(vec3MagSqrd(firstDir) < EPSILON) firstDir = vec3Up();

    Simplex simplex;
    simplex.nPoints = 0;
    bool overlapping = gjkCheckForOverlap(
      simplex,
      object, physicsObjectGjkSupport,
      &tri, meshTriangleGjkSupport,
      firstDir
    );

    if(!overlapping) return false;

    EpaResult epaResult;
    bool epaOk = epaSolve(
      simplex,
      object, physicsObjectGjkSupport,
      &tri, meshTriangleGjkSupport,
      epaResult
    );

    if(!epaOk || epaResult.penetration < EPSILON) return false;

    float combinedFriction = object->collider->friction * 0.8f; // mesh assumed 0.8
    float combinedBounce = object->collider->bounce * 0.2f;

    collideCacheContactConstraint(object, nullptr, epaResult, combinedFriction, combinedBounce, false);

    // Direct velocity correction for mesh contacts
    collideCorrectVelocity(object, epaResult, combinedFriction, combinedBounce);

    // Position correction
    if(object->position && epaResult.penetration > EPSILON) {
      float correction = epaResult.penetration * 0.8f;
      *object->position = vec3Add(*object->position, vec3Scale(epaResult.normal, correction));
    }

    // Mark grounded if normal points up
    if(epaResult.normal.y > 0.5f) {
      object->isGrounded = true;
    }

    return true;
  }

  // ── Object-to-mesh ────────────────────────────────────────────────

  void collideDetectObjectToMesh(PhysicsObject *object, const MeshCollider &mesh) {
    if(!object->collider || object->isSleeping) return;
    if(mesh.triangleCount == 0) return;

    // Query AABB tree for candidate triangles
    NodeProxy candidates[32];
    int count = mesh.aabbTree.queryBounds(object->boundingBox, candidates, 32);

    for(int i = 0; i < count; ++i) {
      void *data = mesh.aabbTree.getNodeData(candidates[i]);
      if(!data) continue;
      int triIndex = static_cast<int>(reinterpret_cast<intptr_t>(data));
      collideDetectObjectToTriangle(object, mesh, triIndex);
    }
  }

  // ── Object-to-object ──────────────────────────────────────────────

  void collideDetectObjectToObject(PhysicsObject *a, PhysicsObject *b) {
    if(!a->collider || !b->collider) return;
    if(a->isSleeping && b->isSleeping) return;

    // Check collision layers
    if((a->collisionLayers & b->collisionLayers) == 0) return;

    // Check collision groups — same group doesn't collide (except 0)
    if(a->collisionGroup != 0 && a->collisionGroup == b->collisionGroup) return;

    bool isTrigger = a->isTrigger || b->isTrigger;

    float combinedFriction = a->collider->friction * b->collider->friction;
    float combinedBounce = fmaxf(a->collider->bounce, b->collider->bounce);

    // Try analytical tests first
    EpaResult result;
    bool analyticalHit = false;

    if(a->collider->type == ShapeType::Sphere && b->collider->type == ShapeType::Sphere) {
      analyticalHit = analyticalSphereSphere(a, b, result);
    } else if(a->collider->type == ShapeType::Sphere && b->collider->type == ShapeType::Box) {
      analyticalHit = analyticalSphereBox(a, b, result);
    } else if(a->collider->type == ShapeType::Sphere && b->collider->type == ShapeType::Capsule) {
      analyticalHit = analyticalSphereCapsule(a, b, result);
    } else if(b->collider->type == ShapeType::Sphere && a->collider->type == ShapeType::Box) {
      analyticalHit = analyticalSphereBox(b, a, result);
      if(analyticalHit) {
        result.normal = vec3Negate(result.normal);
        fm_vec3_t tmp = result.contactA;
        result.contactA = result.contactB;
        result.contactB = tmp;
      }
    } else if(b->collider->type == ShapeType::Sphere && a->collider->type == ShapeType::Capsule) {
      analyticalHit = analyticalSphereCapsule(b, a, result);
      if(analyticalHit) {
        result.normal = vec3Negate(result.normal);
        fm_vec3_t tmp = result.contactA;
        result.contactA = result.contactB;
        result.contactB = tmp;
      }
    }

    if(!analyticalHit) {
      // Fall back to GJK + EPA
      fm_vec3_t firstDir = vec3Sub(a->worldCenterOfMass, b->worldCenterOfMass);
      if(vec3MagSqrd(firstDir) < EPSILON) firstDir = vec3Up();

      Simplex simplex;
      simplex.nPoints = 0;
      bool overlapping = gjkCheckForOverlap(
        simplex,
        a, physicsObjectGjkSupport,
        b, physicsObjectGjkSupport,
        firstDir
      );

      if(!overlapping) return;

      bool epaOk = epaSolve(
        simplex,
        a, physicsObjectGjkSupport,
        b, physicsObjectGjkSupport,
        result
      );

      if(!epaOk || result.penetration < EPSILON) return;
    }

    // Wake sleeping objects
    if(a->isSleeping) a->wake();
    if(b->isSleeping) b->wake();

    // Cache the constraint
    ContactConstraint *cc = collideCacheContactConstraint(
      a, b, result, combinedFriction, combinedBounce, isTrigger
    );

    if(!cc) return;

    // For triggers, skip physics response
    if(isTrigger) return;

    // Immediate velocity correction for stability
    if(!a->isKinematic && !b->isKinematic) {
      // Two-body correction
      float totalInvMass = a->invMass + b->invMass;
      if(totalInvMass > EPSILON) {
        float relVelN = vec3Dot(vec3Sub(a->velocity, b->velocity), result.normal);
        if(relVelN < 0.0f) {
          float jN = -(1.0f + combinedBounce) * relVelN / totalInvMass;
          a->velocity = vec3Add(a->velocity, vec3Scale(result.normal, jN * a->invMass));
          b->velocity = vec3Sub(b->velocity, vec3Scale(result.normal, jN * b->invMass));
        }
      }
    } else if(a->isKinematic) {
      collideCorrectVelocity(b, result, combinedFriction, combinedBounce);
    } else {
      EpaResult flipped = result;
      flipped.normal = vec3Negate(result.normal);
      fm_vec3_t tmp = result.contactA;
      flipped.contactA = result.contactB;
      flipped.contactB = tmp;
      collideCorrectVelocity(a, flipped, combinedFriction, combinedBounce);
    }
  }

} // namespace P64::CollNew
