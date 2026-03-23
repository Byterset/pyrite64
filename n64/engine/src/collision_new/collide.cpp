/**
 * @copyright 2024 - Max Bebök
 * @license MIT
 */
#include "collision_new/collide.h"
#include "collision_new/collision_scene.h"
#include "collision_new/gjk.h"
#include "scene/scene.h"

#include <cmath>
#include <cstdlib>

namespace P64::CollNew {

  struct ColliderProxy {
    const Collider *collider{nullptr};
    fm_vec3_t worldCenter{};
    Matrix3x3 rotation{};
    Matrix3x3 rotationT{};
  };

  static fm_quat_t colliderOrientation(const Collider *collider) {
    if(collider && collider->owner) {
      return collider->owner->rot;
    }
    return fm_quat_t{0.0f, 0.0f, 0.0f, 1.0f};
  }

  static Matrix3x3 colliderRotationMatrix(const Collider *collider) {
    return quatToMatrix3(colliderOrientation(collider));
  }

  static ConstraintCacheKeyPart makeConstraintCacheKeyPart(Collider *collider, Object *object) {
    if(collider) return ConstraintCacheKeyPart{collider, 1};
    if(object) return ConstraintCacheKeyPart{object, 2};
    return ConstraintCacheKeyPart{};
  }

  static bool shouldSwapConstraintOrder(Collider *colliderA, Object *objectA, Collider *colliderB, Object *objectB) {
    const ConstraintCacheKeyPart keyA = makeConstraintCacheKeyPart(colliderA, objectA);
    const ConstraintCacheKeyPart keyB = makeConstraintCacheKeyPart(colliderB, objectB);
    return keyB < keyA;
  }

  static void swapConstraintOrder(
    RigidBody *&rigidBodyA, Collider *&colliderA, Object *&objectA,
    RigidBody *&rigidBodyB, Collider *&colliderB, Object *&objectB,
    EpaResult &result) {
    std::swap(rigidBodyA, rigidBodyB);
    std::swap(colliderA, colliderB);
    std::swap(objectA, objectB);
    result.normal = vec3Negate(result.normal);
    std::swap(result.contactA, result.contactB);
  }

  static void colliderProxyGjkSupport(const void *data, const fm_vec3_t &direction, fm_vec3_t &output) {
    const auto *proxy = static_cast<const ColliderProxy *>(data);
    if(!proxy || !proxy->collider) {
      output = vec3Zero();
      return;
    }

    fm_vec3_t localDir = matrix3Vec3Mul(proxy->rotationT, direction);
    fm_vec3_t localSupport = proxy->collider->support(localDir);
    output = vec3Add(matrix3Vec3Mul(proxy->rotation, localSupport), proxy->worldCenter);
  }

  // ── Analytical collision helpers ──────────────────────────────────

  static bool analyticalSphereSphere(const Collider *a, const Collider *b, EpaResult &result) {
    if(!a || !b) return false;
    if(a->type != ShapeType::Sphere || b->type != ShapeType::Sphere) return false;

    float rA = a->sphere.radius;
    float rB = b->sphere.radius;
    float combinedRadius = rA + rB;

    fm_vec3_t diff = vec3Sub(a->worldCenter, b->worldCenter);
    float distSq = vec3MagSqrd(diff);

    if(distSq >= combinedRadius * combinedRadius) return false;

    float dist = sqrtf(distSq);
    if(dist < EPSILON) {
      result.normal = vec3Up();
      result.penetration = combinedRadius;
      result.contactA = vec3Sub(a->worldCenter, vec3Scale(result.normal, rA));
      result.contactB = vec3Add(b->worldCenter, vec3Scale(result.normal, rB));
      return true;
    }

    result.normal = vec3Scale(diff, 1.0f / dist);
    result.penetration = combinedRadius - dist;
    result.contactA = vec3Sub(a->worldCenter, vec3Scale(result.normal, rA));
    result.contactB = vec3Add(b->worldCenter, vec3Scale(result.normal, rB));
    return true;
  }

  static bool analyticalSphereBox(const Collider *sphere, const Collider *box, EpaResult &result) {
    if(!sphere || !box) return false;
    if(sphere->type != ShapeType::Sphere || box->type != ShapeType::Box) return false;

    float radius = sphere->sphere.radius;
    fm_vec3_t halfSize = box->box.halfSize;

    // Transform sphere center to box local space
    Matrix3x3 boxRot = colliderRotationMatrix(box);
    Matrix3x3 boxRotT = matrix3Transpose(boxRot);
    fm_vec3_t localCenter = vec3Sub(sphere->worldCenter, box->worldCenter);
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

    result.normal = matrix3Vec3Mul(boxRot, localNormal);
    fm_vec3_t worldClosest = vec3Add(matrix3Vec3Mul(boxRot, closest), box->worldCenter);
    result.contactB = worldClosest;
    result.contactA = vec3Sub(sphere->worldCenter, vec3Scale(result.normal, radius));
    return true;
  }

  static bool analyticalSphereCapsule(const Collider *sphere, const Collider *capsule, EpaResult &result) {
    if(!sphere || !capsule) return false;
    if(sphere->type != ShapeType::Sphere || capsule->type != ShapeType::Capsule) return false;

    float rS = sphere->sphere.radius;
    float rC = capsule->capsule.radius;
    float hh = capsule->capsule.innerHalfHeight;

    // Capsule axis endpoints in world space
    fm_vec3_t localUp = vec3(0.0f, hh, 0.0f);
    const fm_quat_t capsuleRot = colliderOrientation(capsule);
    fm_vec3_t capTop = vec3Add(capsule->worldCenter, quatRotateVec(capsuleRot, localUp));
    fm_vec3_t capBot = vec3Sub(capsule->worldCenter, quatRotateVec(capsuleRot, localUp));

    // Closest point on capsule segment to sphere center
    fm_vec3_t seg = vec3Sub(capTop, capBot);
    float segLenSq = vec3MagSqrd(seg);
    float t = 0.5f;
    if(segLenSq > EPSILON) {
      t = vec3Dot(vec3Sub(sphere->worldCenter, capBot), seg) / segLenSq;
      if(t < 0.0f) t = 0.0f;
      if(t > 1.0f) t = 1.0f;
    }
    fm_vec3_t closestOnSeg = vec3Add(capBot, vec3Scale(seg, t));

    float combinedRadius = rS + rC;
    fm_vec3_t diff = vec3Sub(sphere->worldCenter, closestOnSeg);
    float distSq = vec3MagSqrd(diff);

    if(distSq >= combinedRadius * combinedRadius) return false;

    float dist = sqrtf(distSq);
    if(dist < EPSILON) {
      result.normal = vec3Up();
    } else {
      result.normal = vec3Scale(diff, 1.0f / dist);
    }

    result.penetration = combinedRadius - dist;
    result.contactA = vec3Sub(sphere->worldCenter, vec3Scale(result.normal, rS));
    result.contactB = vec3Add(closestOnSeg, vec3Scale(result.normal, rC));
    return true;
  }


  // ── Contact constraint caching ────────────────────────────────────

  ContactConstraint *collideCacheContactConstraint(
    RigidBody *rigidBodyA, Collider *colliderA, Object *objectA,
    RigidBody *rigidBodyB, Collider *colliderB, Object *objectB, const EpaResult &result,
    float combinedFriction, float combinedBounce, bool isTrigger) {

    CollisionScene *scene = collisionSceneGetInstance();
    EpaResult orderedResult = result;

    if(shouldSwapConstraintOrder(colliderA, objectA, colliderB, objectB)) {
      swapConstraintOrder(rigidBodyA, colliderA, objectA, rigidBodyB, colliderB, objectB, orderedResult);
    }

    // Search for existing constraint with matching collider pair and similar normal
    ContactConstraint *existing = scene->findCachedConstraintByPair(
      colliderA, objectA, colliderB, objectB, orderedResult.normal, 0.9f);

    if(existing) {
      if(existing->rigidBodyA != rigidBodyA || existing->colliderA != colliderA || existing->objectA != objectA ||
         existing->rigidBodyB != rigidBodyB || existing->colliderB != colliderB || existing->objectB != objectB) {
        return nullptr;
      }

      // Update existing constraint
      existing->isActive = true;
      existing->normal = orderedResult.normal;
      vec3CalculateTangents(orderedResult.normal, existing->tangentU, existing->tangentV);

      // Try to match new contact to an existing point by proximity
      constexpr float MATCH_DIST_SQ = 0.02f;
      int matchedIdx = -1;
      float bestDistSq = MATCH_DIST_SQ;
      for(int i = 0; i < existing->pointCount; ++i) {
        float distA = vec3DistSqrd(existing->points[i].contactA, orderedResult.contactA);
        float distB = vec3DistSqrd(existing->points[i].contactB, orderedResult.contactB);
        float minDist = fminf(distA, distB);
        if(minDist < bestDistSq) {
          bestDistSq = minDist;
          matchedIdx = i;
        }
      }

      ContactPoint *target = nullptr;
      if(matchedIdx >= 0) {
        // Reuse existing point (preserves accumulated impulses for warm starting)
        target = &existing->points[matchedIdx];
      } else if(existing->pointCount < MAX_CONTACT_POINTS_PER_PAIR) {
        // Add new point
        target = &existing->points[existing->pointCount];
        existing->pointCount++;
        target->accumulatedNormalImpulse = 0.0f;
        target->accumulatedTangentImpulseU = 0.0f;
        target->accumulatedTangentImpulseV = 0.0f;
      } else {
        // Full: replace the shallowest point if new one is deeper
        int minPenIdx = 0;
        float minPen = existing->points[0].penetration;
        for(int i = 1; i < existing->pointCount; ++i) {
          if(existing->points[i].penetration < minPen) {
            minPen = existing->points[i].penetration;
            minPenIdx = i;
          }
        }
        if(orderedResult.penetration > minPen) {
          target = &existing->points[minPenIdx];
          target->accumulatedNormalImpulse = 0.0f;
          target->accumulatedTangentImpulseU = 0.0f;
          target->accumulatedTangentImpulseV = 0.0f;
        }
      }

      if(target) {
        target->contactA = orderedResult.contactA;
        target->contactB = orderedResult.contactB;
        target->point = vec3Scale(vec3Add(orderedResult.contactA, orderedResult.contactB), 0.5f);
        target->penetration = orderedResult.penetration;
        target->active = true;

        if(rigidBodyA) {
          fm_vec3_t relA = vec3Sub(target->contactA, *rigidBodyA->position);
          target->localPointA = rigidBodyA->rotation
            ? quatRotateVec(quatConjugate(*rigidBodyA->rotation), relA)
            : relA;
        } else {
          target->localPointA = target->contactA;
        }
        if(rigidBodyB) {
          fm_vec3_t relB = vec3Sub(target->contactB, *rigidBodyB->position);
          target->localPointB = rigidBodyB->rotation
            ? quatRotateVec(quatConjugate(*rigidBodyB->rotation), relB)
            : relB;
        } else {
          // Mesh contact: store world-space contact point directly
          target->localPointB = target->contactB;
        }
      }

      // Validate other points against the updated normal
      for(int i = 0; i < existing->pointCount; ++i) {
        ContactPoint &cp = existing->points[i];
        if(cp.active) continue; // already processed

        fm_vec3_t diff = vec3Sub(cp.contactA, cp.contactB);
        float pen = -vec3Dot(diff, existing->normal);
        if(pen > -0.05f) {
          cp.penetration = pen;
          cp.active = true;
        }
      }

      return existing;
    }

    // Create new constraint
    ContactConstraint *cc = scene->createCachedConstraint(
      rigidBodyA, colliderA, objectA,
      rigidBodyB, colliderB, objectB);
    if(!cc) return nullptr;
    cc->normal = orderedResult.normal;
    vec3CalculateTangents(orderedResult.normal, cc->tangentU, cc->tangentV);
    cc->combinedFriction = combinedFriction;
    cc->combinedBounce = combinedBounce;
    cc->isActive = true;
    cc->isTrigger = isTrigger;
    cc->pointCount = 1;

    ContactPoint &cp = cc->points[0];
    cp = ContactPoint{};
    cp.contactA = orderedResult.contactA;
    cp.contactB = orderedResult.contactB;
    cp.point = vec3Scale(vec3Add(orderedResult.contactA, orderedResult.contactB), 0.5f);
    cp.penetration = orderedResult.penetration;
    cp.active = true;

    if(rigidBodyA) {
      fm_vec3_t relA = vec3Sub(cp.contactA, *rigidBodyA->position);
      cp.localPointA = rigidBodyA->rotation
        ? quatRotateVec(quatConjugate(*rigidBodyA->rotation), relA)
        : relA;
    } else {
      cp.localPointA = cp.contactA;
    }
    if(rigidBodyB) {
      fm_vec3_t relB = vec3Sub(cp.contactB, *rigidBodyB->position);
      cp.localPointB = rigidBodyB->rotation
        ? quatRotateVec(quatConjugate(*rigidBodyB->rotation), relB)
        : relB;
    } else {
      // Mesh contact: store world-space contact point directly
      cp.localPointB = cp.contactB;
    }


    return cc;
  }

  // ── Object-to-triangle (internal helper, returns EPA result) ────

  static bool testObjectToTriangle(Collider *collider, const MeshCollider &mesh,
                                    int triangleIndex, EpaResult &epaResult) {
    if(!collider) return false;
    if(triangleIndex < 0 || triangleIndex >= mesh.triangleCount) return false;

    MeshTriangle tri;
    tri.vertices = mesh.vertices;
    tri.tri = mesh.triangles[triangleIndex];
    tri.normal = mesh.normals[triangleIndex];
    tri.mesh = &mesh; // Enable world-space transforms in GJK support

    // Quick face-culling: skip if rigidBody is far behind the triangle (in world space)
    float cullDist = 2.0f;
    if(collider->type == ShapeType::Sphere) {
      cullDist = collider->sphere.radius * 2.0f;
    }
    if(tri.comparePoint(collider->worldCenter) < -cullDist) {
      return false;
    }

    // GJK overlap test (MeshTriangle now returns world-space vertices via gjkSupport)
    fm_vec3_t worldV0 = tri.worldVertex(0);
    fm_vec3_t firstDir = vec3Sub(collider->worldCenter, worldV0);
    if(vec3MagSqrd(firstDir) < EPSILON) firstDir = vec3Up();

    ColliderProxy colliderProxy;
    colliderProxy.collider = collider;
    colliderProxy.worldCenter = collider->worldCenter;
    colliderProxy.rotation = colliderRotationMatrix(collider);
    colliderProxy.rotationT = matrix3Transpose(colliderProxy.rotation);

    Simplex simplex;
    simplex.nPoints = 0;
    bool overlapping = gjkCheckForOverlap(
      simplex,
      &colliderProxy, colliderProxyGjkSupport,
      &tri, meshTriangleGjkSupport,
      firstDir
    );

    if(!overlapping) return false;

    bool epaOk = epaSolve(
      simplex,
      &colliderProxy, colliderProxyGjkSupport,
      &tri, meshTriangleGjkSupport,
      epaResult
    );

    return epaOk && epaResult.penetration > EPSILON;
  }

  // ── Object-to-triangle (public, single-triangle test + cache) ────

  bool collideDetectObjectToTriangle(Collider *collider, RigidBody *rigidBody, const MeshCollider &mesh, int triangleIndex) {
    EpaResult epaResult;
    if(!testObjectToTriangle(collider, mesh, triangleIndex, epaResult)) return false;

    float combinedFriction = fmin(collider->friction, mesh.friction);
    float combinedBounce = fmax(collider->bounce, mesh.bounce);

    Object *objectA = collider->owner;
    Object *objectB = mesh.owner;

    collideCacheContactConstraint(
      rigidBody, collider, objectA,
      nullptr, nullptr, objectB,
      epaResult, combinedFriction, combinedBounce, false);

    return true;
  }

  // ── Object-to-mesh ──────────────────────────────────────────────
  // Simple per-triangle detection matching the reference implementation.

  void collideDetectObjectToMesh(Collider *collider, RigidBody *rigidBody, const MeshCollider &mesh) {
    if(!collider) return;
    if(rigidBody && rigidBody->isSleeping) return;
    if(mesh.triangleCount == 0) return;

    // Transform the collider's world AABB into the mesh's local space for tree query
    AABB queryAABB = mesh.hasTransform()
      ? mesh.worldAABBToLocal(collider->worldAABB)
      : collider->worldAABB;

    // Query local-space AABB tree for candidate triangles
    NodeProxy candidates[64];
    int count = mesh.aabbTree.queryBounds(queryAABB, candidates, 64);

    for(int i = 0; i < count; ++i) {
      void *data = mesh.aabbTree.getNodeData(candidates[i]);
      if(!data) continue;
      int triIndex = static_cast<int>(reinterpret_cast<intptr_t>(data)) - 1; // stored as index+1 to avoid nullptr
      collideDetectObjectToTriangle(collider, rigidBody, mesh, triIndex);
    }
  }

  
  /// @brief Detects collision between two colliders and caches contact constraints if needed.
  /// @param colliderA The first collider.
  /// @param rbA The rigid body associated with the first collider.
  /// @param colliderB The second collider.
  /// @param rbB The rigid body associated with the second collider.
  void collideDetectObjectToObject(Collider *colliderA, RigidBody *rbA, Collider *colliderB, RigidBody *rbB) {
    if(!colliderA || !colliderB) return;

    if(rbA && rbB && rbA->isSleeping && rbB->isSleeping) return;

    // If both have rigidbodies, evaluate rigidbody-level filters.
    if(colliderA && colliderB) {
      if((colliderA->maskRead & colliderB->maskWrite) == 0) return;
      if(colliderA->isTrigger && colliderB->isTrigger) return;
    }

    // Try analytical tests first before falling back to GJK+EPA for general convex shapes.
    
    EpaResult result;
    bool analyticalHit = false;
    bool hasAnalyticalPath = false;

    if(colliderA->type == ShapeType::Sphere && colliderB->type == ShapeType::Sphere) {
      hasAnalyticalPath = true;
      analyticalHit = analyticalSphereSphere(colliderA, colliderB, result);
    } else if(colliderA->type == ShapeType::Sphere && colliderB->type == ShapeType::Box) {
      hasAnalyticalPath = true;
      analyticalHit = analyticalSphereBox(colliderA, colliderB, result);
    } else if(colliderA->type == ShapeType::Sphere && colliderB->type == ShapeType::Capsule) {
      hasAnalyticalPath = true;
      analyticalHit = analyticalSphereCapsule(colliderA, colliderB, result);
    } else if(colliderB->type == ShapeType::Sphere && colliderA->type == ShapeType::Box) {
      hasAnalyticalPath = true;
      analyticalHit = analyticalSphereBox(colliderB, colliderA, result);
      if(analyticalHit) {
        result.normal = vec3Negate(result.normal);
        fm_vec3_t tmp = result.contactA;
        result.contactA = result.contactB;
        result.contactB = tmp;
      }
    } else if(colliderB->type == ShapeType::Sphere && colliderA->type == ShapeType::Capsule) {
      hasAnalyticalPath = true;
      analyticalHit = analyticalSphereCapsule(colliderB, colliderA, result);
      if(analyticalHit) {
        result.normal = vec3Negate(result.normal);
        fm_vec3_t tmp = result.contactA;
        result.contactA = result.contactB;
        result.contactB = tmp;
      }
    }
    if(hasAnalyticalPath && !analyticalHit) return;

    bool doEpa = false;
    Simplex simplex;
    ColliderProxy proxyA;
    ColliderProxy proxyB;

    if(!hasAnalyticalPath) {
      // Fall back to GJK + EPA
      fm_vec3_t firstDir = vec3Sub(colliderA->worldCenter, colliderB->worldCenter);
      if(vec3MagSqrd(firstDir) < EPSILON) firstDir = vec3Up();

      proxyA.collider = colliderA;
      proxyA.worldCenter = colliderA->worldCenter;
      proxyA.rotation = colliderRotationMatrix(colliderA);
      proxyA.rotationT = matrix3Transpose(proxyA.rotation);

      proxyB.collider = colliderB;
      proxyB.worldCenter = colliderB->worldCenter;
      proxyB.rotation = colliderRotationMatrix(colliderB);
      proxyB.rotationT = matrix3Transpose(proxyB.rotation);

      simplex.nPoints = 0;
      bool overlapping = gjkCheckForOverlap(
        simplex,
        &proxyA, colliderProxyGjkSupport,
        &proxyB, colliderProxyGjkSupport,
        firstDir
      );

      if(!overlapping) return;
      doEpa = true;

    }

    //handle trigger events
    if ((colliderA && colliderA->isTrigger) || (colliderB && colliderB->isTrigger)) {
      EpaResult dummyResult;
      dummyResult.normal = vec3Zero();
      dummyResult.penetration = 0.0f;
      dummyResult.contactA = colliderA->worldCenter;
      dummyResult.contactB = colliderB->worldCenter;
      // Cache the trigger constraint without velocity correction
      collideCacheContactConstraint(
        rbA, colliderA, colliderA->owner,
        rbB, colliderB, colliderB->owner,
        dummyResult,
        0.0f, 0.0f, // friction and bounce don't matter for triggers
        true // isTrigger
      );
      return;
    }

    if (doEpa)
    {
      bool epaOk = epaSolve(
          simplex,
          &proxyA, colliderProxyGjkSupport,
          &proxyB, colliderProxyGjkSupport,
          result);

      if (!epaOk || result.penetration < EPSILON)
        return;
    }

    // Wake sleeping rigidBodies
    if(rbA && rbA->isSleeping) rbA->wake();
    if(rbB && rbB->isSleeping) rbB->wake();

    float combinedFriction = fmin(colliderA->friction, colliderB->friction);
    float combinedBounce = fmaxf(colliderA->bounce, colliderB->bounce);

    // Cache the constraint
    collideCacheContactConstraint(
      rbA, colliderA, colliderA->owner,
      rbB, colliderB, colliderB->owner,
      result, combinedFriction, combinedBounce, false);
  }

} // namespace P64::CollNew
