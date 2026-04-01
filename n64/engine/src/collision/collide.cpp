/**
 * @file collide.cpp
 * @author Kevin Reier <github.com/Byterset>
 * @brief Functions to detect collisions between different shapes and objects and record them (see collide.h)
 */
#include "collision/collide.h"
#include "collision/collision_scene.h"
#include "collision/contact_utils.h"
#include "collision/gjk.h"
#include "scene/scene.h"

#include <cmath>
#include <cstdlib>

namespace P64::Coll {

  struct ColliderProxy {
    Collider *collider{nullptr};
    fm_vec3_t worldCenter{};
    Matrix3x3 rotation{};
    Matrix3x3 rotationT{};
  };

  static fm_vec3_t makeSafeContactNormal(const fm_vec3_t &normal, const fm_vec3_t &contactA, const fm_vec3_t &contactB) {
    if(fm_vec3_len2(&normal) > FM_EPSILON * FM_EPSILON) {
      fm_vec3_t normalized;
      fm_vec3_norm(&normalized, &normal);
      return normalized;
    }

    fm_vec3_t fallback = contactA - contactB;
    if(fm_vec3_len2(&fallback) > FM_EPSILON * FM_EPSILON) {
      fm_vec3_t normalized;
      fm_vec3_norm(&normalized, &fallback);
      return normalized;
    }

    return VEC3_UP;
  }

  static void swapConstraintOrder(
    RigidBody *&rigidBodyA, Collider *&colliderA, MeshCollider *&meshColliderA, Object *&objectA,
    RigidBody *&rigidBodyB, Collider *&colliderB, MeshCollider *&meshColliderB, Object *&objectB,
    EpaResult &result) {
    std::swap(rigidBodyA, rigidBodyB);
    std::swap(colliderA, colliderB);
    std::swap(meshColliderA, meshColliderB);
    std::swap(objectA, objectB);
    result.normal = -result.normal;
    std::swap(result.contactA, result.contactB);
  }

  static void colliderProxyGjkSupport(const void *data, const fm_vec3_t &direction, fm_vec3_t &output) {
    const auto *proxy = static_cast<const ColliderProxy *>(data);
    if(!proxy || !proxy->collider) {
      output = VEC3_ZERO;
      return;
    }

    fm_vec3_t localDir = matrix3Vec3Mul(proxy->rotationT, direction);
    fm_vec3_t localSupport = proxy->collider->support(localDir);
    output = matrix3Vec3Mul(proxy->rotation, localSupport) + proxy->worldCenter;
  }

  static bool barycentricIsInsideTriangle(const fm_vec3_t &barycentric, float tolerance) {
    return barycentric.x >= -tolerance && barycentric.y >= -tolerance && barycentric.z >= -tolerance &&
           barycentric.x <= 1.0f + tolerance && barycentric.y <= 1.0f + tolerance && barycentric.z <= 1.0f + tolerance;
  }



  static fm_vec3_t closestPointOnTriangle(const fm_vec3_t &point, const fm_vec3_t &a, const fm_vec3_t &b, const fm_vec3_t &c) {
    const fm_vec3_t ab = b - a;
    const fm_vec3_t ac = c - a;
    const fm_vec3_t ap = point - a;
    const float d1 = fm_vec3_dot(&ab, &ap);
    const float d2 = fm_vec3_dot(&ac, &ap);
    if(d1 <= 0.0f && d2 <= 0.0f) {
      return a;
    }

    const fm_vec3_t bp = point - b;
    const float d3 = fm_vec3_dot(&ab, &bp);
    const float d4 = fm_vec3_dot(&ac, &bp);
    if(d3 >= 0.0f && d4 <= d3) {
      return b;
    }

    const float vc = d1 * d4 - d3 * d2;
    if(vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f) {
      const float v = d1 / (d1 - d3);
      return a + (ab * v);
    }

    const fm_vec3_t cp = point - c;
    const float d5 = fm_vec3_dot(&ab, &cp);
    const float d6 = fm_vec3_dot(&ac, &cp);
    if(d6 >= 0.0f && d5 <= d6) {
      return c;
    }

    const float vb = d5 * d2 - d1 * d6;
    if(vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f) {
      const float w = d2 / (d2 - d6);
      return a + (ac * w);
    }

    const float va = d3 * d6 - d5 * d4;
    if(va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f) {
      const fm_vec3_t bc = c - b;
      const float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
      return b + (bc * w);
    }

    const float denom = 1.0f / (va + vb + vc);
    const float v = vb * denom;
    const float w = vc * denom;
    return a + (ab * v) + (ac * w);
  }

  static void projectTriangleOntoAxis(
    const fm_vec3_t &v0,
    const fm_vec3_t &v1,
    const fm_vec3_t &v2,
    const fm_vec3_t &axis,
    float &outMin,
    float &outMax) {
    outMin = fm_vec3_dot(&v0, &axis);
    outMax = outMin;

    const float p1 = fm_vec3_dot(&v1, &axis);
    outMin = fminf(outMin, p1);
    outMax = fmaxf(outMax, p1);

    const float p2 = fm_vec3_dot(&v2, &axis);
    outMin = fminf(outMin, p2);
    outMax = fmaxf(outMax, p2);
  }

  static void meshLocalResultToWorld(EpaResult &result, const MeshCollider &mesh) {
    result.normal = mesh.rotateToWorld(result.normal);
    result.contactA = mesh.toWorldSpace(result.contactA);
    result.contactB = mesh.toWorldSpace(result.contactB);
  }

  static int minimumTriangleReuseContacts(const Collider &collider) {
    switch(collider.shapeType()) {
      case ShapeType::Sphere:
      case ShapeType::Capsule:
        return 1;
      case ShapeType::Cylinder:
      case ShapeType::Cone:
        return 2;
      case ShapeType::Box:
      case ShapeType::Pyramid:
        return 3;
    }

    return MAX_CONTACT_POINTS_PER_PAIR;
  }

  static float triangleReuseShapeScale(const Collider &collider) {
    switch(collider.shapeType()) {
      case ShapeType::Sphere:
        return collider.sphereShape().radius;
      case ShapeType::Capsule:
        return fmaxf(collider.capsuleShape().radius, collider.capsuleShape().innerHalfHeight);
      case ShapeType::Box:
        return fmaxf(collider.boxShape().halfSize.x, fmaxf(collider.boxShape().halfSize.y, collider.boxShape().halfSize.z));
      case ShapeType::Cylinder:
        return fmaxf(collider.cylinderShape().radius, collider.cylinderShape().halfHeight);
      case ShapeType::Cone:
        return fmaxf(collider.coneShape().radius, collider.coneShape().halfHeight);
      case ShapeType::Pyramid:
        return fmaxf(collider.pyramidShape().baseHalfWidthX, fmaxf(collider.pyramidShape().baseHalfWidthZ, collider.pyramidShape().halfHeight));
    }

    return 0.0f;
  }

  static float cachedTriangleContactSpanSq(const ContactConstraint &constraint) {
    float maxSpanSq = 0.0f;
    for(int i = 0; i < constraint.pointCount; ++i) {
      const ContactPoint &pointA = constraint.points[i];
      if(!pointA.active) continue;

      for(int j = i + 1; j < constraint.pointCount; ++j) {
        const ContactPoint &pointB = constraint.points[j];
        if(!pointB.active) continue;

        const fm_vec3_t diff = pointA.contactA - pointB.contactA;
        maxSpanSq = fmaxf(maxSpanSq, fm_vec3_len2(&diff));
      }
    }

    return maxSpanSq;
  }

  static bool canSkipTriangleEpaWithCachedConstraint(const ContactConstraint &constraint, const Collider &collider) {
    const int minContacts = minimumTriangleReuseContacts(collider);
    if(constraint.pointCount < minContacts) {
      return false;
    }

    if(minContacts <= 1) {
      return true;
    }

    const float minSpan = fmaxf(triangleReuseShapeScale(collider) * 0.2f, 0.02f);
    return cachedTriangleContactSpanSq(constraint) >= (minSpan * minSpan);
  }

  static bool refreshCachedTriangleConstraint(
    ContactConstraint &constraint,
    RigidBody *rigidBody,
    const MeshTriangle &triangle,
    const MeshCollider &mesh,
    float combinedFriction,
    float combinedBounce,
    bool respondsA,
    bool respondsB) {
    if(constraint.isTrigger || constraint.pointCount <= 0 || !rigidBody || !rigidBody->positionPtr()) {
      return false;
    }

    const fm_vec3_t localV0 = triangle.localVertex(0);
    const fm_vec3_t localV1 = triangle.localVertex(1);
    const fm_vec3_t localV2 = triangle.localVertex(2);
    const Plane trianglePlaneLocal = planeFromNormalAndPoint(triangle.normal, localV0);
    fm_vec3_t triangleNormalWorld = triangle.worldNormal();
    triangleNormalWorld = makeSafeContactNormal(triangleNormalWorld, constraint.points[0].contactA, constraint.points[0].contactB);
    if(fm_vec3_dot(&triangleNormalWorld, &constraint.normal) < 0.0f) {
      triangleNormalWorld = -triangleNormalWorld;
    }

    constraint.isActive = true;
    constraint.isTrigger = false;
    constraint.normal = triangleNormalWorld;
    vec3CalculateTangents(constraint.normal, constraint.tangentU, constraint.tangentV);
    constraint.combinedFriction = combinedFriction;
    constraint.combinedBounce = combinedBounce;
    constraint.respondsA = respondsA;
    constraint.respondsB = respondsB;

    constexpr float BARY_TOLERANCE = 0.02f;
    constexpr float REUSE_MAX_SEPARATION = 0.05f;
    int writeIndex = 0;

    for(int pointIndex = 0; pointIndex < constraint.pointCount; ++pointIndex) {
      ContactPoint &point = constraint.points[pointIndex];
      if(!point.active) continue;

      fm_vec3_t localMeshPoint = planeProjectPoint(trianglePlaneLocal, point.localPointB);
      const fm_vec3_t barycentric = calculateBarycentricCoords(localV0, localV1, localV2, localMeshPoint);
      if(!barycentricIsInsideTriangle(barycentric, BARY_TOLERANCE)) {
        point.active = false;
        continue;
      }

      point.localPointB = evaluateBarycentricCoords(localV0, localV1, localV2, barycentric);

      point.contactA = contactWorldPointFromLocalPoint(point.localPointA, rigidBody, nullptr, nullptr);
      point.contactB = contactWorldPointFromLocalPoint(point.localPointB, nullptr, nullptr, &mesh);
      point.point = (point.contactA + point.contactB) * 0.5f;

      fm_vec3_t diff = point.contactA - point.contactB;
      point.penetration = -fm_vec3_dot(&diff, &constraint.normal);
      if(point.penetration < -REUSE_MAX_SEPARATION) {
        point.active = false;
        continue;
      }

      if(writeIndex != pointIndex) {
        constraint.points[writeIndex] = point;
      }
      ++writeIndex;
    }

    constraint.pointCount = writeIndex;
    return writeIndex > 0;
  }

  // ── Analytical collision helpers ──────────────────────────────────

  static bool analyticalSphereSphere(const Collider *a, const Collider *b, EpaResult &result) {
    if(!a || !b) return false;
    if(a->shapeType() != ShapeType::Sphere || b->shapeType() != ShapeType::Sphere) return false;

    float rA = a->sphereShape().radius;
    float rB = b->sphereShape().radius;
    float combinedRadius = rA + rB;

    fm_vec3_t diff = a->worldCenter() - b->worldCenter();
    float distSq = fm_vec3_len2(&diff);

    if(distSq >= combinedRadius * combinedRadius) return false;

    float dist = sqrtf(distSq);
    if(dist < FM_EPSILON) {
      result.normal = VEC3_UP;
      result.penetration = combinedRadius;
      result.contactA = a->worldCenter() - result.normal * rA;
      result.contactB = b->worldCenter() + result.normal * rB;
      return true;
    }

    result.normal = diff * (1.0f / dist);
    result.penetration = combinedRadius - dist;
    result.contactA = a->worldCenter() - result.normal * rA;
    result.contactB = b->worldCenter() + result.normal * rB;
    return true;
  }

  static bool analyticalSphereBox(const Collider *sphere, const Collider *box, EpaResult &result) {
    if(!sphere || !box) return false;
    if(sphere->shapeType() != ShapeType::Sphere || box->shapeType() != ShapeType::Box) return false;

    float radius = sphere->sphereShape().radius;
    fm_vec3_t halfSize = box->boxShape().halfSize;

    // Transform sphere center to box local space
    const Matrix3x3 &boxRot = box->rotationMatrix();
    const Matrix3x3 &boxRotT = box->inverseRotationMatrix();
    fm_vec3_t localCenter = sphere->worldCenter() - box->worldCenter();
    fm_vec3_t localSpherePos = matrix3Vec3Mul(boxRotT, localCenter);

    // Clamp to box surface
    fm_vec3_t closest = fm_vec3_t{{ 
      fmaxf(-halfSize.x, fminf(localSpherePos.x, halfSize.x)),
      fmaxf(-halfSize.y, fminf(localSpherePos.y, halfSize.y)),
      fmaxf(-halfSize.z, fminf(localSpherePos.z, halfSize.z))
    }};

    fm_vec3_t diff = localSpherePos - closest;
    float distSq = fm_vec3_len2(&diff);

    if(distSq >= radius * radius) return false;

    float dist = sqrtf(distSq);
    fm_vec3_t localNormal;
    if(dist < FM_EPSILON) {
      // Sphere center is inside box — find shortest escape axis
      float dx = halfSize.x - fabsf(localSpherePos.x);
      float dy = halfSize.y - fabsf(localSpherePos.y);
      float dz = halfSize.z - fabsf(localSpherePos.z);
      if(dx <= dy && dx <= dz) {
        localNormal = fm_vec3_t{{copysignf(1.0f, localSpherePos.x), 0.0f, 0.0f}};
        result.penetration = dx + radius;
      } else if(dy <= dz) {
        localNormal = fm_vec3_t{{0.0f, copysignf(1.0f, localSpherePos.y), 0.0f}};
        result.penetration = dy + radius;
      } else {
        localNormal = fm_vec3_t{{0.0f, 0.0f, copysignf(1.0f, localSpherePos.z)}};
        result.penetration = dz + radius;
      }
    } else {
      localNormal = fm_vec3_t{{diff.x * (1.0f / dist), diff.y * (1.0f / dist), diff.z * (1.0f / dist)}};
      result.penetration = radius - dist;
    }

    result.normal = matrix3Vec3Mul(boxRot, localNormal);
    fm_vec3_t worldClosest = matrix3Vec3Mul(boxRot, closest) + box->worldCenter();
    result.contactB = worldClosest;
    result.contactA = sphere->worldCenter() - result.normal * radius;
    return true;
  }

  static bool analyticalSphereCapsule(const Collider *sphere, const Collider *capsule, EpaResult &result) {
    if(!sphere || !capsule) return false;
    if(sphere->shapeType() != ShapeType::Sphere || capsule->shapeType() != ShapeType::Capsule) return false;

    float rS = sphere->sphereShape().radius;
    float rC = capsule->capsuleShape().radius;
    float hh = capsule->capsuleShape().innerHalfHeight;

    // Capsule axis endpoints in world space
    fm_vec3_t localUp = fm_vec3_t{{0.0f, hh, 0.0f}};
    fm_vec3_t rotUp = matrix3Vec3Mul(capsule->rotationMatrix(), localUp);
    fm_vec3_t capTop = capsule->worldCenter() + rotUp;
    fm_vec3_t capBot = capsule->worldCenter() - rotUp;

    // Closest point on capsule segment to sphere center
    fm_vec3_t seg = capTop - capBot;
    float segLenSq = fm_vec3_len2(&seg);
    float t = 0.5f;
    if(segLenSq > FM_EPSILON) {
      fm_vec3_t capToSphere = sphere->worldCenter() - capBot;
      t = fm_vec3_dot(&capToSphere, &seg) / segLenSq;
      if(t < 0.0f) t = 0.0f;
      if(t > 1.0f) t = 1.0f;
    }
    fm_vec3_t closestOnSeg = capBot + seg * t;

    float combinedRadius = rS + rC;
  fm_vec3_t diff = sphere->worldCenter() - closestOnSeg;
    float distSq = fm_vec3_len2(&diff);

    if(distSq >= combinedRadius * combinedRadius) return false;

    float dist = sqrtf(distSq);
    if(dist < FM_EPSILON) {
      result.normal = VEC3_UP;
    } else {
      result.normal = diff * (1.0f / dist);
    }

    result.penetration = combinedRadius - dist;
    result.contactA = sphere->worldCenter() - result.normal * rS;
    result.contactB = closestOnSeg + result.normal * rC;
    return true;
  }


  // ── Contact constraint caching ────────────────────────────────────

  ContactConstraint *collideCacheContactConstraint(
    RigidBody *rigidBodyA, Collider *colliderA, MeshCollider *meshColliderA, Object *objectA,
    RigidBody *rigidBodyB, Collider *colliderB, MeshCollider *meshColliderB, Object *objectB, const EpaResult &result,
    float combinedFriction, float combinedBounce, bool isTrigger, bool respondsA, bool respondsB, int triangleIndex) {

    CollisionScene *scene = collisionSceneGetInstance();
    EpaResult orderedResult = result;

    orderedResult.normal = makeSafeContactNormal(orderedResult.normal, orderedResult.contactA, orderedResult.contactB);

    const bool isColliderPair = colliderA && colliderB && !meshColliderA && !meshColliderB;
    const bool hasMeshOnA = meshColliderA && !meshColliderB && !colliderA && colliderB;

    if(hasMeshOnA) {
      swapConstraintOrder(rigidBodyA, colliderA, meshColliderA, objectA, rigidBodyB, colliderB, meshColliderB, objectB, orderedResult);
      std::swap(respondsA, respondsB);
    }

    if(isColliderPair && shouldSwapColliderPairOrder(colliderA, colliderB)) {
      swapConstraintOrder(rigidBodyA, colliderA, meshColliderA, objectA, rigidBodyB, colliderB, meshColliderB, objectB, orderedResult);
      std::swap(respondsA, respondsB);
    }

    ContactConstraintKey key;
    if(colliderA && colliderB) {
      key = makeColliderPairConstraintKey(colliderA, colliderB);
    } else if(colliderA && meshColliderB && triangleIndex >= 0) {
      key = isTrigger
        ? makeColliderMeshConstraintKey(colliderA, meshColliderB)
        : makeColliderMeshConstraintKey(colliderA, meshColliderB, static_cast<uint16_t>(triangleIndex));
    } else {
      return nullptr;
    }

    ContactConstraint *existing = scene->findCachedConstraint(key);

    if(existing) {
      // Update existing constraint
      existing->rigidBodyA = rigidBodyA;
      existing->colliderA = colliderA;
      existing->meshColliderA = meshColliderA;
      existing->objectA = objectA;
      existing->rigidBodyB = rigidBodyB;
      existing->colliderB = colliderB;
      existing->meshColliderB = meshColliderB;
      existing->objectB = objectB;
      existing->isActive = true;
      existing->isTrigger = isTrigger;
      existing->normal = orderedResult.normal;
      vec3CalculateTangents(orderedResult.normal, existing->tangentU, existing->tangentV);
      existing->combinedFriction = combinedFriction;
      existing->combinedBounce = combinedBounce;
      existing->respondsA = respondsA;
      existing->respondsB = respondsB;

      if(isTrigger) {
        existing->pointCount = 1;
        ContactPoint &point = existing->points[0];
        point = ContactPoint{};
        point.contactA = orderedResult.contactA;
        point.contactB = orderedResult.contactB;
        point.point = (orderedResult.contactA + orderedResult.contactB) * 0.5f;
        point.penetration = orderedResult.penetration;
        point.active = true;
        return existing;
      }

      // Try to match new contact to an existing point by proximity
      constexpr float MATCH_DIST_SQ = 0.02f;
      int matchedIdx = -1;
      float bestDistSq = MATCH_DIST_SQ;
      for(int i = 0; i < existing->pointCount; ++i) {
        fm_vec3_t diff = existing->points[i].contactA - orderedResult.contactA;
        float distA = fm_vec3_len2(&diff);
        diff = existing->points[i].contactB - orderedResult.contactB;
        float distB = fm_vec3_len2(&diff);
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
        target->point = (orderedResult.contactA + orderedResult.contactB) * 0.5f;
        target->penetration = orderedResult.penetration;
        target->active = true;

        target->localPointA = contactLocalPointFromWorldPoint(target->contactA, rigidBodyA, colliderA, meshColliderA);
        target->localPointB = contactLocalPointFromWorldPoint(target->contactB, rigidBodyB, colliderB, meshColliderB);
      }

      // Validate other points against the updated normal
      for(int i = 0; i < existing->pointCount; ++i) {
        ContactPoint &cp = existing->points[i];
        if(cp.active) continue; // already processed

        fm_vec3_t diff = cp.contactA - cp.contactB;
        float pen = -fm_vec3_dot(&diff, &existing->normal);
        if(pen > -0.05f) {
          cp.penetration = pen;
          cp.active = true;
        }
      }

      return existing;
    }

    // Create new constraint
    ContactConstraint *cc = scene->createCachedConstraint(
      key,
      rigidBodyA, colliderA, meshColliderA, objectA,
      rigidBodyB, colliderB, meshColliderB, objectB);
    if(!cc) return nullptr;
    cc->normal = orderedResult.normal;
    vec3CalculateTangents(orderedResult.normal, cc->tangentU, cc->tangentV);
    cc->combinedFriction = combinedFriction;
    cc->combinedBounce = combinedBounce;
    cc->isActive = true;
    cc->isTrigger = isTrigger;
    cc->respondsA = respondsA;
    cc->respondsB = respondsB;
    cc->pointCount = 1;

    ContactPoint &cp = cc->points[0];
    cp = ContactPoint{};
    cp.contactA = orderedResult.contactA;
    cp.contactB = orderedResult.contactB;
    cp.point = (orderedResult.contactA + orderedResult.contactB) * 0.5f;
    cp.penetration = orderedResult.penetration;
    cp.active = true;

    if(isTrigger) {
      return cc;
    }

    cp.localPointA = contactLocalPointFromWorldPoint(cp.contactA, rigidBodyA, colliderA, meshColliderA);
    cp.localPointB = contactLocalPointFromWorldPoint(cp.contactB, rigidBodyB, colliderB, meshColliderB);


    return cc;
  }


  /// @brief Performs a collision test between a collider and a single Mesh triangle. Used as a subroutine for object-to-mesh collision detection.
  ///
  /// Hint: This function is designed to be called with the collider already transformed into the mesh's local space.
  /// @param colliderProxyMeshSpace Collider proxy containing the original collider and additional mesh space data (world center, rotation) for GJK support function.
  /// @param rigidBody Pointer to the rigid body associated with the collider, if any.
  /// @param mesh Reference to the mesh collider.
  /// @param triangleIndex Index of the triangle within the mesh to test against.
  /// @return True if a collision is detected, false otherwise.
  bool collideDetectObjectToTriangle(ColliderProxy *colliderProxyMeshSpace, RigidBody *rigidBody, const MeshCollider &mesh, int triangleIndex) {
    const bool isTriggerContact = colliderProxyMeshSpace->collider->isTrigger();
    const bool colliderRespondsToMesh = colliderProxyMeshSpace->collider->readsMeshCollider(&mesh);

    Object *objectA = colliderProxyMeshSpace->collider->ownerObject();
    Object *objectB = mesh.ownerObject();

    if(!colliderRespondsToMesh && !mesh.readsCollider(colliderProxyMeshSpace->collider)) {
      return false;
    }

    const float combinedFriction = fminf(colliderProxyMeshSpace->collider->friction(), mesh.friction());
    const float combinedBounce = fmaxf(colliderProxyMeshSpace->collider->bounce(), mesh.bounce());

    MeshTriangle tri;
    tri.vertices = nullptr;
    tri.tri = mesh.triangleIndices(triangleIndex);
    tri.normal = mesh.triangleNormal(triangleIndex);
    tri.mesh = &mesh;

    if(!isTriggerContact) {
      CollisionScene *scene = collisionSceneGetInstance();
      ContactConstraint *existing = scene->findCachedConstraint(
        makeColliderMeshConstraintKey(colliderProxyMeshSpace->collider, const_cast<MeshCollider *>(&mesh), static_cast<uint16_t>(triangleIndex)));
      if(existing && refreshCachedTriangleConstraint(*existing, rigidBody, tri, mesh, combinedFriction, combinedBounce, colliderRespondsToMesh, false) &&
         canSkipTriangleEpaWithCachedConstraint(*existing, *colliderProxyMeshSpace->collider)) {
        existing->rigidBodyA = rigidBody;
        existing->colliderA = colliderProxyMeshSpace->collider;
        existing->meshColliderA = nullptr;
        existing->objectA = objectA;
        existing->rigidBodyB = nullptr;
        existing->colliderB = nullptr;
        existing->meshColliderB = const_cast<MeshCollider *>(&mesh);
        existing->objectB = objectB;
        return true;
      }
    }

    Simplex simplex;
    fm_vec3_t firstDir = ((tri.localVertex(0) + tri.localVertex(1) + tri.localVertex(2)) / 3.0f) - colliderProxyMeshSpace->worldCenter;
    if(fm_vec3_len2(&firstDir) < FM_EPSILON * FM_EPSILON) firstDir = VEC3_RIGHT;

    bool gjkOverlap = gjkCheckForOverlap(
      simplex,
      colliderProxyMeshSpace, colliderProxyGjkSupport,
      &tri, meshTriangleGjkSupport,
      firstDir
    );
    if(!gjkOverlap) return false;

    // If the collider is a trigger we only need to check for overlap
    if (isTriggerContact)
    {
      const fm_vec3_t v0 = tri.worldVertex(0);
      const fm_vec3_t v1 = tri.worldVertex(1);
      const fm_vec3_t v2 = tri.worldVertex(2);
      const fm_vec3_t triCenter = (v0 + v1 + v2) / 3.0f;

      // Cache a dummy contact constraint to report the trigger collision.
      EpaResult dummyResult;
      dummyResult.normal = makeSafeContactNormal(tri.normal, colliderProxyMeshSpace->collider->worldCenter(), triCenter);
      dummyResult.penetration = 0.0f;
      dummyResult.contactA = colliderProxyMeshSpace->collider->worldCenter();
      dummyResult.contactB = triCenter;

      collideCacheContactConstraint(
          rigidBody, colliderProxyMeshSpace->collider, nullptr, objectA,
          nullptr, nullptr, const_cast<MeshCollider *>(&mesh), objectB,
          dummyResult, 0.0f, 0.0f, true, false, false, triangleIndex);

      return true;
    }

    struct EpaResult epaResult;

    bool epaSuccess = epaSolve(
            simplex,
            colliderProxyMeshSpace, colliderProxyGjkSupport,
            &tri, meshTriangleGjkSupport,
            epaResult);
    if (epaSuccess)
    {
      meshLocalResultToWorld(epaResult, mesh);

      // Cache the contact constraint for this collision
      collideCacheContactConstraint(
          rigidBody, colliderProxyMeshSpace->collider, nullptr, objectA,
          nullptr, nullptr, const_cast<MeshCollider *>(&mesh), objectB,
          epaResult, combinedFriction, combinedBounce, isTriggerContact, colliderRespondsToMesh, false, triangleIndex);

      return true;
    }
    return false;
  }

  // ── Object-to-mesh ──────────────────────────────────────────────

  void collideDetectObjectToMesh(Collider *collider, RigidBody *rigidBody, const MeshCollider &mesh) {

    // Transform the collider's world AABB into the mesh's local space for tree query
    AABB queryAABB = mesh.hasTransform()
      ? mesh.worldAabbToLocal(collider->worldAabb())
      : collider->worldAabb();

    // Query local-space mesh AABB tree for candidate triangles
    constexpr int MAX_CANDIDATES = 20;
    NodeProxy candidates[MAX_CANDIDATES];
    int count = mesh.queryTriangleNodes(queryAABB, candidates, MAX_CANDIDATES);

    if(count <= 0) return;

    // Precompute collider proxy in mesh local space for reuse across candidates
    ColliderProxy colliderInMeshSpace;
    colliderInMeshSpace.collider = collider;
    bool meshHasTransform = mesh.hasTransform();
    colliderInMeshSpace.worldCenter = meshHasTransform ? mesh.toLocalSpace(collider->worldCenter()) : collider->worldCenter();
    // apply mesh rotation to collider's orientation so GJK can work in mesh local space
    colliderInMeshSpace.rotation = meshHasTransform
                       ? matrix3Mul(mesh.inverseRotationMatrix(), collider->rotationMatrix())
                       : collider->rotationMatrix();
    colliderInMeshSpace.rotationT = matrix3Transpose(colliderInMeshSpace.rotation);


    // For every candidate triangle perform precise collision test
    for(int i = 0; i < count; ++i) {
      int triIndex = mesh.triangleIndexForNode(candidates[i]);
      if(triIndex < 0) continue;

      // If the triangle is overlapping and the collider is a Trigger
      // we can skip the rest of the candidates since triggers just need to report that a collision happened
      if(collideDetectObjectToTriangle(&colliderInMeshSpace, rigidBody, mesh, triIndex) && collider->isTrigger()) {
        return;
      }
    }


  }

  
  /// @brief Detects collision between two colliders and caches contact constraints if needed.
  /// @param colliderA The first collider.
  /// @param rbA The rigid body associated with the first collider.
  /// @param colliderB The second collider.
  /// @param rbB The rigid body associated with the second collider.
  void collideDetectObjectToObject(Collider *colliderA, RigidBody *rbA, Collider *colliderB, RigidBody *rbB) {
    if(!colliderA || !colliderB) return;

    CollisionScene *scene = collisionSceneGetInstance();
    const bool aReadsB = colliderA->readsCollider(colliderB);
    const bool bReadsA = colliderB->readsCollider(colliderA);

    if(rbA && rbB && rbA->isSleeping() && rbB->isSleeping() && !colliderA->isTrigger() && !colliderB->isTrigger()) return;

    // If both have rigidbodies, evaluate rigidbody-level filters.
    if(colliderA && colliderB) {
      if(!aReadsB && !bReadsA) return;
      if(colliderA->isTrigger() && colliderB->isTrigger()) return;
    }

    // Try analytical tests first before falling back to GJK+EPA for general convex shapes.
    
    EpaResult result;
    bool analyticalHit = false;
    bool hasAnalyticalPath = false;

    if(colliderA->shapeType() == ShapeType::Sphere && colliderB->shapeType() == ShapeType::Sphere) {
      hasAnalyticalPath = true;
      analyticalHit = analyticalSphereSphere(colliderA, colliderB, result);
    } else if(colliderA->shapeType() == ShapeType::Sphere && colliderB->shapeType() == ShapeType::Box) {
      hasAnalyticalPath = true;
      analyticalHit = analyticalSphereBox(colliderA, colliderB, result);
    } else if(colliderA->shapeType() == ShapeType::Sphere && colliderB->shapeType() == ShapeType::Capsule) {
      hasAnalyticalPath = true;
      analyticalHit = analyticalSphereCapsule(colliderA, colliderB, result);
    } else if(colliderB->shapeType() == ShapeType::Sphere && colliderA->shapeType() == ShapeType::Box) {
      hasAnalyticalPath = true;
      analyticalHit = analyticalSphereBox(colliderB, colliderA, result);
      if(analyticalHit) {
        result.normal = -result.normal;
        fm_vec3_t tmp = result.contactA;
        result.contactA = result.contactB;
        result.contactB = tmp;
      }
    } else if(colliderB->shapeType() == ShapeType::Sphere && colliderA->shapeType() == ShapeType::Capsule) {
      hasAnalyticalPath = true;
      analyticalHit = analyticalSphereCapsule(colliderB, colliderA, result);
      if(analyticalHit) {
        result.normal = -result.normal;
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
      // Try to reuse cached separating axis from previous frame (Bullet-style optimization)
      fm_vec3_t firstDir = VEC3_ZERO;
      ContactConstraintKey lookupKey = makeColliderPairConstraintKey(colliderA, colliderB);
      ContactConstraint *cachedCC = scene->findCachedConstraint(lookupKey);
      if(cachedCC && fm_vec3_len2(&cachedCC->cachedSeparatingAxis) > FM_EPSILON * FM_EPSILON) {
        firstDir = cachedCC->cachedSeparatingAxis;
      } else {
        firstDir = colliderA->worldCenter() - colliderB->worldCenter();
      }
      if(fm_vec3_len2(&firstDir) < FM_EPSILON * FM_EPSILON) firstDir = VEC3_UP;

      proxyA.collider = colliderA;
      proxyA.worldCenter = colliderA->worldCenter();
      proxyA.rotation = colliderA->rotationMatrix();
      proxyA.rotationT = colliderA->inverseRotationMatrix();

      proxyB.collider = colliderB;
      proxyB.worldCenter = colliderB->worldCenter();
      proxyB.rotation = colliderB->rotationMatrix();
      proxyB.rotationT = colliderB->inverseRotationMatrix();

      simplex.nPoints = 0;
      fm_vec3_t separatingAxis{};
      bool overlapping = gjkCheckForOverlap(
        simplex,
        &proxyA, colliderProxyGjkSupport,
        &proxyB, colliderProxyGjkSupport,
        firstDir,
        &separatingAxis
      );

      if(!overlapping) {
        // Cache the last GJK search direction as separating axis for next frame
        if(cachedCC) {
          cachedCC->cachedSeparatingAxis = separatingAxis;
        }
        return;
      }
      doEpa = true;

    }

    // Trigger pairs only need overlap confirmation, not a full contact manifold.
    if ((colliderA && colliderA->isTrigger()) || (colliderB && colliderB->isTrigger())) {
      EpaResult dummyResult;
      dummyResult.normal = makeSafeContactNormal(VEC3_ZERO, colliderA->worldCenter(), colliderB->worldCenter());
      dummyResult.penetration = 0.0f;
      dummyResult.contactA = colliderA->worldCenter();
      dummyResult.contactB = colliderB->worldCenter();

      collideCacheContactConstraint(
        rbA, colliderA, nullptr, colliderA->ownerObject(),
        rbB, colliderB, nullptr, colliderB->ownerObject(),
        dummyResult,
        0.0f, 0.0f,
        true,
        false, false
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

      if (!epaOk || result.penetration < FM_EPSILON)
        return;
    }

    // Wake sleeping rigidBodies
    if(aReadsB && rbA && rbA->isSleeping()) scene->wakeRigidBodyIsland(rbA);
    if(bReadsA && rbB && rbB->isSleeping()) scene->wakeRigidBodyIsland(rbB);

    float combinedFriction = fmin(colliderA->friction(), colliderB->friction());
    float combinedBounce = fmaxf(colliderA->bounce(), colliderB->bounce());

    // Cache the constraint
    collideCacheContactConstraint(
      rbA, colliderA, nullptr, colliderA->ownerObject(),
      rbB, colliderB, nullptr, colliderB->ownerObject(),
      result, combinedFriction, combinedBounce, false, aReadsB, bReadsA);
  }

} // namespace P64::Coll
