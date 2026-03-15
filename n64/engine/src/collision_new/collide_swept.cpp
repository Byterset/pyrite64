/**
 * @copyright 2024 - Max Bebök
 * @license MIT
 */
#include "collision_new/collide_swept.h"
#include "collision_new/collision_scene.h"
#include "collision_new/gjk.h"

#include <cmath>

namespace P64::CollNew {

  // Swept physics object wraps a PhysicsObject with a motion offset for CCD
  struct SweptPhysicsObject {
    const PhysicsObject *object;
    fm_vec3_t offset; // swept direction (prevPos → currentPos)
  };

  static void sweptGjkSupport(const void *data, const fm_vec3_t &direction, fm_vec3_t &output) {
    auto *swept = static_cast<const SweptPhysicsObject *>(data);
    swept->object->gjkSupport(direction, output);
    // Extend support in swept direction
    if(vec3Dot(swept->offset, direction) > 0.0f) {
      output = vec3Add(output, swept->offset);
    }
  }

  // Test single triangle with swept detection
  static bool sweptTriangleTest(PhysicsObject *object, const MeshCollider &mesh,
                                int triangleIndex, const fm_vec3_t &motion) {
    if(triangleIndex < 0 || triangleIndex >= mesh.triangleCount) return false;

    MeshTriangle tri;
    tri.vertices = mesh.vertices;
    tri.tri = mesh.triangles[triangleIndex];
    tri.normal = mesh.normals[triangleIndex];

    // Skip if moving away from triangle
    float normalDot = vec3Dot(motion, tri.normal);
    if(normalDot > EPSILON) return false;

    // Try raycast first for tunneling detection
    float rayDist = 0.0f;
    Plane triPlane = planeFromNormalAndPoint(tri.normal, mesh.vertices[tri.tri.indices[0]]);
    fm_vec3_t rayDir = vec3Normalize(motion);
    float motionLen = vec3Mag(motion);

    if(planeRayIntersection(triPlane, object->worldCenterOfMass, rayDir, rayDist)) {
      if(rayDist > 0.0f && rayDist < motionLen) {
        // The object crossed the triangle plane — do GJK check at intersection
        fm_vec3_t hitPoint = vec3Add(object->worldCenterOfMass, vec3Scale(rayDir, rayDist));
        if(tri.comparePoint(hitPoint) < EPSILON) {
          // Resolve: push back to surface
          float penetration = motionLen - rayDist;
          if(penetration > EPSILON && object->position) {
            *object->position = vec3Add(*object->position, vec3Scale(tri.normal, penetration));

            // Correct velocity: remove component into surface
            float velN = vec3Dot(object->velocity, tri.normal);
            if(velN < 0.0f) {
              fm_vec3_t normalImpulse = vec3Scale(tri.normal, -velN);
              object->velocity = vec3Add(object->velocity, normalImpulse);

              // Apply friction
              fm_vec3_t tangentVel = vec3Sub(object->velocity,
                vec3Scale(tri.normal, vec3Dot(object->velocity, tri.normal)));
              float tangentSpeed = vec3Mag(tangentVel);
              if(tangentSpeed > EPSILON) {
                float frictionImpulse = fminf(object->collider->friction * fabsf(velN), tangentSpeed);
                fm_vec3_t frictionDir = vec3Scale(tangentVel, -frictionImpulse / tangentSpeed);
                object->velocity = vec3Add(object->velocity, frictionDir);
              }

              // Apply bounce
              float bounce = object->collider->bounce * 0.2f;
              if(bounce > EPSILON) {
                object->velocity = vec3Add(object->velocity, vec3Scale(tri.normal, fabsf(velN) * bounce));
              }
            }

            if(tri.normal.y > 0.5f) object->isGrounded = true;
            return true;
          }
        }
      }
    }

    // Swept GJK + EPA fallback
    SweptPhysicsObject swept;
    swept.object = object;
    swept.offset = motion;

    fm_vec3_t firstDir = vec3Sub(object->worldCenterOfMass, mesh.vertices[tri.tri.indices[0]]);
    if(vec3MagSqrd(firstDir) < EPSILON) firstDir = vec3Up();

    Simplex simplex;
    simplex.nPoints = 0;
    bool overlapping = gjkCheckForOverlap(
      simplex,
      &swept, sweptGjkSupport,
      &tri, meshTriangleGjkSupport,
      firstDir
    );

    if(!overlapping) return false;

    // Use swept EPA
    fm_vec3_t sweepStart = object->worldCenterOfMass;
    fm_vec3_t sweepEnd = vec3Add(object->worldCenterOfMass, motion);

    EpaResult epaResult;
    bool epaOk = epaSolveSwept(
      simplex,
      object, physicsObjectGjkSupport,
      &tri, meshTriangleGjkSupport,
      sweepStart, sweepEnd,
      epaResult
    );

    if(!epaOk || epaResult.penetration < EPSILON) {
      // Fallback to static EPA
      simplex.nPoints = 0;
      overlapping = gjkCheckForOverlap(
        simplex,
        object, physicsObjectGjkSupport,
        &tri, meshTriangleGjkSupport,
        firstDir
      );
      if(!overlapping) return false;

      epaOk = epaSolve(
        simplex,
        object, physicsObjectGjkSupport,
        &tri, meshTriangleGjkSupport,
        epaResult
      );
      if(!epaOk || epaResult.penetration < EPSILON) return false;
    }

    // Resolve
    if(object->position) {
      float correction = epaResult.penetration * 0.9f;
      *object->position = vec3Add(*object->position, vec3Scale(epaResult.normal, correction));
    }

    // Correct velocity
    float velN = vec3Dot(object->velocity, epaResult.normal);
    if(velN < 0.0f) {
      // Remove velocity into surface
      object->velocity = vec3Add(object->velocity, vec3Scale(epaResult.normal, -velN));

      // Bounce
      float bounce = object->collider->bounce * 0.2f;
      if(bounce > EPSILON) {
        object->velocity = vec3Add(object->velocity, vec3Scale(epaResult.normal, fabsf(velN) * bounce));
      }
    }

    if(epaResult.normal.y > 0.5f) object->isGrounded = true;

    return true;
  }

  // ── Main swept collision function ─────────────────────────────────

  bool collideObjectToMeshSwept(PhysicsObject *object, MeshCollider *mesh, fm_vec3_t *prevPos) {
    if(!object || !mesh || !object->collider || !object->position || !prevPos) return false;
    if(object->isTrigger || object->isKinematic) return false;
    if(mesh->triangleCount == 0) return false;

    fm_vec3_t motion = vec3Sub(*object->position, *prevPos);
    float motionLenSq = vec3MagSqrd(motion);
    if(motionLenSq < EPSILON) return false;

    // Compute expanded AABB covering the full motion sweep
    AABB expandedAABB = object->boundingBox;
    AABB prevAABB = object->boundingBox;
    fm_vec3_t negMotion = vec3Negate(motion);
    prevAABB.min = vec3Add(prevAABB.min, negMotion);
    prevAABB.max = vec3Add(prevAABB.max, negMotion);
    expandedAABB = aabbUnion(expandedAABB, prevAABB);

    // Query AABB tree for candidate triangles
    NodeProxy candidates[32];
    int count = mesh->aabbTree.queryBounds(expandedAABB, candidates, 32);

    bool anyHit = false;
    for(int i = 0; i < count; ++i) {
      void *data = mesh->aabbTree.getNodeData(candidates[i]);
      if(!data) continue;
      int triIndex = static_cast<int>(reinterpret_cast<intptr_t>(data));
      if(sweptTriangleTest(object, *mesh, triIndex, motion)) {
        anyHit = true;
      }
    }

    return anyHit;
  }

} // namespace P64::CollNew
