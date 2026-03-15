/**
 * @copyright 2024 - Max Bebök
 * @license MIT
 *
 * Gilbert–Johnson–Keerthi distance algorithm (GJK) implementation for collision detection.
 *
 * Efficiently determines if two convex shapes overlap by building a simplex (tetrahedron)
 * that contains the origin in Minkowski difference space.
 */
#include "collision_new/gjk.h"

using namespace P64::CollNew;

namespace {
  constexpr int GJK_MAX_ITERATIONS = 24;

  void simplexMovePoint(Simplex &simplex, int to, int from) {
    simplex.points[to] = simplex.points[from];
    simplex.objectAPoint[to] = simplex.objectAPoint[from];
  }
}

fm_vec3_t *P64::CollNew::simplexAddPoint(Simplex &simplex, const fm_vec3_t &aPoint, const fm_vec3_t &bPoint) {
  if(simplex.nPoints == GJK_MAX_SIMPLEX_SIZE) {
    return nullptr;
  }

  int index = simplex.nPoints;
  simplex.objectAPoint[index] = aPoint;
  simplex.points[index] = vec3Sub(aPoint, bPoint);
  ++simplex.nPoints;

  return &simplex.points[index];
}

bool P64::CollNew::simplexCheck(Simplex &simplex, fm_vec3_t &nextDirection) {
  auto &lastAdded = simplex.points[simplex.nPoints - 1];
  auto aToOrigin = vec3Negate(lastAdded);

  if(simplex.nPoints == 2) {
    auto lastAddedToOther = vec3Sub(simplex.points[0], lastAdded);
    nextDirection = vec3TripleProduct(lastAddedToOther, aToOrigin, lastAddedToOther);

    if(vec3MagSqrd(nextDirection) <= 0.0000001f) {
      nextDirection = vec3Perpendicular(lastAddedToOther);
    }
    return false;

  } else if(simplex.nPoints == 3) {
    auto ab = vec3Sub(simplex.points[1], lastAdded);
    auto ac = vec3Sub(simplex.points[0], lastAdded);
    auto normal = vec3Cross(ab, ac);

    auto dirCheck = vec3Cross(ab, normal);
    if(vec3Dot(dirCheck, aToOrigin) > 0.0f) {
      nextDirection = vec3TripleProduct(ab, aToOrigin, ab);
      if(vec3MagSqrd(nextDirection) <= 0.0000001f) {
        nextDirection = normal;
      }
      // remove c
      simplexMovePoint(simplex, 0, 1);
      simplexMovePoint(simplex, 1, 2);
      simplex.nPoints = 2;
      return false;
    }

    dirCheck = vec3Cross(normal, ac);
    if(vec3Dot(dirCheck, aToOrigin) > 0.0f) {
      nextDirection = vec3TripleProduct(ac, aToOrigin, ac);
      if(vec3MagSqrd(nextDirection) <= 0.0000001f) {
        nextDirection = normal;
      }
      // remove b
      simplexMovePoint(simplex, 1, 2);
      simplex.nPoints = 2;
      return false;
    }

    if(vec3Dot(normal, aToOrigin) > 0.0f) {
      nextDirection = normal;
      return false;
    }

    // change triangle winding
    simplexMovePoint(simplex, 3, 0);
    simplexMovePoint(simplex, 0, 1);
    simplexMovePoint(simplex, 1, 3);
    nextDirection = vec3Negate(normal);

  } else if(simplex.nPoints == 4) {
    int lastBehindIndex = -1;
    int lastInFrontIndex = -1;
    int isFrontCount = 0;

    fm_vec3_t normals[3];

    for(int i = 0; i < 3; ++i) {
      auto firstEdge = vec3Sub(lastAdded, simplex.points[i]);
      auto secondEdge = vec3Sub(
        (i == 2) ? simplex.points[0] : simplex.points[i + 1],
        simplex.points[i]
      );
      normals[i] = vec3Cross(firstEdge, secondEdge);

      if(vec3Dot(aToOrigin, normals[i]) > 0.0f) {
        ++isFrontCount;
        lastInFrontIndex = i;
      } else {
        lastBehindIndex = i;
      }
    }

    if(isFrontCount == 0) {
      return true; // origin enclosed
    } else if(isFrontCount == 1) {
      nextDirection = normals[lastInFrontIndex];
      if(lastInFrontIndex == 1) {
        simplexMovePoint(simplex, 0, 1);
        simplexMovePoint(simplex, 1, 2);
      } else if(lastInFrontIndex == 2) {
        simplexMovePoint(simplex, 1, 0);
        simplexMovePoint(simplex, 0, 2);
      }
      simplexMovePoint(simplex, 2, 3);
      simplex.nPoints = 3;
    } else if(isFrontCount == 2) {
      if(lastBehindIndex == 0) {
        simplexMovePoint(simplex, 0, 2);
      } else if(lastBehindIndex == 2) {
        simplexMovePoint(simplex, 0, 1);
      }
      simplexMovePoint(simplex, 1, 3);
      simplex.nPoints = 2;

      auto ab = vec3Sub(simplex.points[0], simplex.points[1]);
      nextDirection = vec3TripleProduct(ab, aToOrigin, ab);
      if(vec3MagSqrd(nextDirection) <= 0.0000001f) {
        nextDirection = vec3Perpendicular(ab);
      }
    } else {
      // origin in front of all three faces — reset to single point
      simplexMovePoint(simplex, 0, 3);
      simplex.nPoints = 1;
      nextDirection = aToOrigin;
    }
  }

  return false;
}

bool P64::CollNew::gjkCheckForOverlap(
  Simplex &simplex,
  const void *objectA, GjkSupportFunction objectASupport,
  const void *objectB, GjkSupportFunction objectBSupport,
  const fm_vec3_t &firstDirection
) {
  fm_vec3_t aPoint{};
  fm_vec3_t bPoint{};
  fm_vec3_t nextDirection{};
  fm_vec3_t dir = firstDirection;

  simplex.nPoints = 0;

  if(vec3IsZero(dir)) {
    dir = vec3Right();
  }

  objectASupport(objectA, dir, aPoint);
  nextDirection = vec3Negate(dir);
  objectBSupport(objectB, nextDirection, bPoint);
  simplexAddPoint(simplex, aPoint, bPoint);

  for(int iteration = 0; iteration < GJK_MAX_ITERATIONS; ++iteration) {
    auto reverseDirection = vec3Negate(nextDirection);
    objectASupport(objectA, nextDirection, aPoint);
    objectBSupport(objectB, reverseDirection, bPoint);

    auto *addedPoint = simplexAddPoint(simplex, aPoint, bPoint);
    if(!addedPoint) return false;

    if(vec3Dot(*addedPoint, nextDirection) <= 0.0f) return false;

    if(simplexCheck(simplex, nextDirection)) return true;
  }

  return false;
}
