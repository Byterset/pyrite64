/**
 * @copyright 2026 - Max Bebök
 * @license MIT
 */
#include "physics/gjk.h"
#include <cmath>

namespace P64::Physics
{
  namespace {
    // Helper: compute triple product (A × B) × C
    fm_vec3_t tripleProduct(const fm_vec3_t& a, const fm_vec3_t& b, const fm_vec3_t& c) {
      // (A × B) × C = B(A·C) - A(B·C)
      float ac = t3d_vec3_dot(a, c);
      float bc = t3d_vec3_dot(b, c);
      return b * ac - a * bc;
    }
    
    // Helper: get perpendicular vector
    fm_vec3_t getPerpendicular(const fm_vec3_t& v) {
      if (fabsf(v.x) < 0.9f) {
        return fm_vec3_t{1.0f, 0.0f, 0.0f};
      }
      return fm_vec3_t{0.0f, 1.0f, 0.0f};
    }
  }
  
  bool simplexContainsOrigin(Simplex& simplex, fm_vec3_t& direction) {
    if (simplex.count == 2) {
      // Line case
      fm_vec3_t a = simplex.points[1];
      fm_vec3_t b = simplex.points[0];
      fm_vec3_t ao = fm_vec3_t{0,0,0} - a;
      fm_vec3_t ab = b - a;
      
      // Direction perpendicular to AB toward origin
      direction = tripleProduct(ab, ao, ab);
      
      // If direction is too small, use perpendicular
      if (fm_vec3_len2(&direction) < 0.0001f) {
        fm_vec3_t perp = getPerpendicular(ab);
        direction = tripleProduct(ab, perp, ab);
      }
      return false;
    }
    
    if (simplex.count == 3) {
      // Triangle case
      fm_vec3_t a = simplex.points[2];
      fm_vec3_t b = simplex.points[1];
      fm_vec3_t c = simplex.points[0];
      fm_vec3_t ao = fm_vec3_t{0,0,0} - a;
      fm_vec3_t ab = b - a;
      fm_vec3_t ac = c - a;
      
      // Triangle normal
      fm_vec3_t abc;
      t3d_vec3_cross(&abc, &ab, &ac);
      
      // Check which side of triangle origin is on
      fm_vec3_t abPerp;
      t3d_vec3_cross(&abPerp, &abc, &ab);
      
      if (t3d_vec3_dot(abPerp, ao) > 0) {
        // Origin is on AB side
        simplex.points[0] = b;
        simplex.pointsA[0] = simplex.pointsA[1];
        simplex.pointsB[0] = simplex.pointsB[1];
        simplex.points[1] = a;
        simplex.count = 2;
        direction = tripleProduct(ab, ao, ab);
        return false;
      }
      
      fm_vec3_t acPerp;
      t3d_vec3_cross(&acPerp, &ac, &abc);
      
      if (t3d_vec3_dot(acPerp, ao) > 0) {
        // Origin is on AC side
        simplex.points[0] = c;
        simplex.points[1] = a;
        simplex.pointsA[1] = simplex.pointsA[2];
        simplex.pointsB[1] = simplex.pointsB[2];
        simplex.count = 2;
        direction = tripleProduct(ac, ao, ac);
        return false;
      }
      
      // Origin is above or below triangle
      if (t3d_vec3_dot(abc, ao) > 0) {
        direction = abc;
      } else {
        // Flip winding
        auto temp = simplex.points[0];
        auto tempA = simplex.pointsA[0];
        auto tempB = simplex.pointsB[0];
        simplex.points[0] = simplex.points[1];
        simplex.pointsA[0] = simplex.pointsA[1];
        simplex.pointsB[0] = simplex.pointsB[1];
        simplex.points[1] = temp;
        simplex.pointsA[1] = tempA;
        simplex.pointsB[1] = tempB;
        direction = abc * -1.0f;
      }
      return false;
    }
    
    if (simplex.count == 4) {
      // Tetrahedron case
      fm_vec3_t a = simplex.points[3];
      fm_vec3_t b = simplex.points[2];
      fm_vec3_t c = simplex.points[1];
      fm_vec3_t d = simplex.points[0];
      fm_vec3_t ao = fm_vec3_t{0,0,0} - a;
      
      fm_vec3_t ab = b - a;
      fm_vec3_t ac = c - a;
      fm_vec3_t ad = d - a;
      
      // Check each face
      fm_vec3_t abc, acd, adb;
      t3d_vec3_cross(&abc, &ab, &ac);
      t3d_vec3_cross(&acd, &ac, &ad);
      t3d_vec3_cross(&adb, &ad, &ab);
      
      int facesInFront = 0;
      bool abcFront = t3d_vec3_dot(abc, ao) > 0;
      bool acdFront = t3d_vec3_dot(acd, ao) > 0;
      bool adbFront = t3d_vec3_dot(adb, ao) > 0;
      
      if (abcFront) facesInFront++;
      if (acdFront) facesInFront++;
      if (adbFront) facesInFront++;
      
      if (facesInFront == 0) {
        // Origin is inside tetrahedron
        return true;
      }
      
      // Origin is outside - reduce to triangle closest to origin
      if (abcFront) {
        simplex.points[0] = c;
        simplex.points[1] = b;
        simplex.points[2] = a;
        simplex.pointsA[2] = simplex.pointsA[3];
        simplex.pointsB[2] = simplex.pointsB[3];
        simplex.count = 3;
        direction = abc;
      } else if (acdFront) {
        simplex.points[0] = d;
        simplex.points[2] = a;
        simplex.pointsA[2] = simplex.pointsA[3];
        simplex.pointsB[2] = simplex.pointsB[3];
        simplex.count = 3;
        direction = acd;
      } else {
        simplex.points[0] = d;
        simplex.points[1] = b;
        simplex.points[2] = a;
        simplex.pointsA[2] = simplex.pointsA[3];
        simplex.pointsB[2] = simplex.pointsB[3];
        simplex.count = 3;
        direction = adb;
      }
      return false;
    }
    
    return false;
  }
  
  bool gjkCheckOverlap(Simplex& simplex,
                       const void* dataA, SupportFunction supportA,
                       const void* dataB, SupportFunction supportB,
                       const fm_vec3_t& initialDir) {
    simplex.clear();
    fm_vec3_t direction = initialDir;
    
    // Normalize initial direction
    float len = fm_vec3_len(&direction);
    if (len < 0.0001f) {
      direction = fm_vec3_t{1, 0, 0};
    } else {
      direction = direction / len;
    }
    
    // Get first support point (Minkowski difference A - B)
    fm_vec3_t supportPointA = supportA(dataA, direction);
    fm_vec3_t supportPointB = supportB(dataB, direction * -1.0f);
    fm_vec3_t supportPoint = supportPointA - supportPointB;
    simplex.add(supportPoint, supportPointA, supportPointB);
    
    // Next direction toward origin
    direction = supportPoint * -1.0f;
    
    for (int i = 0; i < GJK_MAX_ITERATIONS; i++) {
      // Normalize direction
      len = fm_vec3_len(&direction);
      if (len < 0.0001f) {
        break;
      }
      direction = direction / len;
      
      // Get new support point
      supportPointA = supportA(dataA, direction);
      supportPointB = supportB(dataB, direction * -1.0f);
      supportPoint = supportPointA - supportPointB;
      
      // Check if we passed the origin
      if (t3d_vec3_dot(supportPoint, direction) < 0) {
        return false;  // No collision
      }
      
      simplex.add(supportPoint, supportPointA, supportPointB);
      
      if (simplexContainsOrigin(simplex, direction)) {
        return true;  // Collision found
      }
    }
    
    return false;
  }
}
