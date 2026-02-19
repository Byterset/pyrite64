/**
 * @copyright 2026 - Max Bebök
 * @license MIT
 */
#pragma once
#include <t3d/t3dmath.h>

namespace P64::Physics
{
  constexpr int GJK_MAX_ITERATIONS = 32;
  constexpr int GJK_MAX_SIMPLEX_SIZE = 4;
  
  /**
   * Simplex structure for GJK algorithm
   */
  struct Simplex {
    fm_vec3_t points[GJK_MAX_SIMPLEX_SIZE];
    fm_vec3_t pointsA[GJK_MAX_SIMPLEX_SIZE];  // Points on shape A
    fm_vec3_t pointsB[GJK_MAX_SIMPLEX_SIZE];  // Points on shape B
    int count{0};
    
    void add(const fm_vec3_t& p, const fm_vec3_t& pA, const fm_vec3_t& pB) {
      points[count] = p;
      pointsA[count] = pA;
      pointsB[count] = pB;
      count++;
    }
    
    void clear() {
      count = 0;
    }
  };
  
  /**
   * GJK support function type
   */
  using SupportFunction = fm_vec3_t(*)(const void* data, const fm_vec3_t& direction);
  
  /**
   * Check if simplex contains origin and update search direction
   * @return true if origin is enclosed
   */
  bool simplexContainsOrigin(Simplex& simplex, fm_vec3_t& direction);
  
  /**
   * Run GJK algorithm to check if two shapes overlap
   * @param simplex Output simplex (used for EPA if collision found)
   * @param dataA Data for shape A
   * @param supportA Support function for shape A
   * @param dataB Data for shape B
   * @param supportB Support function for shape B
   * @param initialDir Initial search direction
   * @return true if shapes overlap
   */
  bool gjkCheckOverlap(Simplex& simplex,
                       const void* dataA, SupportFunction supportA,
                       const void* dataB, SupportFunction supportB,
                       const fm_vec3_t& initialDir);
}
