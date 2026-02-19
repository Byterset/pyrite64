/**
 * @copyright 2026 - Max Bebök
 * @license MIT
 */
#pragma once
#include "gjk.h"

namespace P64::Physics
{
  constexpr int EPA_MAX_ITERATIONS = 32;
  constexpr int EPA_MAX_FACES = 64;
  constexpr float EPA_TOLERANCE = 0.001f;
  
  /**
   * EPA result structure
   */
  struct EpaResult {
    fm_vec3_t contactA{};      // Contact point on shape A
    fm_vec3_t contactB{};      // Contact point on shape B
    fm_vec3_t normal{};        // Normal from B toward A
    float penetration{};       // Penetration depth
    bool valid{false};
  };
  
  /**
   * EPA polytope face
   */
  struct EpaFace {
    int indices[3];            // Vertex indices
    fm_vec3_t normal{};
    float distance{};          // Distance from origin
  };
  
  /**
   * Expanding Polytope Algorithm - finds penetration depth and contact normal
   * @param simplex GJK simplex containing the origin (4-point tetrahedron)
   * @param dataA Data for shape A
   * @param supportA Support function for shape A
   * @param dataB Data for shape B
   * @param supportB Support function for shape B
   * @param result Output penetration information
   * @return true if successful
   */
  bool epaSolve(Simplex& simplex,
                const void* dataA, SupportFunction supportA,
                const void* dataB, SupportFunction supportB,
                EpaResult& result);
}
