/**
 * @copyright 2026 - Max Bebök
 * @license MIT
 */
#include "physics/epa.h"
#include <cmath>
#include <algorithm>

namespace P64::Physics
{
  namespace {
    struct EpaPolytope {
      fm_vec3_t vertices[EPA_MAX_FACES];
      fm_vec3_t verticesA[EPA_MAX_FACES];
      fm_vec3_t verticesB[EPA_MAX_FACES];
      EpaFace faces[EPA_MAX_FACES];
      int vertexCount;
      int faceCount;
      
      void addVertex(const fm_vec3_t& v, const fm_vec3_t& vA, const fm_vec3_t& vB) {
        if (vertexCount < EPA_MAX_FACES) {
          vertices[vertexCount] = v;
          verticesA[vertexCount] = vA;
          verticesB[vertexCount] = vB;
          vertexCount++;
        }
      }
      
      void addFace(int a, int b, int c) {
        if (faceCount >= EPA_MAX_FACES) return;
        
        EpaFace& face = faces[faceCount];
        face.indices[0] = a;
        face.indices[1] = b;
        face.indices[2] = c;
        
        // Calculate face normal and distance
        fm_vec3_t ab = vertices[b] - vertices[a];
        fm_vec3_t ac = vertices[c] - vertices[a];
        t3d_vec3_cross(&face.normal, &ab, &ac);
        
        float len = fm_vec3_len(&face.normal);
        if (len > 0.0001f) {
          face.normal = face.normal / len;
        }
        
        // Distance from origin to face
        face.distance = t3d_vec3_dot(face.normal, vertices[a]);
        
        // Ensure normal points outward (away from origin)
        if (face.distance < 0) {
          face.normal = face.normal * -1.0f;
          face.distance = -face.distance;
          // Swap winding
          int temp = face.indices[1];
          face.indices[1] = face.indices[2];
          face.indices[2] = temp;
        }
        
        faceCount++;
      }
      
      int findClosestFace() const {
        int closest = 0;
        float minDist = faces[0].distance;
        
        for (int i = 1; i < faceCount; i++) {
          if (faces[i].distance < minDist) {
            minDist = faces[i].distance;
            closest = i;
          }
        }
        
        return closest;
      }
    };
  }
  
  bool epaSolve(Simplex& simplex,
                const void* dataA, SupportFunction supportA,
                const void* dataB, SupportFunction supportB,
                EpaResult& result) {
    if (simplex.count != 4) {
      return false;  // Need full tetrahedron from GJK
    }
    
    // Initialize polytope from GJK simplex
    EpaPolytope polytope;
    polytope.vertexCount = 0;
    polytope.faceCount = 0;
    
    for (int i = 0; i < 4; i++) {
      polytope.addVertex(simplex.points[i], simplex.pointsA[i], simplex.pointsB[i]);
    }
    
    // Create initial tetrahedron faces
    polytope.addFace(0, 1, 2);
    polytope.addFace(0, 3, 1);
    polytope.addFace(0, 2, 3);
    polytope.addFace(1, 3, 2);
    
    // Iteratively expand polytope toward boundary
    for (int iter = 0; iter < EPA_MAX_ITERATIONS; iter++) {
      // Find face closest to origin
      int closestFaceIdx = polytope.findClosestFace();
      const EpaFace& closestFace = polytope.faces[closestFaceIdx];
      
      // Get new support point in direction of closest face normal
      fm_vec3_t supportPointA = supportA(dataA, closestFace.normal);
      fm_vec3_t supportPointB = supportB(dataB, closestFace.normal * -1.0f);
      fm_vec3_t supportPoint = supportPointA - supportPointB;
      
      // Check if we've reached the boundary
      float distance = t3d_vec3_dot(supportPoint, closestFace.normal);
      if (distance - closestFace.distance < EPA_TOLERANCE) {
        // We've converged - extract contact information
        result.normal = closestFace.normal;
        result.penetration = closestFace.distance;
        
        // Calculate contact points using barycentric coordinates
        int i0 = closestFace.indices[0];
        int i1 = closestFace.indices[1];
        int i2 = closestFace.indices[2];
        
        // Project origin onto face plane
        fm_vec3_t p = closestFace.normal * closestFace.distance;
        
        // Compute barycentric coordinates (simplified - use equal weights for now)
        float w0 = 1.0f / 3.0f;
        float w1 = 1.0f / 3.0f;
        float w2 = 1.0f / 3.0f;
        
        result.contactA = polytope.verticesA[i0] * w0 + 
                         polytope.verticesA[i1] * w1 + 
                         polytope.verticesA[i2] * w2;
        result.contactB = polytope.verticesB[i0] * w0 + 
                         polytope.verticesB[i1] * w1 + 
                         polytope.verticesB[i2] * w2;
        
        result.valid = true;
        return true;
      }
      
      // Add new vertex
      int newVertexIdx = polytope.vertexCount;
      polytope.addVertex(supportPoint, supportPointA, supportPointB);
      
      // Remove closest face and add new faces
      // This is simplified - a full implementation would handle edge cases better
      if (polytope.faceCount < EPA_MAX_FACES - 3) {
        // Remove the closest face by swapping with last
        polytope.faces[closestFaceIdx] = polytope.faces[polytope.faceCount - 1];
        polytope.faceCount--;
        
        // Add 3 new faces connecting edges of removed face to new vertex
        polytope.addFace(closestFace.indices[0], closestFace.indices[1], newVertexIdx);
        polytope.addFace(closestFace.indices[1], closestFace.indices[2], newVertexIdx);
        polytope.addFace(closestFace.indices[2], closestFace.indices[0], newVertexIdx);
      } else {
        break;  // Out of space
      }
    }
    
    // Fallback if we didn't converge
    int closestFaceIdx = polytope.findClosestFace();
    const EpaFace& closestFace = polytope.faces[closestFaceIdx];
    result.normal = closestFace.normal;
    result.penetration = closestFace.distance;
    
    int i0 = closestFace.indices[0];
    int i1 = closestFace.indices[1];
    int i2 = closestFace.indices[2];
    
    result.contactA = (polytope.verticesA[i0] + polytope.verticesA[i1] + polytope.verticesA[i2]) / 3.0f;
    result.contactB = (polytope.verticesB[i0] + polytope.verticesB[i1] + polytope.verticesB[i2]) / 3.0f;
    
    result.valid = true;
    return true;
  }
}
