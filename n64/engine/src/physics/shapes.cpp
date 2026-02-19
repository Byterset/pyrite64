/**
 * @copyright 2026 - Max Bebök
 * @license MIT
 */
#include "physics/shapes.h"
#include <cmath>

namespace P64::Physics
{
  fm_vec3_t ColliderShape::getLocalAABBMin() const {
    fm_vec3_t min = localOffset;
    
    switch (type) {
      case ShapeType::SPHERE:
        min.x -= data.sphere.radius;
        min.y -= data.sphere.radius;
        min.z -= data.sphere.radius;
        break;
        
      case ShapeType::BOX:
        min = min - data.box.halfSize;
        break;
        
      case ShapeType::CYLINDER:
        min.x -= data.cylinder.radius;
        min.y -= data.cylinder.halfHeight;
        min.z -= data.cylinder.radius;
        break;
        
      case ShapeType::CAPSULE:
        min.x -= data.capsule.radius;
        min.y -= (data.capsule.innerHalfHeight + data.capsule.radius);
        min.z -= data.capsule.radius;
        break;
    }
    
    return min;
  }
  
  fm_vec3_t ColliderShape::getLocalAABBMax() const {
    fm_vec3_t max = localOffset;
    
    switch (type) {
      case ShapeType::SPHERE:
        max.x += data.sphere.radius;
        max.y += data.sphere.radius;
        max.z += data.sphere.radius;
        break;
        
      case ShapeType::BOX:
        max = max + data.box.halfSize;
        break;
        
      case ShapeType::CYLINDER:
        max.x += data.cylinder.radius;
        max.y += data.cylinder.halfHeight;
        max.z += data.cylinder.radius;
        break;
        
      case ShapeType::CAPSULE:
        max.x += data.capsule.radius;
        max.y += (data.capsule.innerHalfHeight + data.capsule.radius);
        max.z += data.capsule.radius;
        break;
    }
    
    return max;
  }
  
  fm_vec3_t ColliderShape::support(const fm_vec3_t& direction) const {
    fm_vec3_t result = localOffset;
    
    switch (type) {
      case ShapeType::SPHERE: {
        // Furthest point on sphere surface in direction
        float len = fm_vec3_len(&direction);
        if (len > 0.0001f) {
          fm_vec3_t normDir = direction / len;
          result = result + normDir * data.sphere.radius;
        }
        break;
      }
      
      case ShapeType::BOX: {
        // Furthest corner in direction
        result.x += (direction.x >= 0) ? data.box.halfSize.x : -data.box.halfSize.x;
        result.y += (direction.y >= 0) ? data.box.halfSize.y : -data.box.halfSize.y;
        result.z += (direction.z >= 0) ? data.box.halfSize.z : -data.box.halfSize.z;
        break;
      }
      
      case ShapeType::CYLINDER: {
        // Cylinder aligned along Y-axis
        // Top/bottom cap selection
        float y = (direction.y >= 0) ? data.cylinder.halfHeight : -data.cylinder.halfHeight;
        
        // Radial component in XZ plane
        float lenXZ = sqrtf(direction.x * direction.x + direction.z * direction.z);
        if (lenXZ > 0.0001f) {
          result.x += (direction.x / lenXZ) * data.cylinder.radius;
          result.z += (direction.z / lenXZ) * data.cylinder.radius;
        }
        result.y += y;
        break;
      }
      
      case ShapeType::CAPSULE: {
        // Capsule aligned along Y-axis
        // Choose top or bottom sphere center
        float y = copysignf(data.capsule.innerHalfHeight, direction.y);
        
        // Add radius in direction
        float len = fm_vec3_len(&direction);
        if (len > 0.0001f) {
          fm_vec3_t normDir = direction / len;
          result.x += normDir.x * data.capsule.radius;
          result.y += normDir.y * data.capsule.radius + y;
          result.z += normDir.z * data.capsule.radius;
        } else {
          result.y += y;
        }
        break;
      }
    }
    
    return result;
  }
}
