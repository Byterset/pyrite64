/**
 * @copyright 2024 - Max Bebök
 * @license MIT
 */
#include "collision/raycast.h"

namespace P64::Coll {

  Raycast Raycast::create(const fm_vec3_t &origin, const fm_vec3_t &dir, float maxDist,
                          RaycastMask mask, bool interactTrigger,
                          uint16_t collisionLayers, uint16_t ignoreLayers) {
    Raycast r;
    r.origin = origin;

    // Normalize direction
    float mag2 = fm_vec3_len2(&dir);
    if(mag2 < FM_EPSILON * FM_EPSILON) {
      r.dir = fm_vec3_t{{0.0f, -1.0f, 0.0f}};
    } else {
      r.dir = dir / sqrtf(mag2);
    }

    // Safe inverse direction for AABB ray tests
    constexpr float INV_SAFE = 1e-6f;
    r.invDir = fm_vec3_t{{ 
      fabsf(r.dir.x) > FM_EPSILON ? 1.0f / r.dir.x : copysignf(1.0f / FM_EPSILON, r.dir.x),
      fabsf(r.dir.y) > FM_EPSILON ? 1.0f / r.dir.y : copysignf(1.0f / FM_EPSILON, r.dir.y),
      fabsf(r.dir.z) > FM_EPSILON ? 1.0f / r.dir.z : copysignf(1.0f / FM_EPSILON, r.dir.z)
    }};

    r.maxDistance = (maxDist > 0.0f && maxDist < RAYCAST_MAX_DISTANCE) ? maxDist : RAYCAST_MAX_DISTANCE;
    r.mask = mask;
    r.interactTrigger = interactTrigger;
    r.collisionLayers = collisionLayers;
    r.ignoreLayers = ignoreLayers;
    return r;
  }

} // namespace P64::Coll
