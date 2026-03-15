/**
 * @copyright 2024 - Max Bebök
 * @license MIT
 */
#include "collision_new/raycast.h"

namespace P64::CollNew {

  Raycast Raycast::create(const fm_vec3_t &origin, const fm_vec3_t &dir, float maxDist,
                          RaycastMask mask, bool interactTrigger,
                          uint16_t collisionLayers, uint16_t ignoreLayers) {
    Raycast r;
    r.origin = origin;

    // Normalize direction
    float mag = vec3Mag(dir);
    if(mag < EPSILON) {
      r.dir = vec3(0.0f, -1.0f, 0.0f);
    } else {
      r.dir = vec3Scale(dir, 1.0f / mag);
    }

    // Safe inverse direction for AABB ray tests
    constexpr float INV_SAFE = 1e-6f;
    r.invDir = vec3(
      fabsf(r.dir.x) > INV_SAFE ? 1.0f / r.dir.x : copysignf(1.0f / INV_SAFE, r.dir.x),
      fabsf(r.dir.y) > INV_SAFE ? 1.0f / r.dir.y : copysignf(1.0f / INV_SAFE, r.dir.y),
      fabsf(r.dir.z) > INV_SAFE ? 1.0f / r.dir.z : copysignf(1.0f / INV_SAFE, r.dir.z)
    );

    r.maxDistance = (maxDist > 0.0f && maxDist < RAYCAST_MAX_DISTANCE) ? maxDist : RAYCAST_MAX_DISTANCE;
    r.mask = mask;
    r.interactTrigger = interactTrigger;
    r.collisionLayers = collisionLayers;
    r.ignoreLayers = ignoreLayers;
    return r;
  }

} // namespace P64::CollNew
