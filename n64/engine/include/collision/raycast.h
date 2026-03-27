/**
 * @file raycast.h
 * @author Kevin Reier <https://github.com/Byterset>
 * @brief Raycast definitions and functions
 */
#pragma once

#include "vec_math.h"
#include <cstdint>
#include <cmath>

namespace P64::Coll {

  constexpr float RAYCAST_MAX_DISTANCE = 2000.0f;
  constexpr int RAYCAST_MAX_OBJECT_TESTS = 10;
  constexpr int RAYCAST_MAX_TRIANGLE_TESTS = 15;

  enum class RaycastMask : uint8_t {
    MESH_COLLIDERS = (1 << 0),
    COLLIDER_BODIES  = (1 << 1),
    All             = 0xFF
  };

  inline RaycastMask operator|(RaycastMask a, RaycastMask b) {
    return static_cast<RaycastMask>(static_cast<uint8_t>(a) | static_cast<uint8_t>(b));
  }
  inline bool hasFlag(RaycastMask m, RaycastMask f) {
    return (static_cast<uint8_t>(m) & static_cast<uint8_t>(f)) != 0;
  }

  struct Raycast {
    fm_vec3_t origin{};
    fm_vec3_t dir{};
    fm_vec3_t invDir{};
    float maxDistance{RAYCAST_MAX_DISTANCE};
    RaycastMask mask{RaycastMask::All};
    uint16_t collisionLayers{0xFFFF};
    uint16_t ignoreLayers{0};
    bool interactTrigger{false};

    static Raycast create(const fm_vec3_t &origin, const fm_vec3_t &dir, float maxDist,
                          RaycastMask mask = RaycastMask::All, bool interactTrigger = false,
                          uint16_t collisionLayers = 0xFFFF, uint16_t ignoreLayers = 0);
  };

  struct RaycastHit {
    fm_vec3_t point{};
    fm_vec3_t normal{};
    float distance{1e30f};
    uint16_t hitId{0};
    bool didHit{false};
  };

} // namespace P64::Coll
