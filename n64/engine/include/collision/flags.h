/**
* @copyright 2025 - Max Bebök
* @license MIT
*/
#pragma once
#include <cstdint>

namespace P64::Coll
{
  namespace TriType {
    constexpr uint8_t FLOOR = 1 << 0;
    constexpr uint8_t WALL  = 1 << 1;
    constexpr uint8_t CEIL  = 1 << 2;
    constexpr uint8_t BCS   = 1 << 3;
  }

  namespace BCSFlags {
    // Shape type flags (bits 0-1 for 4 shape types)
    constexpr uint8_t SHAPE_MASK     = 0x03;  // Mask for shape type bits
    constexpr uint8_t SHAPE_SPHERE   = 0x00;  // Default/no flags
    constexpr uint8_t SHAPE_BOX      = 0x01;
    constexpr uint8_t SHAPE_CYLINDER = 0x02;
    constexpr uint8_t SHAPE_CAPSULE  = 0x03;

    constexpr uint8_t TRIGGER   = 1 << 2;
    constexpr uint8_t BOUNCY    = 1 << 3;
    constexpr uint8_t FIXED_XYZ = 1 << 4;
  }
}
