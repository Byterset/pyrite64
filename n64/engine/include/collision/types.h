/**
 * @file types.h
 * @author Kevin Reier (Byterset)
 * @brief Contains the different existing Basic Collider Shape Types
 */

#pragma once

namespace P64::Coll
{
  enum class ShapeType : uint8_t
  {
    Sphere,
    Capsule,
    Box,
    Cone,
    Cylinder,
    Pyramid
  };
}