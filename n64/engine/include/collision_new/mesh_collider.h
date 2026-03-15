/**
 * @copyright 2024 - Max Bebök
 * @license MIT
 */
#pragma once

#include "vec_math.h"
#include "aabb_tree.h"
#include <cstdint>

namespace P64::CollNew {

  struct MeshTriangleIndices {
    uint16_t indices[3];
  };

  struct MeshTriangle {
    const fm_vec3_t *vertices;
    fm_vec3_t normal;
    MeshTriangleIndices tri;

    void gjkSupport(const fm_vec3_t &direction, fm_vec3_t &output) const;
    float comparePoint(const fm_vec3_t &point) const;
  };

  /// GJK-compatible wrapper for MeshTriangle
  void meshTriangleGjkSupport(const void *data, const fm_vec3_t &direction, fm_vec3_t &output);

  struct MeshCollider {
    AABBTree aabbTree;
    fm_vec3_t *vertices{nullptr};
    MeshTriangleIndices *triangles{nullptr};
    fm_vec3_t *normals{nullptr};
    uint16_t triangleCount{0};
    uint16_t vertexCount{0};
    fm_vec3_t *offset{nullptr};
    float scale{1.0f};
  };

} // namespace P64::CollNew
