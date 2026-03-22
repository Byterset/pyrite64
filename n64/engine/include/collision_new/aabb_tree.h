/**
 * @copyright 2024 - Max Bebök
 * @license MIT
 */
#pragma once

#include "shapes.h"
#include "vec_math.h"
#include <cstdint>
#include <cassert>
#include <memory>

namespace P64::CollNew {

  using NodeProxy = int16_t;
  constexpr NodeProxy NULL_NODE = -1;
  constexpr float AABB_DISPLACEMENT_MULTIPLIER = 10.0f;
  constexpr float AABB_NODE_BOUNDS_MARGIN = 1.2f;
  constexpr int AABB_QUERY_STACK_SIZE = 256;

  // Forward declare Raycast for ray queries
  struct Raycast;

  // ── AABB utility functions ────────────────────────────────────────

  inline bool aabbOverlap(const AABB &a, const AABB &b) {
    return (a.max.x >= b.min.x) && (a.min.x <= b.max.x)
        && (a.max.y >= b.min.y) && (a.min.y <= b.max.y)
        && (a.max.z >= b.min.z) && (a.min.z <= b.max.z);
  }

  inline bool aabbContains(const AABB &outer, const AABB &inner) {
    return (outer.min.x <= inner.min.x) && (outer.max.x >= inner.max.x)
        && (outer.min.y <= inner.min.y) && (outer.max.y >= inner.max.y)
        && (outer.min.z <= inner.min.z) && (outer.max.z >= inner.max.z);
  }

  inline bool aabbContainsPoint(const AABB &box, const fm_vec3_t &p) {
    return (p.x >= box.min.x) && (p.x <= box.max.x)
        && (p.y >= box.min.y) && (p.y <= box.max.y)
        && (p.z >= box.min.z) && (p.z <= box.max.z);
  }

  inline AABB aabbUnion(const AABB &a, const AABB &b) {
    return {vec3Min(a.min, b.min), vec3Max(a.max, b.max)};
  }

  inline float aabbArea(const AABB &box) {
    float dx = box.max.x - box.min.x;
    float dy = box.max.y - box.min.y;
    float dz = box.max.z - box.min.z;
    return 2.0f * (dx*dy + dy*dz + dz*dx);
  }

  inline void aabbExtendDirection(const AABB &in, const fm_vec3_t &dir, AABB &out) {
    out = in;
    if(dir.x > 0.0f) out.max.x += dir.x; else out.min.x += dir.x;
    if(dir.y > 0.0f) out.max.y += dir.y; else out.min.y += dir.y;
    if(dir.z > 0.0f) out.max.z += dir.z; else out.min.z += dir.z;
  }

  bool aabbIntersectsRay(const AABB &box, const fm_vec3_t &origin, const fm_vec3_t &invDir, float maxDist);

  // ── Tree node ─────────────────────────────────────────────────────

  struct AABBTreeNode {
    AABB bounds{};
    NodeProxy parent{NULL_NODE};
    NodeProxy left{NULL_NODE};
    NodeProxy right{NULL_NODE};
    NodeProxy next{NULL_NODE};
    void *data{nullptr};
  };

  // ── AABB Tree (dynamic BVH) ───────────────────────────────────────

  class AABBTree {
  public:
    AABBTree() = default;
    ~AABBTree();

    void init(int capacity);
    void destroy();

    NodeProxy createNode(const AABB &bounds, void *data);
    bool moveNode(NodeProxy node, const AABB &aabb, const fm_vec3_t &displacement);
    void removeLeaf(NodeProxy leaf, bool freeIt);

    [[nodiscard]] void *getNodeData(NodeProxy node) const;
    [[nodiscard]] const AABB *getNodeBounds(NodeProxy node) const;
    [[nodiscard]] bool isLeaf(NodeProxy node) const;

    int queryBounds(const AABB &queryBox, NodeProxy *results, int maxResults) const;
    int queryPoint(const fm_vec3_t &point, NodeProxy *results, int maxResults) const;
    int queryRay(const fm_vec3_t &origin, const fm_vec3_t &invDir, float maxDist,
                 NodeProxy *results, int maxResults) const;

    NodeProxy root{NULL_NODE};

  private:
    NodeProxy allocateNode();
    void freeNode(NodeProxy node);
    NodeProxy insertLeaf(NodeProxy leaf);
    void rotateNode(NodeProxy node);

    std::unique_ptr<AABBTreeNode[]> nodes_;
    int16_t nodeCount_{0};
    int16_t nodeCapacity_{0};
    NodeProxy freeList_{NULL_NODE};
  };

} // namespace P64::CollNew
