/**
 * @file mesh_collider.h
 * @author Kevin Reier <https://github.com/Byterset>
 * @brief Mesh Collider definitions and functions
 */
#pragma once

#include "vec_math.h"
#include "aabb_tree.h"
#include "rigid_body.h"
#include <cstdint>

namespace P64 { class Object; }

namespace P64::Coll {

  struct Collider;

  struct MeshCollider; // forward declare

  struct MeshTriangleIndices {
    uint16_t indices[3];
  };

  struct MeshTriangle {
    const fm_vec3_t *vertices;
    fm_vec3_t normal;
    MeshTriangleIndices tri;
    const MeshCollider *mesh{nullptr}; ///< Parent mesh for world-space transforms

    void gjkSupport(const fm_vec3_t &direction, fm_vec3_t &output) const;
    float comparePoint(const fm_vec3_t &point) const;

    /// Get triangle vertex in world space (applies mesh transform if present)
    fm_vec3_t worldVertex(int localIndex) const;
    /// Get triangle normal in world space
    fm_vec3_t worldNormal() const;
  };

  /// GJK-compatible wrapper for MeshTriangle
  void meshTriangleGjkSupport(const void *data, const fm_vec3_t &direction, fm_vec3_t &output);

  struct MeshCollider {
    AABBTree aabbTree;           ///< Local-space AABB tree for triangle broadphase
    fm_vec3_t *vertices{nullptr};
    MeshTriangleIndices *triangles{nullptr};
    fm_vec3_t *normals{nullptr};
    uint16_t triangleCount{0};
    uint16_t vertexCount{0};
    uint8_t maskRead{0x00};  // which collision layers this collider get's affected by
    uint8_t maskWrite{0x00}; // which collision layers this collider affects

    // Owner tracking
    P64::Object* owner{};          ///< Pointer to the owning Object (for event lookups)

    // Material
    float friction{1.0f};
    float bounce{0.0f};

    // Scene integration
    NodeProxy aabbTreeNodeId{NULL_NODE}; ///< Node ID in the scene's dynamic AABB tree
    AABB localRootAABB{};                ///< Root AABB of the local triangle tree
    AABB worldBoundingBox{};             ///< Cached world-space AABB
    fm_vec3_t lastOwnerPos{};
    fm_quat_t lastOwnerRot{QUAT_IDENTITY};
    fm_vec3_t lastOwnerScale{1.0f, 1.0f, 1.0f};
    bool hasCachedOwnerTransform{false};
    bool transformChanged{false}; ///< Set to true when owner transform changes, used to trigger AABB tree updates

     // Cached transform data for fast vertex/normal transforms without needing to access the owner Object

    Matrix3x3 inverseRotation{}; ///< Cached inverse of owner's rotation for fast local-space queries

    /// Transform a local-space point to world space
    fm_vec3_t toWorldSpace(const fm_vec3_t &localPoint) const;
    /// Transform a world-space point to local space
    fm_vec3_t toLocalSpace(const fm_vec3_t &worldPoint) const;
    /// Rotate a local-space direction/normal to world space (no translation)
    fm_vec3_t rotateToWorld(const fm_vec3_t &localDir) const;
    /// Rotate a world-space direction/normal to local space (no translation)
    fm_vec3_t rotateToLocal(const fm_vec3_t &worldDir) const;

    /// Recompute worldBoundingBox from localRootAABB + current transform
    void recalculateWorldAABB();
    /// Returns true if the owner's transform differs from the cached transform snapshot
    bool ownerTransformChanged() const;
    /// Updates the cached owner transform snapshot to the current owner transform
    void syncOwnerTransform();
    /// Compute localRootAABB from the internal AABB tree root node
    void computeLocalRootAABB();
    /// Transform a world-space AABB into a conservative local-space AABB for tree queries
    AABB worldAABBToLocal(const AABB &worldAABB) const;

    /// Returns true if the mesh has a non-identity transform
    bool hasTransform() const;

    /// Returns true if the mesh has a non-identity rotation
    bool hasRotation() const;
    /// Returns true if the mesh has a non-zero position
    bool hasPosition() const;
    /// Returns true if the mesh has a non-uniform (1,1,1) scale
    bool hasScale() const;

    bool readsCollider(const Collider *other) const;
    bool readsMeshCollider(const MeshCollider *other) const;

    /// Create a MeshCollider directly from collision asset raw data, binding to the given Object's transform.
    /// The returned collider owns newly allocated arrays (vertices, triangles, normals).
    /// Call destroyData() to free them.
    static MeshCollider *createFromRawData(void *rawData, Object *obj);

    /// Free owned vertex/triangle/normal arrays and destroy the AABB tree.
    void destroyData();
  };

} // namespace P64::Coll
