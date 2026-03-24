/**
 * @copyright 2024 - Max Bebök
 * @license MIT
 */
#include "collision/mesh_collider.h"
#include "scene/object.h"

namespace P64::Coll {

  namespace {
    struct RawCollisionHeader {
      uint32_t triCount;
      uint32_t vertCount;
      float collScale;
      uint32_t vertexPtr;
      uint32_t normalsPtr;
      uint32_t bvhPtr;
    };

    struct PackedNormal {
      int16_t v[3];
    };

    char *alignPtr(char *ptr, size_t alignment) {
      return reinterpret_cast<char *>((reinterpret_cast<uintptr_t>(ptr) + alignment - 1) & ~(alignment - 1));
    }
  }

  // ── MeshTriangle ──────────────────────────────────────────────────

  fm_vec3_t MeshTriangle::worldVertex(int localIndex) const {
    const fm_vec3_t &v = vertices[tri.indices[localIndex]];
    if(mesh) return mesh->toWorldSpace(v);
    return v;
  }

  fm_vec3_t MeshTriangle::worldNormal() const {
    if(mesh) return mesh->rotateToWorld(normal);
    return normal;
  }

  void MeshTriangle::gjkSupport(const fm_vec3_t &direction, fm_vec3_t &output) const {
    fm_vec3_t w0 = worldVertex(0);
    fm_vec3_t w1 = worldVertex(1);
    fm_vec3_t w2 = worldVertex(2);

    float d0 = vec3Dot(w0, direction);
    float d1 = vec3Dot(w1, direction);
    float d2 = vec3Dot(w2, direction);

    if(d0 >= d1 && d0 >= d2) {
      output = w0;
    } else if(d1 >= d2) {
      output = w1;
    } else {
      output = w2;
    }
  }

  float MeshTriangle::comparePoint(const fm_vec3_t &point) const {
    fm_vec3_t w0 = worldVertex(0);
    fm_vec3_t wn = worldNormal();
    fm_vec3_t diff = vec3Sub(point, w0);
    return vec3Dot(wn, diff);
  }

  void meshTriangleGjkSupport(const void *data, const fm_vec3_t &direction, fm_vec3_t &output) {
    auto *tri = static_cast<const MeshTriangle *>(data);
    tri->gjkSupport(direction, output);
  }

  // ── MeshCollider transform ────────────────────────────────────────

  fm_vec3_t MeshCollider::toWorldSpace(const fm_vec3_t &localPoint) const {
    fm_vec3_t position = owner ? owner->pos : vec3Zero();
    fm_quat_t rotation = owner ? owner->rot : QUAT_IDENTITY;
    fm_vec3_t scale = owner ? owner->scale : vec3(1.0f, 1.0f, 1.0f);
    fm_vec3_t scaled = vec3(localPoint.x * scale.x, localPoint.y * scale.y, localPoint.z * scale.z);
    if(!quatIsIdentical(&rotation, &QUAT_IDENTITY)) {
      scaled = quatRotateVec(rotation, scaled);
    }
    if(vec3MagSqrd(position) > EPSILON_SQUARED) {
      scaled = vec3Add(scaled, position);
    }
    return scaled;
  }

  fm_vec3_t MeshCollider::toLocalSpace(const fm_vec3_t &worldPoint) const {
    fm_vec3_t p = worldPoint;
    fm_quat_t rotation = owner ? owner->rot : QUAT_IDENTITY;
    fm_vec3_t scale = owner ? owner->scale : vec3(1.0f, 1.0f, 1.0f);
    if(hasPosition()) {
      p = p - owner->pos;
    }
    if(hasRotation()) {
      p = quatConjugate(rotation) * p;
    }
    if(hasScale()) {
      if(fabsf(scale.x) > EPSILON) p.x /= scale.x;
      if(fabsf(scale.y) > EPSILON) p.y /= scale.y;
      if(fabsf(scale.z) > EPSILON) p.z /= scale.z;
    }
    return p;
  }

  fm_vec3_t MeshCollider::rotateToWorld(const fm_vec3_t &localDir) const {
    if(!hasRotation()) return localDir;
    return owner->rot * localDir;
  }

  fm_vec3_t MeshCollider::rotateToLocal(const fm_vec3_t &worldDir) const {
    if(!hasRotation()) return worldDir;
    return quatConjugate(owner->rot) * worldDir;
  }

  bool MeshCollider::hasTransform() const {
    return (hasRotation() || hasPosition() || hasScale());
  }

  bool MeshCollider::hasRotation() const {
    if(!owner) return false;
    return !quatIsIdentical(&owner->rot, &QUAT_IDENTITY);
  }

  bool MeshCollider::hasPosition() const {
    if(!owner) return false;
    return vec3MagSqrd(owner->pos) > EPSILON_SQUARED;
  }

  bool MeshCollider::hasScale() const {
    if(!owner) return false;
    return (fabsf(owner->scale.x - 1.0f) > EPSILON) || (fabsf(owner->scale.y - 1.0f) > EPSILON) || (fabsf(owner->scale.z - 1.0f) > EPSILON);
  }

  bool MeshCollider::ownerTransformChanged() const {
    if(!owner) return false;
    if(!hasCachedOwnerTransform) return true;

    if(vec3DistSqrd(owner->pos, lastOwnerPos) > EPSILON_SQUARED) return true;
    if(vec3DistSqrd(owner->scale, lastOwnerScale) > EPSILON_SQUARED) return true;

    const float rotSim = fabsf(quatDot(owner->rot, lastOwnerRot));
    return rotSim < (1.0f - EPSILON);
  }

  void MeshCollider::syncOwnerTransform() {
    if(!owner) {
      lastOwnerPos = vec3Zero();
      lastOwnerRot = QUAT_IDENTITY;
      lastOwnerScale = vec3(1.0f, 1.0f, 1.0f);
    } else {
      lastOwnerPos = owner->pos;
      lastOwnerRot = owner->rot;
      lastOwnerScale = owner->scale;
    }
    hasCachedOwnerTransform = true;
  }

  void MeshCollider::computeLocalRootAABB() {
    if(aabbTree.root != NULL_NODE) {
      const AABB *rootBounds = aabbTree.getNodeBounds(aabbTree.root);
      if(rootBounds) {
        localRootAABB = *rootBounds;
        return;
      }
    }
    // Fallback: compute from vertices
    if(vertexCount == 0) return;
    fm_vec3_t minV = vertices[0];
    fm_vec3_t maxV = vertices[0];
    for(int i = 1; i < vertexCount; ++i) {
      minV = vec3Min(minV, vertices[i]);
      maxV = vec3Max(maxV, vertices[i]);
    }
    localRootAABB = {minV, maxV};
  }

  void MeshCollider::recalculateWorldAABB() {
    // Transform all 8 corners of the local AABB to world space and take the enclosing AABB
    fm_vec3_t corners[8] = {
      vec3(localRootAABB.min.x, localRootAABB.min.y, localRootAABB.min.z),
      vec3(localRootAABB.max.x, localRootAABB.min.y, localRootAABB.min.z),
      vec3(localRootAABB.min.x, localRootAABB.max.y, localRootAABB.min.z),
      vec3(localRootAABB.max.x, localRootAABB.max.y, localRootAABB.min.z),
      vec3(localRootAABB.min.x, localRootAABB.min.y, localRootAABB.max.z),
      vec3(localRootAABB.max.x, localRootAABB.min.y, localRootAABB.max.z),
      vec3(localRootAABB.min.x, localRootAABB.max.y, localRootAABB.max.z),
      vec3(localRootAABB.max.x, localRootAABB.max.y, localRootAABB.max.z),
    };

    fm_vec3_t worldMin = toWorldSpace(corners[0]);
    fm_vec3_t worldMax = worldMin;
    for(int i = 1; i < 8; ++i) {
      fm_vec3_t w = toWorldSpace(corners[i]);
      worldMin = vec3Min(worldMin, w);
      worldMax = vec3Max(worldMax, w);
    }
    worldBoundingBox = {worldMin, worldMax};
  }

  AABB MeshCollider::worldAABBToLocal(const AABB &worldAABB) const {
    // Transform all 8 corners of the world AABB into local space
    fm_vec3_t corners[8] = {
      vec3(worldAABB.min.x, worldAABB.min.y, worldAABB.min.z),
      vec3(worldAABB.max.x, worldAABB.min.y, worldAABB.min.z),
      vec3(worldAABB.min.x, worldAABB.max.y, worldAABB.min.z),
      vec3(worldAABB.max.x, worldAABB.max.y, worldAABB.min.z),
      vec3(worldAABB.min.x, worldAABB.min.y, worldAABB.max.z),
      vec3(worldAABB.max.x, worldAABB.min.y, worldAABB.max.z),
      vec3(worldAABB.min.x, worldAABB.max.y, worldAABB.max.z),
      vec3(worldAABB.max.x, worldAABB.max.y, worldAABB.max.z),
    };

    fm_vec3_t localMin = toLocalSpace(corners[0]);
    fm_vec3_t localMax = localMin;
    for(int i = 1; i < 8; ++i) {
      fm_vec3_t l = toLocalSpace(corners[i]);
      localMin = vec3Min(localMin, l);
      localMax = vec3Max(localMax, l);
    }
    return {localMin, localMax};
  }

  // ── Load Mesh Collider from Raw Data and build AABB Tree ────────────────────────────────────────

  MeshCollider *MeshCollider::createFromRawData(void *rawData, Object *obj) {
    if(!rawData) return nullptr;
    if(!obj) return nullptr;

    auto *header = static_cast<RawCollisionHeader *>(rawData);
    if(header->triCount == 0 || header->vertCount == 0) return nullptr;
    if(header->triCount > 0xFFFFu || header->vertCount > 0xFFFFu) return nullptr;

    char *data = reinterpret_cast<char *>(header + 1);

    auto *indexData = reinterpret_cast<uint16_t *>(data);
    data += header->triCount * sizeof(uint16_t) * 3;

    data = alignPtr(data, 4);
    auto *normalData = reinterpret_cast<PackedNormal *>(data);

    data += header->triCount * sizeof(PackedNormal);
    data = alignPtr(data, 4);
    auto *vertexData = reinterpret_cast<fm_vec3_t *>(data);

    auto *collider = new MeshCollider();

    collider->triangleCount = static_cast<uint16_t>(header->triCount);
    collider->vertexCount = static_cast<uint16_t>(header->vertCount);

    // Copy vertex data
    collider->vertices = new fm_vec3_t[header->vertCount];
    for(uint32_t i = 0; i < header->vertCount; ++i) {
      collider->vertices[i] = vertexData[i];
    }

    // Copy triangle indices
    collider->triangles = new MeshTriangleIndices[header->triCount];
    for(uint32_t t = 0; t < header->triCount; ++t) {
      collider->triangles[t].indices[0] = indexData[t * 3 + 0];
      collider->triangles[t].indices[1] = indexData[t * 3 + 1];
      collider->triangles[t].indices[2] = indexData[t * 3 + 2];
    }

    // Convert packed normals (int16_t scaled by 32767) to fm_vec3_t
    constexpr float NORM_SCALE = 1.0f / 32767.0f;
    collider->normals = new fm_vec3_t[header->triCount];
    for(uint32_t t = 0; t < header->triCount; ++t) {
      collider->normals[t] = vec3(
        static_cast<float>(normalData[t].v[0]) * NORM_SCALE,
        static_cast<float>(normalData[t].v[1]) * NORM_SCALE,
        static_cast<float>(normalData[t].v[2]) * NORM_SCALE
      );
    }

    // Bind to owner object
    collider->owner = obj;

    // Build AABB tree from triangle bounding boxes
    // Need 2*N-1 internal nodes for N leaves, plus some margin
    int treeCapacity = static_cast<int>(header->triCount) * 2 + 1;
    collider->aabbTree.init(treeCapacity);

    for(uint32_t t = 0; t < header->triCount; ++t) {
      const fm_vec3_t &v0 = collider->vertices[collider->triangles[t].indices[0]];
      const fm_vec3_t &v1 = collider->vertices[collider->triangles[t].indices[1]];
      const fm_vec3_t &v2 = collider->vertices[collider->triangles[t].indices[2]];

      AABB triAABB;
      triAABB.min = vec3Min(vec3Min(v0, v1), v2);
      triAABB.max = vec3Max(vec3Max(v0, v1), v2);

      // Store triangle index + 1 as data pointer (index 0 would be nullptr and get skipped)
      collider->aabbTree.createNode(triAABB, reinterpret_cast<void *>(static_cast<intptr_t>(t + 1)));
    }

    collider->computeLocalRootAABB();
    collider->recalculateWorldAABB();
    collider->syncOwnerTransform();

    return collider;
  }

  void MeshCollider::destroyData() {
    aabbTree.destroy();
    delete[] vertices;
    delete[] triangles;
    delete[] normals;
    vertices = nullptr;
    triangles = nullptr;
    normals = nullptr;
    triangleCount = 0;
    vertexCount = 0;
  }

} // namespace P64::Coll
