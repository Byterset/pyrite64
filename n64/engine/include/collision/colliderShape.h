/**
 * @file colliderShape.h
 * @author Kevin Reier <https://github.com/Byterset>
 * @brief Defines the Basic (non-mesh) Colliders 
 */
#pragma once

#include "gjk.h"
#include "types.h"
#include "shapes.h"
#include "matrix3x3.h"
#include "aabbTree.h"

namespace P64
{
  class Object;
}

namespace P64::Coll {

  class CollisionScene;
  struct MeshCollider;
  struct RigidBody;

  struct Collider {
    // Dimensions of colliders themselves are always in world scale (the object scale is already baked in) and
    // read-only. To change the size of a collider use one of the setters. Together with
    // setParentOffset() they keep the world AABB (and with it the broadphase) plus the mass
    // properties of an attached rigid body in sync, and wake up whatever the change may touch.
    // Note that a collider created by the 'CollBody' component gets its
    // size rebuilt from the component whenever the object scale changes, see
    // 'Comp::CollBody::setHalfExtend()' to resize those permanently.

    /// Changes the shape type, this resets all dimensions back to zero.
    void setShapeType(ShapeType newType) {
      type_ = newType;
      switch(type_) {
        case ShapeType::Sphere:   sphere_ = {}; break;
        case ShapeType::Box:      box_ = {}; break;
        case ShapeType::Capsule:  capsule_ = {}; break;
        case ShapeType::Cylinder: cylinder_ = {}; break;
        case ShapeType::Cone:     cone_ = {}; break;
        case ShapeType::Pyramid:  pyramid_ = {}; break;
      }
      markGeometryChanged();
    }
    ShapeType shapeType() const { return type_; }

    const SphereShape &sphereShape() const { return sphere_; }
    const BoxShape &boxShape() const { return box_; }
    const CapsuleShape &capsuleShape() const { return capsule_; }
    const CylinderShape &cylinderShape() const { return cylinder_; }
    const ConeShape &coneShape() const { return cone_; }
    const PyramidShape &pyramidShape() const { return pyramid_; }

    /// Makes this a sphere collider of the given size.
    void setSphereShape(float radius) {
      if(type_ == ShapeType::Sphere && sphere_.radius == radius) return;
      type_ = ShapeType::Sphere;
      sphere_.radius = radius;
      markGeometryChanged();
    }

    /// Makes this a box collider of the given size.
    void setBoxShape(const fm_vec3_t &halfSize) {
      if(type_ == ShapeType::Box && box_.halfSize == halfSize) return;
      type_ = ShapeType::Box;
      box_.halfSize = halfSize;
      markGeometryChanged();
    }

    /// Makes this a capsule collider of the given size ('innerHalfHeight' excludes the round caps).
    void setCapsuleShape(float radius, float innerHalfHeight) {
      if(type_ == ShapeType::Capsule && capsule_.radius == radius && capsule_.innerHalfHeight == innerHalfHeight) return;
      type_ = ShapeType::Capsule;
      capsule_.radius = radius;
      capsule_.innerHalfHeight = innerHalfHeight;
      markGeometryChanged();
    }

    /// Makes this a cylinder collider of the given size.
    void setCylinderShape(float radius, float halfHeight) {
      if(type_ == ShapeType::Cylinder && cylinder_.radius == radius && cylinder_.halfHeight == halfHeight) return;
      type_ = ShapeType::Cylinder;
      cylinder_.radius = radius;
      cylinder_.halfHeight = halfHeight;
      markGeometryChanged();
    }

    /// Makes this a cone collider of the given size.
    void setConeShape(float radius, float halfHeight) {
      if(type_ == ShapeType::Cone && cone_.radius == radius && cone_.halfHeight == halfHeight) return;
      type_ = ShapeType::Cone;
      cone_.radius = radius;
      cone_.halfHeight = halfHeight;
      markGeometryChanged();
    }

    /// Makes this a pyramid collider of the given size.
    void setPyramidShape(float baseHalfWidthX, float baseHalfWidthZ, float halfHeight) {
      if(type_ == ShapeType::Pyramid && pyramid_.baseHalfWidthX == baseHalfWidthX && pyramid_.baseHalfWidthZ == baseHalfWidthZ && pyramid_.halfHeight == halfHeight) return;
      type_ = ShapeType::Pyramid;
      pyramid_.baseHalfWidthX = baseHalfWidthX;
      pyramid_.baseHalfWidthZ = baseHalfWidthZ;
      pyramid_.halfHeight = halfHeight;
      markGeometryChanged();
    }

    /// Resizes the current shape from a half-extend box, keeping the shape type the same.
    /// Axes a shape has no use for are folded in, e.g. a cylinder takes its radius from
    /// max(x, z) and its half height from y. Negative values are mirrored.
    void setHalfExtend(const fm_vec3_t &newHalfExtend);

    /// Size of the current shape as a half-extend box.
    fm_vec3_t halfExtend() const;

    void setOwner(P64::Object *newOwner) {
      owner_ = newOwner;
      hasCachedOwnerTransform_ = false;
    }
    P64::Object *ownerObject() const { return owner_; }

    /// Moves the shape's center relative to the owner's origin. In the owner's local space,
    /// the object scale is applied on top of it.
    void setParentOffset(const fm_vec3_t &newParentOffset) {
      if(parentOffset_ == newParentOffset) return;
      parentOffset_ = newParentOffset;
      markGeometryChanged();
    }
    const fm_vec3_t &parentOffset() const { return parentOffset_; }

    void setBounce(float newBounce) { bounce_ = newBounce; }
    float bounce() const { return bounce_; }
    void setFriction(float newFriction) { friction_ = newFriction; }
    float friction() const { return friction_; }

    void setTrigger(bool newIsTrigger) { isTrigger_ = newIsTrigger; }
    bool isTrigger() const { return isTrigger_; }

    void setCollisionMask(uint8_t newReadMask, uint8_t newWriteMask) {
      readMask_ = newReadMask;
      writeMask_ = newWriteMask;
    }
    uint8_t readMask() const { return readMask_; }
    uint8_t writeMask() const { return writeMask_; }

    const fm_vec3_t &worldCenter() const { return worldCenter_; }
    const AABB &worldAabb() const { return worldAabb_; }
    const Matrix3x3 &rotationMatrix() const { return rotationMatrix_; }
    const Matrix3x3 &inverseRotationMatrix() const { return inverseRotationMatrix_; }
    uint32_t worldStateVersion() const { return worldStateVersion_; }

    fm_vec3_t support(const fm_vec3_t &dir) const;
    AABB boundingBox(const fm_quat_t *rotation) const;
    fm_vec3_t inertiaTensor(float mass) const;
    fm_vec3_t toWorldSpace(const fm_vec3_t &localPoint) const;
    fm_vec3_t toLocalSpace(const fm_vec3_t &worldPoint) const;
    fm_vec3_t rotateToWorld(const fm_vec3_t &localDir) const;
    fm_vec3_t rotateToLocal(const fm_vec3_t &worldDir) const;
    bool hasOwnerTransformChanged() const;
    void syncOwnerTransform();
    bool syncFromRigidBody(const fm_vec3_t& rbPosition, const fm_quat_t& rbRotation);
    bool syncWorldState();
    bool readsCollider(const Collider *other) const;
    bool readsMeshCollider(const MeshCollider *other) const;

  private:
    friend class CollisionScene;

    /// Flags the shape's size or local placement as changed. On the next collision step the
    /// world AABB (and the mass properties of an attached rigid body) are rebuilt and sleeping
    /// bodies the change may touch are woken. All shape setters call this.
    void markGeometryChanged();

    union {
      SphereShape sphere_;
      BoxShape box_;
      CapsuleShape capsule_;
      CylinderShape cylinder_;
      ConeShape cone_;
      PyramidShape pyramid_;
    };

    P64::Object *owner_{nullptr};
    // RigidBody registered for the same owner, maintained by the CollisionScene on add/remove
    RigidBody *rigidBody_{nullptr};
    Matrix3x3 rotationMatrix_{Matrix3x3::identity()};
    Matrix3x3 inverseRotationMatrix_{Matrix3x3::identity()};
    AABB worldAabb_{};
    fm_vec3_t worldCenter_{};
    fm_vec3_t parentOffset_{};
    fm_vec3_t lastOwnerPosition_{};
    fm_quat_t lastOwnerRotation_{QUAT_IDENTITY};
    fm_vec3_t lastOwnerScale_{1.0f, 1.0f, 1.0f};
    float bounce_{0.0f};
    float friction_{0.8f};
    uint32_t worldStateVersion_{0};
    NodeProxy aabbTreeNodeId_{NULL_NODE};
    ShapeType type_{ShapeType::Sphere};
    uint8_t readMask_{0x00};
    uint8_t writeMask_{0x00};
    bool hasCachedOwnerTransform_{false};
    bool isTrigger_{false};
    // set by the shape setters, forces a world AABB rebuild even if the transform didn't change
    bool geometryDirty_{false};
  };

  /// GJK-compatible support wrapper
  void colliderGjkSupport(const void *data, const fm_vec3_t &direction, fm_vec3_t &output);

} // namespace P64::Coll
