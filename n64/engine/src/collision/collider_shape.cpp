/**
 * @file collider_shape.cpp
 * @author Kevin Reier <https://github.com/Byterset>
 * @brief Defines the Basic (non-mesh) Colliders (see collider_shape.h)
 */
#include "collision/collider_shape.h"
#include "collision/mesh_collider.h"
#include "scene/object.h"

using namespace P64::Coll;

fm_vec3_t Collider::support(const fm_vec3_t &dir) const {
  switch(type) {
    case ShapeType::Sphere:   return sphere.support(dir);
    case ShapeType::Box:      return box.support(dir);
    case ShapeType::Capsule:  return capsule.support(dir);
    case ShapeType::Cylinder: return cylinder.support(dir);
    case ShapeType::Cone:     return cone.support(dir);
    case ShapeType::Pyramid:  return pyramid.support(dir);
  }
  __builtin_unreachable();
}

AABB Collider::boundingBox(const fm_quat_t *rotation) const {
  switch(type) {
    case ShapeType::Sphere:   return sphere.boundingBox(rotation);
    case ShapeType::Box:      return box.boundingBox(rotation);
    case ShapeType::Capsule:  return capsule.boundingBox(rotation);
    case ShapeType::Cylinder: return cylinder.boundingBox(rotation);
    case ShapeType::Cone:     return cone.boundingBox(rotation);
    case ShapeType::Pyramid:  return pyramid.boundingBox(rotation);
  }
  __builtin_unreachable();
}

fm_vec3_t Collider::inertiaTensor(float mass) const {
  switch(type) {
    case ShapeType::Sphere:   return sphere.inertiaTensor(mass);
    case ShapeType::Box:      return box.inertiaTensor(mass);
    case ShapeType::Capsule:  return capsule.inertiaTensor(mass);
    case ShapeType::Cylinder: return cylinder.inertiaTensor(mass);
    case ShapeType::Cone:     return cone.inertiaTensor(mass);
    case ShapeType::Pyramid:  return pyramid.inertiaTensor(mass);
  }
  __builtin_unreachable();
}

fm_vec3_t Collider::toWorldSpace(const fm_vec3_t &localPoint) const {
  return worldCenter + rotateToWorld(localPoint);
}

fm_vec3_t Collider::toLocalSpace(const fm_vec3_t &worldPoint) const {
  return rotateToLocal(worldPoint - worldCenter);
}

fm_vec3_t Collider::rotateToWorld(const fm_vec3_t &localDir) const {
  return matrix3Vec3Mul(rotationMatrix, localDir);
}

fm_vec3_t Collider::rotateToLocal(const fm_vec3_t &worldDir) const {
  return matrix3Vec3Mul(inverseRotationMatrix, worldDir);
}

bool Collider::ownerTransformChanged() const {
  if(!owner) return false;
  if(!hasCachedOwnerTransform) return true;

  if(fm_vec3_distance2(&owner->pos, &lastOwnerPos) > FM_EPSILON * FM_EPSILON) return true;
  if(fm_vec3_distance2(&owner->scale, &lastOwnerScale) > FM_EPSILON * FM_EPSILON) return true;

  const float rotSim = fabsf(quatDot(owner->rot, lastOwnerRot));
  return rotSim < (1.0f - FM_EPSILON);
}

void Collider::syncOwnerTransform() {
  if(!owner) {
    lastOwnerPos = VEC3_ZERO;
    lastOwnerRot = QUAT_IDENTITY;
    lastOwnerScale = fm_vec3_t{{1.0f, 1.0f, 1.0f}};
  } else {
    lastOwnerPos = owner->pos;
    lastOwnerRot = owner->rot;
    lastOwnerScale = owner->scale;
  }

  rotationMatrix = quatToMatrix3(lastOwnerRot);
  inverseRotationMatrix = quatToMatrix3(quatConjugate(lastOwnerRot));
  hasCachedOwnerTransform = true;
}

bool Collider::syncWorldState() {
  if(!owner) return false;

  const bool transformChanged = !hasCachedOwnerTransform || ownerTransformChanged();
  if(!transformChanged) return false;

  syncOwnerTransform();
  worldCenter = lastOwnerPos + matrix3Vec3Mul(rotationMatrix, parentOffset * lastOwnerScale);

  const AABB local = boundingBox(&lastOwnerRot);
  worldAABB.min = local.min + worldCenter;
  worldAABB.max = local.max + worldCenter;
  ++worldStateVersion;
  return true;
}

bool Collider::readsCollider(const Collider *other) const {
  return other && ((maskRead & other->maskWrite) != 0);
}

bool Collider::readsMeshCollider(const MeshCollider *other) const {
  return other && ((maskRead & other->maskWrite) != 0);
}

void P64::Coll::colliderGjkSupport(const void *data, const fm_vec3_t &direction, fm_vec3_t &output) {
  auto *collider = static_cast<const Collider *>(data);
  output = collider->support(direction);
}
