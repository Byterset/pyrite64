/**
 * @copyright 2024 - Max Bebök
 * @license MIT
 */
#include "collision_new/collider_shape.h"

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

void P64::Coll::colliderGjkSupport(const void *data, const fm_vec3_t &direction, fm_vec3_t &output) {
  auto *collider = static_cast<const Collider *>(data);
  output = collider->support(direction);
}
