/**
 * @copyright 2024 - Max Bebök
 * @license MIT
 */
#include "collision_new/shapes/sphere.h"

using namespace P64::CollNew;

void P64::CollNew::sphereSupportFunction(const void *data, const fm_vec3_t &direction, fm_vec3_t &output) {
  auto *collider = static_cast<const ColliderData *>(data);
  float radius = collider->shapeData.sphere.radius;

  output.x = direction.x * radius;
  output.y = direction.y * radius;
  output.z = direction.z * radius;
}

void P64::CollNew::sphereBoundingBox(const void *data, const fm_quat_t * /*rotation*/, AABB &box) {
  auto *collider = static_cast<const ColliderData *>(data);
  float r = collider->shapeData.sphere.radius;

  box.min = vec3(-r, -r, -r);
  box.max = vec3(r, r, r);
}

void P64::CollNew::sphereInertiaTensor(const void *data, float mass, fm_vec3_t &out) {
  auto *collider = static_cast<const ColliderData *>(data);
  float r = collider->shapeData.sphere.radius;
  float inertia = 0.4f * mass * r * r;

  out.x = inertia;
  out.y = inertia;
  out.z = inertia;
}
