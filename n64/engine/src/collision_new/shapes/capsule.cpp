/**
 * @copyright 2024 - Max Bebök
 * @license MIT
 */
#include "collision_new/shapes/capsule.h"

#include <cmath>

using namespace P64::CollNew;

void P64::CollNew::capsuleSupportFunction(const void *data, const fm_vec3_t &direction, fm_vec3_t &output) {
  auto *collider = static_cast<const ColliderData *>(data);
  float halfH = collider->shapeData.capsule.innerHalfHeight;
  float radius = collider->shapeData.capsule.radius;

  float y = copysignf(halfH, direction.y);

  output.x = direction.x * radius;
  output.y = direction.y * radius + y;
  output.z = direction.z * radius;
}

void P64::CollNew::capsuleBoundingBox(const void *data, const fm_quat_t *q, AABB &box) {
  auto *collider = static_cast<const ColliderData *>(data);
  float halfH = collider->shapeData.capsule.innerHalfHeight;
  float radius = collider->shapeData.capsule.radius;

  fm_vec3_t r{};

  if(!q) {
    r = vec3(0.0f, halfH, 0.0f);
  } else {
    float x = q->x, y = q->y, z = q->z, w = q->w;
    float xx = x*x, yy = y*y, zz = z*z;
    float xy = x*y, xz = x*z, yz = y*z;
    float wx = w*x, wy = w*y, wz = w*z;

    float r01 = 2*(xy-wz);
    float r11 = 1 - 2*(xx+zz);
    float r21 = 2*(yz+wx);

    r.x = r01 * halfH;
    r.y = r11 * halfH;
    r.z = r21 * halfH;
  }

  float absX = fabsf(r.x);
  float absY = fabsf(r.y);
  float absZ = fabsf(r.z);

  box.min = vec3(-absX - radius, -absY - radius, -absZ - radius);
  box.max = vec3(absX + radius, absY + radius, absZ + radius);
}

void P64::CollNew::capsuleInertiaTensor(const void *data, float mass, fm_vec3_t &out) {
  auto *collider = static_cast<const ColliderData *>(data);
  float radius = collider->shapeData.capsule.radius;
  float innerHalfH = collider->shapeData.capsule.innerHalfHeight;
  float cylHeight = 2.0f * innerHalfH;

  constexpr float PI = 3.14159265358979f;

  float cylVol = PI * radius * radius * cylHeight;
  float sphereVol = (4.0f / 3.0f) * PI * radius * radius * radius;
  float totalVol = cylVol + sphereVol;

  float cylMass = mass * (cylVol / totalVol);
  float sphereMass = mass * (sphereVol / totalVol);

  float rSq = radius * radius;
  float hSq = cylHeight * cylHeight;
  float cylPerp = cylMass * (3.0f * rSq + hSq) / 12.0f;
  float cylAxial = 0.5f * cylMass * rSq;

  float sphereInertia = 0.4f * sphereMass * rSq;
  float offsetSq = innerHalfH * innerHalfH;
  float hemiMass = sphereMass * 0.5f;
  float spherePerp = sphereInertia + 2.0f * hemiMass * offsetSq;

  out.x = cylPerp + spherePerp;
  out.y = cylAxial + sphereInertia;
  out.z = cylPerp + spherePerp;
}
