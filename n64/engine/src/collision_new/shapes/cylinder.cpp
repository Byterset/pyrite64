/**
 * @copyright 2024 - Max Bebök
 * @license MIT
 */
#include "collision_new/shapes/cylinder.h"

#include <cmath>

using namespace P64::CollNew;

namespace {
  constexpr float SQRT_1_2 = 0.707106781f;
}

void P64::CollNew::cylinderSupportFunction(const void *data, const fm_vec3_t &direction, fm_vec3_t &output) {
  auto *collider = static_cast<const ColliderData *>(data);
  float r = collider->shapeData.cylinder.radius;
  float h = collider->shapeData.cylinder.halfHeight;

  float x = direction.x;
  float y = direction.y;
  float z = direction.z;

  output.y = copysignf(h, y);

  float absX = fabsf(x);
  float absZ = fabsf(z);

  if(absX < SQRT_1_2 * (absX + absZ) && absZ < SQRT_1_2 * (absX + absZ)) {
    output.x = (x >= 0.0f) ? r * SQRT_1_2 : -r * SQRT_1_2;
    output.z = (z >= 0.0f) ? r * SQRT_1_2 : -r * SQRT_1_2;
  } else if(absX > absZ) {
    output.x = (x >= 0.0f) ? r : -r;
    output.z = 0.0f;
  } else {
    output.x = 0.0f;
    output.z = (z >= 0.0f) ? r : -r;
  }
}

void P64::CollNew::cylinderBoundingBox(const void *data, const fm_quat_t *q, AABB &box) {
  auto *collider = static_cast<const ColliderData *>(data);
  float h = collider->shapeData.cylinder.halfHeight;
  float r = collider->shapeData.cylinder.radius;

  float ex = r, ey = h, ez = r;

  if(!q) {
    box.min = vec3(-ex, -ey, -ez);
    box.max = vec3(ex, ey, ez);
    return;
  }

  float x = q->x, y = q->y, z = q->z, w = q->w;
  float xx = x*x, yy = y*y, zz = z*z;
  float xy = x*y, xz = x*z, yz = y*z;
  float wx = w*x, wy = w*y, wz = w*z;

  float r00 = 1.0f - 2.0f*(yy+zz);
  float r01 =        2.0f*(xy-wz);
  float r02 =        2.0f*(xz+wy);
  float r10 =        2.0f*(xy+wz);
  float r11 = 1.0f - 2.0f*(xx+zz);
  float r12 =        2.0f*(yz-wx);
  float r20 =        2.0f*(xz-wy);
  float r21 =        2.0f*(yz+wx);
  float r22 = 1.0f - 2.0f*(xx+yy);

  float wxe = fabsf(r00)*ex + fabsf(r01)*ey + fabsf(r02)*ez;
  float wye = fabsf(r10)*ex + fabsf(r11)*ey + fabsf(r12)*ez;
  float wze = fabsf(r20)*ex + fabsf(r21)*ey + fabsf(r22)*ez;

  box.min = vec3(-wxe, -wye, -wze);
  box.max = vec3(wxe, wye, wze);
}

void P64::CollNew::cylinderInertiaTensor(const void *data, float mass, fm_vec3_t &out) {
  auto *collider = static_cast<const ColliderData *>(data);
  float r = collider->shapeData.cylinder.radius;
  float h = 2.0f * collider->shapeData.cylinder.halfHeight;

  float rSq = r * r;
  float hSq = h * h;

  float perpInertia = mass * (3.0f * rSq + hSq) / 12.0f;
  float axialInertia = 0.5f * mass * rSq;

  out.x = perpInertia;
  out.y = axialInertia;
  out.z = perpInertia;
}
