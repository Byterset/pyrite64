/**
 * @copyright 2024 - Max Bebök
 * @license MIT
 */
#include "collision_new/shapes/box.h"

#include <cmath>

using namespace P64::CollNew;

void P64::CollNew::boxSupportFunction(const void *data, const fm_vec3_t &direction, fm_vec3_t &output) {
  auto *collider = static_cast<const ColliderData *>(data);
  auto &hs = collider->shapeData.box.halfSize;

  output.x = copysignf(hs.x, direction.x);
  output.y = copysignf(hs.y, direction.y);
  output.z = copysignf(hs.z, direction.z);
}

void P64::CollNew::boxBoundingBox(const void *data, const fm_quat_t *q, AABB &box) {
  auto *collider = static_cast<const ColliderData *>(data);
  auto &h = collider->shapeData.box.halfSize;

  float ex, ey, ez;

  if(!q) {
    ex = h.x; ey = h.y; ez = h.z;
  } else {
    float x = q->x, y = q->y, z = q->z, w = q->w;
    float xx = x*x, yy = y*y, zz = z*z;
    float xy = x*y, xz = x*z, yz = y*z;
    float wx = w*x, wy = w*y, wz = w*z;

    float r00 = 1 - 2*(yy+zz), r01 = 2*(xy-wz), r02 = 2*(xz+wy);
    float r10 = 2*(xy+wz), r11 = 1 - 2*(xx+zz), r12 = 2*(yz-wx);
    float r20 = 2*(xz-wy), r21 = 2*(yz+wx), r22 = 1 - 2*(xx+yy);

    ex = h.x*fabsf(r00) + h.y*fabsf(r01) + h.z*fabsf(r02);
    ey = h.x*fabsf(r10) + h.y*fabsf(r11) + h.z*fabsf(r12);
    ez = h.x*fabsf(r20) + h.y*fabsf(r21) + h.z*fabsf(r22);
  }

  box.min = vec3(-ex, -ey, -ez);
  box.max = vec3(ex, ey, ez);
}

void P64::CollNew::boxInertiaTensor(const void *data, float mass, fm_vec3_t &out) {
  auto *collider = static_cast<const ColliderData *>(data);
  auto &hs = collider->shapeData.box.halfSize;

  float hxSq = hs.x * hs.x;
  float hySq = hs.y * hs.y;
  float hzSq = hs.z * hs.z;
  float scale = mass / 3.0f;

  out.x = scale * (hySq + hzSq);
  out.y = scale * (hxSq + hzSq);
  out.z = scale * (hxSq + hySq);
}
