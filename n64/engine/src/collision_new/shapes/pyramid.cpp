/**
 * @copyright 2024 - Max Bebök
 * @license MIT
 */
#include "collision_new/shapes/pyramid.h"

#include <cmath>

using namespace P64::CollNew;

void P64::CollNew::pyramidSupportFunction(const void *data, const fm_vec3_t &direction, fm_vec3_t &output) {
  auto *collider = static_cast<const ColliderData *>(data);
  float hwx = collider->shapeData.pyramid.baseHalfWidthX;
  float hwz = collider->shapeData.pyramid.baseHalfWidthZ;
  float hh = collider->shapeData.pyramid.halfHeight;

  float apexDot = hh * direction.y;
  float baseDot = fabsf(direction.x) * hwx + fabsf(direction.z) * hwz - hh * direction.y;

  if(apexDot > baseDot) {
    output = vec3(0.0f, hh, 0.0f);
  } else {
    output = vec3(
      copysignf(hwx, direction.x),
      -hh,
      copysignf(hwz, direction.z)
    );
  }
}

void P64::CollNew::pyramidBoundingBox(const void *data, const fm_quat_t *q, AABB &box) {
  auto *collider = static_cast<const ColliderData *>(data);
  float hh = collider->shapeData.pyramid.halfHeight;
  float hwx = collider->shapeData.pyramid.baseHalfWidthX;
  float hwz = collider->shapeData.pyramid.baseHalfWidthZ;

  float ex, ey, ez;

  if(!q) {
    ex = hwx; ey = hh; ez = hwz;
  } else {
    float x = q->x, y = q->y, z = q->z, w = q->w;
    float xx = x*x, yy = y*y, zz = z*z;
    float xy = x*y, xz = x*z, yz = y*z;
    float wx = w*x, wy = w*y, wz = w*z;

    float r00 = 1 - 2*(yy+zz), r01 = 2*(xy-wz), r02 = 2*(xz+wy);
    float r10 = 2*(xy+wz), r11 = 1 - 2*(xx+zz), r12 = 2*(yz-wx);
    float r20 = 2*(xz-wy), r21 = 2*(yz+wx), r22 = 1 - 2*(xx+yy);

    ex = hwx*fabsf(r00) + hh*fabsf(r01) + hwz*fabsf(r02);
    ey = hwx*fabsf(r10) + hh*fabsf(r11) + hwz*fabsf(r12);
    ez = hwx*fabsf(r20) + hh*fabsf(r21) + hwz*fabsf(r22);
  }

  box.min = vec3(-ex, -ey, -ez);
  box.max = vec3(ex, ey, ez);
}

void P64::CollNew::pyramidInertiaTensor(const void *data, float mass, fm_vec3_t &out) {
  auto *collider = static_cast<const ColliderData *>(data);
  float hh = collider->shapeData.pyramid.halfHeight;
  float hwx = collider->shapeData.pyramid.baseHalfWidthX;
  float hwz = collider->shapeData.pyramid.baseHalfWidthZ;

  float mDiv20 = mass * 0.05f;
  float mDiv5 = mass * 0.2f;

  float Ixx = (mDiv5 * hwz * hwz) + (mDiv20 * 3.0f * hh * hh);
  float Iyy = (mDiv5 * hwx * hwx) + (mDiv5 * hwz * hwz);
  float Izz = (mDiv5 * hwx * hwx) + (mDiv20 * 3.0f * hh * hh);

  out.x = Ixx;
  out.y = Iyy;
  out.z = Izz;
}
