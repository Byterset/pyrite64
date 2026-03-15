/**
 * @copyright 2024 - Max Bebök
 * @license MIT
 */
#include "collision_new/shapes/cone.h"

#include <cmath>

using namespace P64::CollNew;

void P64::CollNew::coneSupportFunction(const void *data, const fm_vec3_t &direction, fm_vec3_t &output) {
  auto *collider = static_cast<const ColliderData *>(data);
  float r = collider->shapeData.cone.radius;
  float h = collider->shapeData.cone.halfHeight;

  float dx = direction.x;
  float dy = direction.y;
  float dz = direction.z;

  float sinAlpha = r / sqrtf(r * r + 4.0f * h * h);
  float sin2 = sinAlpha * sinAlpha;
  float sigma2 = dx * dx + dz * dz;
  float dy2 = dy * dy;
  float d2 = dy2 + sigma2;

  // Case 1: apex (if y-component dominates)
  if(dy > 0.0f && dy2 > d2 * sin2) {
    output = vec3(0.0f, h, 0.0f);
    return;
  }

  // Case 2: base circle (if there's a radial component)
  if(sigma2 > 0.0f) {
    float invSigma = 1.0f / sqrtf(sigma2);
    output = vec3(r * dx * invSigma, -h, r * dz * invSigma);
    return;
  }

  // Case 3: purely vertical downward
  output = vec3(0.0f, -h, 0.0f);
}

void P64::CollNew::coneBoundingBox(const void *data, const fm_quat_t *q, AABB &box) {
  auto *collider = static_cast<const ColliderData *>(data);
  float h = collider->shapeData.cone.halfHeight;
  float r = collider->shapeData.cone.radius;

  if(!q) {
    box.min = vec3(-r, -h, -r);
    box.max = vec3(r, h, r);
    return;
  }

  float x = q->x, y = q->y, z = q->z, w = q->w;
  float xx = x*x, yy = y*y, zz = z*z;
  float xy = x*y, xz = x*z, yz = y*z;
  float wx = w*x, wy = w*y, wz = w*z;

  float r00 = 1 - 2*(yy+zz), r01 = 2*(xy-wz), r02 = 2*(xz+wy);
  float r10 = 2*(xy+wz), r11 = 1 - 2*(xx+zz), r12 = 2*(yz-wx);
  float r20 = 2*(xz-wy), r21 = 2*(yz+wx), r22 = 1 - 2*(xx+yy);

  // Rotate key points: apex and 4 base circle extremes
  auto rotate = [&](float px, float py, float pz) -> fm_vec3_t {
    return vec3(
      r00*px + r01*py + r02*pz,
      r10*px + r11*py + r12*pz,
      r20*px + r21*py + r22*pz
    );
  };

  auto apex = rotate(0.0f, h, 0.0f);
  fm_vec3_t bmin = apex, bmax = apex;

  fm_vec3_t basePts[4] = {
    rotate(r, -h, 0.0f),
    rotate(-r, -h, 0.0f),
    rotate(0.0f, -h, r),
    rotate(0.0f, -h, -r)
  };

  for(int i = 0; i < 4; ++i) {
    bmin = vec3Min(bmin, basePts[i]);
    bmax = vec3Max(bmax, basePts[i]);
  }

  box.min = bmin;
  box.max = bmax;
}

void P64::CollNew::coneInertiaTensor(const void *data, float mass, fm_vec3_t &out) {
  auto *collider = static_cast<const ColliderData *>(data);
  float r = collider->shapeData.cone.radius;
  float h = 2.0f * collider->shapeData.cone.halfHeight;

  float rSq = r * r;
  float hSq = h * h;

  float perpInertia = (3.0f / 80.0f) * mass * (4.0f * rSq + hSq);
  float axialInertia = 0.3f * mass * rSq;

  out.x = perpInertia;
  out.y = axialInertia;
  out.z = perpInertia;
}
