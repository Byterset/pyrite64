/**
 * @copyright 2024 - Max Bebök
 * @license MIT
 */
#include "collision_new/shapes/sweep.h"

#include <cmath>

using namespace P64::CollNew;

void P64::CollNew::sweepSupportFunction(const void *data, const fm_vec3_t &direction, fm_vec3_t &output) {
  auto *collider = static_cast<const ColliderData *>(data);
  auto &s = collider->shapeData.sweep;

  float dirX = direction.x;
  float dirZ = direction.z;
  float distance = 0.0f;
  float resX = 0.0f;
  float resZ = 0.0f;

  float armX = copysignf(s.rangeX, direction.x);
  float armZ = s.rangeY;
  float test = dirX * armX + dirZ * armZ;

  if(test > distance) {
    distance = test;
    resX = armX;
    resZ = armZ;
  }

  if(dirZ > distance) {
    resX = 0.0f;
    resZ = 1.0f;
  }

  output.x = resX * s.radius;
  output.y = copysignf(s.halfHeight, direction.y);
  output.z = resZ * s.radius;
}

void P64::CollNew::sweepBoundingBox(const void *data, const fm_quat_t * /*rotation*/, AABB &box) {
  auto *collider = static_cast<const ColliderData *>(data);
  (void)collider;

  box.min = vec3Zero();
  box.max = vec3Zero();
}
