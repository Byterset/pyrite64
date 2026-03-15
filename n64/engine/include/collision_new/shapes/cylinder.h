/**
 * @copyright 2024 - Max Bebök
 * @license MIT
 */
#pragma once

#include "../collider_shape.h"

namespace P64::CollNew {

  void cylinderSupportFunction(const void *data, const fm_vec3_t &direction, fm_vec3_t &output);
  void cylinderBoundingBox(const void *data, const fm_quat_t *rotation, AABB &box);
  void cylinderInertiaTensor(const void *data, float mass, fm_vec3_t &out);

} // namespace P64::CollNew
