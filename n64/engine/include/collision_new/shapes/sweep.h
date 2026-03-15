/**
 * @copyright 2024 - Max Bebök
 * @license MIT
 */
#pragma once

#include "../collider_shape.h"

namespace P64::CollNew {

  void sweepSupportFunction(const void *data, const fm_vec3_t &direction, fm_vec3_t &output);
  void sweepBoundingBox(const void *data, const fm_quat_t *rotation, AABB &box);

} // namespace P64::CollNew
