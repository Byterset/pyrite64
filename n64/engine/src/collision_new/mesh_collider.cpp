/**
 * @copyright 2024 - Max Bebök
 * @license MIT
 */
#include "collision_new/mesh_collider.h"

namespace P64::CollNew {

  void MeshTriangle::gjkSupport(const fm_vec3_t &direction, fm_vec3_t &output) const {
    const fm_vec3_t &v0 = vertices[tri.indices[0]];
    const fm_vec3_t &v1 = vertices[tri.indices[1]];
    const fm_vec3_t &v2 = vertices[tri.indices[2]];

    float d0 = vec3Dot(v0, direction);
    float d1 = vec3Dot(v1, direction);
    float d2 = vec3Dot(v2, direction);

    if(d0 >= d1 && d0 >= d2) {
      output = v0;
    } else if(d1 >= d2) {
      output = v1;
    } else {
      output = v2;
    }
  }

  float MeshTriangle::comparePoint(const fm_vec3_t &point) const {
    const fm_vec3_t &v0 = vertices[tri.indices[0]];
    fm_vec3_t diff = vec3Sub(point, v0);
    return vec3Dot(normal, diff);
  }

  void meshTriangleGjkSupport(const void *data, const fm_vec3_t &direction, fm_vec3_t &output) {
    auto *tri = static_cast<const MeshTriangle *>(data);
    tri->gjkSupport(direction, output);
  }

} // namespace P64::CollNew
