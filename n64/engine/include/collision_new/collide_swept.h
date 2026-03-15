/**
 * @copyright 2024 - Max Bebök
 * @license MIT
 */
#pragma once

#include "physics_object.h"
#include "mesh_collider.h"
#include "epa.h"

namespace P64::CollNew {

  bool collideObjectToMeshSwept(PhysicsObject *object, MeshCollider *mesh, fm_vec3_t *prevPos);

} // namespace P64::CollNew
