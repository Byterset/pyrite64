/**
* @copyright 2024 - Max Bebök
* @license MIT
*/
#include "scene/camera.h"
#include "lib/logger.h"
#include "renderer/renderScale.h"
#include "scene/globalState.h"

namespace
{
  constexpr fm_vec3_t FORWARD{0,0,-1};
  constexpr fm_vec3_t UP{0,1,0};
}

P64::Camera::Camera() {
  viewports = t3d_viewport_create_buffered(3);
}

P64::Camera::~Camera() {
  t3d_viewport_destroy(&viewports);
}

void P64::Camera::update([[maybe_unused]] float deltaTime)
{
  // camera values are in meters, the viewport/view-matrix operate in render units
  float renderScale = Renderer::getRenderScale();
  if(projection == Projection::ORTHOGRAPHIC) {
    float halfHeight = orthoSize * renderScale;
    float halfWidth = halfHeight * aspectRatio;
    t3d_viewport_set_ortho(&viewports, -halfWidth, halfWidth, -halfHeight, halfHeight, near * renderScale, far * renderScale);
  } else {
    t3d_viewport_set_perspective(&viewports, fov, aspectRatio, near * renderScale, far * renderScale);
  }
  t3d_viewport_set_view_matrix(&viewports, &viewMatrix);
}

void P64::Camera::attach() {
  t3d_viewport_attach(viewports);
}

void P64::Camera::setScreenArea(int x, int y, int width, int height) {
  t3d_viewport_set_area(viewports, x,y, width, height);
}

void P64::Camera::setLookAt(const fm_vec3_t &newPos, const fm_vec3_t &newTarget, const fm_vec3_t &newUp) {
  target = newTarget;
  up = newUp;
  pos = newPos;
  // build the view matrix in render units (only affects the translation, the rotation basis is scale-free)
  fm_vec3_t eyeScaled = newPos * Renderer::getRenderScale();
  fm_vec3_t targetScaled = newTarget * Renderer::getRenderScale();
  t3d_mat4_look_at(&viewMatrix, &eyeScaled, &targetScaled, &newUp);
  needsProjUpdate = true;
}

void P64::Camera::setPosRot(const fm_vec3_t &newPos, const fm_quat_t&rot) {
  setLookAt(newPos, newPos + (rot * FORWARD), rot * UP);
}

fm_vec3_t P64::Camera::getScreenPos(const fm_vec3_t &worldPos)
{
  fm_vec3_t res{};
  fm_vec3_t worldPosScaled = worldPos * Renderer::getRenderScale();
  t3d_viewport_calc_viewspace_pos(viewports, res, worldPosScaled);
  return res;
}

