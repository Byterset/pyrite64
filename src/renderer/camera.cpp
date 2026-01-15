/**
* @copyright 2025 - Max Bebök
* @license MIT
*/
#include "camera.h"

#include "glm/ext/matrix_clip_space.hpp"
#include "glm/ext/matrix_transform.hpp"
//#include "glm/gtx/quaternion.hpp"

namespace
{
  constexpr glm::vec3 WORLD_UP{0,1,0};
  constexpr glm::vec3 WORLD_FORWARD{0,0,-1};
}

Renderer::Camera::Camera() {
  rot = glm::rotate(
    glm::identity<glm::quat>(),
    glm::radians(-180.0f),
    glm::vec3(1,0,0)
  );
  posOffset = {0,220,220};
}

void Renderer::Camera::update() {
  pos += velocity;
  velocity *= 0.9f;
}

void Renderer::Camera::apply(UniformGlobal &uniGlobal)
{
  float aspect = screenSize.x / screenSize.y;
  uniGlobal.projMat = glm::perspective(glm::radians(70.0f), aspect, 10.0f, 10'000.0f);

  const glm::vec3 direction = glm::normalize(rot * WORLD_FORWARD);
  const glm::vec3 dynamicUp = glm::normalize(rot * WORLD_UP);
  const glm::vec3 target = pos + posOffset + direction;
  uniGlobal.cameraMat = glm::lookAt(pos + posOffset, target, dynamicUp);


/*
  uniGlobal.cameraMat = glm::mat4_cast(rot);
  uniGlobal.cameraMat = glm::translate(uniGlobal.cameraMat, -pos * rot);
  */
}

void Renderer::Camera::rotateDelta(glm::vec2 screenDelta)
{
  if (!isRotating) {
    rotBase = rot;
    isRotating = true;
  }

  constexpr float rotSpeed = 0.0025f;
  // rotate based on screen delta
  float angleX = screenDelta.x * -rotSpeed;
  float angleY = screenDelta.y * -rotSpeed;
  glm::quat qx = glm::angleAxis(angleX, glm::vec3(0, 1, 0));
  glm::quat qy = glm::angleAxis(angleY, glm::vec3(1, 0, 0));
  rot = qx * rotBase * qy;
  rot = glm::normalize(rot);

}

void Renderer::Camera::moveDelta(glm::vec2 screenDelta) {
  if (!isMoving) {
    posBase = pos;
    isMoving = true;
  }
  float moveX = screenDelta.x * 0.001f;
  float moveY = screenDelta.y * -0.001f;

  glm::vec3 right = rot * glm::vec3(1, 0, 0);
  glm::vec3 up = rot * glm::vec3(0, 1, 0);

  pos = posBase + (right * moveX) + (up * moveY);
}
