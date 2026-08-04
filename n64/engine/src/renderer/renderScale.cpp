/**
* @copyright 2026 - Max Bebök
* @license MIT
*/
#include "renderer/renderScale.h"

namespace P64::Renderer
{
  namespace
  {
    constexpr float DEFAULT_RENDER_SCALE = 100.0f;

    constinit float renderScale_{DEFAULT_RENDER_SCALE};
    constinit float inverseRenderScale_{1.0f / DEFAULT_RENDER_SCALE};
  }

  float getRenderScale() { return renderScale_; }
  float getInvRenderScale() { return inverseRenderScale_; }

  void setRenderScale(float renderScale) {
    renderScale_ = renderScale;
    inverseRenderScale_ = 1.0f / renderScale;
  }

  void fillModelMatrixFP(T3DMat4FP *mat,
    const fm_vec3_t &scale, const fm_quat_t &rot, const fm_vec3_t &pos,
    float vertexScale)
  {
    t3d_mat4fp_from_srt(mat, scale * (vertexScale * renderScale_), rot, pos * renderScale_);
  }
}
