/**
* @copyright 2025 - Max Bebök
* @license MIT
*/
#pragma once
#include "assets/assetManager.h"
#include "scene/object.h"
#include "assets/assetManager.h"
#include <t3d/t3dmodel.h>

#include "collision/colliderShape.h"

namespace P64::Comp
{
  struct CollBody
  {
    static constexpr uint32_t ID = 5;

    Coll::Collider collider{};

    /// Resizes the collider. Like the half extend set in the editor this is in the object's
    /// local space, the object scale gets applied on top of it and stays applied when the
    /// object is scaled later on. Shapes that don't use all three axes fold them in,
    /// see 'Coll::Collider::setHalfExtend()'.
    void setHalfExtend(const fm_vec3_t &newHalfExtend) {
      halfExtend_ = newHalfExtend;
      applyObjectScale(appliedScale_);
    }
    /// Unscaled half extend, multiply with the object scale to get the collider's world size.
    const fm_vec3_t &getHalfExtend() const { return halfExtend_; }

    /// Changes the shape type and keeps the current half extend, unlike the collider's own
    /// setShapeType() which resets the dimensions of the new shape to zero.
    void setShapeType(Coll::ShapeType type) {
      collider.setShapeType(type);
      applyObjectScale(appliedScale_);
    }

    /// Rebuilds the collider size from the half extend scaled by the given object scale.
    void applyObjectScale(const fm_vec3_t &objectScale) {
      appliedScale_ = objectScale;
      collider.setHalfExtend(halfExtend_ * objectScale);
    }

    static uint32_t getAllocSize([[maybe_unused]] uint16_t* initData)
    {
      return sizeof(CollBody);
    }

    static void initDelete([[maybe_unused]] Object& obj, CollBody* data, void* initData);

    static void onEvent(Object& obj, CollBody* data, const ObjectEvent& event);

    static void update(Object& obj, CollBody* data, float deltaTime);

  private:
    fm_vec3_t halfExtend_{};
    // object scale the collider size was last built with, used to detect scale changes
    fm_vec3_t appliedScale_{{1.0f, 1.0f, 1.0f}};
  };
}
