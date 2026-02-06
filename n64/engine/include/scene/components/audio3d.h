/**
 * @author Kevin Reier
 * @license MIT
 */
#pragma once
#include <t3d/t3dmodel.h>
#include <libdragon.h>
#include "assets/assetManager.h"
#include "audio/audioManager.h"
#include "scene/object.h"
#include "scene/sceneManager.h"
#include "script/scriptTable.h"

namespace P64::Comp
{
  struct Audio3D
  {
    static constexpr uint32_t ID = 11;

    static constexpr uint8_t FLAG_LOOP = 1 << 0;
    static constexpr uint8_t FLAG_AUTO_PLAY = 1 << 1;

    wav64_t *audio{};
    float volume{};
    float radius{};
    uint16_t listenerObjId{};
    uint8_t flags{};
    Audio::Handle handle{};

    static uint32_t getAllocSize([[maybe_unused]] uint16_t* initData)
    {
      return sizeof(Audio3D);
    }

    static void initDelete([[maybe_unused]] Object& obj, Audio3D* data, uint16_t* initData);
    static void update(Object& obj, Audio3D* data, float deltaTime);
  };
}
