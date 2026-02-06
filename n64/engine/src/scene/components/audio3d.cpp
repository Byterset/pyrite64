/**
* @copyright 2026 - GitHub Copilot
* @license MIT
*/
#include "scene/object.h"
#include "scene/components/audio3d.h"
#include "audio/audioManager.h"
#include "scene/sceneManager.h"
#include "scene/scene.h"
#include <math.h>

namespace
{
  struct InitData
  {
    uint16_t assetIdx;
    uint16_t volume;
    uint16_t listenerObjId;
    uint8_t flags;
    uint8_t padding;
    float radius;
  } __attribute__((packed));
}

namespace P64::Comp
{
  void Audio3D::initDelete(Object &obj, Audio3D* data, uint16_t* initData_)
  {
    auto initData = (InitData*)initData_;
    if (initData == nullptr) {
      data->handle.stop();
      data->~Audio3D();
      return;
    }

    new(data) Audio3D();

    data->audio = (wav64_t*)AssetManager::getByIndex(initData->assetIdx);
    
    data->volume = (float)initData->volume * (1.0f / 0xFFFF);
    data->listenerObjId = initData->listenerObjId;
    data->radius = initData->radius;
    data->flags = initData->flags;

    if(data->audio) {
        wav64_set_loop(data->audio, (data->flags & FLAG_LOOP) != 0);

        if(data->flags & FLAG_AUTO_PLAY) {
            data->handle = AudioManager::play2D(data->audio);
            // Start silent, update will set correct volume
            data->handle.setVolume(0.0f);
        }
    }
  }

  void Audio3D::update(Object& obj, Audio3D* data, [[maybe_unused]] float deltaTime)
  {
    if(data->handle.isDone()) return;

    auto &sc = SceneManager::getCurrent();

    auto listener = sc.getObjectById(data->listenerObjId);
    
    if(!listener) {
        return;
    }
    
    float dx = obj.pos.x - listener->pos.x;
    float dy = obj.pos.y - listener->pos.y;
    float dz = obj.pos.z - listener->pos.z;
    float dist = sqrtf(dx*dx + dy*dy + dz*dz);
    
    //scale volume with distance, simple linear falloff
    //TODO: coult add curve option or make it depend on listener orientation for directional audio
    float volScale = 0.0f;
    if (dist < data->radius) {
        volScale = 1.0f - (dist / data->radius);
        if (volScale < 0.0f) volScale = 0.0f;
    }
    
    data->handle.setVolume(data->volume * volScale);
  }
}
