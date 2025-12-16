/**
* @copyright 2025 - Max Bebök
* @license MIT
*/
#include "scene/object.h"
#include "scene/components/collBody.h"
#include "scene/sceneManager.h"

namespace
{
  struct InitData
  {
    fm_vec3_t halfExtend{};
    fm_vec3_t offset{};
    uint8_t type{};
  };
}

namespace P64::Comp
{
  void CollBody::initDelete([[maybe_unused]] Object& obj, CollBody* data, void* initData_)
  {
    InitData* initData = static_cast<InitData*>(initData_);
    auto &coll = SceneManager::getCurrent().getCollision();

    if (initData == nullptr) {
      coll.unregisterBCS(&data->bcs);
      data->~CollBody();
      return;
    }

    new(data) CollBody();

    data->offset = initData->offset;
    data->bcs = {
      .center = obj.pos + data->offset,
      .halfExtend = initData->halfExtend,
      .maskRead = 0xFF,
      .maskWrite = 0xFF,
      .flags = initData->type == 0 ? Coll::BCSFlags::SHAPE_BOX : (uint8_t)0,
    };
    coll.registerBCS(&data->bcs);
  }
}
