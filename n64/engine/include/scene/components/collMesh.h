/**
* @copyright 2025 - Max Bebök
* @license MIT
*/
#pragma once
#include <t3d/t3dmodel.h>

#include "assets/assetManager.h"
#include "lib/matrixManager.h"
#include "scene/object.h"
#include "script/scriptTable.h"

namespace P64::Comp
{
  struct CollMesh
  {
    uint32_t dummy;

    static uint32_t getAllocSize([[maybe_unused]] uint16_t* initData)
    {
      return sizeof(CollMesh);
    }

    static void initDelete([[maybe_unused]] Object& obj, CollMesh* data, uint16_t* initData)
    {
      if (initData == nullptr) {
        data->~CollMesh();
        return;
      }

      uint16_t assetIdx = initData[0];
      new(data) CollMesh();
      debugf("Coll-asset: %d\n", assetIdx);

      //data->model = (T3DModel*)AssetManager::getByIndex(assetIdx);
      //assert(data->model != nullptr);
    }

    static void update(Object& obj, CollMesh* data, float deltaTime) {
    }
  };
}