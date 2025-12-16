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
  struct Model
  {
    static constexpr uint32_t ID = 1;

    T3DModel *model{};
    RingMat4FP matFP{};

    static uint32_t getAllocSize([[maybe_unused]] uint16_t* initData)
    {
      return sizeof(Model);
    }

    static void initDelete([[maybe_unused]] Object& obj, Model* data, uint16_t* initData)
    {
      if (initData == nullptr) {
        data->~Model();
        return;
      }

      uint16_t assetIdx = initData[0];
      new(data) Model();

      data->model = (T3DModel*)AssetManager::getByIndex(assetIdx);
      assert(data->model != nullptr);

      // auto scriptPtr = Script::getCodeByIndex(initData[0]);
      // reserved: initData[1];
    }

    static void update(Object& obj, Model* data, float deltaTime) {
      auto mat = data->matFP.getNext();
      t3d_mat4fp_from_srt(mat,
        obj.scale,
        obj.rot,
        obj.pos
      );
    }

    static void draw([[maybe_unused]] Object& obj, Model* data, float deltaTime) {
      t3d_matrix_set(data->matFP.get(), true);
      /*auto it = t3d_model_iter_create(data->model, T3D_CHUNK_TYPE_OBJECT);
      while(t3d_model_iter_next(&it)) {
        auto &mat = it.object->material;
        mat->colorCombiner = RDPQ_COMBINER_SHADE;
      }*/

      rspq_block_run(data->model->userBlock);
      //t3d_model_draw(data->model);

      //char* funcData = (char*)data + sizeof(Code);
      //if(data->funcDraw)data->funcDraw(obj, funcData);
    }
  };
}