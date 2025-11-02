/**
* @copyright 2025 - Max Bebök
* @license MIT
*/
#include "scene/object.h"

#include "scene/componentTable.h"
#include "scene/sceneManager.h"

P64::Object::~Object()
{
  auto compRefs = getCompRefs();
  for (uint32_t i=0; i<compCount; ++i) {
    const auto &compDef = COMP_TABLE[compRefs[i].type];
    char* dataPtr = (char*)this + compRefs[i].offset;
    compDef.initDel(*this, dataPtr, nullptr);
  }
}

void P64::Object::remove()
{
  SceneManager::getCurrent().removeObject(*this);
}
