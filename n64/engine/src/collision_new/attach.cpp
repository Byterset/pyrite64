/**
* @copyright 2026 - Max Bebök
* @license MIT
*/
#include "collision_new/attach.h"
#include "scene/components/collMesh.h"
#include "scene/scene.h"

fm_vec3_t P64::CollNew::Attach::update(const fm_vec3_t &ownPos)
{
  auto trackedObj = SceneManager::getCurrent().getObjectById(refId);
  auto trackedColl = trackedObj ? trackedObj->getComponent<Comp::CollMesh>() : nullptr;

  fm_vec3_t diff{};
  if(trackedColl && trackedColl->meshCollider)
  {
    if(lastRefId == refId) {
      diff = refPos - trackedColl->meshCollider->toWorldSpace(refPosLocal);
    }

    lastRefId = refId;
    refPos = ownPos;
    refPosLocal = trackedColl->meshCollider->toLocalSpace(refPos);
  } else {
    lastRefId = 0;
  }
  refId = 0;
  return diff;
}

void P64::CollNew::Attach::setReference(const CollNew::MeshCollider *meshCollider)
{
  if(meshCollider) {
    refId = meshCollider->owner->id;
  }
}
