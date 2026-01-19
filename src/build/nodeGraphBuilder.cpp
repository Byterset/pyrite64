/**
* @copyright 2025 - Max Bebök
* @license MIT
*/
#include "projectBuilder.h"
#include "../utils/string.h"
#include "../utils/fs.h"
#include "../utils/logger.h"
#include "../utils/proc.h"
#include <filesystem>

#include "../project/graph/graph.h"

namespace fs = std::filesystem;

bool Build::buildNodeGraphAssets(Project::Project &project, SceneCtx &sceneCtx)
{
  fs::path mkAsset = fs::path{project.conf.pathN64Inst} / "bin" / "mkasset";
  auto &assets = sceneCtx.project->getAssets().getTypeEntries(Project::FileType::NODE_GRAPH);
  for (auto &asset : assets)
  {
    if(asset.conf.exclude)continue;

    auto projectPath = fs::path{project.getPath()};
    auto outPath = projectPath / asset.outPath;
    auto outDir = outPath.parent_path();
    Utils::FS::ensureDir(outPath.parent_path());

    sceneCtx.files.push_back(asset.outPath);
    if(!assetBuildNeeded(asset, outPath))continue;

    //printf("Prefab: %s -> %s\n", asset.path.c_str(), outPath.string().c_str());
    auto json = Utils::FS::loadTextFile(asset.path);
    Project::Graph::Graph graph{};
    graph.deserialize(json);
    auto data = graph.build();
    data.writeToFile(outPath);
  }
  return true;
}