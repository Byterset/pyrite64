/**
* @copyright 2025 - Max Bebök
* @license MIT
*/
#include "./sceneManager.h"
#include "../project.h"
#include <filesystem>

namespace
{
  std::string getScenePath(Project::Project *project) {
    auto scenesPath = project->getPath() + "/data/scenes";
    if (!std::filesystem::exists(scenesPath)) {
      std::filesystem::create_directory(scenesPath);
    }
    return scenesPath;
  }
}

void Project::SceneManager::reload()
{
  entries.clear();

  auto scenesPath = getScenePath(project);

  // list directories
  for (const auto &entry : std::filesystem::directory_iterator{scenesPath}) {
    if (entry.is_directory()) {
      auto path = entry.path();
      auto name = path.filename().string();
      try {
        int id = std::stoi(name);
        entries.push_back({id, name});
      } catch(...) {
        // ignore
      }
    }
  }

}

Project::SceneManager::~SceneManager() {
  delete loadedScene;
}

void Project::SceneManager::save() {
  if (loadedScene) {
    loadedScene->save();
  }
}

void Project::SceneManager::add() {
  auto scenesPath = getScenePath(project);
  int newId = 1;
  for (const auto &entry : entries) {
    if (entry.id >= newId) {
      newId = entry.id + 1;
    }
  }
  auto newPath = std::filesystem::path{scenesPath} / std::to_string(newId);
  printf("Create-Scene: %s\n", newPath.c_str());
  std::filesystem::create_directory(newPath);

  reload();
}

void Project::SceneManager::loadScene(int id) {
  if (loadedScene) {
    loadedScene->save();
    delete loadedScene;
  }
  loadedScene = new Scene(id);
}
