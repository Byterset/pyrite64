/**
* @copyright 2025 - Max Bebök
* @license MIT
*/
#include "project.h"

#include <filesystem>

#include "../utils/fs.h"

#include "../utils/json.h"
#include "../utils/jsonBuilder.h"

using namespace simdjson;

std::string Project::ProjectConf::serialize() const {
  Utils::JSON::Builder builder{};
  builder.set("name", name);
  builder.set("romName", romName);
  builder.set("pathEmu", pathEmu);
  builder.set("pathN64Inst", pathN64Inst);
  builder.set("sceneIdOnBoot", sceneIdOnBoot);
  builder.set("sceneIdOnReset", sceneIdOnReset);
  return builder.toString();
}

void Project::Project::deserialize(const simdjson_result<dom::element> &doc) {
  conf.name = Utils::JSON::readString(doc, "name");
  conf.romName = Utils::JSON::readString(doc, "romName");
  conf.pathEmu = Utils::JSON::readString(doc, "pathEmu");
  conf.pathN64Inst = Utils::JSON::readString(doc, "pathN64Inst");
  conf.sceneIdOnBoot = Utils::JSON::readU32(doc, "sceneIdOnBoot", 1);
  conf.sceneIdOnReset = Utils::JSON::readU32(doc, "sceneIdOnReset", 1);
}

Project::Project::Project(const std::string &path)
  : path{path}, pathConfig{path + "/project.json"}
{
  Utils::FS::ensureDir(path);
  Utils::FS::ensureDir(path + "/data");
  Utils::FS::ensureDir(path + "/data/scenes");
  Utils::FS::ensureDir(path + "/assets");
  Utils::FS::ensureDir(path + "/assets/p64");
  Utils::FS::ensureDir(path + "/src");
  Utils::FS::ensureDir(path + "/src/p64");
  Utils::FS::ensureDir(path + "/src/user");

  Utils::FS::ensureFile(path + "/.gitignore", "data/build/baseGitignore");
  Utils::FS::ensureFile(path + "/Makefile.custom", "data/build/baseMakefile.custom");
  Utils::FS::ensureFile(path + "/assets/p64/font.ia4.png", "data/build/assets/font.ia4.png");

  deserialize(Utils::JSON::loadFile(pathConfig));
  assets.reload();
  scenes.reload();
}

void Project::Project::save() {
  Utils::FS::saveTextFile(pathConfig, conf.serialize());
  assets.save();
  scenes.save();
}
