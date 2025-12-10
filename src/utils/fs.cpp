/**
* @copyright 2025 - Max Bebök
* @license MIT
*/
#include "fs.h"
#include <filesystem>

void Utils::FS::ensureDir(const std::string &path)
{
  std::filesystem::path fsPath{path};
  if(!std::filesystem::exists(fsPath)) {
    std::filesystem::create_directories(fsPath);
  }
}

void Utils::FS::ensureFile(const std::string &path, const std::string &pathTemplate)
{
  std::filesystem::path fsPath{path};
  if(!std::filesystem::exists(fsPath)) {
    std::filesystem::create_directories(fsPath.parent_path());
    std::filesystem::copy_file(pathTemplate, fsPath);
  }
}
