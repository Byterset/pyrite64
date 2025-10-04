/**
* @copyright 2025 - Max Bebök
* @license MIT
*/
#include "assetManager.h"
#include "../context.h"
#include <filesystem>

#include "SHA256.h"
#include "../utils/fs.h"
#include "../utils/hash.h"
#include "../utils/json.h"

namespace
{
}

std::string Project::AssetManager::AssetConf::serialize() const {
  simdjson::builder::string_builder builder{};
  builder.start_object();
  builder.append_key_value<"format">(format);
  builder.append_comma();
  builder.append_key_value<"baseScale">(baseScale);
  builder.append_comma();
  builder.append_key_value<"compression">(static_cast<int>(compression));
  builder.append_comma();
  builder.append_key_value<"gltfBVH">(gltfBVH);
  builder.end_object();
  return {builder.c_str()};
}

void Project::AssetManager::reload() {
  entries.clear();

  auto assetPath = std::filesystem::path{project->getPath()} / "assets";
  if (!std::filesystem::exists(assetPath)) {
    std::filesystem::create_directory(assetPath);
  }

  // scan all files
  for (const auto &entry : std::filesystem::directory_iterator{assetPath}) {
    if (entry.is_regular_file()) {
      auto path = entry.path();
      auto ext = path.extension().string();

      FileType type = FileType::UNKNOWN;
      if (ext == ".png") {
        type = FileType::IMAGE;
      } else if (ext == ".wav" || ext == ".mp3") {
        type = FileType::AUDIO;
      } else if (ext == ".glb" || ext == ".gltf") {
        type = FileType::MODEL_3D;
      }

      uint64_t uuid = Utils::Hash::sha256_64bit("ASSET:" + path.string());
      Entry entry{
        .uuid = uuid,
        .name = path.filename().string(),
        .path = path.string(),
        .type = type,
      };

      if (type == FileType::IMAGE) {
        entry.texture = new Renderer::Texture{ctx.gpu, path.string()};
      }

      // check if meta-data exists
      auto pathMeta = path;
      pathMeta += ".conf";
      if (type != FileType::UNKNOWN && std::filesystem::exists(pathMeta))
      {
        auto doc = Utils::JSON::loadFile(pathMeta);
        if (doc.is_object()) {
          auto &conf = entry.conf;
          JSON_GET_INT(format);
          JSON_GET_INT(baseScale);
          JSON_GET_INT(compression);
          JSON_GET_BOOL(gltfBVH);
        }
      }

      entries.push_back(entry);
    }
  }
}

void Project::AssetManager::save()
{
  for(auto &entry : entries)
  {
    if(entry.type == FileType::UNKNOWN)continue;

    auto pathMeta = entry.path + ".conf";
    auto json = entry.conf.serialize();
    Utils::FS::saveTextFile(pathMeta, entry.conf.serialize());
  }
}
