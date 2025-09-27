/**
* @copyright 2025 - Max Bebök
* @license MIT
*/
#include "filePicker.h"

#include <atomic>
#include <mutex>
#include <SDL3/SDL.h>

#include "../context.h"

namespace
{
  constinit bool isPickerOpen{false};
  constinit std::mutex mtxResult{};
  constinit std::atomic_bool hasResult{false};
  constinit std::string result{};

  std::function<void(const std::string&path)> resultUserCb;

  void cbResult(void *userdata, const char * const *filelist, int filter) {
    std::lock_guard lock{mtxResult};
    result = (filelist && filelist[0]) ? filelist[0] : "";
    hasResult = true;
  }
}

bool Utils::FilePicker::open(std::function<void(const std::string&path)> cb, bool isDirectory) {
  if (isPickerOpen) return false;

  resultUserCb = cb;
  if (isDirectory) {
    SDL_ShowOpenFolderDialog(cbResult, nullptr, ctx.window, nullptr, false);
  } else {
    SDL_ShowOpenFileDialog(cbResult, nullptr, ctx.window, nullptr, 0, nullptr, false);
  }
  isPickerOpen = true;
  return true;
}

void Utils::FilePicker::poll()
{
  if (hasResult) {
    mtxResult.lock();
    auto resLocal = result;
    mtxResult.unlock();

    resultUserCb(resLocal);

    isPickerOpen = false;
    hasResult = false;
  }
}
