/**
* @copyright 2026 - Max Bebök
* @license MIT
*/
#include "scriptHost.h"
#include "../../utils/logger.h"
#include "quickjs.h"

namespace
{
  JSRuntime *rt{nullptr};
  JSContext *jsCtx{nullptr};
}

namespace Editor::ScriptHost
{
  bool start(Project::Project &project)
  {
    stop();

    rt = JS_NewRuntime();
    if(!rt) {
      Utils::Logger::log("Failed to create editor script runtime", Utils::Logger::LEVEL_ERROR);
      return false;
    }

    jsCtx = JS_NewContext(rt);
    if(!jsCtx) {
      Utils::Logger::log("Failed to create editor script context", Utils::Logger::LEVEL_ERROR);
      JS_FreeRuntime(rt);
      rt = nullptr;
      return false;
    }

    Utils::Logger::log("Editor scripts: context started");
    return true;
  }

  void stop()
  {
    if(!rt)return;

    JS_FreeContext(jsCtx);
    JS_FreeRuntime(rt);
    jsCtx = nullptr;
    rt = nullptr;
    Utils::Logger::log("Editor scripts: context stopped");
  }

  bool isRunning() {
    return jsCtx != nullptr;
  }
}
