/**
* @copyright 2026 - Max Bebök
* @license MIT
*/
#include "scriptHost.h"
#include "../../utils/logger.h"
#include "../actions.h"
#include "quickjs.h"

namespace
{
  JSRuntime *rt{nullptr};
  JSContext *jsCtx{nullptr};

  std::string toString(JSValueConst val)
  {
    const char* str = JS_ToCString(jsCtx, val);
    std::string res = str ? str : "";
    if(str)JS_FreeCString(jsCtx, str);
    return res;
  }

  const JSCFunctionListEntry EditorAPI[] = {
    JS_CFUNC_DEF("build", 1, (
      [](JSContext *c, JSValueConst thisVal, int argc, JSValueConst *argv) -> JSValue {
        bool doRun = argc > 0 ? JS_ToBool(jsCtx, argv[0]) : false;
        bool res = Editor::Actions::call(Editor::Actions::Type::PROJECT_BUILD, doRun ? "run" : "");
        return JS_NewBool(c, res);
      }
    )),
    JS_CFUNC_DEF("close", 0, (
      [](JSContext *c, JSValueConst thisVal, int argc, JSValueConst *argv) -> JSValue {
        return JS_NewBool(c, Editor::Actions::call(Editor::Actions::Type::PROJECT_CLOSE));
      }
    )),
    JS_CFUNC_DEF("clean", 0, (
      [](JSContext *c, JSValueConst thisVal, int argc, JSValueConst *argv) -> JSValue {
        return JS_NewBool(c, Editor::Actions::call(Editor::Actions::Type::PROJECT_CLEAN));
      }
    )),
  };

  const JSCFunctionListEntry ConsoleAPI[] = {
    JS_CFUNC_DEF("clear", 0, (
      [](JSContext *c, JSValueConst thisVal, int argc, JSValueConst *argv) -> JSValue {
        Utils::Logger::clear();
        return JS_UNDEFINED;
      }
    )),
  };
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

    JSValue global = JS_GetGlobalObject(jsCtx);
    
    auto addObj = [&](const char* name, const JSCFunctionListEntry *entries, int len){
      JSValue api = JS_NewObject(jsCtx);
      JS_SetPropertyFunctionList(jsCtx, api, entries, len);
      JS_SetPropertyStr(jsCtx, global, name, api);
    };

    addObj("editor", EditorAPI, std::size(EditorAPI));
    addObj("console", ConsoleAPI, std::size(ConsoleAPI));

    JS_FreeValue(jsCtx, global);

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

  EvalResult eval(const std::string &code)
  {
    if(!jsCtx)return {"No project loaded!", false};

    JSValue val = JS_Eval(jsCtx, code.c_str(), code.size(), "<shell>", JS_EVAL_TYPE_GLOBAL);

    if(JS_IsException(val)) {
      JSValue ex = JS_GetException(jsCtx);
      std::string text = toString(ex);
      JSValue stack = JS_GetPropertyStr(jsCtx, ex, "stack");

      if(JS_IsString(stack)) {
        text += "\n" + toString(stack);
      }
      JS_FreeValue(jsCtx, stack);
      JS_FreeValue(jsCtx, ex);
      return {text, false};
    }

    EvalResult res{.success = true};
    if(JS_IsObject(val) && !JS_IsFunction(jsCtx, val)) {
      JSValue json = JS_JSONStringify(jsCtx, val, JS_UNDEFINED, JS_UNDEFINED);
      res.text = JS_IsException(json) ? toString(val) : toString(json);
      JS_FreeValue(jsCtx, json);
    } else {
      res.text = toString(val);
    }
    JS_FreeValue(jsCtx, val);
    return res;
  }
}
