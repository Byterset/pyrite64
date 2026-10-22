/**
* @copyright 2026 - Max Bebök
* @license MIT
*/
#pragma once
#include <string>

namespace Project { class Project; }

namespace Editor::ScriptHost
{
  struct EvalResult
  {
    std::string text{};
    bool success{};
  };

  bool start(Project::Project &project);
  void stop();

  [[nodiscard]] bool isRunning();

  /**
   * Evaluates code in the global editor scope.
   * Result is JSON as a string, or some raw text for exceptions.
   */
  EvalResult eval(const std::string &code);
}
