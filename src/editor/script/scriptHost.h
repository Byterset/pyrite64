/**
* @copyright 2026 - Max Bebök
* @license MIT
*/
#pragma once

namespace Project { class Project; }

namespace Editor::ScriptHost
{
  bool start(Project::Project &project);
  void stop();

  [[nodiscard]] bool isRunning();
}
