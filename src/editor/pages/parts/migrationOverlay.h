/**
* @copyright 2026 - Max Bebök
* @license MIT
*/
#pragma once
#include <functional>

#include "../../../project/scene/migration.h"

/**
 * Asks the user before scene/prefab files are upgraded to a newer format.
 *
 * Migration rewrites project files in place, so it never runs unattended: the caller hands over
 * what it found and what it wants to do afterwards, and only gets its callback when the user
 * agrees. On Decline, nothing is changed and the project is left as-is.
 */
namespace Editor::MigrationOverlay
{
  /**
   * Opens the confirmation dialog.
   * @param scan documents that need migrating, must not be empty
   * @param title what the user was trying to do, e.g. "Open Scene"
   * @param onConfirm runs after the migration completed
   * @param onCancel runs when the user declined, optional
   */
  void open(const Project::Migration::ScanResult &scan, const char *title,
            std::function<void()> onConfirm, std::function<void()> onCancel = {});

  /// Call once per frame, draws the dialog if open.
  void draw();

  /**
   * Runs `onConfirm` right away when nothing needs migrating, otherwise asks first.
   */
  void guard(const Project::Migration::ScanResult &scan, const char *title,
             std::function<void()> onConfirm, std::function<void()> onCancel = {});
}
