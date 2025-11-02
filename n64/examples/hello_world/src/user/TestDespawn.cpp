#include "scene/sceneManager.h"

namespace P64::Script::C54E2E8B498612FE
{
  struct Data
  {
    // Put your arguments here if needed, those will show up in the editor.
    //
    // Allowed types:
    // - uint8_t, int8_t, uint16_t, int16_t, uint32_t, int32_t
    // - float
  };

  void update(Object& obj, Data *data)
  {
    auto pressed = joypad_get_buttons_pressed(JOYPAD_PORT_1);
    if(pressed.z)obj.remove();
  }
}
