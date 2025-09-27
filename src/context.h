/**
* @copyright 2025 - Max Bebök
* @license MIT
*/
#pragma once
#include "project/project.h"
#include "SDL3/SDL_video.h"

struct Context
{
  Project::Project *project{nullptr};
  SDL_Window* window{nullptr};
};

extern Context ctx;