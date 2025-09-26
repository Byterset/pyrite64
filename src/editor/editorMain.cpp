/**
* @copyright 2025 - Max Bebök
* @license MIT
*/
#include "editorMain.h"

#include <atomic>
#include <cstdio>
#include <mutex>

#include "imgui.h"
#include "backends/imgui_impl_sdlgpu3.h"
#include "SDL3/SDL_dialog.h"

void ImDrawCallback_ImplSDLGPU3_SetSamplerRepeat(const ImDrawList* parent_list, const ImDrawCmd* cmd);

namespace
{
  bool isHoverAdd = false;
  bool isHoverLast = false;

  constinit bool fileSelectIsOpen{false};
  constinit std::mutex mtxProjectPath{};
  constinit std::atomic_bool hasProjectPath{false};
  constinit std::string projectPath{};

  void cbOpenProject(void *userdata, const char * const *filelist, int filter) {
    std::lock_guard lock{mtxProjectPath};
    projectPath = (filelist && filelist[0]) ? filelist[0] : "";
    hasProjectPath = true;
  }
}

Editor::Main::Main(SDL_GPUDevice* device)
  : texTitle{device, "data/img/titleLogo.png"},
  texBtnAdd{device, "data/img/cardAdd.svg"},
  texBtnOpen{device, "data/img/cardLast.svg"},
  texBG{device, "data/img/splashBG.png"}
{
}

Editor::Main::~Main() {
}

void Editor::Main::draw()
{
  auto &io = ImGui::GetIO();

  ImGui::SetNextWindowPos({0,0}, ImGuiCond_Appearing, {0.0f, 0.0f});
  ImGui::SetNextWindowSize({io.DisplaySize.x, io.DisplaySize.y}, ImGuiCond_Always);
  ImGui::Begin("WIN_MAIN", 0,
    ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoTitleBar
    | ImGuiWindowFlags_NoBackground | ImGuiWindowFlags_NoScrollbar
    | ImGuiWindowFlags_NoScrollWithMouse
  );

  ImVec2 centerPos = {io.DisplaySize.x / 2, io.DisplaySize.y / 2};

  // BG
  ImGui::GetWindowDrawList()->AddCallback(ImDrawCallback_ImplSDLGPU3_SetSamplerRepeat, nullptr);

  float topBgHeight = 7;
  float bottomBgHeight = 3;
  float bgRepeatsX = io.DisplaySize.x / texBG.getWidth();
  ImGui::SetCursorPos({0,0});
  ImGui::Image(ImTextureID(texBG.getGPUTex()),
    {io.DisplaySize.x, (float)texBG.getHeight() * topBgHeight},
    {0,topBgHeight}, {bgRepeatsX,0}
  );
  // bottom

  ImGui::SetCursorPos({0, io.DisplaySize.y - ((float)texBG.getHeight() * bottomBgHeight)});
  ImGui::Image(ImTextureID(texBG.getGPUTex()),
    {io.DisplaySize.x, (float)texBG.getHeight() * bottomBgHeight},
    {0,0}, {bgRepeatsX,bottomBgHeight}
  );

  float midBgPointY = (float)texBG.getHeight() * topBgHeight;
  midBgPointY += io.DisplaySize.y - ((float)texBG.getHeight() * bottomBgHeight);
  midBgPointY /= 2.0f;

  ImGui::GetWindowDrawList()->AddCallback(ImDrawCallback_ResetRenderState, nullptr);

  // Title
  if (isHoverAdd || isHoverLast) {
    ImGui::SetMouseCursor(ImGuiMouseCursor_Hand);
  } else {
    ImGui::SetMouseCursor(ImGuiMouseCursor_Arrow);
  }

  auto logoSize = texTitle.getSize(0.65f);
  ImGui::SetCursorPos({
    centerPos.x - (logoSize.x/2) + 16,
    28
  });
  ImGui::Image(ImTextureID(texTitle.getGPUTex()),logoSize);

  constexpr float BTN_SPACING = 170;

  auto getBtnPos = [&](ImVec2 size, bool isLeft) {
    return ImVec2{
      centerPos.x - (size.x/2) + (isLeft ? -BTN_SPACING : BTN_SPACING),
      midBgPointY - (size.y/2)
    };
  };

  // Buttons
  ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.f, 0.f, 0.f, 0.f));
  ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.f, 0.f, 0.f, 0.f));
  ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.f, 0.f, 0.f, 0.f));

  auto btnSizeAdd = texBtnAdd.getSize(isHoverAdd ? 0.85f : 0.8f);
  ImGui::SetCursorPos(getBtnPos(btnSizeAdd, true));
  ImGui::ImageButton("New Project",
      ImTextureID(texBtnAdd.getGPUTex()),
      btnSizeAdd, {0,0}, {1,1}, {0,0,0,0},
      {1,1,1, isHoverAdd ? 1 : 0.8f}
  );
  isHoverAdd = ImGui::IsItemHovered(ImGuiHoveredFlags_RectOnly);

  auto btnSizeLast = texBtnOpen.getSize(isHoverLast ? 0.85f : 0.8f);
  ImGui::SetCursorPos(getBtnPos(btnSizeLast, false));

  bool wantsOpen = ImGui::ImageButton("Open Project",
      ImTextureID(texBtnOpen.getGPUTex()),
      btnSizeLast, {0,0}, {1,1}, {0,0,0,0},
      {1,1,1,isHoverLast ? 1 : 0.8f}
  );

  isHoverLast = ImGui::IsItemHovered(ImGuiHoveredFlags_RectOnly);

  ImGui::PopStyleColor(3);

  // version
  ImGui::SetCursorPos({14, io.DisplaySize.y - 30});
  ImGui::Text("Pyrite64 [v0.0.0-alpha]");

  constexpr const char* creditsStr = "© 2025 Max Bebök (HailToDodongo)";
  ImGui::SetCursorPos({
    io.DisplaySize.x - 14 - ImGui::CalcTextSize(creditsStr).x,
    io.DisplaySize.y - 30
  });
  ImGui::Text(creditsStr);

  ImGui::End();

  if (wantsOpen && !fileSelectIsOpen) {
    SDL_ShowOpenFolderDialog(cbOpenProject, nullptr, SDL_GL_GetCurrentWindow(), nullptr, false);
    fileSelectIsOpen = true;
  }

  if (hasProjectPath) {
    std::lock_guard lock{mtxProjectPath};
    if (!projectPath.empty()) {
      printf("Open project: %s\n", projectPath.c_str());
    }
    fileSelectIsOpen = false;
    hasProjectPath = false;
  }
}
