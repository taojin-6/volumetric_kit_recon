// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The render pass both live viewers record (examples/viewer/viewer_common.hpp):
// one GPU-timed stage spans the whole pass, and the mesh and overlay draws
// inside it are CPU-only stages; the overlay's colours show as authored on an
// _SRGB target, as the swapchain's default is. Runs on a headless device,
// which the test skips without.

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <imgui.h>

#include "test_check.hpp"
#include "viewer_common.hpp"
#include "volumetric_kit/core/testing/vulkan_policy.hpp"

#include "volumetric_kit/core/base/stage_metrics.hpp"
#include "volumetric_kit/core/vulkan/vulkan.hpp"
#include "volumetric_kit/gfx/app/headless_app.hpp"
#include "volumetric_kit/gfx/core/frame_metrics.hpp"
#include "volumetric_kit/gfx/core/offscreen_target.hpp"
#include "volumetric_kit/gfx/core/profiler.hpp"
#include "volumetric_kit/gfx/ui/imgui_overlay.hpp"

namespace {

namespace vkc = volumetric_kit::core;
namespace vg = volumetric_kit::gfx;

constexpr VkExtent2D kExtent{64, 64};
// ImGui's dark WindowBg, as an sRGB byte.
constexpr int kWindowBg = 15;

const vkc::StageRow* find_row(const vg::FrameMetrics& metrics,
                              const char* name) {
  for (const vkc::StageRow& row : metrics.sections) {
    if (std::strcmp(row.name, name) == 0) return &row;
  }
  return nullptr;
}

int records_the_viewer_pass(vg::app::HeadlessApp& app) {
  vg::OffscreenTargetDesc desc;
  desc.extent = kExtent;
  desc.color_format = VK_FORMAT_R8G8B8A8_SRGB;  // as the swapchain's default
  auto target = vg::OffscreenTarget::create(app.allocator(), desc);
  CHECK(target.ok());
  vg::ui::ImGuiOverlayConfig overlay_config;
  overlay_config.layout = target.value().layout();
  auto overlay = vg::ui::ImGuiOverlay::create(
      app.device(), app.instance().handle(), overlay_config);
  CHECK(overlay.ok());
  vg::ProfilerConfig profiler_config;
  profiler_config.frames_in_flight = 1;
  auto profiler = vg::Profiler::create(app.device(), profiler_config);
  CHECK(profiler.ok());

  // Two frames: the second's begin_frame publishes the first's stages.
  int meshes = 0;
  for (int frame = 0; frame < 2; ++frame) {
    ImGui::GetIO().DisplaySize = ImVec2(static_cast<float>(kExtent.width),
                                        static_cast<float>(kExtent.height));
    overlay.value().new_frame();
    ImGui::GetBackgroundDrawList()->AddRectFilled(
        ImVec2(0.0f, 0.0f), ImGui::GetIO().DisplaySize,
        IM_COL32(kWindowBg, kWindowBg, kWindowBg, 255));
    const vkc::Status drawn =
        app.device().submit_single_time([&](VkCommandBuffer cmd) {
          profiler.value().begin_frame(0, cmd);
          target.value().prepare(cmd);
          fuse_viewer::record_view_pass(
              profiler.value(), cmd, target.value().target(),
              [&](VkCommandBuffer) { ++meshes; }, &overlay.value());
          target.value().record_readback(cmd);
          profiler.value().end_frame();
        });
    CHECK(drawn.ok());
  }
  CHECK(meshes == 2);

  // gfx #144 and on: the target encodes the byte once, not twice (69).
  const auto* pixels =
      static_cast<const std::uint8_t*>(target.value().pixels());
  CHECK(pixels != nullptr);
  const std::size_t centre =
      (std::size_t{kExtent.height} / 2 * kExtent.width + kExtent.width / 2) * 4;
  std::printf("WindowBg %d reads %d\n", kWindowBg, pixels[centre]);
  for (std::size_t c = 0; c < 3; ++c) {
    CHECK(std::abs(pixels[centre + c] - kWindowBg) <= 1);
  }

  const vg::FrameMetrics& metrics = profiler.value().metrics();
  const vkc::StageRow* pass = find_row(metrics, "render pass");
  const vkc::StageRow* mesh = find_row(metrics, "mesh draw");
  const vkc::StageRow* ui = find_row(metrics, "overlay draw");
  CHECK(pass != nullptr && mesh != nullptr && ui != nullptr);
  std::printf("GPU timing %s\n",
              profiler.value().gpu_timing() ? "available" : "unavailable");
  CHECK(pass->has_gpu == profiler.value().gpu_timing());
  CHECK(!mesh->has_gpu);
  CHECK(!ui->has_gpu);
  return 0;
}

int run_pass_tests() {
  vg::app::HeadlessAppConfig config;
  config.app_name = "recon_example_viewer_pass_test";
  config.enable_validation = vkc::test::instance_config().enable_validation;
  auto app = vg::app::HeadlessApp::create(config);
  if (!app.ok()) {
    return vkc::test::no_device_exit_code("no Vulkan device (" +
                                          app.status().message() + ")");
  }
  if (const vkc::Status loaded =
          vkc::test::check_layer_loaded(app.value().instance());
      !loaded.ok()) {
    std::fprintf(stderr, "%s\n", loaded.message().c_str());
    return 1;
  }
  if (const int rc = records_the_viewer_pass(app.value())) return rc;
  std::puts("viewer_pass: OK");
  return 0;
}

}  // namespace

int main() {
  const vkc::test::ValidationSession validation;
  const vkc::test::LogCapture log;
  return log.exit_code(run_pass_tests());
}
