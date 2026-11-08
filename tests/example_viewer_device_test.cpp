// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The device the live viewers share (examples/viewer/viewer_common.hpp): it is
// built with the app's name and graphics requirements, plus line fill for the
// wireframe, and the window's validation setting, not the app's own, and
// without a window it is refused with gfx's reason; and the View options' and
// the shading's flags (viewer_panels.hpp). Host only: no window, instance or
// device is made.

#include <cstdio>
#include <memory>

#include "viewer_common.hpp"
#include "viewer_panels.hpp"
#include "volumetric_kit/core/testing/test_check.hpp"

namespace vkc = volumetric_kit::core;
namespace vg = volumetric_kit::gfx;

int main() {
  vg::app::WindowedAppConfig app;
  app.app_name = "example_viewer_device_test";
  app.device.features.samplerAnisotropy = VK_TRUE;
  app.device.extensions.push_back("VK_EXT_test_only");
  // WindowedApp::adopt ignores this; the window's setting is the one used.
  app.enable_validation = true;

  const vg::app::WindowDeviceConfig on =
      fuse_viewer::viewer_device_config(app, true);
  CHECK(on.instance.app_name == app.app_name);
  CHECK(on.instance.enable_validation);
  CHECK(on.graphics.features.samplerAnisotropy == VK_TRUE);
  // Line fill, for the View panel's wireframe, though the app asked none.
  CHECK(app.device.features.fillModeNonSolid == VK_FALSE);
  CHECK(on.graphics.features.fillModeNonSolid == VK_TRUE);
  CHECK(on.graphics.extensions == app.device.extensions);
  CHECK(on.graphics.dynamic_rendering == app.device.dynamic_rendering);
  CHECK(on.graphics.api_version == app.device.api_version);

  const vg::app::WindowDeviceConfig off =
      fuse_viewer::viewer_device_config(app, false);
  CHECK(!off.instance.enable_validation);
  CHECK(off.instance.app_name == app.app_name);

  // No window: refused before GLFW or Vulkan is reached.
  const vkc::Result<std::unique_ptr<vkc::SharedDevice>> made =
      fuse_viewer::make_viewer_device(nullptr, app, false);
  CHECK(!made.ok());
  CHECK(made.status().domain() == vkc::Status::Code::InvalidArgument);

  // The View options' flags: none at first, each its gfx bit.
  fuse_viewer::MeshView view;
  CHECK(fuse_viewer::mesh_view_flags(view) == 0u);
  view.wireframe = true;
  CHECK(fuse_viewer::mesh_view_flags(view) ==
        vg::pipelines::kHybridMeshWireframe);
  view.cull_back_faces = true;
  CHECK(fuse_viewer::mesh_view_flags(view) ==
        (vg::pipelines::kHybridMeshWireframe |
         vg::pipelines::kHybridMeshCullBackFaces));
  view.wireframe = false;
  CHECK(fuse_viewer::mesh_view_flags(view) ==
        vg::pipelines::kHybridMeshCullBackFaces);

  // The shading's flags: lit, none for unlit, normals.
  using Shading = volumetric_kit::core::config::Shading;
  CHECK(fuse_viewer::shading_flags(Shading::Lit) ==
        vg::pipelines::kHybridMeshLit);
  CHECK(fuse_viewer::shading_flags(Shading::Unlit) == 0u);
  CHECK(fuse_viewer::shading_flags(Shading::Normals) ==
        vg::pipelines::kHybridMeshNormals);

  std::puts("example_viewer_device: OK");
  return 0;
}
