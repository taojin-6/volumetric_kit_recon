// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The fusing examples' side of the integration boundary: every fusing
// template loads and maps into the codec's runtime options (grid's
// codec_config.hpp), core-validated settings reach the sender's config, the
// Replica sensor's options and, where each driver is built, the Orbbec streams
// and the RealSense camera's options (and, with sensor_ffs, FFS depth's), and
// the DeckLink stereo camera's (its cameras always; the driver's and FFS
// depth's where built) unchanged, and the live templates hold what they
// document. The codec's side is grid's example_config_test.cpp. No file
// assets, device or camera are opened.
#include <cstdio>
#include <filesystem>
#include <string>
#include <utility>

#include "codec_config.hpp"
#include "fuse_sender.hpp"
#include "replica_config.hpp"
#include "volumetric_kit/core/testing/test_check.hpp"
#include "volumetric_kit/grid/networking/stream_server.hpp"
#ifdef VR_TEST_ORBBEC_CONFIG
#include "orbbec_config.hpp"
#endif
#ifdef VR_TEST_REALSENSE_CONFIG
#include "realsense_camera.hpp"
#endif
#include "decklink_camera.hpp"

namespace vkc = volumetric_kit::core;

namespace cfg = volumetric_kit::core::config;
namespace vr = volumetric_kit::recon;
namespace sensor = volumetric_kit::sensor;
namespace vgrid = volumetric_kit::grid;

int main() {
  struct Example {
    const char* file;
    cfg::Application application;
  };
  const Example examples[] = {
      {"fuse_replica", cfg::Application::FuseReplica},
      {"codec_replica", cfg::Application::CodecReplica},
      {"fuse_orbbec", cfg::Application::FuseOrbbec},
      {"codec_orbbec", cfg::Application::FuseOrbbec},
      {"serve_orbbec", cfg::Application::FuseOrbbec},
      {"stream_orbbec", cfg::Application::FuseOrbbec},
      {"readback_orbbec", cfg::Application::FuseOrbbec},
      {"dump_orbbec", cfg::Application::FuseOrbbec},
      {"fuse_realsense", cfg::Application::FuseRealsense},
      {"fuse_realsense_ffs", cfg::Application::FuseRealsense},
      {"fuse_decklink", cfg::Application::FuseDecklink},
      {"fuse_render", cfg::Application::FuseRender},
      {"fuse_viewer", cfg::Application::FuseViewer},
      {"rig_viewer", cfg::Application::RigViewer},
  };
  for (const auto& example : examples) {
    const auto loaded = cfg::load(
        std::string(VR_CONFIG_FIXTURES) + "/" + example.file + ".json",
        example.application);
    if (!loaded)
      std::fprintf(stderr, "%s: %s\n", example.file,
                   loaded.status().message().c_str());
    CHECK(loaded);
    // codec_orbbec alone writes codec rows, and it and serve_orbbec frames;
    // serve_orbbec alone serves them, on the default port to every interface,
    // to the server's default count of viewers, a joiner waiting for the key
    // the stream makes for it.
    // Every live example fuses Dynamic, by core's default, the rest Classic.
    const bool coded = std::string(example.file) == "codec_orbbec";
    const bool served = std::string(example.file) == "serve_orbbec";
    const bool live = example.application == cfg::Application::FuseOrbbec ||
                      example.application == cfg::Application::FuseRealsense ||
                      example.application == cfg::Application::FuseDecklink ||
                      example.application == cfg::Application::RigViewer;
    CHECK(loaded->fusion.dynamic == live);
    CHECK(loaded->output.csv.empty() != coded);
    CHECK(loaded->output.stream.empty() != (coded || served));
    CHECK(loaded->output.serve == (served ? 47801 : 0));
    const vgrid::codec_stream::CodecOutputs outputs =
        vgrid_example::codec_outputs(loaded.value().output);
    CHECK(outputs.serve.has_value() == served);
    CHECK(!served || (outputs.serve->host.empty() &&
                      outputs.serve->port == vgrid::networking::kDefaultPort &&
                      outputs.serve->max_viewers ==
                          vgrid::networking::ServerOptions{}.max_viewers &&
                      !outputs.serve->replay_gop));
    // No scenario codes a true SDF: codec_mesh alone sets it, in code.
    const auto encoder = vgrid_example::encoder_config(loaded->codec);
    CHECK(encoder.ok() && !encoder->true_sdf);
  }

  // fuse_orbbec serves as many viewers as its scenario's output.max_viewers,
  // through the outputs it codes to.
  const auto many = cfg::parse(
      R"({"version":1,"operation":"fuse","source":{"kind":"live","sync_config":"sync.json"},"codec":{"intra_period":30},"output":{"serve":47801,"max_viewers":16}})",
      cfg::Application::FuseOrbbec);
  CHECK(many);
  const vgrid::codec_stream::CodecOutputs many_outputs =
      vgrid_example::set_codec_outputs(many.value());
  CHECK(many_outputs.serve && many_outputs.serve->port == 47801 &&
        many_outputs.serve->max_viewers == 16);

  // codec_replica codes every frame, then its sweep, at period 30 with U = R,
  // as the freeze measured it (DECISIONS, the 2026-10-17 freeze).
  const auto replica =
      cfg::load(std::string(VR_CONFIG_FIXTURES) + "/codec_replica.json",
                cfg::Application::CodecReplica);
  CHECK(replica && replica->codec.encode_every == 1 && replica->codec.sweep);
  CHECK(replica->codec.intra_period == 30 && replica->codec.update_size == 0);
  const auto replica_encoder = vgrid_example::encoder_config(replica->codec);
  CHECK(replica_encoder.ok() && replica_encoder->intra_period == 30);
  CHECK(replica_encoder->segment_size == 64 &&
        replica_encoder->resolved_update_size() ==
            replica_encoder->segment_size);

  const auto readback =
      cfg::load(std::string(VR_CONFIG_FIXTURES) + "/readback_orbbec.json",
                cfg::Application::FuseOrbbec);
  CHECK(readback && readback->operation == cfg::Operation::Stream);
  CHECK(readback->diagnostics.readback);
  CHECK(readback->diagnostics.dump_sets == 0);
  CHECK(readback->diagnostics.dump_directory.empty());
  // The first sets' JPEGs and luma, into a directory beside the scenario.
  const auto dump =
      cfg::load(std::string(VR_CONFIG_FIXTURES) + "/dump_orbbec.json",
                cfg::Application::FuseOrbbec);
  CHECK(dump && dump->operation == cfg::Operation::Stream);
  CHECK(dump->source.color_codec == cfg::ColorCodec::Mjpeg);
  CHECK(dump->diagnostics.readback && dump->diagnostics.dump_sets == 20);
  CHECK(dump->diagnostics.dump_directory ==
        (std::filesystem::path(VR_CONFIG_FIXTURES) / "results" / "dump")
            .lexically_normal()
            .string());
  const auto fusion =
      cfg::load(std::string(VR_CONFIG_FIXTURES) + "/fuse_orbbec.json",
                cfg::Application::FuseOrbbec);
  CHECK(fusion && !fusion->diagnostics.readback);
  // The rig coded set by set at 1 cm, fused Dynamic and freeing stale blocks
  // every 30 sets as rig_viewer does, at the codec templates' period; its
  // outputs beside the scenario.
  const auto coding =
      cfg::load(std::string(VR_CONFIG_FIXTURES) + "/codec_orbbec.json",
                cfg::Application::FuseOrbbec);
  CHECK(coding && coding->operation == cfg::Operation::Fuse);
  CHECK(coding->fusion.voxel_size_m == 0.01F &&
        coding->fusion.free_after == 30 && coding->fusion.max_weight == 20.F);
  CHECK(coding->codec.intra_period == 30 && !coding->codec.sweep);
  const auto results = std::filesystem::path(VR_CONFIG_FIXTURES) / "results";
  CHECK(coding->output.stream ==
        (results / "rig.vrst").lexically_normal().string());
  CHECK(coding->output.csv ==
        (results / "rig_frames.csv").lexically_normal().string());
  CHECK(coding->output.path ==
        (results / "rig_end.ply").lexically_normal().string());
  // The rig served live as it is coded and recorded, at the same period, for
  // 216000 camera frames: 36 minutes of four cameras at 25 fps.
  const auto serving =
      cfg::load(std::string(VR_CONFIG_FIXTURES) + "/serve_orbbec.json",
                cfg::Application::FuseOrbbec);
  CHECK(serving && serving->operation == cfg::Operation::Fuse);
  CHECK(serving->run.frames == 216000 && serving->fusion.voxel_size_m == 0.01F);
  CHECK(serving->codec.intra_period == 30 && !serving->codec.sweep);
  CHECK(serving->output.stream ==
        (results / "rig.vrst").lexically_normal().string());
  CHECK(serving->output.path ==
        (results / "rig_end.ply").lexically_normal().string());

  const auto dataset =
      cfg::parse(R"({
    "version": 1, "operation": "fuse",
    "source": {"kind": "dataset", "path": "room0",
               "camera_parameters": "intrinsics.json", "preload": true},
    "fusion": {"voxel_size_m": 0.01, "stride": 3,
               "min_depth_m": 0.3, "max_depth_m": 4.0},
    "run": {"frames": 17}
  })",
                 cfg::Application::FuseReplica, "/tmp/scenarios");
  CHECK(dataset);
  const auto source = vr_example::replica_options(dataset.value());
  CHECK(source.frame_limit == 17 && source.frame_stride == 3);
  CHECK(source.min_depth == 0.3F && source.max_depth == 4.F);
  CHECK(dataset.value().source.path == "/tmp/scenarios/room0");
  CHECK(dataset.value().source.camera_parameters ==
        "/tmp/scenarios/intrinsics.json");
  CHECK(dataset.value().fusion.truncation_m == 0.04F);

  // The sender fuses as `fusion` says and codes every code_every-th set.
  cfg::Config fusing;
  fusing.fusion.max_weight = 7.5F;
  fusing.fusion.dynamic = true;
  fusing.fusion.free_after = 12;
  const vr::sender::SetSender::Config sending =
      vr_example::sender_config(fusing, 3);
  CHECK(sending.max_weight == 7.5F && sending.free_after == 12);
  CHECK(sending.mode == vr::tsdf::IntegrationMode::Dynamic);
  CHECK(sending.code_every == 3);

#ifdef VR_TEST_ORBBEC_CONFIG
  const auto live = cfg::parse(R"({
    "version": 1, "operation": "stream",
    "source": {"kind": "live", "sync_config": "sync.json",
               "calibration": "poses.json", "color_codec": "mjpeg",
               "color_width": 3840, "color_height": 2160, "fps": 25},
    "run": {"frames": 60000}
  })",
                               cfg::Application::FuseOrbbec);
  CHECK(live);
  const auto streams = vr_example::orbbec_stream_options(live.value());
  CHECK(streams.color_codec == sensor::OrbbecColorCodec::Mjpeg);
  CHECK(streams.color_width == 3840 && streams.color_height == 2160);
  CHECK(streams.depth_width == 640 && streams.depth_height == 576);
  CHECK(streams.fps == 25);
  CHECK(streams.min_depth == 0.25F && streams.max_depth == 5.F);
  CHECK(streams.device == nullptr && streams.allocator == nullptr);

  // A live scenario that names no colour mode streams H.265 3840x2160 at 25
  // fps, depth 640x576: the driver's own defaults, and no colour controls
  // (the calibration carries those).
  for (const auto& [app, operation] :
       {std::pair{cfg::Application::FuseOrbbec, "fuse"},
        std::pair{cfg::Application::RigViewer, "view"}}) {
    const auto unnamed = cfg::parse(
        std::string(R"({"version": 1, "operation": ")") + operation +
            R"(", "source": {"kind": "live", "sync_config": "sync.json"}})",
        app);
    CHECK(unnamed);
    const auto mapped = vr_example::orbbec_stream_options(unnamed.value());
    const sensor::OrbbecStreamOptions driver;
    CHECK(mapped.color_codec == sensor::OrbbecColorCodec::Hevc);
    CHECK(mapped.color_width == 3840 && mapped.color_height == 2160);
    CHECK(mapped.depth_width == 640 && mapped.depth_height == 576);
    CHECK(mapped.fps == 25);
    CHECK(mapped.color_codec == driver.color_codec &&
          mapped.color_width == driver.color_width &&
          mapped.color_height == driver.color_height &&
          mapped.depth_width == driver.depth_width &&
          mapped.depth_height == driver.depth_height &&
          mapped.fps == driver.fps);
    CHECK(!mapped.color_controls);
  }
#endif
#ifdef VR_TEST_REALSENSE_CONFIG
  // fuse_realsense's template opens the only camera attached at the driver's
  // own modes, the live depth gate, its projector on and its colour at auto
  // exposure.
  const auto realsense =
      cfg::load(std::string(VR_CONFIG_FIXTURES) + "/fuse_realsense.json",
                cfg::Application::FuseRealsense);
  CHECK(realsense);
  const sensor::RealSenseSensor::Options template_options =
      vr_example::realsense_options(realsense.value());
  const sensor::RealSenseSensor::Options driver;
  CHECK(template_options.serial.empty());
  CHECK(template_options.depth_width == driver.depth_width &&
        template_options.depth_height == driver.depth_height &&
        template_options.color_width == driver.color_width &&
        template_options.color_height == driver.color_height &&
        template_options.fps == driver.fps);
  CHECK(template_options.min_depth == 0.25F &&
        template_options.max_depth == 5.F);
  CHECK(template_options.emitter && !template_options.color_controls);
  CHECK(realsense->output.path ==
        (results / "realsense.ply").lexically_normal().string());
  // Every field the profile takes reaches its option, none crossed.
  const auto named = cfg::parse(R"({
    "version": 1, "operation": "fuse",
    "source": {"kind": "live", "driver": "realsense", "serial": "215122251234",
               "color_width": 640, "color_height": 480,
               "depth_width": 848, "depth_height": 100, "fps": 15},
    "fusion": {"min_depth_m": 0.4, "max_depth_m": 3.5}
  })",
                                cfg::Application::FuseRealsense);
  CHECK(named);
  const sensor::RealSenseSensor::Options opened =
      vr_example::realsense_options(named.value());
  CHECK(opened.serial == "215122251234");
  CHECK(opened.color_width == 640 && opened.color_height == 480);
  CHECK(opened.depth_width == 848 && opened.depth_height == 100);
  CHECK(opened.fps == 15);
  CHECK(opened.min_depth == 0.4F && opened.max_depth == 3.5F);
  CHECK(opened.emitter);
  CHECK(vr_example::check_realsense_depth(named.value()).ok());
  CHECK(vr_example::realsense_depth_line(named.value()) ==
        "depth: the camera's own, projector on\n");
  // The projector off when the scenario says so.
  const auto dark = cfg::parse(
      R"({"version":1,"operation":"fuse","source":{"kind":"live","emitter":false}})",
      cfg::Application::FuseRealsense);
  CHECK(dark && !vr_example::realsense_options(dark.value()).emitter);
  CHECK(vr_example::realsense_depth_line(dark.value()) ==
        "depth: the camera's own, projector off\n");

  // fuse_realsense_ffs's template: FFS on the infrared pair at 960x544, the
  // projector off, the camera's colour and rate as the driver's own, and the
  // model and its plans beside the scenario.
  const auto ffs =
      cfg::load(std::string(VR_CONFIG_FIXTURES) + "/fuse_realsense_ffs.json",
                cfg::Application::FuseRealsense);
  CHECK(ffs && ffs->source.depth == cfg::DepthSource::Ffs);
  const sensor::RealSenseStereo::Options pair =
      vr_example::realsense_stereo_options(ffs.value());
  const sensor::RealSenseStereo::Options pair_driver;
  CHECK(pair.serial.empty() && !pair.emitter && !pair.color_controls);
  CHECK(pair.infrared_width == pair_driver.infrared_width &&
        pair.infrared_height == pair_driver.infrared_height &&
        pair.color_width == pair_driver.color_width &&
        pair.color_height == pair_driver.color_height &&
        pair.fps == pair_driver.fps);
  const auto models = std::filesystem::path(VR_CONFIG_FIXTURES) / "models";
  CHECK(ffs->source.ffs_model ==
        (models / "ffs_23-36-37_960x544.onnx").lexically_normal().string());
  CHECK(ffs->source.ffs_plan_cache ==
        (models / "ffs_plans").lexically_normal().string());
  CHECK(ffs->output.path ==
        (results / "realsense_ffs.ply").lexically_normal().string());
  CHECK(vr_example::realsense_depth_line(ffs.value()) ==
        "depth: FFS on the infrared cameras at 960x544 (" +
            ffs->source.ffs_model + "), projector off\n");
  // Every field the pair takes reaches its option, none crossed.
  const auto lit = cfg::parse(R"({
    "version": 1, "operation": "fuse",
    "source": {"kind": "live", "serial": "215122251234", "depth": "ffs",
               "emitter": true, "color_width": 640, "color_height": 480,
               "fps": 15, "depth_width": 1280, "depth_height": 736,
               "ffs_model": "/m/ffs.onnx", "ffs_plan_cache": "/m/plans"},
    "fusion": {"min_depth_m": 0.4, "max_depth_m": 3.5}
  })",
                              cfg::Application::FuseRealsense);
  CHECK(lit);
  const sensor::RealSenseStereo::Options lit_pair =
      vr_example::realsense_stereo_options(lit.value());
  CHECK(lit_pair.serial == "215122251234" && lit_pair.emitter);
  CHECK(lit_pair.color_width == 640 && lit_pair.color_height == 480);
  CHECK(lit_pair.fps == 15);
  CHECK(lit_pair.infrared_width == pair_driver.infrared_width &&
        lit_pair.infrared_height == pair_driver.infrared_height);
#ifdef VR_EXAMPLE_FFS
  // With sensor_ffs: FFS is opened, the pair rectified to the depth size as
  // luma at alpha 0, the model, its plans and the depth gate passed on, and
  // the pair's measured disparity noise stated, so its frames state their
  // depth noise (sensor's FfsStereo defaults it to unknown).
  CHECK(vr_example::check_realsense_depth(ffs.value()).ok());
  const sensor::StereoDepthSensor::Options depth =
      vr_example::realsense_ffs_options(ffs.value());
  CHECK(depth.rectify.size.width == 960 && depth.rectify.size.height == 544);
  CHECK(depth.rectify.alpha == 0.0f);
  CHECK(depth.rectify.output == sensor::StereoRectifier::Output::Luma8);
  CHECK(depth.ffs.model.string() == ffs->source.ffs_model);
  CHECK(depth.ffs.plan_cache.string() == ffs->source.ffs_plan_cache);
  CHECK(depth.ffs.min_depth == 0.25F && depth.ffs.max_depth == 5.F);
  CHECK(sensor::FfsStereo::Options{}.disparity_sigma == 0.0f);
  CHECK(depth.ffs.disparity_sigma == 0.04f);
  const sensor::StereoDepthSensor::Options lit_depth =
      vr_example::realsense_ffs_options(lit.value());
  CHECK(lit_depth.rectify.size.width == 1280 &&
        lit_depth.rectify.size.height == 736);
  CHECK(lit_depth.ffs.model.string() == "/m/ffs.onnx" &&
        lit_depth.ffs.plan_cache.string() == "/m/plans");
  CHECK(lit_depth.ffs.min_depth == 0.4F && lit_depth.ffs.max_depth == 3.5F);
  CHECK(lit_depth.ffs.disparity_sigma == 0.04f);
#else
  // Without it, FFS is refused, saying how to build it.
  const vkc::Status refused = vr_example::check_realsense_depth(ffs.value());
  CHECK(refused.domain() == vkc::Status::Code::Unsupported);
  CHECK(refused.message().find("VS_WITH_FFS=ON") != std::string::npos);
#endif
#endif

  // fuse_decklink's template: the stereo camera the provisional calibration
  // made, bm-1 on SDI 2 its left camera and bm-2 on SDI 1 its right one,
  // free-running at the cameras' default mode, so left and right may be
  // half a frame apart; the user's calibration file under rigs/, and FFS's
  // model and plans under models/, beside the scenario.
  const auto decklink =
      cfg::load(std::string(VR_CONFIG_FIXTURES) + "/fuse_decklink.json",
                cfg::Application::FuseDecklink);
  CHECK(decklink);
  const vr_example::DecklinkCameras shipped =
      vr_example::decklink_cameras(decklink.value());
  CHECK(shipped.left.id == "bm-1" && shipped.left.input == "SDI 2");
  CHECK(shipped.right.id == "bm-2" && shipped.right.input == "SDI 1");
  CHECK(decklink->source.max_pair_skew_us == 16666);
  CHECK(decklink->source.calibration ==
        (std::filesystem::path(VR_CONFIG_FIXTURES) / "rigs" / "decklink" /
         "array_calibration.json")
            .lexically_normal()
            .string());
  const auto ffs_models = std::filesystem::path(VR_CONFIG_FIXTURES) / "models";
  CHECK(decklink->source.ffs_model ==
        (ffs_models / "ffs_23-36-37_960x544.onnx").lexically_normal().string());
  CHECK(decklink->source.ffs_plan_cache ==
        (ffs_models / "ffs_plans").lexically_normal().string());
  CHECK(decklink->output.path ==
        (results / "decklink.ply").lexically_normal().string());
  // The stereo camera is the pair's two cameras, by id, whatever order
  // source.cameras lists them in; every field reaches its option.
  const auto listed = cfg::parse(R"({
    "version": 1, "operation": "fuse",
    "source": {"kind": "live", "driver": "decklink",
               "cameras": [{"id": "east", "input": "SDI 3"},
                           {"id": "west", "input": "SDI 4"}],
               "pairs": [{"left": "west", "right": "east"}],
               "color_width": 1920, "color_height": 1080, "fps": 25,
               "depth_width": 1280, "depth_height": 736,
               "max_pair_skew_us": 500, "calibration": "/r/rig.json",
               "ffs_model": "/m/ffs.onnx", "ffs_plan_cache": "/m/plans"},
    "fusion": {"min_depth_m": 0.8, "max_depth_m": 4.5}
  })",
                                 cfg::Application::FuseDecklink);
  CHECK(listed);
  const vr_example::DecklinkCameras west_east =
      vr_example::decklink_cameras(listed.value());
  CHECK(west_east.left.id == "west" && west_east.left.input == "SDI 4");
  CHECK(west_east.right.id == "east" && west_east.right.input == "SDI 3");
  // The camera opens only with the driver and FFS both built; a build
  // without either refuses, naming each part missing and how to build it.
  const vkc::Status build = vr_example::check_decklink_build();
#if defined(VR_EXAMPLE_DECKLINK) && defined(VR_EXAMPLE_FFS)
  CHECK(build.ok());
#else
  CHECK(build.domain() == vkc::Status::Code::Unsupported);
#ifdef VR_EXAMPLE_DECKLINK
  CHECK(build.message().find("VS_WITH_DECKLINK") == std::string::npos);
#else
  CHECK(build.message().find("VS_WITH_DECKLINK=ON") != std::string::npos);
#endif
#ifdef VR_EXAMPLE_FFS
  CHECK(build.message().find("VS_WITH_FFS") == std::string::npos);
#else
  CHECK(build.message().find("VS_WITH_FFS=ON") != std::string::npos);
#endif
#endif
#ifdef VR_EXAMPLE_DECKLINK
  // The driver's options: the connectors as the pair names them, the mode,
  // the bound in nanoseconds and the calibration; the start's wait the
  // driver's own.
  sensor::StereoCalibration lenses;
  lenses.left.size = lenses.right.size = {1920, 1080};
  lenses.left_to_right[3] = {-0.4, 0.0, 0.0, 1.0};
  const sensor::DecklinkStereo::Options wired =
      vr_example::decklink_options(listed.value(), lenses);
  const sensor::DecklinkStereo::Options driver_defaults;
  CHECK(wired.left.id == "west" && wired.left.input == "SDI 4");
  CHECK(wired.right.id == "east" && wired.right.input == "SDI 3");
  CHECK(wired.width == 1920 && wired.height == 1080 && wired.fps == 25);
  CHECK(wired.max_skew_ns == 500'000);
  CHECK(wired.calibration && wired.calibration->left.size.width == 1920 &&
        wired.calibration->left_to_right[3].x == -0.4);
  CHECK(wired.lock_timeout == driver_defaults.lock_timeout);
  // The template's are the driver's defaults: 2160p30, half a frame apart.
  const sensor::DecklinkStereo::Options shipped_options =
      vr_example::decklink_options(decklink.value(), lenses);
  CHECK(shipped_options.left.input == "SDI 2" &&
        shipped_options.right.input == "SDI 1");
  CHECK(shipped_options.width == driver_defaults.width &&
        shipped_options.height == driver_defaults.height &&
        shipped_options.fps == driver_defaults.fps &&
        shipped_options.max_skew_ns == driver_defaults.max_skew_ns);
#endif
#ifdef VR_EXAMPLE_FFS
  // FFS depth on the DeckLink cameras: R'G'B' views at the depth size and
  // alpha 0, the model, its plans and the depth gate passed on, and the
  // cameras' disparity noise stated, so every frame states its depth noise
  // (Dynamic refuses a frame without one).
  const sensor::StereoDepthSensor::Options decklink_depth =
      vr_example::decklink_ffs_options(decklink.value());
  CHECK(decklink_depth.rectify.output ==
        sensor::StereoRectifier::Output::Rgba8);
  CHECK(decklink_depth.rectify.size.width == 960 &&
        decklink_depth.rectify.size.height == 544);
  CHECK(decklink_depth.rectify.alpha == 0.0f);
  CHECK(decklink_depth.ffs.model.string() == decklink->source.ffs_model);
  CHECK(decklink_depth.ffs.plan_cache.string() ==
        decklink->source.ffs_plan_cache);
  CHECK(decklink_depth.ffs.min_depth == 0.5F &&
        decklink_depth.ffs.max_depth == 5.F);
  CHECK(decklink_depth.ffs.disparity_sigma == 0.08f);
  const sensor::StereoDepthSensor::Options listed_depth =
      vr_example::decklink_ffs_options(listed.value());
  CHECK(listed_depth.rectify.size.width == 1280 &&
        listed_depth.rectify.size.height == 736);
  CHECK(listed_depth.ffs.model.string() == "/m/ffs.onnx" &&
        listed_depth.ffs.plan_cache.string() == "/m/plans");
  CHECK(listed_depth.ffs.min_depth == 0.8F &&
        listed_depth.ffs.max_depth == 4.5F);
#endif
  return 0;
}
