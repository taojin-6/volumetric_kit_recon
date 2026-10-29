// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The fusing examples' side of the integration boundary: every fusing
// template loads and maps into the codec's runtime options (grid's
// codec_config.hpp), core-validated settings reach the sender's config, the
// Replica sensor's options and, where the driver is built, the Orbbec streams
// unchanged, and the live templates hold what they document. The codec's side
// is grid's example_config_test.cpp. No file assets, device or camera are
// opened.
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
  return 0;
}
