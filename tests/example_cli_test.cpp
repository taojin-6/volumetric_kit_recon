// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The examples' one command-line parser (examples/common/cli.hpp) and the
// flag sets every example declares through it: numbers read whole and
// finite, a flag's value taken whatever it looks like, unknown and missing
// arguments refused with the usage line, required flags and exclusive
// switches, checks run after the arguments, and the fusion, Replica, codec
// and Orbbec flags' defaults and validation. Host only.

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <optional>
#include <string>
#include <vector>

#include "cli.hpp"
#include "codec_flags.hpp"
#include "fusion_flags.hpp"
#include "replica_flags.hpp"
#if VR_TEST_ORBBEC_FLAGS
#include "orbbec_flags.hpp"
#endif

namespace vkc = volumetric_kit::core;
namespace codec = volumetric_kit::recon::codec;

#define CHECK(cond)                                                        \
  do {                                                                     \
    if (!(cond)) {                                                         \
      std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
      return 1;                                                            \
    }                                                                      \
  } while (0)

namespace {

// Parse `args` (the program name is prepended) with `cli`.
vkc::Status parse(const vr_example::Cli& cli,
                  const std::vector<const char*>& args) {
  std::vector<const char*> argv{"prog"};
  argv.insert(argv.end(), args.begin(), args.end());
  return cli.parse(static_cast<int>(argv.size()), argv.data());
}

bool has(const vkc::Status& s, const std::string& text) {
  return s.message().find(text) != std::string::npos;
}

int numbers() {
  double d = 7.0;
  CHECK(vr_example::parse_number("--x", "0.25", d).ok() && d == 0.25);
  CHECK(vr_example::parse_number("--x", "-1e3", d).ok() && d == -1000.0);
  // The whole text, finite, in range; a refusal leaves the value alone.
  for (const char* bad : {"", "10x", "0.5 ", "nan", "inf", "1e999", "x"}) {
    d = 7.0;
    const vkc::Status s = vr_example::parse_number("--x", bad, d);
    CHECK(s.domain() == vkc::Status::Code::InvalidArgument);
    CHECK(has(s, "--x") && d == 7.0);
  }
  float f = 0.0f;
  CHECK(!vr_example::parse_number("--x", "1e39", f).ok());
  CHECK(vr_example::parse_number("--x", "-2.5", f).ok() && f == -2.5f);
  int i = 3;
  CHECK(vr_example::parse_number("--n", "-42", i).ok() && i == -42);
  for (const char* bad : {"10x", "", "1.5", "2147483648", "0x10"}) {
    i = 3;
    CHECK(!vr_example::parse_number("--n", bad, i).ok() && i == 3);
  }
  std::uint32_t u = 9;
  CHECK(vr_example::parse_number("--n", "4294967295", u).ok() &&
        u == 4294967295u);
  CHECK(!vr_example::parse_number("--n", "-1", u).ok() && u == 4294967295u);
  CHECK(!vr_example::parse_number("--n", "4294967296", u).ok());
  return 0;
}

int parser() {
  struct {
    std::string scene, out = "a.ply", up;
    float voxel = 0.02f;
    int frames = 5;
    bool preload = false, texture = true, lit = false;
    std::optional<float> depth;
  } o;
  int checks = 0;
  vr_example::Cli cli("prog");
  cli.positional("scene_dir", o.scene)
      .option({"-o", "--out"}, "out.ply", o.out)
      .option("--voxel", "m", o.voxel)
      .option("--frames", "N", o.frames, 1)
      .option("--depth", "m", o.depth)
      .option("--up", "axis", o.up)
      .flag("--preload", o.preload)
      .flag("--no-texture", o.texture, false)
      .choice<bool>({{"--lit", true}, {"--unlit", false}}, o.lit)
      .check([&checks] {
        ++checks;
        return vkc::Status{};
      });

  // Every kind of declaration sets its variable; an alias is the flag; a
  // value is taken however it starts.
  CHECK(parse(cli, {"room0", "--out", "b.ply", "--voxel", "0.01", "--frames",
                    "7", "--depth", "3", "--up", "-y", "--preload",
                    "--no-texture", "--lit", "--lit"})
            .ok());
  CHECK(o.scene == "room0" && o.out == "b.ply" && o.voxel == 0.01f);
  CHECK(o.frames == 7 && o.depth == 3.0f && o.up == "-y");
  CHECK(o.preload && !o.texture && o.lit && checks == 1);
  CHECK(parse(cli, {"-o", "c.ply", "room0"}).ok() && o.out == "c.ply");
  // Each parse stands alone: the earlier --lit does not refuse --unlit.
  CHECK(parse(cli, {"room0", "--unlit"}).ok() && !o.lit);

  // Each refusal names its program and its cause, then the usage line.
  const auto refused = [&](std::vector<const char*> args, const char* why) {
    const vkc::Status s = parse(cli, args);
    return s.domain() == vkc::Status::Code::InvalidArgument &&
           s.message().rfind(std::string("prog: ") + why, 0) == 0 &&
           has(s, "\nusage: prog <scene_dir>");
  };
  CHECK(refused({}, "needs <scene_dir>"));
  CHECK(refused({"a", "b"}, "unexpected argument: b"));
  CHECK(refused({"a", "--bogus"}, "unknown flag: --bogus"));
  CHECK(refused({"a", "--voxel"}, "--voxel needs a value"));
  CHECK(refused({"a", "--frames", "10x"}, "--frames: not an integer: 10x"));
  CHECK(refused({"a", "--frames", "0"}, "--frames must be >= 1"));
  CHECK(refused({"a", "--voxel", "nan"}, "--voxel: not a finite number"));
  CHECK(refused({"a", "--lit", "--unlit"}, "--lit or --unlit, not both"));

  // A check runs after every argument, and its refusal is the parse's.
  vr_example::Cli checked("prog");
  int order = 0;
  checked.option("--frames", "N", o.frames).check([&] {
    order = o.frames;
    return vkc::Status::invalid_argument("frames is " +
                                         std::to_string(o.frames));
  });
  const vkc::Status late = parse(checked, {"--frames", "3"});
  CHECK(order == 3 && has(late, "prog: frames is 3\nusage: prog"));

  // A required flag is refused when absent and shown without brackets.
  vr_example::Cli required("prog");
  std::string rig;
  required.option("--rig", "sync.json", rig).require("--rig");
  CHECK(has(parse(required, {}), "prog: needs --rig"));
  CHECK(parse(required, {"--rig", "r.json"}).ok() && rig == "r.json");
  CHECK(required.usage() == "usage: prog --rig sync.json");
  return 0;
}

int usage() {
  std::string s;
  bool b = false;
  int n = 0;
  vr_example::Cli cli("tool");
  cli.positional("in", s)
      .option({"-o", "--out"}, "path", s)
      .flag("--fast", b)
      .choice<int>({{"--one", 1}, {"--two", 2}}, n)
      .option("--a-rather-long-flag-name", "value", s)
      .option("--another-long-flag", "value", s)
      .option("--third", "value", s);
  CHECK(cli.usage() ==
        "usage: tool <in> [-o path] [--fast] [--one | --two]\n"
        "            [--a-rather-long-flag-name value] "
        "[--another-long-flag value]\n"
        "            [--third value]");
  return 0;
}

int fusion_flags() {
  {
    vr_example::FusionFlags f(0.02f);
    vr_example::Cli cli("prog");
    f.add_to(cli);
    CHECK(parse(cli, {}).ok());
    // The band defaults to four voxels, the gate to the source's own.
    CHECK(f.voxel == 0.02f && f.trunc == vr_example::default_trunc(0.02f));
    CHECK(f.trunc == 0.08f && !f.min_depth && !f.max_depth);
    CHECK(f.max_weight == 20.0f);
    struct {
      float min_depth = 0.1f, max_depth = 8.0f;
    } source;
    f.apply_depth_gate(source);
    CHECK(source.min_depth == 0.1f && source.max_depth == 8.0f);
  }
  {
    vr_example::FusionFlags f(0.02f);
    vr_example::Cli cli("prog");
    f.add_to(cli);
    CHECK(parse(cli, {"--voxel", "0.01", "--trunc", "0.05", "--max-depth", "3",
                      "--max-weight", "50"})
              .ok());
    CHECK(f.voxel == 0.01f && f.trunc == 0.05f && f.max_weight == 50.0f);
    struct {
      float min_depth = 0.1f, max_depth = 8.0f;
    } source;
    f.apply_depth_gate(source);
    CHECK(source.min_depth == 0.1f && source.max_depth == 3.0f);
  }
  const auto refused = [](std::vector<const char*> args, const char* why) {
    vr_example::FusionFlags f(0.02f);
    vr_example::Cli cli("prog");
    f.add_to(cli);
    return has(parse(cli, args), why);
  };
  CHECK(refused({"--voxel", "0"}, "--voxel must be > 0"));
  CHECK(refused({"--voxel", "-0.01"}, "--voxel must be > 0"));
  CHECK(refused({"--voxel", "3e38"}, "--voxel or --trunc is too large"));
  CHECK(refused({"--min-depth", "0"}, "must be > 0"));
  CHECK(refused({"--max-depth", "-1"}, "must be > 0"));
  CHECK(refused({"--min-depth", "2", "--max-depth", "2"},
                "--min-depth must be below --max-depth"));
  CHECK(refused({"--max-weight", "0"}, "--max-weight must be > 0"));
  CHECK(refused({"--trunc", "inf"}, "--trunc: not a finite number"));
  return 0;
}

int replica_flags() {
  vr_example::ReplicaFlags r(400);
  vr_example::Cli cli("prog");
  r.add_to(cli);
  CHECK(has(parse(cli, {}), "needs <scene_dir>"));
  CHECK(parse(cli, {"data/room0"}).ok());
  CHECK(r.max_frames == 400 && !r.preload);
  CHECK(r.cam_params_path() == "data/room0/../cam_params.json");
  CHECK(parse(cli, {"room0", "--cam-params", "c.json", "--max-frames", "9",
                    "--preload"})
            .ok());
  CHECK(r.cam_params_path() == "c.json" && r.max_frames == 9 && r.preload);
  CHECK(has(parse(cli, {"room0", "--max-frames", "0"}),
            "--max-frames must be >= 1"));
  return 0;
}

int codec_flags() {
  {
    vr_example::CodecFlags c;
    vr_example::Cli cli("prog");
    c.add_to(cli);
    CHECK(parse(cli, {"--k", "84", "--step", "0.1", "--quant-table", "band",
                      "--entropy", "host", "--segment-size", "16"})
              .ok());
    CHECK(c.config.params.coefficient_count == 84u);
    CHECK(c.config.params.quantization_scale == 0.1f);
    CHECK(c.config.entropy == codec::EntropyCoding::kHost);
    CHECK(c.config.segment_size == 16u && c.quant_table == "band");
    // The table was applied: band weights rise with total frequency.
    CHECK(c.config.params.quantization_weights[0] == 1.0f);
    CHECK(c.config.params.quantization_weights[1] == 1.25f);
  }
  const auto refused = [](std::vector<const char*> args, const char* why) {
    vr_example::CodecFlags c;
    vr_example::Cli cli("prog");
    c.add_to(cli);
    return has(parse(cli, args), why);
  };
  CHECK(refused({"--k", "0"}, "--k must be >= 1"));
  CHECK(refused({"--k", "513"}, "coefficient_count"));
  CHECK(refused({"--segment-size", "0"}, "--segment-size must be >= 1"));
  CHECK(refused({"--entropy", "gpu"}, "--entropy needs auto, host or device"));
  CHECK(refused({"--quant-table", "jpeg"}, "unknown quantization table"));
  return 0;
}

#if VR_TEST_ORBBEC_FLAGS
int orbbec_flags() {
  {
    vr_example::OrbbecFlags c;
    vr_example::Cli cli("prog");
    c.add_to(cli);
    CHECK(parse(cli, {}).ok());
    CHECK(c.hevc && c.color_width == 0 && c.fps == 0 && c.rig.empty());
    CHECK(parse(cli, {"--rig", "s.json", "--apply-sync", "--mjpeg", "--color",
                      "3840x2160", "--fps", "25", "--calibration", "c.json"})
              .ok());
    CHECK(!c.hevc && c.apply_sync && c.rig == "s.json");
    CHECK(c.calibration == "c.json" && c.fps == 25u);
    CHECK(c.color_width == 3840u && c.color_height == 2160u);
  }
  const auto refused = [](std::vector<const char*> args, const char* why) {
    vr_example::OrbbecFlags c;
    vr_example::Cli cli("prog");
    c.add_to(cli);
    return has(parse(cli, args), why);
  };
  for (const char* bad : {"1920", "1920x", "0x1080", "1920x1080p", "x"}) {
    CHECK(refused({"--color", bad}, "--color needs WxH"));
  }
  CHECK(refused({"--fps", "0"}, "--fps must be >= 1"));
  CHECK(refused({"--hevc", "--mjpeg"}, "--hevc or --mjpeg, not both"));
  CHECK(refused({"--apply-sync"}, "--apply-sync needs --rig"));
  return 0;
}
#endif

}  // namespace

int main() {
  CHECK(numbers() == 0);
  CHECK(parser() == 0);
  CHECK(usage() == 0);
  CHECK(fusion_flags() == 0);
  CHECK(replica_flags() == 0);
  CHECK(codec_flags() == 0);
#if VR_TEST_ORBBEC_FLAGS
  CHECK(orbbec_flags() == 0);
#endif
  std::puts("example_cli: OK");
  return 0;
}
