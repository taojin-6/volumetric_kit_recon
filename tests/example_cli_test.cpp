// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The examples' one command-line parser (examples/common/cli.hpp) and the
// flag sets every example declares through it: numbers read whole and
// finite, a flag's value taken whatever it looks like, unknown and missing
// arguments refused with the usage line, required flags, one-of groups and
// exclusions, checks run after the arguments, the voxel rule, and the fusion,
// Replica, codec, Orbbec and fuse_orbbec flags' defaults and validation, among
// them --no-fuse refusing the flags only fusion uses. Host only.

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <iterator>
#include <optional>
#include <string>
#include <vector>

#include "cli.hpp"
#include "codec_flags.hpp"
#include "fusion_flags.hpp"
#include "grid_layout.hpp"
#include "replica_flags.hpp"
#include "test_check.hpp"
#if VR_TEST_ORBBEC_FLAGS
#include "fuse_orbbec_flags.hpp"
#include "orbbec_flags.hpp"
#endif

namespace vkc = volumetric_kit::core;
namespace codec = volumetric_kit::recon::codec;

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
  for (const char* bad :
       {"", "10x", "0.5 ", " 0.5", "nan", "inf", "1e999", "x"}) {
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
  for (const char* bad : {"10x", "", " 5", "1.5", "2147483648", "0x10"}) {
    i = 3;
    CHECK(!vr_example::parse_number("--n", bad, i).ok() && i == 3);
  }
  std::uint32_t u = 9;
  CHECK(vr_example::parse_number("--n", "4294967295", u).ok() &&
        u == 4294967295u);
  // A count is digits only: no sign, no white space.
  for (const char* bad : {"-1", "-0", "+1", " 1", "4294967296", "1e3", ""}) {
    CHECK(!vr_example::parse_number("--n", bad, u).ok() && u == 4294967295u);
  }
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

  // A one_of group of flags that take values, declared apart: one of them,
  // the last when repeated; a second, different one is refused.
  vr_example::Cli grouped("prog");
  std::string serial;
  grouped.option("--serial", "SN", serial)
      .option("--frames", "N", o.frames)
      .option("--rig", "sync.json", rig)
      .one_of({"--serial", "--rig"});
  CHECK(parse(grouped, {}).ok());
  CHECK(parse(grouped, {"--rig", "a.json", "--rig", "b.json"}).ok() &&
        rig == "b.json");
  CHECK(has(parse(grouped, {"--serial", "1", "--rig", "r.json"}),
            "prog: --serial or --rig, not both"));
  CHECK(
      has(parse(grouped, {"--rig", "r.json", "--frames", "2", "--serial", "1"}),
          "prog: --serial or --rig, not both"));
  // Required, the group needs one of its flags.
  grouped.require("--rig");
  CHECK(
      has(parse(grouped, {"--frames", "2"}), "prog: needs --serial or --rig"));
  CHECK(parse(grouped, {"--serial", "7"}).ok() && serial == "7");

  // A flag that excludes others refuses each, in either order and under any
  // spelling, naming its first; each parse stands alone, and the others still
  // parse together.
  vr_example::Cli modes("prog");
  bool stream = false;
  modes.flag("--stream", stream)
      .option("--voxel", "m", o.voxel)
      .option({"-o", "--out"}, "out.ply", o.out)
      .excludes("--stream", {"--voxel", "-o"});
  CHECK(parse(modes, {"--stream"}).ok() && stream);
  CHECK(parse(modes, {"--voxel", "0.5", "--out", "d.ply"}).ok());
  CHECK(has(parse(modes, {"--stream", "--voxel", "1"}),
            "prog: --stream refuses --voxel\nusage: prog"));
  CHECK(has(parse(modes, {"--voxel", "1", "--stream"}),
            "prog: --stream refuses --voxel"));
  CHECK(has(parse(modes, {"--out", "e.ply", "--stream"}),
            "prog: --stream refuses -o"));
  CHECK(modes.usage() == "usage: prog [--stream] [--voxel m] [-o out.ply]");
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

  // A group is shown where its first flag was declared: in brackets when
  // optional, in parentheses when required.
  vr_example::Cli grouped("tool");
  grouped.option("--serial", "SN", s)
      .option("--frames", "N", n)
      .option("--rig", "sync.json", s)
      .one_of({"--serial", "--rig"});
  CHECK(grouped.usage() ==
        "usage: tool [--serial SN | --rig sync.json] [--frames N]");
  grouped.require("--serial");
  CHECK(grouped.usage() ==
        "usage: tool (--serial SN | --rig sync.json) [--frames N]");
  return 0;
}

int fusion_flags() {
  {
    vr_example::FusionFlags f(0.02f);
    vr_example::Cli cli("prog");
    f.add_to(cli);
    // kNames is every flag add_to declares, in its order, so a mode that
    // fuses nothing refuses them all (excluded_by).
    const std::string line = cli.usage();
    std::vector<std::string> declared;
    for (std::size_t at = line.find("[-"); at != std::string::npos;
         at = line.find("[-", at + 1)) {
      declared.push_back(line.substr(at + 1, line.find(' ', at) - at - 1));
    }
    CHECK(declared ==
          std::vector<std::string>(std::begin(vr_example::FusionFlags::kNames),
                                   std::end(vr_example::FusionFlags::kNames)));
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
  CHECK(refused({"--voxel", "3e38"}, "--voxel is too large"));
  CHECK(
      refused({"--voxel", "3e38", "--trunc", "0.05"}, "--voxel is too large"));
  CHECK(refused({"--min-depth", "0"}, "must be > 0"));
  CHECK(refused({"--max-depth", "-1"}, "must be > 0"));
  CHECK(refused({"--min-depth", "2", "--max-depth", "2"},
                "--min-depth must be below --max-depth"));
  CHECK(refused({"--max-weight", "0"}, "--max-weight must be > 0"));
  CHECK(refused({"--trunc", "inf"}, "--trunc: not a finite number"));
  return 0;
}

int voxel_rule() {
  CHECK(vr_example::check_voxel(0.01f).ok());
  CHECK(vr_example::check_voxel(8e37f).ok());  // a band of 3.2e38
  for (const float bad : {0.0f, -0.01f, std::nanf("")}) {
    CHECK(has(vr_example::check_voxel(bad), "--voxel must be > 0"));
  }
  CHECK(has(vr_example::check_voxel(9e37f), "--voxel is too large"));
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
  for (const char* bad :
       {"1920", "1920x", "0x1080", "1920x1080p", "x", "-1x1080", "1920x 1080",
        "+1920x1080", " 1920x1080", "1920x1080x2"}) {
    CHECK(refused({"--color", bad}, "--color needs WxH"));
  }
  CHECK(refused({"--fps", "0"}, "--fps must be >= 1"));
  CHECK(refused({"--hevc", "--mjpeg"}, "--hevc or --mjpeg, not both"));
  CHECK(refused({"--apply-sync"}, "--apply-sync needs --rig"));
  return 0;
}

int fuse_orbbec_flags() {
  {
    vr_example::FuseOrbbecFlags f;
    vr_example::Cli cli("prog");
    f.add_to(cli);
    CHECK(parse(cli, {}).ok());
    CHECK(!f.no_fuse && f.frames == 300 && f.out == "fuse_orbbec.ply");
    CHECK(f.serial.empty() && !f.host_clock && f.fusion.voxel == 0.02f);
    // The rig streamed at the calibration app's modes, fusing nothing.
    CHECK(parse(cli, {"--rig", "s.json", "--no-fuse", "--mjpeg", "--color",
                      "3840x2160", "--fps", "25", "--frames", "60000"})
              .ok());
    CHECK(f.no_fuse && f.frames == 60000 && f.cameras.rig == "s.json");
    CHECK(!f.cameras.hevc && f.cameras.fps == 25u &&
          f.cameras.color_width == 3840u);
  }
  {
    // One camera, its clock and its pose are a stream's too.
    vr_example::FuseOrbbecFlags f;
    vr_example::Cli cli("prog");
    f.add_to(cli);
    CHECK(parse(cli, {"--serial", "7", "--host-clock", "--calibration",
                      "c.json", "--no-fuse"})
              .ok());
    CHECK(f.serial == "7" && f.host_clock && f.no_fuse);
  }
  const auto refused = [](std::vector<const char*> args, const char* why) {
    vr_example::FuseOrbbecFlags f;
    vr_example::Cli cli("prog");
    f.add_to(cli);
    return has(parse(cli, args), why);
  };
  // Each flag that only fusion uses (every one FusionFlags declares, and
  // -o), on either side of --no-fuse; each is taken without it.
  std::vector<std::string> fusion_only(
      std::begin(vr_example::FusionFlags::kNames),
      std::end(vr_example::FusionFlags::kNames));
  fusion_only.insert(fusion_only.end(), {"-o", "--out"});
  for (const std::string& flag : fusion_only) {
    const std::string why =
        "prog: --no-fuse refuses " + (flag == "--out" ? "-o" : flag) + "\n";
    CHECK(refused({"--no-fuse", flag.c_str(), "0.5"}, why.c_str()));
    CHECK(refused({flag.c_str(), "0.5", "--no-fuse"}, why.c_str()));
    vr_example::FuseOrbbecFlags f;
    vr_example::Cli cli("prog");
    f.add_to(cli);
    CHECK(parse(cli, {flag.c_str(), "0.5"}).ok());
  }
  CHECK(refused({"--serial", "7", "--rig", "s.json"},
                "--serial or --rig, not both"));
  CHECK(refused({"--rig", "s.json", "--host-clock"},
                "--host-clock is for one camera"));
  CHECK(refused({"--no-fuse", "--frames", "0"}, "--frames must be >= 1"));
  return 0;
}
#endif

}  // namespace

int main() {
  CHECK(numbers() == 0);
  CHECK(parser() == 0);
  CHECK(usage() == 0);
  CHECK(fusion_flags() == 0);
  CHECK(voxel_rule() == 0);
  CHECK(replica_flags() == 0);
  CHECK(codec_flags() == 0);
#if VR_TEST_ORBBEC_FLAGS
  CHECK(orbbec_flags() == 0);
  CHECK(fuse_orbbec_flags() == 0);
#endif
  std::puts("example_cli: OK");
  return 0;
}
