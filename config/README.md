# Running an example

Each executable reads the versioned JSON format owned by
`volumetric_kit::core_config`. Run `program --config path.json` or
`program --help`. The former per-setting flags and positional paths have been
removed. Core validates the document before the executable opens a GPU,
camera, window or output file.

The files in [examples](examples) are starting points. Edit their source paths
and create the output directories before a real run. Paths are relative to the
configuration file, not the shell's current directory; environment variables
and home-directory shortcuts are not expanded.

| Executable | Starting configuration | Work |
| --- | --- | --- |
| `fuse_replica` | [fuse_replica.json](examples/fuse_replica.json) | Dataset to PLY |
| `codec_replica` | [codec_replica.json](examples/codec_replica.json) | Dataset fusion and codec study |
| `codec_mesh` | [codec_mesh.json](examples/codec_mesh.json) | Height-normalized mesh and codec study |
| `fuse_orbbec` | [fuse_orbbec.json](examples/fuse_orbbec.json) | Live camera/rig to PLY |
| `fuse_orbbec` | [stream_orbbec.json](examples/stream_orbbec.json) | Live stream/decode only |
| `fuse_orbbec` | [readback_orbbec.json](examples/readback_orbbec.json) | Live stream with luma readback |
| `fuse_orbbec` | [dump_orbbec.json](examples/dump_orbbec.json) | Live stream with readback, first sets' JPEGs and luma written |
| `fuse_render` | [fuse_render.json](examples/fuse_render.json) | Dataset to image |
| `fuse_viewer` | [fuse_viewer.json](examples/fuse_viewer.json) | Dataset in a window |
| `rig_viewer` | [rig_viewer.json](examples/rig_viewer.json) | Live rig in a window |

A scenario requires `version: 1`, an explicit `operation`, and a `source`
object with an explicit `kind`. Settings are grouped into `source`, `fusion`, `codec`,
`window`, `render`, `run`, `output` and `diagnostics`. Core's application
profile supplies defaults and rejects unknown fields, wrong types, out-of-range numbers and
unsupported combinations. Dataset paths, a mesh's height and up direction, or
a rig's sync path remain required where the application needs them.

| Former command-line input | JSON setting |
| --- | --- |
| Dataset positional path / `--cam-params` | `source.path` / `source.camera_parameters` |
| `--serial` / `--rig` / `--calibration` | `source.serial` / `source.sync_config` / `source.calibration` |
| `--hevc` / `--mjpeg` | `source.color_codec` |
| `--color 3840x2160` / `--fps 25` | `source.color_width`, `source.color_height` / `source.fps` |
| `--no-fuse` | `operation: "stream"` |
| `--readback` | `diagnostics.readback: true` |
| `--voxel` / `--trunc` | `fusion.voxel_size_m` / `fusion.truncation_m` |
| `--min-depth` / `--max-depth` | `fusion.min_depth_m` / `fusion.max_depth_m` |
| `--max-frames` / `--frames` / `--sets` | `run.frames` / `run.frames` / `run.sets` |
| `--width` / `--height` for a window or image | `window.width` / `window.height` |
| `--height` / `--up-vector` for a mesh | `source.height_m` / `source.up_vector` (three numbers) |
| `-o` for fusion/render or a codec | `output.path` or `output.prefix` |

Omitted values retain the previous application's defaults. A truncation value
of zero or less requests four voxels. Depth limits use the source's defaults
(Replica 0.1–8 m; live camera 0.25–5 m). Dataset frame limits retain their old
per-example defaults; `rig_viewer` uses zero for no frame/set limit. Counts
remain distinct: `fuse_orbbec` counts camera frames, while `rig_viewer` can
limit both acquired sets and drawn frames.

`fuse_orbbec` accepts `diagnostics.readback: true` in either a stream or fusion
scenario. It copies every set's luma to host memory and reports a separate
`readback` stage. It defaults to false, so a stream without it measures only
streaming and decoding.

`diagnostics.dump_sets: N` with `diagnostics.dump_directory` writes the first
N sets' files into that directory, which must not exist yet (its parent must):
`set_NNN_<serial>.jpg`, each camera's JPEG as it came, and with `readback`
`set_NNN_<serial>_luma.png`, the luma the device decoded, as R = G = B. It
needs `source.color_codec: "mjpeg"`. Writing slows the sets it writes.

`operation: "stream"` refuses an explicitly supplied `fusion` or `output`
section, including settings equal to their defaults. Leave those sections out.
`source.serial` and `source.sync_config` are mutually exclusive;
`source.apply_sync` needs the sync file, and `source.host_clock` is a
single-camera option. Loading or validating a scenario never changes camera
settings; camera setup performs that action only when requested.

For mesh inspection, set only `codec.inspect_only: true` in the `codec` section
and omit `fusion`, `source.sdf_mode` and `source.shell_voxels`; these settings
are unused until mesh conversion runs. An optional `output.prefix` still
exports the normalized input mesh.

The sync document remains SDK-compatible, and the calibration document remains
recon's domain format. A scenario references these resources rather than
embedding their schemas. Runtime camera discovery, asset readability,
supported device modes and GPU availability remain runtime API checks.
