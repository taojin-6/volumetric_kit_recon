# Changelog

All notable changes to this project are documented here. The format is based on
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and the project aims to
follow [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- `camera`: **the camera vocabulary**, a tier of its own
  (`volumetric_kit::recon_camera`) that links the core's base tier and GLM,
  and no Vulkan, so a driver uses it without a GPU API:
  - `CameraModel` (image size, intrinsics and OpenCV's rational lens, in
    double) and `check_camera_model`;
  - `distort_rational<T>`, the lens model the sensor tier's float lens and
    its GLSL mirror follow;
  - rigid transforms and the rotation of a Rodrigues vector
    (`geometry.hpp`);
  - the sensor array's calibration file (`array_calibration.hpp`): each
    sensor's pose by serial, read from the family's `device_calibration`
    layout.
  Tests: `recon_camera_model`, `recon_camera_geometry`,
  `recon_camera_array_calibration`.
- `camera`: **`nearest_rotation`**, the polar factor of a matrix near a
  rotation.
- `sensor`: **`IRgbdSensor`** (`rgbd_sensor.hpp`), the interface every
  driver implements: `SensorInfo` (id, the cameras' factory models,
  `depth_to_color`, rig role, clock, pose source, rate), a queue of
  `set_queue_depth` frames, `poll` for the newest and `drain` for all,
  oldest first, and `SensorStats`. Test: `recon_sensor_rgbd_sensor`.
- `sensor/array`: **`SensorArray`** (`volumetric_kit::recon_sensor_array`),
  several `IRgbdSensor`s read as one: secondaries started before the
  primary, frames grouped into a `FrameSet` by trigger on the host clock,
  each posed by an `ArrayCalibration`. Test: `recon_sensor_array`.
- `sensor/array`: **`SensorArray::process`**, a set prepared on the GPU in
  one batch, through the new **`GpuFramePrep::prepare_batch`** (every
  camera's uploads, then every camera's passes, in one `CommandBatch`).
  Test:
  `recon_sensor_array_process`.
- `sensor`: **`TriggerGrouper`** (`trigger_grouper.hpp`), the rig's grouping
  of frames into triggers, moved out of the Orbbec driver. Test:
  `recon_sensor_trigger_grouper`; the driver's start order is tested by
  `recon_sensor_orbbec_start_order`, formerly `recon_sensor_orbbec_grouping`.
- `sensor/orbbec`: **`OrbbecSensor`**, the Femto Mega as an `IRgbdSensor`:
  frames as captured, for the GPU pass, the pose in double, and
  `sync_clock_to_host` for the host's clock. `fuse_orbbec` reads one camera
  through it, which is how it is checked, as tests use no hardware.
- `volume`: **`VoxelHashMap::compact_active_blocks_in_frusta_on_device`**,
  the active blocks inside any of several frusta, compacted in one scan and
  left on the device in a list of their own, which
  `check_device_block_list` refuses as not the active set. Tests:
  `recon_volume_frustum`, `recon_tsdf_integrate_cull`.
- `mesh` / `volume`: **marching cubes over a device block list**.
  `MarchingCubes::extract_device(grid, iso, const DeviceBlockList&, timings)`
  meshes exactly the listed blocks, bound in place, with no hole at the cull
  edge; `VoxelHashMap::check_device_block_subset` accepts the map's active set
  or its frusta list while it holds, and the extract refuses any other before
  it claims a ring slot. A later frustum compaction now makes an earlier
  frusta list stale. Test: `recon_mesh_marching_cubes_sparse`.

### Changed

- `tsdf`: **`TsdfIntegrator::integrate` fuses only the blocks its frames
  reach**: one frustum a frame from its depth camera, no near cut, far at
  `max_depth + trunc_dist`, the union compacted in one scan. The result is
  bit-identical, Dynamic's clearing included; the dispatch scales with the
  reached blocks rather than the map (room0 at 1 cm replicated 16 times on an
  M5 Max: 0.5-0.7 ms against 3.3-5.2 ms of device time a frame). A frame
  whose principal point lies outside its image fuses the whole active set.
  The dispatch-size refusal counts the reached blocks, and a fuse no longer
  leaves the map's full list for an extract to reuse.
- `fuse_viewer`, `rig_viewer`: **mesh only what the render camera sees**.
  The render thread publishes its `view_proj`; the fuse thread meshes the
  blocks inside it, 0.25 m wider, after fusing (at `--remesh-every`) and
  whenever the view moves while nothing fuses, so both fuse threads now run
  until the window closes. The whole map is meshed until a view exists.
- `mesh`: **a culled extract records its triangle density**, as a full one
  does, so a viewer that only culls stops refitting on every extract. A full
  extract after a culled one may plan high once.
- `sensor`: **`RawFrame` is `RgbdFrame`** (`sensor/rgbd_frame.hpp`). Each
  camera is a `camera::CameraModel` in double; the poses are
  `color_to_world` and the sensor's `depth_to_color`, in double, both
  refused by `GpuFramePrep` unless rigid; frames carry a `sequence`; and a
  frame holds its host pixels (`pixels`) rather than borrowing them until
  the next poll. Migrating: `RawFrame` is `RgbdFrame`;
  `depth_camera`/`color_camera` are `CameraModel`s (`size`, `intrinsics`,
  `distortion`); `color_cam_to_world` is `color_to_world` and
  `depth_cam_to_world` is `color_to_world * depth_to_color`.
  `sensor/lens.hpp` (`LensCamera`, `LensDistortion`, `distort_normalized`)
  is gone: use `camera::CameraModel` and `camera::distort_rational`.
  `OrbbecCapture::Options::cam_to_world` is a `camera::Mat4d`, refused at
  `open` unless rigid. The examples' own frame type is `OwnedFrame`
  (`examples/common/owned_frame.hpp`), so it is not taken for this one.
- `sensor/orbbec`: **the raw path's depth-to-colour extrinsic is a
  rotation.** The Femto Mega's factory one is 1.2% off orthonormal; the
  driver takes its nearest rotation, which moves depth up to about 4 px
  along x in a 720p colour image, and `open` refuses one that does not come
  out a rotation.
- `sensor/orbbec`: **raw frames need `min_depth > 0`**, as the GPU pass
  does: `OrbbecCapture` and `OrbbecRig` opened `raw` refuse 0 at `open`
  instead of handing out frames `GpuFramePrep` refuses. **A raw MJPEG pair
  the JPEG decoder had no time for is counted `dropped`**, as the mailbox
  counts one, not `lost`.
- `sensor/orbbec`: **every frame is handed out as captured**, for the GPU
  pass. `OrbbecRig` is no longer an `ICameraCapture`: its one read is
  `poll_set`, formerly `poll_raw_set`, handing out an `OrbbecRigSet` of
  `RgbdFrame`s, and `color_camera(i)` is the colour camera's factory
  `camera::CameraModel`, its pose `color_to_world(i)`. `OrbbecRigSet` is no
  longer a template: what was `OrbbecRigSet<RgbdFrame>` (`OrbbecRigRawSet`) is
  plain `OrbbecRigSet`, and `OrbbecRigFrameSet` is gone. A colour camera is
  read off the H.265 or MJPG mode streamed, no longer the RGB mode, so `open`
  no longer refuses a camera whose two modes' calibrations differ. The
  shared types move out of `orbbec_capture.hpp`: `OrbbecStreamOptions`,
  `OrbbecDeviceInfo`, `OrbbecColorCodec` and `OrbbecStreamStats` (formerly
  `OrbbecCaptureStats`) to `orbbec_stream.hpp`, and `OrbbecSyncMode`,
  `waits_for_primary` and its `to_string` to `orbbec_sync_config.hpp`.
- build: **`VR_WITH_ORBBEC` needs `VR_WITH_FFMPEG`**, refused at configure
  without it: the driver decodes each camera's colour itself.
- `examples`: **`fuse_orbbec` always prepares its frames on the GPU**, H.265
  unless `--mjpeg`, a camera or a rig's set through `prepare_batch` and
  `fuse_set`.
- `sensor/video`: **the decoders hand out pictures on the device only**, on
  the build's one hardware path: VideoToolbox on Apple, NVDEC and nvJPEG's
  hardware engine through CUDA elsewhere. Their status codes are the
  contract: `IoError` for data that does not decode, `Unsupported` for a
  stream the hardware does not take (sticky for H.265) or a `create` with no
  device path, `Backend` or `OutOfMemory` for a device path that failed.
  `HevcDecoder::Options` is `unlabelled_color`, `configure_ffmpeg_logging`,
  `device` (required) and `allocator` (required with NVDEC);
  `JpegDecoder::Options` is `device` and `allocator`. Migrating: pass the
  device and allocator, and read pictures from `DecodedPicture::device` or
  `image`.
- `sensor/orbbec`: **a colour stream the hardware cannot decode stops the
  camera**: `OrbbecSensor::poll` and `OrbbecRig::poll_set` return the
  decoder's `Unsupported`, `Backend` or `OutOfMemory`; the driver's start
  returns `Unsupported` without a device.
- build: **off Apple, `VR_WITH_FFMPEG` needs `VR_WITH_CUDA`**, and the FFmpeg
  floor is 6.1 (libavcodec 60.31); libswscale is no longer used.

### Fixed

- `sensor/video`: preserve VideoToolbox JPEG session, decode and callback
  errors, so hardware malfunctions and allocation failures stop the Orbbec
  colour decoder instead of looking like corrupt frames. Bad JPEG data
  remains recoverable.
- `sensor/video`: validate a candidate cropped HEVC SPS before refusing the
  stream on Apple. A truncated SPS no longer poisons subsequent valid frames,
  including after `HevcDecoder::reset`.

### Removed

- `mesh`: **incremental extraction** (BREAKING).
  `MarchingCubes::extract_device_incremental`,
  `MarchingCubesConfig::track_block_spans`, `BlockSpan`, `block_spans()`,
  `block_span_capacity()`, `block_spans_generation()`, `block_span_valid()`,
  and `ExtractTimings::incremental`, `remeshed_blocks` and `input_upload_ms`.
  Every extract meshes the whole active set or a caller's block list. The sparse kernels
  lose their span and stamp bindings, three push constants and a scratch
  word (the push block is 52 bytes, the command buffer 28). The `changed`
  stamp, its writers and `read_block_stamps` stay. `fuse_replica` loses
  `--incremental`. The iOS scanner's `incremental_benchmark` mode, compiled
  into every scanner build, must be removed before it re-pins.
- `mesh` / `volume`: **the host-list culled extract** (BREAKING):
  `MarchingCubes::extract_device(grid, iso, const BlockList&, timings)` and
  `VoxelHashMap::compact_active_blocks_in_frustum` (both overloads). Cull with
  `compact_active_blocks_in_frusta_on_device` and the `DeviceBlockList`
  overload. `BlockList`, `VoxelBlockGrid::block_list` and `check_block_list`
  stay for the codec. Nothing is uploaded, so `ExtractTimings::input_upload_ms`
  goes; the iOS scanner drops its two `inputs` rows when it re-pins.
- **Test-only public API**, which no example, sibling or app called:
  - `core`: `device_macros.hpp` (`VR_DEVICE_HOST`), `vr::normalize`,
    `Vec3u8`;
  - `volume`: `Voxel`, `VoxelData`, `HashTable`,
    `VoxelHashMap::allocate_from_points`, `VoxelBlockGrid::stamp_blocks`,
    `VoxelGridParams::defaults()`; `VoxelBlockGrid::create` now refuses an
    attribute whose block is not whole 4-byte words;
  - `mesh`: `ExtractTimings::uncached_cells_per_block`;
  - `texture`: `pack_atlas` and `texture(DeviceMesh, Buffer depth, cam)`;
  - `codec`: `Encoder::config()`;
  - `eval`: `compare_meshes` (use `ReferenceMesh`) and `MeshDistance::reach`.
  Test: `recon_volume_types` goes.
- **Windows and MSVC**: the MSVC compiler flags and `_MSC_VER`/`WIN32`
  branches in the build, the codec and the tests. recon builds with GCC or
  Clang on Linux, Android, macOS and iOS.
- `sensor/video`: **software decoding, VAAPI and host pictures**:
  `VideoDecodeBackend`, `HevcDecoder::hardware_backends` and `backend`, the
  `layout`, `threads` and `label` options, `JpegDecodeBackend`,
  `JpegDecoder::backend` and its `label` and `configure_ffmpeg_logging`,
  `VideoPixelLayout::Rgb24`, `DecodedPicture::plane`, nvJPEG's `GPU_HYBRID`
  back end, and the warning a decoder gave when its pictures came to the
  host. Test: `recon_sensor_video_backend` goes, and
  `recon_sensor_video_converter` becomes `recon_sensor_video_frame_color`.
- `sensor/orbbec`: **`OrbbecStreamStats::host_pictures`**, with nothing left
  to count.
- `sensor`: **host colour planes**: `YuvImage::plane`. `GpuFramePrep` takes
  colour on the device only, as a buffer or NV12 images, and stages only
  depth, on the calling thread.

- `sensor/orbbec`: **`OrbbecCapture` and the SDK's host path**: the
  undistortion and registration on the host (`ob::UnDistortionFilter`,
  `ob::Align`), the conversion to metres and packed RGB, the H.265
  decoder's RGB frames, and `OrbbecStreamOptions::raw`. With them go
  `OrbbecRig::poll`, its processed `poll_set` and `poll_raw`, and
  `ICameraCapture::poll_raw` and `raw_frames`, which only the two Orbbec
  classes implemented, and `to_string(OrbbecColorCodec)`, which had no
  caller. Migrating: open one camera as an `OrbbecSensor`.
- `examples`: **`fuse_orbbec --gpu`.** Every frame is prepared on the GPU,
  so the switch chose nothing; `--host-clock` stays, for one camera.

- `sensor`: **`prepare_set`**, a thread and a submit per camera, replaced by
  `GpuFramePrep::prepare_batch`, which takes the same passes and frames and
  returns the same frames. `fuse_orbbec --rig` and `rig_viewer` prepare
  their sets through it.
- tests: **the tests that opened real cameras**,
  `recon_sensor_orbbec_capture`, `recon_sensor_orbbec_gpu_prep` and
  `recon_sensor_orbbec_rig`, with their `VR_ORBBEC_TEST_SERIAL`,
  `VR_ORBBEC_TEST_RIG`, `VR_ORBBEC_TEST_COLOR` and `VR_ORBBEC_TEST_FPS`
  variables.

- `sensor`: **`rig_calibration.hpp`**, replaced by
  `camera/array_calibration.hpp`. Migrating: `read_rig_calibration` is
  `camera::read_array_calibration`, which returns an `ArrayCalibration`; a
  `RigCameraCalibration`'s `serial` and `cam_to_world` are a
  `SensorCalibration`'s `id` and `color_to_world` (in double);
  `OrbbecRig::Options::calibration` takes the `ArrayCalibration`. The file's
  lens fields are no longer read.

- `core`: **recon's names for the core's types and macros.** `vr::Status`,
  `vr::Result`, `vr::Device`, `vr::Buffer` and every other core type or
  function `core/` re-exported into `vr::` are gone, with the headers that did
  it (`result.hpp`, `check.hpp`, `vk_result.hpp`, `vulkan.hpp`,
  `allocator.hpp`, `buffer.hpp`, `command_batch.hpp`, `compute_kernel.hpp`,
  `compute_pipeline.hpp`, `compute_util.hpp`, `descriptor.hpp`,
  `external_memory.hpp`, `gpu_timer.hpp`, `image.hpp`, `instance.hpp`,
  `shader.hpp`, `stage_metrics.hpp`, `unique_handle.hpp`), and so are
  `VR_TRY`, `VR_ASSIGN`, `VR_CHECK` and `VR_VK_TRY`. Migrating:
  - `vr::X` is `volumetric_kit::core::X`; include the core's header of the
    same name, `volumetric_kit/core/base/` for `result`, `check` and
    `stage_metrics`, `volumetric_kit/core/vulkan/` for the rest.
  - `VR_TRY`, `VR_ASSIGN`, `VR_CHECK` and `VR_VK_TRY` are `VKC_TRY`,
    `VKC_ASSIGN`, `VKC_CHECK` and `VKC_VK_TRY`.
  - `core/device.hpp` is `core/device_requirements.hpp`, holding only
    `vr::device_requirements()` and `vr::check_device_requirements()`.
  - `vr::log_message` and `vr::kLogSource` stay; `LogLevel`, `LogHandler` and
    `set_log_handler` are the core's.
  - `core/fwd.hpp` forward-declares the core's classes in the core's namespace
    only.

- `volume`: **`VoxelBlockGrid`'s move assignment**. A grid is move-
  constructible only; replace one with `std::optional::emplace`, as the iOS
  scanner and the codec examples already do. The assignment had to name every
  member to guard against `std::vector` self-move, had missed one before, and
  nothing outside its tests used it.
- `core`: **`Device::command_pool()`**. Each submit takes a pool of its own,
  so there is no one pool to hand out, and a caller recording on it would
  race the device's submits. Nothing outside the tests called it.
- `examples`: **the examples' own dataset API** — `ReplicaDataset`,
  `FrameView` and `example_camera.hpp` (`make_depth_camera`) are gone, replaced
  by `ReplicaCapture` (below); `pack_color_rgba8` with them, since the atlas
  leg now runs through `sensor::to_canonical`. No compatibility layer: the
  examples are the only consumers.
- `mesh`: **the dense marching-cubes entry point**. `MarchingCubes::extract`
  taking a caller-supplied `Voxel` array over a `DenseGrid` is gone, with
  `DenseGrid`, the `marching_cubes.comp` kernel it drove, and its descriptor
  set. It had no consumer outside the tests, it was never in the engine this
  tier re-implements (whose extractor takes only a `VoxelHashMap`), and it
  shared the sparse path's ring — so one large dense call grew whichever slot
  it landed on to the dense worst case and held it for the extractor's
  lifetime. Meshing is sparse-only now. See the 2026-08-31 decision for what
  the removal does and does not cost in coverage: every property the dense
  cases carried is asserted through a sparse fixture instead, including the
  independent growth of the two output buffers, which a first extract over a
  thin shell followed by one over a dense field reaches without a second entry
  point.

### Changed

- `examples`: **the viewers' gfx pin moves from #98 to #106**, after gfx
  moved onto volumetric_kit_core (gfx #100-#106). recon and gfx now share one
  `Status`, `Device` and `StageRow`: the viewers build the shared device from
  their `WindowedAppConfig::device` and hand gfx its `graphics_payload()`
  unconverted, `stage_metrics.hpp` and its `to_sections` mapping are gone
  (recon's stage rows are the overlay's), and the viewers' textures and
  descriptor objects are the core's `vkc::Image`, `vkc::DescriptorPool` and
  `vkc::DescriptorSet`. Their images and buffers are `DeviceOnly`, which
  refuses where gfx's `DeviceLocal` fell back to host memory. `fuse_render`
  renders byte-identically at the new pin.
- `core`: **a device that did not enable what recon's kernels need is
  refused.** `MarchingCubes`, `VoxelHashMap` (and so `VoxelBlockGrid`),
  `TsdfIntegrator`, `MeshIntegrator`, `ProjectiveTexturer`, `GpuFramePrep`
  and the codec's encoder and decoder return `Unsupported` from `create`,
  naming the missing feature, where they used to run their `layout(scalar)`
  kernels on a device without `scalarBlockLayout`. Create or adopt every
  device recon runs on with `vr::device_requirements()`;
  `vr::check_device_requirements(device, who)` is the check.
- `core`: **volumetric_kit_core is pinned at main after its PR #13** (was
  PR #10), for PR #11's `Device::check_enabled`. Migrating:
  - `vr::AdoptedDevice` declares its features in one `enabled_features`
    (`EnabledFeatures`): `enabled_features` → `enabled_features.core`, and
    `enabled_timeline_semaphore`, `enabled_scalar_block_layout` and
    `enabled_dynamic_rendering` → `enabled_features.timeline_semaphore`,
    `.scalar_block_layout` and `.dynamic_rendering`.
  - Vulkan headers 1.3.204 or newer are required, 1.3.208 on Apple; older ones
    fail at configure or in the core's `vulkan.hpp`.
  - `vk_result` returns empty for a backend detail outside `VkResult`'s range.
  - Rebuild after bumping the pin: `Device` changes size.
- `core`: **the Vulkan foundation comes from `volumetric_kit_core`'s vulkan
  tier**. `Instance`, `Device`, `Allocator`, `Buffer`, `Image`, the descriptor
  and pipeline wrappers, `ComputeKernel`, `CommandBatch`, `GpuTimer`,
  `StageMetrics` and the exported buffers are the core's types, named in `vr::`
  as before, so one `VkDevice` and its buffers pass between recon, gfx (once it
  adopts the core) and an embedder unchanged. recon no longer compiles VMA:
  the core's `core_vulkan` does, and `recon_core` links it PUBLIC. Migrating:
  - `vr::device_requirements()` states what recon's kernels need
    (`scalarBlockLayout`, and external memory and Metal objects where
    offered). `Instance::select_physical_device(reqs)` returns a
    `PhysicalDeviceInfo`, and `Device::create(instance, gpu, reqs)` and
    `Device::adopt(payload, reqs)` take the requirements in place of
    `DeviceConfig`, which is gone with `Device::requirements`.
  - `Device::compute_family()` / `compute_queue()` are `queue_family()` /
    `queue()`, and so are `AdoptedDevice`'s fields; an adopted device also
    declares `instance_api_version`.
  - Memory is placed explicitly (`MemoryUsage`): `DeviceOnly`, the default,
    for anything a kernel touches; `DeviceMapped` for device-local memory the
    host maps; `Staging` for host-memory transfer buffers, `TRANSFER` usage
    only. `DeviceLocal`, `HostVisible`, `Auto` and `BufferDesc::mapped` are
    gone; mapped placements are always mapped.
  - `storage_buffer` is `mapped_storage_buffer` (device-local, mapped;
    `Unsupported` where the device has none), and `max_storage_buffer_range`
    takes the `Device`.
  - `HeapStats::usage_bytes` is the heap's usage -- the whole process's where
    `VK_EXT_memory_budget` is enabled -- and `reserved_bytes` is recon's own
    allocator's share; the viewers' memory panels now show the latter.
  - `BufferMemoryInfo` is `MemoryInfo`; an `Image` is adopted from an
    `ImageInfo`; `submit_single_time` takes an optional `keep_alive` and
    `in_flight`, or a `GpuStageScope` to time the work.
  - `create_exported_buffer(device, allocator, bytes)` takes the allocator,
    and its descriptor is a `UniqueFd`. So `HevcDecoder::Options`,
    `JpegDecoder::Options` and `OrbbecStreamOptions` gain an `allocator`
    beside `device`: NVDEC's and nvJPEG's pictures stay on the device only
    when one is given.
  - Headers that only name these types include `core/fwd.hpp`; a forward
    declaration in `vr::` would declare a different class.
  - The marching-cubes span table is device-only, so `block_spans()` returns
    a host copy each extract reads back, and `ExtractTimings::arena_bytes`
    counts it (16 bytes per block).
  - `vr_compile_shaders` and `vr_embed_shaders` forward to the core's
    `vkc_compile_shaders` and `vkc_embed_shaders`; `cmake/embed_spirv.cmake`
    is gone. The core must be made available with `VKC_WITH_VULKAN ON`, which
    configuring now checks.
- `core`: **error handling comes from `volumetric_kit_core`**, fetched pinned by
  commit and re-found by the installed package. `vr::Status` and `vr::Result`
  are the core's types, so they pass to calib unchanged (gfx keeps its own
  `Status` until it adopts the core). Migrating:
  - A log handler takes `(level, source, message)`; recon's messages carry
    source `"vr"`, and `vr::log_message(level, message)` is unchanged.
  - `Status::Code::Numerical` is new, so an exhaustive `switch` over the codes
    needs a case (the iOS app's `RendererErrors.mm` does).
  - `Status` and `Result` are `[[nodiscard]]`; discard one with `(void)`.
  - `Status::backend_error(0, …)` aborts, and so does
    `vk_error(VK_SUCCESS, …)`; `Status::with_context` prefixes a message and
    keeps the domain and detail.
  - `std::move(r).value()` and `*std::move(r)` return the value, not a
    reference into `r`.
  - `Result`'s success constructor refuses a pointer for `Result<bool>` and
    `nullptr` for a string-like `T`; return a `Status` for those errors.
  - `set_log_handler` returns only once no other thread is still in the
    previous handler, so a handler must not wait for a thread that may call
    `set_log_handler`.
  - `VR_TRY` / `VR_ASSIGN` / `VR_CHECK` are the core's `VKC_*` macros under
    recon's names; a failed `VR_CHECK` logs with source `"core"`.
- docs: make `AGENTS.md` the concise shared working guide for Codex and
  Claude Code, with `CLAUDE.md` importing it. Move detailed contracts,
  implementation status, and gotchas into `DESIGN.md`, and put the complete
  decision index beside its dated rationale in `DECISIONS.md`.
- `codec`: **per-basis quantization, frame v3.** `CodecParams::dc_step` and
  `ac_step` are gone, replaced by `quantization_scale` and 512
  `quantization_weights` in `x + 8*y + 64*z` order; each step is their
  product. Only the K kept bases' weights are used, checked and carried, and
  the scale and those weights must be normal. The v3 header carries the K
  weights after its 44-byte prefix (`44 + 4K` bytes, 300 at K = 64), which is
  what `read_frame_info` now needs; v1 and v2 frames are `Unsupported`.
  Callers that set `dc_step` / `ac_step`, such as the iOS scanner and the gfx
  players, must move to the scale and weights. The encoder compacts its
  observed list on the device and keeps its transform scratch between calls.
  See the 2026-10-02 and 2026-10-03 codec decisions.
- `codec`: **frame v2, 37% smaller on room0.** A partial observed mask is
  coded a plane and a line at a time against its neighbours, and a sign
  inside its mantissa's raw field: 11.0 B/block at 1 cm where v1 was 17.4,
  with the same decoded surface, and encode / decode 14.3 / 14.0 ms where
  they were 17.6 / 16.0 (M5 Max, Release). A v1 frame is `Unsupported`. The
  transform is device-resident, one `CommandBatch` a call, with 16-bit
  coefficients. The encoder reuses the map's last active list and drops
  never-observed blocks before the transform. The decoder checks geometry
  off the header first, stamps `changed` only on blocks it alters, and
  reports a broken heap as `InvalidArgument` rather than `IoError`.
  `CodecParams` refuses a step past `kMaxStep` (64). See the 2026-10-01
  codec review decision.
- `volume`: **`VoxelBlockGrid::remove` finds each block on the device**, so
  it no longer compacts and sorts the whole grid to zero what it removes.
- `examples`: **`rig_viewer` textures a missing camera from its last
  frame.** A set missing a camera now textures that camera from its newest
  frame, up to `--hold-ms` old on the rig's clock (2000 by default; 0 turns
  it off). The held frame is a fallback view with its own depth: it takes
  only triangles no present camera does, where that depth still agrees with
  the mesh within the occlusion threshold. Before, each dropped frame
  textured that remesh without the camera, and its triangles flickered to
  fused colour. On the lab rig at 4K, 71 of 823 remeshes were given fewer
  cameras than the rig has without a hold, and none of 888 with the first
  cut's. The Rig panel counts held views and short remeshes. Only texturing
  uses the held frame, so no frame is fused twice.
- `examples`: **`rig_viewer` fuses depth only inside each camera's colour
  view** by default (`GpuFramePrepConfig::depth_within_color`), so the
  untextured white floor and walls outside every colour camera's view are no
  longer fused. On the lab rig this halved the untextured triangles, from
  16.9% to 8.6%. `--all-depth` fuses all of it.
- `examples`: **`rig_viewer` fuses `Dynamic` by default** (`--static` for
  `Classic`), so a surface that moves away is gone on the next set rather
  than fading over `--max-weight` frames. It **textures at a 5 cm occlusion
  threshold** (`--occlusion`), which on the lab rig left 17% of the mesh
  untextured against 28% at 2 cm. The View panel tunes the mode, the max
  weight and the threshold live. `fuse_frame`'s `DeviceFrame` overload takes
  the integration mode, `Classic` by default.
- `examples`: **`rig_viewer` remeshes on every set by default**, not every
  fifth, so the mesh updates at the rig's set rate (about 26-29 a second on
  the lab rig at 1 cm), and its Rig panel and status line report the
  measured mesh update rate. `--remesh-every N` still sets it.
- `examples`: **the viewers' gfx pin moves from #93 to #98**, whose
  `kHybridMeshNormals` is `rig_viewer`'s normals shading mode. `fuse_viewer`
  and `fuse_render` build unchanged at it.
- `texture`: **the multi-view atlas grows in rows.**
  `side_by_side_atlas` lays n views out in floor(sqrt(n)) rows, so up to
  three sit side by side and four make two rows of two: four 4K views make a
  7680 x 4320 atlas rather than 15360 x 2160. A row still ends early at the
  extent.
- `core`: **a `Device` may be submitted to from several threads at once.**
  Each submit records on a command pool no other submit holds, from a free
  list the device keeps, so recording takes no lock. Only `vkQueueSubmit` is
  serialized, under the embedder's mutex on a shared queue and the device's
  own otherwise, so `submit_mutex()` is never null. Before, every submit
  shared one pool and only a shared queue was locked, so two threads' batches
  were undefined: the validation layer reported `THREADING ERROR` on the pool
  and one run segfaulted. A command buffer is now kept for the next submit,
  and its fence with it: creating a fence alone cost an RTX 5090 0.29 ms a
  submit, and room0 fuses there at 431–463 fps against 258–286.
- `sensor`: **`GpuFramePrep` stages its raw frame and runs both passes in one
  submit**: the raw depth and colour planes go up through a batch into
  device-local inputs, rather than a host-visible buffer the kernels read
  across the bus. The three planes are packed into one staging buffer, and
  the `"frame prep"` row's device half counts the copy. The pass keeps that
  buffer, since four passes allocating one each per call made VMA allocate
  and free a block for every rig set: 46 ms a set on an RTX 5090, 4.8 ms
  kept.
- `texture`: **every pass is one batch**: a host depth frame staged, the camera
  inline, the dispatch, and for a host `Mesh` the vertices staged up, then read
  back in a batch of their own. The camera, the depth copy and the several-view
  pass's buffers are device-local and grow-only.
- `core`: **staged uploads rising through one buffer share a barrier**, as
  fills and inline uploads do, so a rig's views stage into one buffer without a
  barrier between them. `StorageInput::buffer` reuses the buffer it is handed
  when it already fits.
- `mesh`: **the arena, index run and draw command are device-local**, as are
  the tables. Each extract attempt is one batch: the active list staged, the
  command reset inline, the dispatch, the command read back. `download` copies
  back through a batch. The span table stays host-visible, since its reader
  is the host. The output buffers' usage now includes `TRANSFER_SRC |
  TRANSFER_DST`.
- `tsdf`: **`integrate` fuses over the active list in place**, one batch after
  the compaction: the frames staged onto the device, the cameras inline. The
  dirty flags, the cameras and `MeshIntegrator`'s per-slot counters are
  device-local. Breaking: `dirty_block_count()` returns `Result<uint32_t>` and
  `reset_dirty()` returns a `[[nodiscard]] Status`, since both now reach the
  device; a boolean test of `dirty_block_count()` now reads success, not a
  non-zero count. `MeshIntegrator` submits each dispatch of a split write on
  its own and reads back only the per-slot counts.
- `volume`: **`compact_active_blocks_on_device`** leaves the compacted list on
  the device as a `DeviceBlockList` and reads back only its count;
  `check_device_block_list` refuses a list that has gone stale.
  `allocate_from_depth` stages a host depth frame onto the device.
- `core`: **`StorageInput::buffer` takes the call's `CommandBatch`** and stages
  a host array into a device-local buffer, rather than binding a host-visible
  upload.
- `volume`: **the hash map's buffers and the grid's attribute arrays are
  device-local**, reached through a `CommandBatch`: `create`, `clear` and
  `remove` zero on the device, before any index is freed, `resize` copies
  there, a call's inputs are uploaded in its first round, and a compaction
  reads its list back in the count's own submit. `AttributeView::buffer` is
  no longer mapped; a test reads and writes an attribute through
  `tests/grid_readback.hpp`. `load_factor()` reads a host copy of the heap
  counter that every allocating or removing round reads back, so it still
  costs no dispatch, and `diagnostics()` reads the device's own.
- `core`: **`device_storage_buffer` adds `TRANSFER_SRC | TRANSFER_DST`** and
  takes extra usage and queue families, as `storage_buffer` does, so a
  `CommandBatch` can fill, copy and stage through it; `GpuFramePrep`'s
  outputs are made by it. `tests/buffer_readback.hpp` reads back through a
  batch.
- `codec`: **`CodecParams`' defaults are K = 64 with one step of 0.2 for DC
  and AC alike**, replacing the prior engine's K = 32 with DC 0.25 / AC 0.05.
  On room0 at 1 cm frames are 7% smaller and 28% more accurate, and host
  coding still fits a 30 fps frame interval. They are provisional until the
  per-band quantization study (see the 2026-09-27 decision). On a smooth
  analytic sphere they are 16% smaller but 17% less accurate on the mean,
  and the worst vertex is 2.4x as far off.
- `volume`: `VoxelBlockGrid::remove` finds each block by binary search over one
  sorted snapshot, where it scanned the whole active set per block. Removing k
  of n blocks drops from O(k·n) to O((k + n) log n).
- `codec`: the frame reader's `max_blocks` refusal is `OutOfMemory`, and it
  comes after the block count is checked against the segment table. A corrupt
  count is still `InvalidArgument`. Its messages say `codec frame:`, not
  `read_intra_frame:`.
- `examples`: **every fuse loop polls its frames through
  `sensor::ICameraCapture&`.** `examples/common/replica_capture.hpp` plays a
  Replica sequence back through the contract — frame cap, stride and the depth
  gate are its `Options`, stamped on each frame; the depth camera is derived
  from the colour one through `depth_from_registered_color` at `open`; the
  frames on disk are probed once, at exactly the strided indices under the
  limit, so `frame_count()` is what will really play (room0: 400, not the
  trajectory's 2000) and a sequence thinned to every N-th frame plays in full
  under `--stride N`. An empty poll is retried until the source reports itself
  `exhausted()`, so a live driver is a construction-site swap and a replay
  ends. A frame kept past the next poll is copied into an `RgbdFrame`
  of its own (`examples/common/rgbd_frame.hpp`, the one frame type: what the
  reader decodes into, what a consumer keeps, and what `CapturedFrame` is the
  view of) — `fuse_render`'s keyframe and
  `fuse_viewer`'s newest fused frame for its final texture pass — never
  borrowed. Each frame fuses through `examples/common/fuse_frame.hpp`, the one
  allocate-and-grow-then-integrate loop three examples had each carried a copy
  of. `fuse_render` and `fuse_viewer` take `--min-depth` beside `--max-depth`.
  See the 2026-09-14 decision.
- `mesh`: **`MarchingCubes::extract` is renamed `extract_host`.**
  Source-breaking, and mechanical at every call site. The pair is symmetric now
  — `extract_host` returns an owned host `Mesh`, `extract_device` a borrowed
  `DeviceMesh` — matching the `Mesh` / `DeviceMesh` types and making the shape
  `extract_<destination>[_<mode>]`, so `extract_device_incremental` and the
  `BlockList` overload read as modes of the live workflow rather than siblings
  of the export one. The old name implied the host path was the default when it
  is the export path. `extract_host` is *not* a wrapper you can inline: it also
  returns its ring slot, which a host-only caller cannot do (a `Result<Mesh>`
  carries no generation, and the public `release_through` is the consumer's
  high-water mark).
  **Note for downstream:** `volumetric_kit_ios` tracks this repo by
  `GIT_TAG main`, and its `compute_smoke` app calls `mc.extract(vbg)` — rename
  it there in the same window, or its next configure fails to compile.

### Added

- `codec`: **rANS decoding on the device.** `DecoderConfig::entropy`
  (`Decoder::create`'s new argument) picks where a frame decodes, as the
  encoder's does; all decode the same blocks and refuse a corrupt frame.
  On the device only the tables and payload go up and the coordinates come
  back; the coefficients stay in VRAM for the inverse. `kAuto` decodes on
  the device from `kMinDeviceDecodeSegments` (80), and
  `kMinDeviceSegments` is renamed `kMinDeviceEncodeSegments`. The device
  refuses segments longer than `kMaxDeviceDecodeSegmentSize` (1,024 blocks),
  so a stream cannot run one invocation past a GPU watchdog. Both coders'
  `kAuto` now tries a failed kernel build once and reports device failures
  other than a frame the device cannot hold.
  `EntropyCoding` moves to `codec_params.hpp`. Room0's decode at 1 cm goes
  from 8.54 to 3.29 ms on M5 Max and from 8.07 to 4.08 ms on RTX 5090. See
  the 2026-10-03 decoding decision.
- `codec`: **rANS encoding on the device.** `EncoderConfig::entropy` picks
  `EntropyCoding::kAuto` (the default: the device from 48 segments, the
  host below or when the device cannot code the frame), `kHost` or
  `kDevice`; all write the same bytes. On the device
  the coefficients stay in VRAM and only symbol counts and the coded frame
  cross the bus. Room0's encode at 1 cm goes from 8.98 to 3.12 ms on M5 Max
  and from 10.67 to 3.06 ms on RTX 5090. See the 2026-10-03 decision.
- `texture`: **`TextureView::fallback`**. The several-view pass gives a
  marked view only the triangles no unmarked view qualifies for, whatever
  their scores: for a frame older than the rest, such as `rig_viewer`'s held
  frames.
- `sensor`: **`GpuFramePrepConfig::depth_within_color`**. It zeroes a depth
  pixel whose point the colour camera did not record: behind that camera, or
  on a pixel its lens missed, found by the colour pass's own coverage test.
  What is fused then lies inside the colour view, so every surface can be
  coloured and textured. The test is the colour camera's view, not its line
  of sight. A frame without colour keeps all its depth, and the option is off
  by default. Test: `recon_sensor_gpu_frame_prep` (the 2026-09-28 GPU
  pre-processing decision, amended).
- `examples`: **`rig_viewer --show-sources`** (a View panel toggle too) fills
  each camera's atlas tile with a colour of its own, so the window shows which
  camera textured each triangle; **`--texture-stats`** reads the mesh back
  every 30 remeshes and prints each camera's share and the untextured one.
- `examples`: **`rig_viewer`**, a live window over a synced rig of Orbbec
  cameras (`VR_BUILD_VIEWER` with `VR_WITH_ORBBEC` and `VR_WITH_FFMPEG`).
  The rig's raw sets are prepared, fused and textured from every camera on
  the GPU. The atlas, one tile per camera, is filled by device copies
  recorded in gfx's frame. It shades unlit, lit or by normal, orbits, and
  looks out of any camera. Defaults: 1 cm voxels and H.265 colour. Ctrl+C
  stops the rig before it exits. See the 2026-09-29 decision.
- `sensor`: **`GpuFramePrepConfig::color_queue_families`**, the queue
  families a pass's colour buffers are shared with, as `MarchingCubesConfig`'s
  are for the mesh. A renderer on another family can then read a frame's
  colour. Depth stays EXCLUSIVE, since only recon reads it, and an empty
  config (the default) leaves both EXCLUSIVE, as before. Test:
  `recon_sensor_gpu_frame_prep`.
- `core`: **`check_queue_family_count`**, the bound `MarchingCubes::create`
  and `GpuFramePrep::create` both put on a config's fixed family array.
- `sensor`: **raw Orbbec frames over MJPEG.** A raw capture or rig no longer
  needs H.265: over MJPEG it streams the camera's JPEGs, decoded on a
  thread per camera by `JpegDecoder`, onto `OrbbecStreamOptions::device`
  where nvJPEG or VideoToolbox takes them. On the four-camera rig at 4K25,
  about 5 s of CPU for 600 frames against 71 s on the host MJPEG path.
  `fuse_orbbec --gpu --mjpeg` streams them; `--gpu` alone stays on H.265.
- `sensor`: **a raw Orbbec frame's colour stays on the GPU.** With
  `OrbbecStreamOptions::device`, a raw capture or rig decodes its H.265
  colour onto that device, and a `RawFrame`'s colour is the picture NVDEC or
  VideoToolbox left there, for `GpuFramePrep` on the same device.
  `fuse_orbbec --gpu` passes its device. On the four-camera rig at 4K25, the
  run's CPU falls by about 1.4 ms a camera frame.
- `sensor`: **`JpegDecoder` decodes on VideoToolbox.** Given a device that
  imports Metal textures, an 8-bit 4:2:0 JPEG decodes on the hardware JPEG
  decoder into NV12 plane images (`DecodedPicture::image`,
  `JpegDecodeBackend::VideoToolbox`). On an M5 Max, per 4K frame: 0.92-0.94
  ms of CPU to decode against 13.1-13.3 in software.
- `sensor`: **VideoToolbox's pictures stay on the GPU.** Given
  `HevcDecoder::Options::device` on a device that imports Metal textures, a
  VideoToolbox decode hands its picture out as two plane images
  (`DecodedPicture::image`), which `GpuFramePrep` takes as `YuvImage::image`
  and copies on the device. On an M5 Max, per 4K picture: 0.003-0.005 ms of
  CPU to receive against 0.57-0.75, each surface's images being made once,
  and 0.31-0.45 to prepare against 0.43-0.58.
- `core`: **`Image`**, a `VkImage` another API made, freed by its maker's
  deleter and kept in a layout a copy reads, and **`CommandBatch::copy`**
  from one into a buffer.
  `Device::create` enables `VK_EXT_metal_objects` where offered
  (`imports_metal_textures`), and `DeviceRequirements::metal_objects` asks a
  shared device for it. The build enables Objective-C++ on Apple.
- `texture`: **a colour camera of its own, and depth and coverage on the
  device, for projective texturing** — what a `GpuFramePrep` frame has. A
  `TextureView` takes an optional `color_camera`, a `depth_buffer` in place
  of the host `depth`, and a `coverage` (the frame's colour, whose zero high
  byte marks what the lens saw nothing of), the buffers held by
  `shared_ptr`. The single-camera pass takes one in
  `texture(const DeviceMesh&, const TextureView&, ...)` and
  `texture(Mesh&, const TextureView&, ...)`. The depth camera decides
  visibility and the colour camera gives each vertex its coordinate, which it
  keeps only where the image recorded it, both cameras see the same side of
  the surface, and the colour camera's line of sight, walked through the
  depth map, is clear; per triangle, the colour camera must also see its
  front. A view's device depth and coverage are copied on the device into the
  pass's buffers, never staged through the host, and timed in its row.
  Without a colour camera the result is unchanged. Tests:
  `recon_texture_multiview`, `recon_texture_device_mesh`. See the 2026-09-28
  decision.
- `core`: **copies join a rising run** in `CommandBatch`: copies from other
  buffers into one at rising, disjoint offsets need no barrier between them,
  as fills and uploads did not, unless a command in the run writes a copy's
  source. Test: `recon_core_command_batch`.
- `sensor`: **`JpegDecoder`** decodes the JPEGs an MJPEG camera sends, to
  I420. Given a device, with `VR_WITH_CUDA`, nvJPEG decodes an 8-bit 4:2:0
  JPEG straight into a Vulkan buffer CUDA has imported, on the GPU's
  hardware JPEG engine where it has one and its cores for the rest: on an
  RTX 5090, 0.38 ms of CPU a 4K frame against 14.3 in software, and the
  picture is already on the device. libnvjpeg is loaded at run time, like
  libcuda. Anything else
  decodes in software to host planes. `DecodedPicture::offset` grows a
  third plane for it.
- `sensor`: **NVDEC's pictures stay on the GPU** (`VR_WITH_CUDA`, Linux, the
  CUDA 13 toolkit's headers; libcuda is loaded at run time). Given `HevcDecoder::Options::device`, a CUDA decode copies each
  picture device to device into a Vulkan buffer CUDA has imported and hands
  it out as NV12 (`DecodedPicture::device`), rather than copying it to the
  host and converting it there: on an RTX 5090, 0.03-0.05 ms of CPU a 4K
  picture against 1.34, before the host path's staging copy and upload.
- `core`: **`create_exported_buffer`** makes a device-local storage buffer on
  memory of its own, with a file descriptor another API on the GPU imports.
  `Device::create` enables `VK_KHR_external_memory_fd` where the GPU offers
  it (`Device::exports_memory`), `adopt` honours it where declared, and
  `DeviceRequirements::external_memory` asks a shared device for it.
- `sensor`: **colour already on the device, and NV12.** `YuvImage` names its
  chroma layout (`layout`: I420 or NV12) and takes its planes from a device
  buffer (`device`, with per-plane `offset`) as well as from the host.
  `GpuFramePrep` binds device planes where they are, so a hardware decoder's
  picture need never cross the bus, after taking the buffer over from the
  queue family that wrote it (`queue_family`: `kQueueFamilyExternal` for
  CUDA). Device planes that overlap are refused.
- `core`: **`CommandBatch::acquire`** takes over a buffer another queue family
  or an API outside Vulkan wrote, before the commands after it use it.
- `texture`: **`ProjectiveTexturer::texture(const DeviceMesh&, const Buffer&
  depth, ...)`** binds a depth frame already on the device, in place. The atlas
  must be registered to the depth camera; a `GpuFramePrep` frame, whose colour
  keeps a camera of its own, takes the `TextureView` overload listed above.
- `core`: **`CommandBatch::reserve_upload`** hands the caller the staging for
  an upload to pack itself, such as strided rows or several planes. It,
  `upload` and `copy` take an optional `GpuStageScope` that times the copy.
- `sensor`: **raw sets from the rig.** `OrbbecRig::poll_raw_set` hands out a
  set of `RawFrame`s from a rig opened with `raw`, and `poll_raw` the same
  frames one at a time, as the contract's `raw_frames()` says. `prepare_set`
  prepares a set on the device, a thread and a `GpuFramePrep` per camera.
  `fuse_orbbec --rig --gpu` fuses them. `OrbbecRigFrameSet` and
  `OrbbecRigRawSet` are one template, `OrbbecRigSet<Frame>`.
- `core`: **`CommandBatch`** (`core/command_batch.hpp`): one call's uploads,
  fills, copies, dispatches, indirect dispatches and readbacks recorded into one
  command buffer and submitted with one fence wait. Small aligned uploads go
  inline, larger ones through a staging buffer the batch allocates, and
  readbacks (small results) through one host buffer; nothing goes through a
  mapping. On an RTX 5090, memory the kernels use in VRAM rather than
  host-visible takes `integrate` from 14.6 to 0.067 ms of device time (the
  2026-09-28 residency decision). `dispatch()` is a batch of one, `zero`
  clears a range at any alignment, and fills and inline uploads rising
  through one buffer without overlap share a barrier, which takes 4 096
  scattered fills from 7.0 to 0.78 ms on the 5090. It refuses a kernel whose
  descriptor set was rewritten after its dispatch was recorded
  (`DescriptorSet::writes`) and a push off 4 bytes or past the kernel's range
  (`ComputeKernel::push_bytes`;
  `KernelSetBuilder::add` now refuses a push range off offset 0).
  `submit_single_time` reports a failed wait through `in_flight`, and the
  batch then leaks its staging. Test: `recon_core_command_batch`.
- `tsdf`: **`MeshIntegrator`** — a triangle mesh's truncated distance field,
  written into a grid's `tsdf` and `weight`, in one of two `MeshSdfMode`s (see
  the 2026-09-27 decision).
  - `Signed`: +-distance, signed by the closest triangle's face normal. For a
    closed, outward-wound mesh; nothing is checked and no topology is built,
    so past an edge sharper than 90 degrees the sign can take the wrong side,
    and an open mesh grows a skirt past its rim.
  - `Shell`: distance minus a half-thickness (1.5 voxels by default), for any
    mesh at all.
  - Every voxel of every band block is overwritten: weight 1 within
    `trunc_dist`, `tsdf = 0, weight = 0` elsewhere, which is how the codec's
    inverse leaves a fresh block. The band must be allocated first with
    `allocate_from_triangles`; a missing block is refused before anything is
    written.
  - Triangles are binned per block (a count and a fill dispatch over the
    allocation's own work items), so no voxel measures the whole mesh. Ties
    break on the triangle index, so the same mesh writes the same bytes.
  - A bin past `MeshIntegrator::kMaxBinTriangles` is refused, and the write
    splits into dispatches of at most `kMaxDispatchBinEntries` bin entries;
    `MeshIntegrateStats::dispatches` reports how many.
- `volume`: **`triangle_candidate_offsets`** — the per-triangle candidate-block
  prefix sum `allocate_from_triangles` dispatches over, public so the mesh
  integrator bins over the same decomposition. Its decode and band test moved
  into `shaders/triangle_candidates.glsl` for the same reason.
- `core`: `vrClosestPointOnTriangleFeature` in `shaders/triangle_common.glsl`
  also reports which vertex, edge or face the closest point lies on.
  `vrClosestPointOnTriangle` is now a wrapper over it, with the same
  arithmetic.
- `volume`: **`VoxelHashMap::allocate_from_triangles`** — the blocks a triangle
  mesh's truncation band covers, which is what a mesh-to-SDF pass then writes.
  Not expressible as `allocate_from_points` over the vertices: that dilates each
  point into the `(2*tb+1)^3` cube, one block (40 mm) wide at the defaults, so
  any triangle wider than that left an unallocated hole through its middle. A
  block is allocated when its centre lies within `trunc_dist` plus the block's
  half-diagonal of some triangle — conservative, so no block holding a voxel in
  the band is missed, and tight enough that a large slanted triangle allocates a
  sheet rather than the solid interior of its bounding box. The work is split
  per *candidate block* rather than per triangle, over a host-computed prefix
  sum, so a single large quad cannot land a quarter-million bucket-locked
  inserts on one lane. That host pass also bounds-checks every index (the kernel
  indexes `vertices` directly and `robustBufferAccess` is enabled nowhere here)
  and drops zero-area and non-finite triangles, which a mesh file routinely
  carries and which should not fail the whole allocation. Opt-in `StageMetrics*`
  reports an `"allocate"` row, as `allocate_from_depth` does. See the 2026-08-31
  decision.
- `core`: `shaders/triangle_common.glsl` — closest-point-on-triangle, shared
  by the block allocation above and (next) the mesh-to-SDF integrators, so the
  blocks one allocates and the voxels the other writes cannot drift apart.
- `texture`: **several views into one atlas.** `ProjectiveTexturer::texture`
  takes `TextureView`s (a depth map, its camera, and the size of the colour
  image registered to it, which may be larger) and an `AtlasLayout`; each
  triangle takes the view that faces it most squarely among those that see
  its front and all three vertices, and its vertices point into that view's
  tile, the colour image at its own resolution. Needs an unshared mesh; a
  shared one is refused, as are a view with no depth range and overlapping
  tiles. `texture_atlas.hpp` lays the images out side by side
  (`side_by_side_atlas`, wrapping past the device's largest image) and packs
  them (`pack_atlas`). gfx is unchanged. Test: `recon_texture_multiview`;
  `recon_texture_device_mesh` checks device against host.
- `sensor`: **`ICameraCapture::poll_raw`** and **`raw_frames`**: a source
  opened for raw frames hands them out through the capture contract, so a
  consumer asks which kind a source serves instead of reaching for its
  concrete type. Both non-pure: a source that serves none keeps
  `Unsupported` and `false`.
- `sensor`: **`OrbbecStreamOptions::raw`** and **`OrbbecCapture::poll_raw`**
  hand out a `RawFrame`: raw depth and the decoded I420 planes (H.265 colour
  only) with the matrix and range the stream codes them in, and each camera's
  lens and pose from the factory calibration, with the depth camera posed
  through its extrinsic to the colour one. The host undistorts, registers and
  converts nothing. `fuse_orbbec --gpu` fuses such frames through
  `GpuFramePrep`, one camera for now (`OrbbecRig` refuses `raw`).
  `ycbcr_weights` gives a `VideoColorMatrix`'s luma weights.
- `sensor`: **`GpuFramePrep`** (`sensor/utils/gpu_frame_prep.hpp`, the new
  `recon_sensor_utils` target): a captured frame's depth and colour
  undistorted on the GPU, and its Y'CbCr 4:2:0 colour converted to R'G'B' in
  the same pass, handed over as device-local buffers the device-input fusion
  overloads read. It takes a `RawFrame` (`sensor/raw_frame.hpp`: raw depth,
  the decoded planes, and each camera's `LensCamera` from `sensor/lens.hpp`,
  which `LensDistortion` moved to), so depth and colour keep their own
  intrinsics and poses and nothing registers one to the other. The
  `DeviceFrame` it returns holds its buffers, so one kept past the next
  frame keeps its contents; colour carries each pixel's coverage in its high
  byte; and a frame is checked whole, its depth range included, before
  anything is uploaded. Test: `recon_sensor_gpu_frame_prep`.
- `volume` / `tsdf`: **device-input overloads** of
  `VoxelHashMap::allocate_from_depth` and `TsdfIntegrator::integrate` that
  take the depth image as a storage `Buffer` already on the GPU, and
  `ColorFrame::buffer` for the colour image, read in place with no upload
  and bound at the image's exact range. The same frame fused both ways gives
  the same grid (`recon_tsdf_device_input`); a buffer that is empty, not a
  storage buffer or smaller than the image is refused before any work, an
  empty grid's call included (`core`'s `StorageInput`, which both tiers bind
  through). **`ColorFrame::coverage_in_alpha`** has fusion skip a colour
  pixel whose high byte is 0, as `GpuFramePrep` marks one its lens maps
  outside the picture.
- `sensor`: **H.265 colour from Orbbec cameras**,
  `OrbbecStreamOptions::color_codec = OrbbecColorCodec::Hevc` (the default
  stays `Mjpeg`; needs `VR_WITH_FFMPEG`). 21.6 Mbit/s of colour per camera at
  720p30 against MJPEG's 37-38, and 21 against 185 at 4K25. Each camera's
  colour is decoded on a thread of its own, every frame in order, as
  BT.601 full range when the stream declares no matrix, which is how the
  Femto Mega codes it without saying so. `open` refuses a camera whose H.265
  mode's calibration is not its RGB mode's.
  `OrbbecCaptureStats::lost` counts the pairs it cannot hand on. Tests:
  `recon_sensor_orbbec_hevc` (no camera); the capture and rig hardware tests
  run both codecs.
- `sensor`: **`HevcDecoder::Options::unlabelled_color`**, the matrix and range
  for a stream that declares no matrix, and **`HevcDecoder::reset()`**, which
  starts the stream afresh after lost access units, skipping a CRA's leading
  pictures.
- `examples`: **`fuse_orbbec --hevc [--color WxH] [--fps N]`**.
- `sensor`: **`HevcDecoder`** (`sensor/video/hevc_decoder.hpp`, the new
  `recon_sensor_video` target behind `VR_WITH_FFMPEG`): H.265 access units to
  host pictures, `Rgb24` or `Yuv420`, over an installed FFmpeg ≥ 4.4. `Auto`
  takes the first hardware back end that decodes HEVC (VideoToolbox, as
  `VTIsHardwareDecodeSupported` answers; CUDA, then VAAPI on Linux and CUDA,
  then D3D11VA on Windows, by decoding a built-in clip), else software. Tests:
  `recon_sensor_video_hevc`, which `VR_TEST_HEVC_BACKEND` can hold to one
  back end, `recon_sensor_video_converter` and `recon_sensor_video_backend`.
  CI builds it on every leg and requires NVDEC on Linux and VideoToolbox on
  macOS. Pictures carry the stream's transfer and primaries
  (`DecodedPicture::encoding`), and a display window off the coded corner is
  cropped on every back end (VideoToolbox, which cannot, refuses such a
  stream, and `Auto` moves it to software). The installed package holds a
  consumer to the FFmpeg major versions it was built against.
- `eval`: **a quality-measurement tier**, `recon_eval`, off `recon_mesh`.
  `MeshDistance` gives point-to-surface distance up to a reach, over a copy
  of the triangles. `compare_meshes` gives accuracy, coverage and an optional
  F-score at a threshold, and `ReferenceMesh` indexes a reference once to
  judge many meshes against. They return `Status` on a bad reach, indices out
  of range, a corner that is not finite or too far out, a reach too small for
  the triangles, or a threshold past the reach. Only the surface is measured:
  triangles collapsed to a point, and vertices no triangle uses, are left
  out. A stride picks vertices by position, so the figures reproduce. The
  metric came from `examples/common`, where the room0 measurement introduced
  it, and it carries that header's review fixes: the closest point is exact
  to float rounding on thin and degenerate triangles, and a pruned search of
  half-reach cells compares room0 at 1 cm in 1.9 s, down from 8.3 s. Tested
  by `recon_eval_mesh_distance`.
- `examples`: **`codec_replica`**, the TSDF codec on real data. It fuses a
  Replica sequence and streams the grid through `Encoder` / `Decoder`,
  reporting bytes, bitrate and stage rows. It then judges the decoded surface
  against the source's, mesh to mesh (accuracy and coverage, each with its
  count beyond reach). `--sweep` prints a rate–distortion table. The metric
  is the `eval` tier's (below). The player grid shares the fusion grid's
  layout, now `examples/common/grid_layout.hpp`.
- `codec`: **`Encoder` and `Decoder`, the codec's public API** (see the
  2026-09-27 decision). This is the third of its five PRs.
  - `Encoder::encode(grid)` gives one intra frame of every block with an
    observed voxel, sorted so the bytes do not depend on hash order.
  - `Decoder::decode(frame, grid)` leaves a caller's grid holding exactly the
    frame. It diffs the grid's block set rather than clearing it, and checks
    everything checkable before touching the grid. A grid too small for the
    frame, in its heap or its hash table, is `OutOfMemory`; resize and decode
    again. Lock contention that outlasts the retries is `IoError`. A grid
    declaring any attribute besides `tsdf` and `weight` is refused, since a
    kept block would carry it over stale.
  - `read_frame_info` reads a frame's header, so a player can build a grid of
    the stream's geometry.
  - Both report `StageMetrics`.
  - On an analytic sphere at the defaults (K = 32): 35.8 B/block, with the
    decoded mesh at worst 0.27 voxels off the true surface.
  - Tests: `recon_codec_encoder` and `recon_codec_decoder` (GPU).
- `volume`: `VoxelBlockGrid::attribute_count()`, the number of attributes a
  grid declared.
- `sensor`: **`OrbbecRig`**, a hardware-synced rig of Orbbec cameras, opened
  from the rig's sync configuration (`orbbec_sync_config.hpp`, the SDK's
  `MultiDeviceSyncConfig.json` layout). It refuses cameras whose settings
  differ, unless `apply_sync_config` writes them; starts the secondaries
  first; keeps the cameras on the host's clock; and hands out one set per
  primary frame (`poll_set()`), or those frames one at a time (`poll()`). A
  missing secondary leaves an empty slot. Tests: `recon_sensor_orbbec_rig`
  (only the rig `VR_ORBBEC_TEST_RIG` names), `recon_sensor_orbbec_grouping`,
  `recon_sensor_orbbec_sync_config`.
- `sensor`: **the rig calibration file** (`rig_calibration.hpp`): read and
  write the family's config layout, each camera's OpenCV `pose` turned into
  camera-to-world, its lens fields kept. Test: `recon_sensor_rig_calibration`.
  Parsed with nlohmann/json 3.12.0, a new pinned header-only dependency.
- `config/`: **the lab rig's sync configuration**, `femto_mega_sync.json`
  (one primary, three secondaries at 160/320/480 µs), kept valid by
  `recon_sensor_orbbec_sync_config`.
- `examples`: **`fuse_orbbec --rig sync.json [--apply-sync]
  [--calibration calib.json]`** fuses the rig; `--serial` with
  `--calibration` poses one camera from the file.
- `codec`: **the rANS reference coder and the v1 intra frame**, both internal.
  This is the second of the codec's five PRs (see the 2026-09-27 decision).
  - `rans.hpp`: static-table rANS with a 32-bit state, 16-bit words and 12-bit
    probabilities, integer only. It is the reference the GPU coder must match
    byte for byte.
  - `bitstream.{hpp,cpp}`: `write_intra_frame` / `read_intra_frame`. A frame
    holds a 44-byte header, a section table, fixed per-frame tables, and
    segments of R sorted blocks (default 64), each an independent rANS stream.
    Every integer is a class plus raw bits.
  - The reader never reads outside its buffer. It refuses unknown required
    sections and skips optional ones, and it takes the caller's `max_blocks`,
    which bounds its allocation (under 2.1 KB per block). Every size is
    checked in 64 bits, so a 32-bit build refuses a frame rather than wrapping.
  - The format requires strictly increasing block coordinates. The reader
    checks the order across segments, where a segment's first coordinate is raw
    bits.
  - Flag bits other than "required" are reserved, so a known section that sets
    one is refused. Table varints must be canonical.
  - Tests: `recon_codec_rans` and `recon_codec_bitstream`, both host-only.
- `sensor`: **the Orbbec (Femto Mega) driver** — `OrbbecCapture`, an
  `ICameraCapture` over one camera, as its own target
  (`volumetric_kit::recon_sensor_orbbec`, built with `VR_WITH_ORBBEC`) so
  `recon_sensor` stays vendor-free. Each `poll()` hands out the newest
  synchronised pair, colour undistorted and depth registered to it, in metres
  and packed colour; `open` reports the camera's rig sync role without changing
  it. See the 2026-09-26 decision.
- `examples`: **`fuse_orbbec`** — live reconstruction from an Orbbec camera to
  a PLY (built with `VR_WITH_ORBBEC`). The grid setup the four examples shared
  is now one `create_fusion_grid` in `examples/common/fuse_frame.hpp`.
- `codec`: **a new tier, `recon_codec`, and the block DCT it is built on.** It
  is the first of five PRs toward a per-frame TSDF geometry codec, with separate
  `Encoder` and `Decoder` classes, a geometry-only intra frame and chunked
  static-table rANS (see the 2026-09-26 decision). This one lands:
  - the public `codec::CodecParams`: coefficients kept per block, and DC / AC
    quantization steps as fractions of `trunc_dist`. `validate()` refuses a
    step fine enough to overflow the ±32767 clamp.
  - the internal `DctTransform`: GLSL forward and inverse 8³ DCT-II kernels
    with one 64-invocation workgroup per block. They turn a `volume::BlockList`
    into a `DctBlocks` and back: the first K zigzag-ordered quantized
    coefficients plus a 1-bit observed mask per voxel, beside the params and
    `trunc_dist` they were made with, which the inverse checks against its
    grid. The forward fills unobserved voxels from the nearest observed ones
    rather than transforming the zeros a fused block holds there, which
    decoded a partially observed block with 2.8x the error. Each workgroup
    checks through the hash table that its entry is a live block, so a free
    slot or a mis-paired coord is refused rather than written. Lists longer
    than the device's workgroup limit are batched.
  - tests: `recon_codec_params` (host-only, pinning the whole zigzag table) and
    `recon_codec_dct`, which checks the kernels against a double-precision
    reference of the same fill and transform.

  The tier links `recon_volume` alone and is in the `volumetric_kit::recon`
  umbrella. There is no encoder, decoder or bitstream yet.
- `volume`: **`kObservedWeight`, `VoxelBlockGrid::check_block_list`, and a
  deleted rvalue `block_list`.** The observed-weight threshold and the O(1)
  `BlockList` checks (null with a count, more blocks than the heap, another
  topology epoch) are now defined once, for `mesh` and `codec` both.
  `block_list(map().compact_active_blocks().value())` no longer compiles: the
  list borrowed a vector that died at the end of the statement.
- build: **the Orbbec SDK as an opt-in prerequisite** — `VR_WITH_ORBBEC`
  (off by default) finds an installed SDK ≥ 2.9.3 and exposes `ob::OrbbecSDK`
  for the Orbbec (Femto Mega) capture driver that follows. Nothing is fetched
  or vendored: the SDK is installed once, outside every repo, and each build
  points at it with `-DOrbbecSDK_ROOT=<sdk>` or the `OrbbecSDK_ROOT`
  environment variable (`cmake/vr_orbbec.cmake`), which is authoritative:
  re-pointing it at another SDK takes effect in an existing build tree. An SDK
  older than 2.9.3 or of another major version, a missing one, or an iOS
  target stops the configure with the fix in the message.
  `recon_orbbec_sdk_smoke` pins that the copy CMake found is the copy that
  loads, and that an SDK context opens and enumerates devices, with or without
  a camera attached. CI builds and runs it on the ubuntu-24.04 and macos-26
  legs, against the pinned 2.9.3 release it installs per job.
- `sensor`: **`ICameraCapture::exhausted()`** — whether a source will never
  hand out another frame. An empty `poll()` says only "nothing this tick", and
  a live device polled faster than it runs and a replay that has played its
  last frame return the same empty optional; a consumer that would wait for
  the first must end on the second. Non-pure and `false` by default (a live
  device is never exhausted, only stopped), so existing drivers compile
  untouched; a finite source overrides it.
- `core`: **GPU-profiler labels** — `VK_EXT_debug_utils` names, so an Nsight
  Graphics or Xcode Metal capture reads `tsdf_integrate` and `tsdf.depth_frame`
  rather than a wall of anonymous dispatches over unnamed handles. Every
  `dispatch()` wraps its submission in a region named by `ComputeKernel::name`,
  which also names the kernel's `VkPipeline` (the object a profiler indexes
  shader cost by), and `Device::set_object_name` names the buffers each tier
  holds — re-applied wherever a handle is replaced, since a name lives on the
  handle. Labels are **not** tied to a `GpuStageScope`: a span is opt-in and
  costs a timestamp, a label is free and unconditional.
  `InstanceConfig::request_debug_utils` defaults **on** and is independent of
  `enable_validation`, because a Release build is the only one worth profiling.
  **Note for downstream:** four source-breaking changes. `KernelSetBuilder`
  takes a `const Device&` rather than a `VkDevice`, and its `add()` gained a
  required `name` second parameter (a borrowed string literal that must outlive
  the kernel). `Device::end_debug_label` takes the name its `begin` was called
  with, so the pair skips on identical conditions. `DeviceRequirements` gained
  `debug_utils`, `DeviceConfig` gained `instance_debug_utils_enabled`, and
  `AdoptedDevice` gained `enabled_debug_utils` — debug utils is an *instance*
  extension, so it cannot ride `enabled_device_extensions` and the creator
  declares it instead; leaving any of them unset costs the capture's names and
  nothing else. Prefer the new `Device::create(const Instance&, …)` overload,
  which fills the declaration in for you.

- `mesh` / `volume`: **view-culled meshing**. `MarchingCubes::extract_device`
  takes a `volume::BlockList` — pointer, count, and the `topology_epoch` it was
  compacted at — and meshes that subset instead of compacting the whole map, so
  a scanning device that renders a small part of a large volume meshes only that
  part. Nothing in the extractor knows a frustum produced it: a region of
  interest or a chunk queue is the same call. `volume::make_frustum_planes`
  gains an overload that reads the six planes off a *render* camera's
  `view_proj` (Gribb-Hartmann), so it holds for any handedness provided depth
  maps to `[0, 1]`, with a `margin_m` in metres for a consumer whose cull runs
  behind its draw. Build the list with `VoxelBlockGrid::block_list`, which
  stamps the epoch off the grid that owns the blocks so the triple cannot be
  mispaired.
  The arena is rebuilt from the culled dispatch alone, so a block outside the
  list costs no triangles and no live bytes; the surface does not hole at the
  cull edge, since the on-device probe still resolves neighbours that were never
  dispatched. Offered on `extract_device` only — an incremental pass keeps the
  triangles of blocks it does not re-mesh, so culling would leave them drawn.
  Alternating the two is safe: a culled pass publishes no arena state, so the
  next incremental request falls back to a full extract and reports it.
  **Note for downstream:** `volume::BlockList::count` is `uint32_t` (it was
  `int32_t`) and the struct has a third member, so a two-element aggregate
  initializer no longer compiles.

- `mesh`: incremental extraction now works under **`share_vertices`**, which is
  the configuration the only device consumer runs and the one that decides
  whether any of this ships. The sharing kernel reserves two per-block ranges
  already (since the stage-2 work), so this teaches it to *reuse* them: in place
  when **both** the vertex and triangle counts fit, appending otherwise. Both,
  not either — the two ranges are allocated independently and a triangle indexes
  into the vertex range beside it, so reusing one while relocating the other
  would leave the kept range's triangles pointing at vertices that moved. That
  coupling is what sharing adds over the default kernel, where `v = 3t` makes one
  test serve both.
  **Retirement is an order of magnitude cheaper here**, because this kernel owns
  its index run: a dead triangle is retired by pointing its three indices at one
  vertex — 12 bytes, zero area, culled before rasterisation — where the default
  kernel, whose run is the identity it cannot touch, overwrites 192 bytes of
  vertices to say the same thing. The dead *vertices* need no writing at all:
  sharing is in-block and the `+face` is duplicated, so once no triangle
  references them they are unreachable rather than merely unused.
  The memory argument this existed to answer: room0 holds **39.5 MB with 310 312
  triangles for 277 506 live — 1.12x inflation**, against 1.82x and a 4 177 MB
  device arena without sharing.
  Occupancy is asked on **both** axes: retirement leaves dead triangles occupying
  index slots while dead vertices are merely unreachable, so the two buffers
  drift apart and either can be the one that reaches
  `maxStorageBufferRange` first. Without sharing the second test is the first
  restated and changes nothing.
  The decisive test now runs over **both** kernels: the field is changed under a
  clean extract, so all-zero flags must return the old surface (a silent fallback
  returns the new one and fails) and all-set flags must reproduce a full extract
  exactly once retired degenerates are dropped. It did not before — the loop that
  selects the emitter never read its own induction variable, so `share_vertices`
  stayed false and the two iterations ran the same kernel twice, `-Werror` silent
  because the loop header used it. Wiring it up failed immediately on
  `remeshed_blocks`, which this kernel declared and never incremented: the
  counter had been added to the sibling alone, on the assumption this change
  reverses.

### Fixed

- `volume`: **removing many blocks at once no longer loses them from the free
  heap.** The delete kernel appended each freed block with a compare-and-swap
  loop capped at 256 tries, and a thread that ran out dropped its block for
  good, counted only in `AllocFailures::terminal`. Removing 2 048 of 16 384
  blocks in one call lost about 960 of them on an M5 Max and 1 100 on an RTX
  5090, and every such call shrank the grid further; the codec's decoder
  removes blocks every frame. One `atomicAdd` now claims the slot. The
  allocate kernels popped the heap with the same loop and reported
  `kFailHeap`, an empty heap, for one that was only contended: on the 5090,
  355 of 16 384 blocks failed that way after every retry round with half the
  heap free, which a caller answers by growing the map. They pop with one
  `atomicAdd` too.
- `volume`: **one `remove` call finishes what bucket-lock contention holds
  up.** Removing 8 192 blocks left 1–6 behind in 5 of 600 calls on the RTX
  5090 (Debug), none lost, which failed `recon_volume_delete`. The retry
  loop stopped after two rounds without progress, reading that as a capacity
  limit a remove never hits, and each round re-ran every coord, so blocks
  already removed took their bucket locks again. Now only a capacity limit
  ends any kernel's rounds early, and the delete kernel flags each coord it
  settles so later rounds skip it; either change alone left none behind in
  600 calls. What contention still holds after every round is counted as
  `AllocFailures::lock`, as allocation's is: call again.
- `volume`: **the hash table's bucket locks live in device memory.** They were
  host-visible like every buffer the map owns, so on a discrete GPU each spin
  was an atomic across PCIe. On an RTX 5090, allocating a 5 000-triangle sheet
  took 1.97 s against 3.4 ms now, and a 320 000-triangle one ran past the
  driver's 7-second watchdog (NVIDIA Xid 8 / 109), which is what failed the
  ubuntu-26.04 legs of #81. Every allocation path gains, not only triangles.
  Apple's unified memory never saw a difference. `core` gains
  `device_storage_buffer` for memory only the kernels touch.
- `mesh`: a triangle the sharing kernel drops for a vertex-claim overflow
  **inside a reused range** kept the previous extract's index triple. Those
  indices are in range, so nothing faults, but they name three unrelated vertices
  of the *new* surface and draw a full-area triangle across the block — and the
  retire pass cannot reach them, since it starts past the live sub-range by
  construction. The slot is retired in place now, with the same degenerate.
- `mesh`: the sharing kernel's dirty dilation had **no `barrier()`** between
  invocation 0 zeroing `s_dirty` and invocations 0..7 `atomicOr`-ing into it, so
  a lane that ORed early had its bit clobbered and a changed block took the clean
  early-return — keeping stale triangles for a surface that had moved, under
  `Status::ok`. Latent where lanes 0..7 share one SIMD group, as on Apple; live
  on any implementation with a smaller subgroup or independent thread
  scheduling. The sibling kernel has the barrier at exactly that point.

- `mesh`: `MarchingCubes::extract_device_incremental` — **stage 3**. Blocks whose
  `+{0,1}³` neighbourhood carries no change keep the triangles they already have,
  at the offsets `block_spans()` already names, and cost one workgroup that
  returns before gathering a corner. A changed block re-meshes into the range it
  owns when the new count fits, and appends past the watermark when it does not.
  **No host work in the loop.** The dilation from the *changed* set to the
  *re-mesh* set happens on-device on the eight blocks `s_neighbour` already
  holds, so there is no readback, no set union and no upload — the same move the
  2026-08-08 decision made for the neighbour table. Retirement is per block and
  local (three identical vertices, culled before raster), so no prefix sum and no
  index rebuild is needed and the run stays the identity.
  What a pass may trust is one `{watermark, epoch, serial}` struct, **cleared at
  the top of both extract paths and re-established only on the publishing
  return**, so no failure — and no dense extract, which rewrites the same arena
  with a kernel that knows nothing about blocks — leaves it describing geometry
  that is gone. Anchors are compared *above* the call that re-anchors them.
  Falls back to a **full** extract, by design, when an incremental one would be
  wrong rather than slower: the first extract against a grid, a topology change
  that retired either anchor, flags the integrator will not vouch for, an arena
  that has to grow (it reallocates without copying), an overflow refit (whose
  retry has already lost the pre-call spans), occupancy past 2x the live
  surface, or more than one slot. Which one the caller got is reported
  as `ExtractTimings::incremental`, beside a device-counted `remeshed_blocks` —
  `dispatches` counts refit rounds and reads 1 on both paths, so it cannot say.
- `tsdf`: `TsdfIntegrator::dirty_flags_buffer()` / `dirty_flags_capacity()` /
  `dirty_epoch()` — the dirty set for an on-device consumer, instead of taking
  `dirty_remesh_blocks` back through the host. All three go null **together** on
  every staleness this tier can see (the topology-stale latch included, which
  `dirty_remesh_blocks` already refused on), and the epoch carries the one it
  cannot: *which grid*. It is the same globally unique token the span table
  anchors on, so one comparison in the consumer covers both "the right grid" and
  "no blocks removed since". `mesh` takes the three as an opaque `DirtyBlocks`
  and gains no dependency on `tsdf`.
  **It costs arena.** On room0 the arena holds 505 511 triangles for 277 506 live
  — 1.82x — because retirement leaves degenerates and, at that scene's dirty
  rate, relocation is constant; the live count is summed off the spans so the
  density estimate is not fed its own dead triangles, and occupancy past 2x live
  schedules a full pass, which is the compaction. **And room0 cannot show the
  win**: 81.67% of its blocks re-mesh per window, so the ceiling there is 1.22x,
  and measured extract_device is 0.98 ms incremental against 0.94 ms full — both
  figures taken at a 100% dirty rate, which is what an unreset flag array
  produces. The ~4x lives at the iPad's 25%, unmeasured here.
  **An in-place re-mesh writes bytes an outstanding generation may be drawing.**
  Every index stays in range and every vertex stays a real vertex, so it is not a
  memory error — but a consumer holding a `DeviceMesh` across the call can catch
  one block mid-update. That trade is what this overload exists to make
  measurable; `extract_device` is unchanged and does not make it.
  `fuse_replica --incremental` drives it: it implies `--device-extract`, is
  refused beside `--dirty-every` (both consume the same flags on unrelated
  cadences), resets the flags immediately after the extract that read them, and
  reports how many extracts were really incremental and what fraction of blocks
  each re-meshed.

- Initial repository scaffolding: tiered layout, MIT license, `.clang-format`,
  `.cmake-format.yaml`, `.pre-commit-config.yaml`, `.gitignore`.
- `CLAUDE.md` — the living source of truth (objective, locked decisions, tier
  architecture, interop contract, salvage + exclusion policy, gotchas).
- `core` tier foundation: backend-neutral `Status` / `Result<T>` error handling
  (`VR_TRY` / `VR_ASSIGN`), `VR_CHECK` contract checks, a pluggable log handler,
  the version API, and portable POD math types.
- Tiered CMake with install/export and package config
  (`find_package(volumetric_kit_recon)` / `FetchContent`), warnings-as-errors,
  and sanitizer support.
- Direction set to a single **Vulkan compute** path (MoltenVK on Apple),
  mirroring `volumetric_kit_gfx`, so the reconstruction backend and renderer
  share one cross-platform API and a trivial same-device interop seam.
- `core` Vulkan **compute foundation**: a VMA `Allocator` (VMA v3.3.0 vendored
  via pinned FetchContent, built in one TU), a RAII `Buffer`, a SPIR-V
  `ShaderModule`, `DescriptorSetLayout` / `DescriptorPool` / `DescriptorSet`
  (storage-buffer binding), a `ComputePipeline` (explicit descriptor layout +
  push constants, no SPIR-V reflection), the `UniqueHandle` owner for
  device-scoped Vulkan handles, and a shared-queue-safe
  `Device::submit_single_time` / `queue_submit` dispatch primitive.
- GLSL → SPIR-V build step: `vr_compile_shaders()` (`cmake/vr_shaders.cmake`),
  targeting Vulkan 1.2 (the device floor; scalar block layout is 1.2 core).
- Vulkan **compute smoke** (`tests/compute_smoke_test.cpp` + `tests/shaders/
  fill.comp`) proving the end-to-end path — allocate → bind → dispatch → read
  back — on MoltenVK, plus move-only RAII tests (`tests/compute_raii_test.cpp`)
  the sanitizer job turns into leak/double-free detectors.
- `volume` tier host math: `VoxelGridParams` (grid + hash-table shape, the
  scalar-layout shader ABI), world/voxel/block coordinate transforms
  (`volume/voxel_coords.hpp`), and the Teschner spatial hash + slot sentinels
  (`volume/hash.hpp`), with CPU tests.
- `volume` **sparse voxel hash map** (first GPU slice): the host `VoxelHashMap`
  owns the device buffers + compute pipelines and drives init /
  allocate-from-coords / compact via GLSL kernels (`volume/shaders/hash_*.comp`,
  scalar-block-layout `HashEntry`/`BlockIndex`), embedded into the now-compiled
  STATIC `recon_volume` via `vr_embed_shaders` (`cmake/vr_embed.cmake` +
  `cmake/embed_spirv.cmake`). A GPU test (`tests/volume_hash_map_test.cpp`) proves
  allocate→compact + the on-device layout round-trip on MoltenVK.
- `core`: `Device` now enables `scalarBlockLayout` (Vulkan 1.2 core) — the
  compute-shader buffer ABI — on create, and `adopt` requires the creator did.
- `volume`: `VoxelHashMap::remove` + the `hash_delete_coords` kernel — delete
  blocks by coordinate (collision-chain splice + successor pull-up) and return
  each freed block to the heap. The delete searches authoritatively under the
  bucket lock — no lock-free existence pre-check, since (unlike allocate) a
  delete cannot trust a lock-free "absent" read — retries and counts a
  found-but-not-deleted coord so a lost lock race is never silently dropped, and
  flags a block that cannot be returned to the heap. A GPU test
  (`tests/volume_delete_test.cpp`) forces overflow chains and a tight heap to
  prove the chain splice, successor pull-up, and real heap reuse on MoltenVK.
- `volume`: `VoxelHashMap::resize` — grow the hash table to more buckets,
  **preserving each block's index** so per-voxel data survives. Snapshots the
  active set, re-inits the larger table, then a `hash_rehash.comp` kernel
  re-inserts each block with its ORIGINAL pointer (the shared `insert_block`'s
  new `preset_ptr` — reused across normal allocation and rehash, so the tested
  insert path stays single-sourced, instead of drawing a fresh block off the
  heap), and the host rebuilds the free-block heap to exclude those live indices.
  The grow is failure-atomic (the larger buffers are built off to the side and
  swapped in together) and re-drives the snapshot to absorb transient lock
  contention. `VoxelBlockGrid::resize` grows every attribute array first (copying
  the old contents forward), so a block keeps its `ptr` and its
  `tsdf`/`weight`/`color` data at the same offset. GPU tests
  (`tests/volume_resize_test.cpp`, `tests/volume_block_grid_test.cpp`) prove
  256 → 1024 growth with block indices preserved, the heap rebuilt to exclude the
  live set, per-voxel attribute data surviving the grow, and allocation past the
  old capacity still working on MoltenVK.
- `volume`: `VoxelHashMap::diagnostics` — host-side occupancy + health stats
  (active / overflow / collision-chain length, load factor, heap utilization)
  from the entries + heap counter; a GPU test
  (`tests/volume_diagnostics_test.cpp`) forces a collision chain and verifies the
  counts. The shared coord-kernel path (allocate + remove) now re-dispatches on
  failure to converge under same-bucket contention (a GPU spin-lock livelock); a
  non-zero return means a genuine capacity limit.
- `volume`: `VoxelBlockGrid` — a structure-of-arrays voxel attribute store
  (Open3D-style) composing a `VoxelHashMap` block index with named, independently
  allocated per-voxel attribute arrays (`tsdf`, `weight`, `color`, …), each
  `num_blocks * voxels_per_block` and keyed by `BlockIndex::ptr`. A consumer
  declares only the channels it needs, so a `volume`-only user allocates no
  per-voxel memory; the TSDF / colour integrators and meshing bind the attribute
  buffers they touch. A GPU test (`tests/volume_block_grid_test.cpp`) proves
  distinct, correctly-sized attribute storage, the composed map, and the error /
  move paths on MoltenVK.
- `tsdf`: new **`tsdf` tier** with `TsdfIntegrator` — classic projective TSDF
  integration of a posed depth frame (float metres, reusing `DepthCameraParams`)
  into a `VoxelBlockGrid`'s `tsdf` + `weight` attributes. One GLSL dispatch
  (`tsdf_integrate.comp`) runs a thread per voxel of each active block: project
  the node-centred voxel into the camera, `sdf = depth - Zc`, truncate at
  `±trunc_dist`, weight by inverse-square with a behind-surface dropoff, and fuse
  by a running average capped at `max_weight` — faithful to the prior engine's
  classic kernel (neural/triplane channels excluded). A GPU test
  (`tests/tsdf_integrate_test.cpp`) fuses a constant-depth plane and checks the
  per-voxel sdf/weight, the truncation boundary, and the two-frame weight cap on
  MoltenVK.
- `tsdf`: `TsdfIntegrator::integrate` gains an `IntegrationMode` (classic /
  dynamic). Dynamic integration adds one kernel branch — a voxel that projects
  into free space past the truncation band (`sdf > trunc_dist`) is cleared if it
  held prior weight, so a receded surface leaves no ghost geometry (classic keeps
  it clamped): the prior engine's stale-free-space clearing. The GPU test
  re-integrates a receding plane and asserts dynamic clears the stale voxel while
  classic keeps it. Bilinear depth sampling and colour follow.
- `tsdf`: the integrate kernel now samples depth **bilinearly** (was
  nearest-neighbour), falling back to the nearest sample when a tap is out of
  bounds, invalid, or the 2x2 taps straddle a depth discontinuity
  (`max - min > trunc_dist`) that would blend across a surface edge — the prior
  engine's `sampleDepthBilinear`. The GPU test projects an on-axis voxel onto a
  half-pixel tap boundary and checks a sub-`trunc` step blends (distinct from the
  nearest sample) while an over-`trunc` step falls back. Colour follows.
- `tsdf`: `TsdfIntegrator::integrate` gains an optional `ColorFrame` — fuse a
  posed color image into the grid's `color` attribute alongside depth. Color uses
  a **separate color camera** — its own `ColorCameraParams` (pinhole intrinsics +
  pose + dimensions, the color analogue of `DepthCameraParams` without the
  depth-range fields), projected per voxel and running-averaged with the same
  weights as the SDF. A voxel's **first color
  observation assigns** the sampled RGB — keyed on whether color was seen
  (`color_attr == 0`), not on the depth weight, so an unregistered color camera
  (or a depth-only warmup) does not blend the first color toward black. Dynamic
  mode clears a receded voxel's color whenever the grid carries the attribute,
  **including on a depth-only frame**, so no color ghost survives a recede. RGB
  is packed in a `uint`'s low three bytes, matching the mesh tier's `color`
  layout so meshing reads it directly. The GPU test fuses a constant color and
  checks the packed RGB per voxel, that an occluded voxel keeps zero, that a
  color camera shifted out of frame skips color while depth still fuses, that the
  first color after a depth-only warmup assigns the full RGB, and that a
  depth-only dynamic recede clears the color ghost.
- `mesh`: a second `MarchingCubes::extract` overload meshes straight off a sparse
  `volume::VoxelBlockGrid` — the real `tsdf`/`weight`/`color` blocks — via
  `mesh/shaders/marching_cubes_sparse.comp`. One invocation per voxel of each
  active block (the tsdf integrator's iteration); a cell on a block's `+face`
  resolves its cross-block corners through a **host-built 2×2×2 neighbour table**
  (each active block plus its seven `+x/+y/+z` neighbours, from the compacted
  active set), so the kernel needs no device-side hash probe and no access to the
  hash table's internal buffers. *(Superseded — the kernel resolves the
  neighbourhood on-device now; see the `Changed` entry below.)* When the grid
  carries a `uint32` `color`
  attribute each vertex's color is interpolated from it, else opaque white; a
  corner whose color is the integrator's `0` "colour unobserved" sentinel (a
  written colour carries alpha `0xFF`) also falls back to white rather than
  dragging the vertex toward black; `uv0` stays the `(-1,-1)` sentinel. Only the
  corner sampling differs from the dense kernel — the shared per-cell body (cube
  index, gradient normal, reversed winding, independent triangles) is factored
  into `mesh/shaders/marching_cubes_common.glsl`, `#include`d by both kernels.
  The host rejects a worst-case vertex arena larger than `maxStorageBufferRange`
  with a clean `Status` instead of an opaque allocation failure. The GPU test
  writes an analytic sphere into a real 6³-block grid so the surface crosses
  interior block boundaries, then proves the sparse mesh matches the dense path
  **triangle-for-triangle** (plus cross-block color, that an unobserved colour
  meshes white, and that a sub-threshold weight gates every cell out) — the
  exact-count equivalence being the cross-block-addressing proof — and checks the
  empty / moved-from / missing-attribute paths.
- `examples`: `fuse_replica` — the first example and the vertical slice running
  end-to-end on real data. Reads a posed Replica-SLAM RGB-D sequence (nvblox's
  `fuse_replica` layout: `results/frameNNNNNN.jpg` + `depthNNNNNN.png`, row-major
  `traj.txt` camera-to-world poses, `cam_params.json` intrinsics/scale), fuses
  each frame into a sparse TSDF+colour volume (`allocate_from_depth`, growing the
  map via the block-index-preserving `resize` on overflow, then `integrate`
  depth+colour), extracts a marching-cubes mesh, and writes a coloured binary
  PLY. A small `examples/common` reader (stb_image colour/depth decode + a
  tinyply PLY writer, via pinned examples-only FetchContent, plus the poses)
  activates the reserved `examples/` slot. Verified on Replica room0 (400
  frames): a coherent
  4.0×4.4×2.8 m room, unit normals, plausible surface colour, triangle count
  converging, at ~60 fps on MoltenVK.
- `examples`: gfx-linked **viewer examples** (`examples/viewer/`, behind an
  off-by-default `VR_BUILD_VIEWER` that FetchContents `volumetric_kit_gfx` + finds
  GLFW — the only place the recon tree touches the renderer; the tiers + default
  build + CI stay renderer-independent). `fuse_render` fuses a Replica sequence
  and renders the coloured reconstruction to a **PNG** headlessly through gfx's
  `HybridMeshPipeline` + an `OffscreenTarget` (per-vertex colour on the `uv0`
  sentinel; optional `--follow N` renders from a sensor pose). `fuse_viewer` opens
  a **live window** — the nvblox `FuserVisualizer` analogue — fusing on a
  background thread (load/decode/integrate/extract off the render thread) while
  the render thread draws the growing mesh each frame following the capture
  trajectory (a host-mesh handoff, interop seam A, two devices).
  `recon_gfx_bridge.hpp` converts `mesh::Vertex` → `gfx::assets::Vertex`
  (synthesizing `tangent`, keeping the `uv0` sentinel). Verified on Replica room0:
  a correct first-person coloured room render.
- `mesh`: `MarchingCubes::block_spans()` publishes **where each block's geometry
  landed** — vertex base/count and triangle base/count per block slot
  (`BlockIndex::ptr / voxels_per_block`), written by both sparse kernels. This is
  the mapping the per-block reservation computes and used to drop, and it is
  **not derivable on the host**: the atomics hand ranges out in workgroup arrival
  order, not block order, so nothing outside the dispatch knows which range
  belongs to which block. It is what stage 3 re-meshes against. Counted in
  vertices and **triangles**, not indices, because a triangle is what a block
  owns and what a re-mesh replaces; the four numbers are independent under
  `share_vertices` and locked at `v = 3t` without it.
  **Opt-in** behind `MarchingCubesConfig::track_block_spans`, off by default: the
  table is sized by the grid rather than the surface (`num_blocks` entries — 24 MB
  at `VoxelGridParams::defaults()`, doubling with every `VoxelHashMap::resize`),
  so a caller who does not read it allocates nothing and the kernel is told not to
  write it, the bargain `TsdfIntegratorConfig::track_dirty_blocks` strikes for the
  same table shape. It is counted in `ExtractTimings::arena_bytes`, and grown
  beside the arena so the allocation lands in `arena_alloc_ms` rather than in the
  descriptor row.
  **Borrowed and retired by generation.** The accessor returns mapped device
  memory, which a grow frees, so `block_spans_generation()` carries the same
  counter as `DeviceMesh::generation`: a consumer compares the two rather than
  being told in prose not to cache the pointer. It returns `nullptr` until an
  extract has left a table describing its own output — both kernels publish a
  span *before* their capacity guard (deliberately: the counters must carry each
  block's full total for the host's refit), so a failed, empty or dense extract
  leaves spans naming triangles the arena never held. **Not per slot**, unlike the
  arena and index run: one table describes the current dispatch, and the
  generation is what makes a mismatch against a held mesh visible.
  A grow carries the existing spans forward and zeroes only the new tail
  (`VoxelHashMap::resize` preserves block indices, which is why
  `topology_epoch()` does not move across one), mirroring
  `TsdfIntegrator::prepare_dirty_flags`.
  The host/device mirror is pinned per field: `BlockSpan` is four same-typed
  `uint32`s, so `sizeof` alone cannot see a transposition — it carries `offsetof`
  asserts like every other mirror in the repo, is declared once in
  `shaders/marching_cubes_block_span.glsl` rather than copied into two kernels
  that could disagree on field order, and both kernels assign it by name.
  A slot is meaningful only against the grid and topology epoch that produced it,
  and only for the blocks the *published* extract meshed — nothing is cleared on
  the way past, so every other entry reads as a well-formed span belonging to an
  earlier one. `block_span_valid` is what separates them; there is no value in
  the table that means "not mine".
- `mesh`: `MarchingCubes::block_span_valid(grid, slot)` — the **anchor** that
  makes a published span mean something on a later extract, and the per-slot half
  of the question `block_spans_generation()` answers for the table as a whole. A
  span is keyed by block slot, and a slot names a block only against a particular
  grid at a particular topology: the block heap is LIFO, so after a `remove()` a
  reused slot names a *different* block and its span reads as that block's
  geometry under `Status::ok`. Anchored on `topology_epoch` alone, which now
  identifies both — see the `volume` entry below.
  It takes the **grid** rather than trusting the caller to re-extract first:
  between a `remove()` and the next extract the per-slot stamps are still set, so
  a query that re-checked nothing would call a stale span live — a staleness the
  caller cannot see, which makes it this tier's to check (the 2026-08-04 rule).
  The stamp is compared for **equality** with the extract that published the
  table, not merely for being set: a block that drops out of the active set keeps
  the stamp its last extract wrote, so "ever meshed" would hand back a span whose
  bases index an arena since rewritten — which is what dirty-only dispatch, where
  meshing a strict subset is the normal case, would make the steady state.
  False whenever the whole table has been retired, so it can never report a slot
  live beside a `block_spans()` of `nullptr`; false too when
  `track_block_spans` is off, on a moved-from extractor, and for a moved-from
  grid, which owns no blocks however its token reads.
  It is **not** the whole answer at `slot_count > 1`: one table serves the whole
  ring, so compare `block_spans_generation()` against your own
  `DeviceMesh::generation` first.
  A `resize()` does not break it: resizing preserves block indices, so spans stay
  true and the table simply grows, its new entries unstamped.
- `volume`: `topology_epoch` moved from `VoxelBlockGrid` down to
  `VoxelHashMap` — the object that hands block indices out and takes them back —
  and became a **globally unique token** drawn from a process-wide counter at
  `create` and at every `remove()` / `clear()`, rather than a per-grid count.
  Two consequences, both of which closed a hole an anchor built on it could not
  see. It can no longer be dodged: `VoxelHashMap::remove` reached through
  `VoxelBlockGrid::map()` moves it exactly as the wrapper does, where a counter
  owned one tier up was moved only by the wrapper. And no two grids, or two
  topologies of one grid, ever share a value, so an anchor holding a dead grid's
  token cannot be revived by a new grid built in the same storage — the ABA a
  raw pointer comparison has no way to detect. `MarchingCubes` consequently
  anchors on the token alone and holds no grid pointer at all.
  `resize()` still does not move it, which is what lets a slot-keyed cache
  survive a grow. It counts nothing now; compare it for equality only.

### Changed

- `mesh`: the **vertex-sharing** sparse kernel emits per block, like the default
  one. It counts a block's vertices and triangles, reserves one range of each
  with a single `atomicAdd`, and only then writes — where it previously appended
  per vertex *and* per triangle through the global counters, so its output
  interleaved across blocks and no per-block range described it. `share_vertices`
  was therefore the one path a dirty-only dispatch could not use, which matters
  because it is the path the iOS scanner runs (memory: an in-block-shared mesh is
  ~3x smaller, and an iPad is where the arena ceiling is real). Two ranges rather
  than the default kernel's one, since sharing breaks `v = 3t`.
  **No measurable cost**: three interleaved samples each of `main` and of this
  kernel span 1.77–1.82 ms on the extract dispatch, and their means differ by
  less than that spread (room0 at 120 frames, `--voxel 0.012 --share-vertices
  --device-extract --preload`, **Release** — the build type belongs with the
  figure, since a bare configure measures `-O0`). The kernel already ran two
  passes over a cached classification, so the count phases reuse it — and
  counting reads the eight corner *signs* alone (`mcCellSigns`), no `sdf`/colour
  arrays copied out of a gather and no sRGB decode, for the one pass that runs
  over 100% of the cells. The default kernel paid ~10% for the same property.
  Geometry is **unchanged as a triangle set and as a vertex count** — not
  byte-identical, since the arena layout is exactly what this changes: the same
  766 117 triangles over the same 668 792 vertices as `main`, in a PLY of
  identical length and different bytes.
  Counting duplicates is a **second** pass, not folded into the first: a
  duplicate is decided by the *owner* cell's state, which pass one is still
  writing. Reading it there races, and under-reserves the block's range so the
  cursor walks off the end of it — invisible on the sphere fixture, a
  6 062-vertex error on room0, which is how it was caught.
  Each cursor is **bounded by the block's own reservation**, the fail-safe the
  default kernel already pays for on the same mechanism: an over-consuming block
  drops geometry rather than writing over the next block's range with exact
  counters and no error anywhere.
  `kVertexDropped` is **removed**: it existed so a dropped vertex would not be
  re-claimed and double-counted, and per-block reservation makes every slot
  deterministic whether or not it lands inside the arena, so the totals are exact
  by construction rather than by bookkeeping.

- `mesh`: the sparse `MarchingCubes` kernel emits **per block contiguously**.
  It counts a block's triangles, reserves one span for all of them with a single
  global `atomicAdd`, and only then writes — where every triangle used to claim
  its own slot and a block's output interleaved with every other block's in
  flight. Per-block ranges are what a dirty-only dispatch needs to leave a clean
  block's geometry in place, so nothing downstream of incremental extraction can
  start without this. Applies to the **default** sparse kernel; the
  `share_vertices` one is the entry above, which reserves two ranges rather than
  one.
  Geometry is **unchanged as a triangle set** — not byte-identical, since the
  arena layout is exactly what this changes: room0 at 120 frames matches `main`
  triangle-for-triangle at `--voxel 0.02` (277 506 triangles) and at
  `--voxel 0.012` (766 117), by a canonical hash over the sorted triangle set.
  **It costs ~10%** on the extract dispatch (1.17 → 1.28 ms at `--voxel 0.012`,
  Release, samples interleaved to cancel thermal drift) and buys **no**
  coalescing win —
  a triangle's three vertices were already written consecutively at `tri * 3`;
  the interleaving that cost coalescing is per-*vertex* and lives in the sharing
  kernel, which this does not touch. Nor is the index run monotonic within a
  block: slots are still handed out in whatever order cells reach the cursor.
  Justified as the precondition, not as a speedup; see the 2026-08-09
  incremental-extraction decision in `DECISIONS.md`, which records what did and
  did not move that number.
  A cell is visited twice but gathered ~1.08 times, and the two visits are
  asymmetric: the counting phase runs over 100% of cells and gathers **signs
  only** (`mcCellSigns` — no `sdf[8]`/colour array copy-out, no sRGB decode, for
  values a count cannot use), caching each cell's triangle count in one byte,
  four to a **private uint**. Private, not `shared`: both phases stride the
  block identically from the same thread id, so the writer is the cache's only
  reader — which costs the default path **zero** bytes of `shared` (44 B in
  total, against the sharing kernel's 8 428 B that the 2026-08-08 two-kernel
  decision exists to keep off this path), and drops an `atomicOr` and a zeroing
  pass with it. The emitting phase then gathers in full, but only the ~8% of
  cells that emit; the rest are rejected on a register byte. A grid whose block
  outgrows the four slots still meshes correctly and now says so, through
  `ExtractTimings::uncached_cells_per_block`.
  Contiguity is asserted directly, and exactly: triangles are attributed to
  blocks by centroid and the owning block must change exactly `distinct - 1`
  times walking the index buffer — on a clean extract, and on one that refit
  against an arena too small, over a 27-block run where the arena boundary falls
  inside one block's span and past others entirely. The golden sparse-vs-dense
  equivalence cannot see this: it compares triangles as a **set**.
  `mcEmitCell` splits into `mcCellTriangleCount` + `mcWriteTriangle`, so the
  per-block emitter and the dense kernel's per-triangle append still write
  through one body and cannot drift, and the cross-block corner addressing
  splits into `mcCornerStorage` so the two gathers resolve a corner through one
  copy of it. Both walks of a `tri_table` row are now bounded by the row
  (`kMaxTrianglesPerCell`) as well as by its `-1` terminator: the terminator is
  data the host uploads, and that count went from a loop trip to the size of a
  block's arena reservation.

- docs: split the locked-decision record out of `CLAUDE.md` into a new
  `DECISIONS.md`, moved verbatim — same 33 entries, same order, byte-identical
  text. `CLAUDE.md` keeps every decision as a one-line rule linking to its full
  entry, and its "Where to start" tour is trimmed to a tier map plus what has
  landed and what is next; the measured-lesson narrative it carried (the
  `ExtractTimings` breakdown that overturned the bottleneck guess) moves to
  `DECISIONS.md` → "Measured lessons". `CLAUDE.md` goes 2 339 → 438 lines, which
  is what an agent loads on every session; nothing is lost, only relocated.

- `mesh`: the sparse `MarchingCubes::extract` kernel resolves its 2×2×2
  neighbourhood **on-device** instead of being handed a host-built table. One
  workgroup per active block (not a flat voxel grid — shared memory is per
  workgroup), threads 0–7 each probing one octant through the new
  `volume/shaders/hash_lookup.glsl` into `shared int s_neighbour[8]`, then every
  thread strides over the block's voxels. The host pass it replaces was an
  `O(active·8)` serial `unordered_map` build measured at **102.2 ms of a 132.7 ms
  extract** at 107 k blocks on an M5 iPad Pro, against 25.9 ms for the dispatch it
  fed; extract drops to **42.0 ms**. Both prior CUDA and Metal implementations of
  this pipeline resolved neighbours the same way.
  `volume::VoxelHashMap::entries_buffer` / `entries_buffer_size` are published for
  it (bound with the real range and checked against `maxStorageBufferRange`, since
  `resize` doubles the table), and `hash_lookup.glsl` shares `hash_common.glsl`'s
  struct layouts and hash constants rather than mirroring them — the `pc`
  push-constant block is opt-out via `VR_HASH_COMMON_NO_PUSH_CONSTANTS`, and both
  headers gained include guards.
  **The probe requires a quiescent table** (no `allocate`/`remove`/`clear`/`resize`
  dispatch in flight), which is stated on `MarchingCubes::extract_device` as well
  as on the accessor: it holds by construction on one thread, and a consumer that
  fuses and meshes concurrently must serialise them. See the 2026-08-08 decision in
  `DECISIONS.md` for what the `mesh`→`volume` coupling costs.
- `mesh`: **removed** `ExtractTimings::neighbour_lut_ms` — the phase it measured no
  longer exists, and a permanently-zero row in the viewer overlay is worse than an
  absent one. `total_ms()` and `fuse_viewer`'s stage table drop it with the field.

### Fixed

- `volume`: `VoxelHashMap::entries_buffer_size()` reported the full table size on a
  **moved-from** map while `entries_buffer()` correctly reported `VK_NULL_HANDLE`,
  so the descriptor write the two accessors exist to spell would pair a null handle
  with a non-zero range — invalid usage, and undefined with layers off, which this
  repo neither enables `nullDescriptor` nor `robustBufferAccess` to survive. Both
  accessors now gate on `valid()`, and `entries_buffer()` documents that a completed
  `resize` destroys the handle (both shipped examples resize mid-scan).
