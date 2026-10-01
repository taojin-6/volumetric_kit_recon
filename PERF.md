# PERF.md — the live rig path, camera to screen

The optimisation plan for `rig_viewer`'s path: capture, decode, frame prep,
allocation, fusion, marching cubes, texturing and gfx's draw. It is a shared
work list. Each item says what it fixes, where the code is, what evidence
motivates it, and how to tell it worked.

- **Claim an item** by writing your branch in its row of the table below, in
  the PR that does the work. Edit only your own row, so parallel branches
  merge cleanly.
- **Land it** by setting the row to `landed (#PR)` with the before/after
  figures, the build type and the hardware. The rationale goes in
  [DECISIONS.md](DECISIONS.md) as usual. This file keeps the plan, not the
  history.
- **Found something new?** Add an item with the same fields. An item whose
  win nobody has measured says "measure first" and names the measurement.

Baseline: `main` at `36b6d86` (2026-09-30).

## Goal

1. **Data stays on the device from arrival to draw.** Depth arrives on the
   host from the camera and goes up once. Colour is decoded on the GPU and
   never comes down. Only small control values are read back.
2. **This holds on a discrete GPU.** An NVIDIA desktop is the reference, not
   Apple's unified memory, where host visibility is free and so hides its
   cost (the 2026-09-28 residency decision and its measured lessons).
3. **A set of N cameras costs a fixed number of submits.** The baseline
   took about 19 fence waits for four cameras. Since P1, fusion's share no
   longer grows with the camera count.

## The path today

For one set of four cameras, one iteration of `rig_viewer`'s fuse thread
(`examples/viewer/rig_viewer.cpp:1270`), at the baseline. P1 has since made
allocate, compaction and integrate one submit each per set; see its row.

| stage | code | submits | order | read back |
|---|---|---|---|---|
| decode | `hevc_color.cpp`, `cuda_pictures.cpp`, `vt_pictures.mm` | own thread per camera | ahead of the poll | nothing |
| frame prep | `prepare_set`, `gpu_frame_prep.cpp:640` | 1 per camera | parallel threads | nothing |
| allocate | `allocate_from_depth` → `dispatch_with_retry`, `voxel_hash_map.cpp:376` | 1 per retry round, per camera | serial | failure tally, heap counter |
| compaction | `compact_active_blocks_on_device`, `tsdf_integrator.cpp:314` | 1 per camera | serial | active count |
| integrate | `tsdf_integrator.cpp:411` | 1 per camera | serial | nothing |
| extract compaction | `compact_active_blocks`, `marching_cubes.cpp:1332` | 1 (2 if the list outgrew its guess) | serial | count and the **whole list** |
| extract | `marching_cubes.cpp:1762` | 1 (2 on a refit) | serial | 32-byte draw command |
| texture | `projective_texturer.cpp:737` | 1 | serial | nothing |
| atlas copy and draw | `rig_viewer.cpp:498`, gfx's frame | gfx's own | render thread | nothing |

Every submit waits on its fence (`device.cpp:710`), so the GPU idles while
the host records the next one. Fusion is **one set of kernels per camera**.
Only texturing takes all the cameras in one dispatch.

Bytes moved per set, four cameras (4K colour, 640 × 576 depth, 1 cm voxels):

| bytes | from → to | how often | item |
|---|---|---|---|
| depth, 0.7 MB per camera | host → device | every set | unavoidable |
| NV12, 12 MB per camera | NVDEC surface → Vulkan buffer (CUDA copy), or VideoToolbox image → buffer | every set | required on NVIDIA; L3 on Apple |
| active block list, 16 B per block (~0.8 MB on the lab rig) | device → host → device | every remesh | P5 |
| colour, 33 MB per camera | prep output → texturer's buffer | every remesh | P4 |
| colour, 33 MB per camera | prep output → gfx atlas image | every remesh | L4 (kept) |
| mesh, ~290 MB at 1.5 M triangles | written by marching cubes, `uv0` rewritten by texturing | every remesh | L1 |
| solid colour, 33 MB per camera | device buffer → gfx atlas image | every remesh with `--show-sources` | D2 |

## Baseline figures

Measured, each from where it is recorded. Quote new figures the same way.

| what | figure | source |
|---|---|---|
| one camera, GPU path, 4K25, M-series Mac, Release | frame prep 2.6 ms host / 0.89 ms device; allocate 1.3 / 0.76; integrate 1.7 / 0.58 | DECISIONS.md, 2026-09-28 GPU pre-processing |
| `prepare_set`, 4 × (640 × 576 depth, 3840 × 2160 colour) | 1.73 ms M5 Max, 4.8 ms RTX 5090 | DECISIONS.md, 2026-09-28 residency, step 5b |
| `rig_viewer` per set, lab rig, 1 cm, M5 Max, Release | fuse 6.0–11.1 ms; remesh (extract + texture) 6.1–7.8 ms at 720p, 6.7–7.1 ms at 4K | DECISIONS.md, 2026-09-29 and its amendments |
| atlas copy, GPU, 4 cameras at 720p | 0.23 ms | DECISIONS.md, 2026-09-29 |
| VideoToolbox image → buffer copy, 4K | 0.28–0.31 ms GPU, M5 Max | `undistort_color.comp:21` |
| one submit and fence wait | ~0.38 ms, RTX 5090 | `voxel_hash_map.cpp:658` |
| one timed submit | ~0.13 ms extra, MoltenVK | CLAUDE.md, `core` |

With one camera, host time is about 2.2 times device time for allocate plus
integrate. Four in a row account for most of the rig's fuse time. The
difference is the cost of submitting, waiting and recording, not of the
kernels.

## Rules for work on this path

- **Measure the phases first**, as the rest of the repo does:
  - Configure with `-DCMAKE_BUILD_TYPE=Release`, since a bare configure is
    `-O0`.
  - Read the stage rows' host and device halves. A stage whose host time
    far exceeds its device time is bound by its submits.
  - `fuse_orbbec --rig sync.json --calibration calib.json --gpu --frames N`
    is the headless loop.
  - An Nsight Systems or Xcode capture shows the gaps between submits, with
    each kernel named by its debug-utils label.
  - On the NVIDIA home box, use the kit in `~/recon-bench`. The Mac is
    shared and noisy, so bench it interleaved A/B only.
- **Test on a discrete GPU before you call something a win.**
- **Kernel memory is `device_storage_buffer`.** The host reaches it through a
  `CommandBatch`, never through a mapping, and never reads VRAM through the
  BAR (the 2026-09-28 residency decision).
- **One `CommandBatch` cannot dispatch the same `ComputeKernel` twice if its
  set is rewritten in between** (`command_batch.cpp:632`). Batch several
  cameras with one kernel instance per camera, or with a kernel that takes
  them all.
- **Run sync validation on any change to batching or barriers**
  (`VK_KHRONOS_VALIDATION_VALIDATE_SYNC=true`). A missing barrier passes a
  plain run and fails only through the test's error counter.
- **Threads:** a kernel's descriptor set, a `Buffer` and a `GpuTimer` stay on
  one thread; `Device` takes submits from several.
- **No GPU wait between recon and gfx.** The mesh ring is released by the
  host's report (the 2026-08-03 decision).
- **A batched path computes what the per-camera path computes.** Show it
  with a test, voxel by voxel or block by block.
- **Changing a tier's contract or a locked decision** means updating
  CLAUDE.md and DECISIONS.md in the same PR.

## Work items

| id | item | expected win | effort | depends on | status |
|---|---|---|---|---|---|
| P8 | Sample the viewer's GPU timing | measured 0.04–0.07 ms/set, not the 1.7 estimated | S | — | not worth it |
| P1 | Fuse a set's cameras in one allocate, one compaction and one integrate | measured −14% a set on the M5 Max, −21% on the RTX 5090 | L | — | landed (#127) |
| P3 | Deduplicate depth allocation before dilating | measured −50% a set on the M5 Max, −62% on the RTX 5090 (over P1) | M | — | landed (#128) |
| P5 | Extract from the fuse's device block list | measured −14% an extract on the M5 Max, −35% on the RTX 5090 | M | P1 for the shared list | landed (#129) |
| P4 | Bind texture views in place, with no per-remesh copies | measured 0.26–0.51 ms GPU per remesh at 4K, 0.04–0.12 at 720p | M | P1's descriptor-array decision | deferred |
| P2 | Record a set's frame prep in one batch | measured no gain; slower for host colour on the Mac | S | — | not worth it |
| P6 | Take the remaining host decisions off the critical path | at most ~0.5 ms/set on the RTX 5090, ~0.7 on the M5 Max (measured gap) | M | P1, P5 | open |
| P7 | Free the blocks nothing asks for or weights | measured: the map 7.3k → 2.9k blocks in 600 sets, integrate's device time −40% on the M5 Max, −50% on the RTX 5090; a static room keeps its size | M | — | in review (#132) |
| P9 | Allocate only the band blocks a sample can weight | ~60% of a static room's active set holds no weight; compaction, integrate and meshing scale with it | M | — | open, measure first |
| D1 | Report a decoder's fallback to host pictures | makes a silent 12 MB/camera/frame PCIe regression visible | S | — | landed (#130) |
| D2 | Put `--show-sources`' buffers on the device | ~133 MB over PCIe per remesh with the view on | S | — | landed (#130) |
| D3 | Keep exported picture buffers out of the BAR | robustness on ReBAR systems | S | — | not needed; in review (#130) |
| L1 | Shared-vertex, incremental remesh for the rig | ~3.4× fewer vertices; remesh cost tracks change, not size | L | gfx | blocked |
| L2 | Pipeline sets | overlaps set N's GPU work with set N+1's host work | L | P6 | later |
| L3 | Read VideoToolbox's plane images directly | 0.28–0.31 ms GPU per 4K frame, Apple only | M | — | later |
| L4 | Sample the atlas in place rather than copy it | measure the copy at 4K first | L | gfx | kept |

The suggested order: P8 first, so every later figure is honest; D1–D3
whenever convenient; then P1, P3, P5, P4, P2 + P6; then P7, and P9 once it
is measured.

### P8 — Sample the viewer's GPU timing

> **Measured, not built (2026-10-01).** Timing every set costs 0.04–0.07 ms
> a set. A throwaway bench ran room0's four-frame sets through `fuse_set`,
> rows on against off, interleaved over 3 × 30 sets: M5 Max 2.87 against
> 2.83 ms, RTX 5090 1.23 against 1.16 ms (within run-to-run noise). The
> estimate below predates P1, which cut the timed submits from about 13 a set
> to 3, and over-stated what one costs.

- **Problem.** `rig_viewer` passes `&fuse_stages` and `&remesh_stages` on
  every set, so about 13 submits a set are timed: 4 × allocate, compaction and
  integrate, plus the texture pass. By the ~0.13 ms figure that is ~1.7 ms a
  set on MoltenVK. This is an estimate, and the overlay's own rows include
  it.
- **Change.** Pass the metrics on every Nth set (say 10), keep the last
  sample on the panel, and time the host span of every set as now.
- **Measure.** Host wall-clock time per set, timing on versus sampled,
  interleaved.

### P1 — Fuse a set's cameras together

> **Landed as** one batch per step with a descriptor set per camera over the
> unchanged kernels: no device feature, and the same bits as fusing the frames
> one at a time. The camera-looping kernel below is deferred until integrate's
> device time is shown to matter. Figures and rationale are in DECISIONS.md,
> 2026-09-30. What is left: the extract still compacts again (P5). A retry
> round still re-dispatches every camera, but since P3 one is rare.

- **Problem.** `rig_viewer.cpp:1323` loops over the cameras through
  `fuse_frame` (`examples/common/fuse_device_frame.hpp`), and per camera:
  - `allocate_band` runs `allocate_from_depth`: at least one submit and
    wait;
  - `integrate` compacts the whole map (`tsdf_integrator.cpp:314`): a
    submit and a wait;
  - `integrate` then dispatches (`:411`): another.

  That is at least 12 serial waits and 4 whole-table compactions, and every
  camera reads and writes every active voxel's `tsdf`, `weight` and `color`.
- **Change.**
  - An `allocate_from_depth` overload that takes N depth maps and cameras:
    one dispatch, each thread a (camera, pixel), the cameras in an array
    buffer, and one retry loop per set.
  - A `TsdfIntegrator::integrate` overload that takes N frames: one
    compaction and one dispatch. Each voxel loops over the cameras in input
    order and carries its `tsdf`, `weight` and colour in registers, so it
    reads and writes memory once.
  - `fuse_set` in `examples/common/fuse_device_frame.hpp`, used by
    `rig_viewer` and `fuse_orbbec --rig --gpu`.
- **Binding N buffers.** Use descriptor arrays with dynamic indexing
  (`shaderStorageBufferArrayDynamicIndexing`, an optional core 1.0 feature).
  - `Device::create` does not enable it today; see the reasoning at
    `projective_texturer.cpp:649`.
  - Enable it where offered, name it in `requirements()`, and report it on
    `adopt`.
  - Keep today's per-camera path as the fallback.
  - **First step:** confirm MoltenVK and the RTX 5090 both report the
    feature. A fixed set of `kMaxCameras` bindings with a switch is the
    alternative that needs no feature.
- **Semantics.** The per-voxel arithmetic is the same, in the same camera
  order, as today's sequential dispatches, Dynamic's clearing included,
  since each voxel belongs to one thread. The one difference is that a
  block first allocated by a later camera in the set is also fused by the
  earlier ones: more observations, not fewer.
- **Measure and accept.**
  - A test fusing a pre-allocated grid both ways, compared voxel by voxel.
    Say whether it matches exactly or within a tolerance, and why.
  - The fuse row, host and device, per set on the M5 Max and the RTX 5090.
  - Submits per set.

### P3 — Deduplicate depth allocation

> **Landed as** 16 x 16 tiles with shared-memory leader election, each
> distinct block's band shared out over the workgroup. See DECISIONS.md,
> 2026-09-30. Allocation's device time fell 79% on the M5 Max and 84% on the
> RTX 5090, and on the Mac 1 set in 30 retries rather than 18–20.

- **Problem.** `hash_allocate_depth.comp` gives each pixel a thread that
  probes the whole (2·tb + 1)³ cube around its block
  (`hash_allocate_common.glsl:358`): 27 probes at the defaults, about 10 M
  for a 640 × 576 frame. Adjacent pixels mostly hit the same few thousand
  blocks. The hot buckets lose lock races, and every lost race means another
  round, another submit (`dispatch_with_retry`).
- **Evidence.**
  - Allocate is the largest fusion kernel on the device: 0.76 ms against
    integrate's 0.58 ms for one camera. After P1 it is most of a set's
    fusion: about 3.7 ms of device time a set on the M5 Max and 2.5 on the
    RTX 5090 (room0, four frames, 1 cm).
  - Since P1 a retry round re-dispatches every camera of the set. 63% of sets
    need a second round on the Mac, so contention now costs a whole set.
  - The dedup is already a `TODO(volume)` (`hash_allocate_depth.comp:12`),
    which the prior engine's `hash_ops.metal` did with leader election.
- **Change.** Deduplicate the centre blocks before dilating: 2-D workgroups
  (a 16 × 16 pixel tile, say), each collecting its tile's distinct centre
  blocks in shared memory or through subgroup ops. Only the distinct ones
  dilate.
- **Accept.**
  - The same allocated block set, compared as sorted coordinates, on the
    existing fixtures and on a recorded frame.
  - Allocate's device time.
  - Lock races and retry rounds per set (`AllocFailures::lock`).

### P5 — Extract from the fuse's device block list

> **Landed**, though not as the fix below has it. With spans off the list
> stays on the device, and the map hands its last compaction back while
> nothing has changed since, so the extract reuses the fuse's with no new
> API. See DECISIONS.md, 2026-09-30.

- **Problem.** `extract_device` compacts the whole map again, reads the list
  back to the host (`collect_compacted`, `voxel_hash_map.cpp:653`) and
  uploads it again (`marching_cubes.cpp:1332`, `:1525`). That is a device →
  host → device trip and two waits on every remesh. The host never reads the
  list unless `track_block_spans` is on (`TODO(mesh)` at
  `marching_cubes.cpp:1505`, and the `TODO(volume)` at
  `voxel_hash_map.cpp:659`).
- **Change.**
  - An `extract_device` overload that takes a `volume::DeviceBlockList`,
    checked by `check_device_block_list`, for use when spans are off.
  - The fuse's single compaction (P1) supplies the list, and its count is
    already on the host, so a remesh compacts nothing.
  - Before P1 lands, the extract can compact to the device list itself.
- **Accept.** `ExtractTimings::compact_ms` and `input_upload_ms` go to about
  zero, and the mesh matches today's triangle for triangle (sorted triangle
  hash).

### P4 — Bind texture views in place

> **Measured, deferred (2026-10-01).** The copy is cheaper than its byte count
> suggests: four views' colour copied into one buffer, as `texture_views`
> does, costs 0.51 ms of GPU at 4K on the M5 Max and 0.26 ms on the RTX 5090
> (0.12 and 0.04 ms at 720p). That is about 7% of a 4K remesh on the Mac, too
> little to buy a caller-declared cache key or a device feature for. Revisit
> if a 4K rig's remesh shows the texture row's device half dominating.

- **Problem.** `texture_views` copies every view's depth, and its whole RGBA
  colour buffer, into packed buffers on every call
  (`projective_texturer.cpp:658-679`).
  - It needs only the colour's coverage byte, which depends on the lens
    alone (`undistort_color.comp:7-9`), so it never changes between
    frames.
  - At 4K that is 33 MB a camera, ~133 MB a remesh, and gfx then copies the
    same bytes into the atlas.
- **Change.**
  - Bind each view's depth and colour where they are, through P1's
    descriptor arrays. The file's own reason for copying is the missing
    feature.
  - The fallback with no feature: keep coverage in the pass between calls,
    and copy it again only when a view's camera changes (the
    `TODO(texture)` at `projective_texturer.hpp:415`).
- **Accept.** The texture row's device time at 4K with four cameras, and the
  existing several-view tests unchanged.

### P2 — Record a set's frame prep in one batch

> **Built, measured, dropped (2026-10-01).** The per-camera threads already
> overlap the four submits, so one batch saves nothing.
>
> `prepare_set` on four frames (640 x 576 depth, 3840 x 2160 colour),
> threads against one batch, median over 70 sets:
>
> | | device colour | host colour |
> |---|---|---|
> | M5 Max | 0.99 vs 1.03 ms | 1.50 vs 1.97 ms |
> | RTX 5090 | 2.64 vs 2.64 ms | 8.1 vs 8.1 ms |
>
> The Mac's host-colour case got slower, because the staging copies ran one
> after another on one thread.

- **Problem.** `prepare_set` starts a thread per camera, and each thread
  makes its own batch, submit and wait (`gpu_frame_prep.cpp:468`, `:573`).
  With colour already on the device, a thread does a 0.7 MB depth memcpy and
  then waits.
- **Change.**
  - A `GpuFramePrep` entry point that records into a caller's
    `CommandBatch`, so one batch carries every camera. Each pass keeps its
    own kernels and sets, so the rewrite refusal is not an issue.
  - Keep the per-pass staging (step 5b's VMA-lock lesson).
  - Keep threads only for host colour, where the memcpy is the cost.
- **Accept.** `prepare_set` time on both machines, against step 5b's table.

### P6 — One submit per set in steady state

> **Headroom measured (2026-10-01).** What P6 can recover is the gap between
> a step's host and device time, from #128's set rows (room0, four frames,
> 1 cm, per set):
>
> | | allocate, host / device | integrate, host / device | gap at most |
> |---|---|---|---|
> | RTX 5090 | 0.69 / 0.47 ms | about 0.65 / 0.34 ms | about 0.5 ms |
> | M5 Max | 1.14 / 0.84 ms | 1.82 / 1.38 ms | about 0.7 ms |
>
> That is about 25–45% of the set. Taking the compaction count off the host
> also means rethinking #129's cached list, which holds that count. P2 was
> dropped, so P6 no longer waits on it.

With P1, P2 and P5 in place, three host decisions still split a set:

- **The compaction count, which sizes the integrate dispatch.** Dispatch
  indirect off the device's count (`CommandBatch::dispatch_indirect`; the
  `TODO(tsdf)` at `tsdf_integrator.cpp:312`).
- **Allocation failures, which decide a retry or a grow.**
  - Grow ahead of need at `kGrowThreshold`, off `load_factor()`, before the
    set.
  - Read the tally back in the set's one submit.
  - A residue of lost lock races lands on the next set.
  - That one-set lag is for the live viewer only. The examples that write a
    mesh (`fuse_replica` and the rest) keep today's synchronous loop.
- **The triangle count, which sizes the texture dispatch and the arena
  refit.** Keep it: a refit is rare once the plan has measured density.

The target is fuse + prep in one submit and remesh in one, about 2 per set
from about 19. Merge further only if the rows show the remaining gap.

### P7 — Free the blocks nothing asks for or weights

> **Built (2026-10-01; the DECISIONS.md entry of that date).** Every block
> slot carries stamps, ticks of a clock on the map: `requested`, written by
> every allocation kernel for each block it asks for, and `weighted`, by the
> grid's block pass. `free_stale_blocks(max_age)` frees the blocks whose
> newer stamp is that old, and `rig_viewer` runs it every 30 sets
> (`--free-after`). On the bench below, at 600 sets:
>
> | | blocks | integrate, device ms/set |
> |---|---|---|
> | M5 Max, without / with | 7 348 / 2 909 | 0.41 / 0.25 |
> | RTX 5090, without / with | 7 345 / 2 909 | 0.08–0.09 / 0.04 |
>
> The blocks holding weight are the same with and without. A pass costs
> about 2 ms of host time on the M5 Max and 1 ms on the RTX 5090, and the
> stamp adds 4% to allocation's device time on the RTX 5090 and nothing
> measurable on the M5 Max. On the lab rig's static room a pass frees
> 1 100–1 800 noise blocks in 4.6–7.4 ms, nearly all of it zero fills, and
> the map keeps its size: what is empty there is the band, which P9 is
> about. The first cut freed on weight alone, and freed that band every
> other pass for the allocator to ask for again, 33 000 blocks and 140 ms
> a pass.

> **Measured before building (2026-10-01).** A throwaway bench put four
> cameras at the rig's calibrated poses, with Femto-like 640 x 576 depth
> ray-cast on the host. They saw a static 0.3 m sphere at the point their
> axes meet, and a 0.25 m sphere walking a 0.8 m loop around it, then the
> same loop shifted 0.5 m. The bench fused every set through `fuse_set`,
> Dynamic, 1 cm (M5 Max, Release):
>
> | set | blocks | with any weight | empty | integrate, host / device |
> |---|---|---|---|---|
> | 50 | 2 117 | 759 | 64% | 0.63 / 0.19 ms |
> | 300 | 5 786 | 1 632 | 72% | 0.81 / 0.45 ms |
> | 600 | 7 348 | 1 947 | 74% | 0.82 / 0.49 ms |
>
> The surface stays at about 1 600–1 950 blocks while the map grows 3.5x.
> Empty blocks come two ways:
>
> - **From the start.** Most of the dilated band never takes weight under
>   Dynamic integration, which clears free space ahead of the surface and
>   does not fuse past the band. That is already 64% at set 50. P7 keeps
>   these, since the allocator asks for them every set; P9 is about them.
> - **Over time.** A receded surface's blocks accumulate as the mover
>   reaches new space. These are what P7 frees.

- **Problem.**
  - Dynamic integration clears a receded surface's voxels to weight 0 (the
    free-space branch in `tsdf_integrate.comp`) but never frees their
    blocks, and nothing on the live path calls `remove`.
  - The active set therefore grows with the history of motion, and so does
    every per-set pass over it: compaction, integrate, marching cubes.
  - Eventually a resize doubles the map.
- **Still to measure.** `rig_viewer` with a subject moving for several
  minutes, the map's size over time at `--free-after 30` and at 0. The
  room measured so far was static, where there is no history to free.
- **Change, as built.**
  - Every K sets, one block pass stamps the active blocks holding weight
    and lists those that neither an allocation nor weight has stamped for
    M ticks. The age keeps a block flickering at the band's edge from being
    freed and allocated again each set, and `requested` keeps the band.
  - The list is read back, the blocks' attributes are zeroed by the ptrs
    the pass read, and they are removed by coordinate.
  - Removing moves `topology_epoch`, which invalidates the dirty flags and
    span tables; incremental extraction already falls back when that
    happens.

### P9 — Allocate only the band blocks a sample can weight

- **Problem.**
  - In the lab rig's static room about 60% of the active set holds no
    weight at any moment: 33 000 of about 58 000 blocks at 1 cm, which the
    first P7 cut found by freeing them.
  - Depth allocation dilates every surface block into the (2tb+1)³ cube,
    27 blocks at the defaults. With 8 cm blocks and 4 cm of truncation, a
    sample reaches only the neighbours within 4 cm of it. Dynamic
    integration clears free space ahead of the surface to weight 0, so the
    cube's far blocks never take weight.
  - Every pass over the active set pays for them: compaction, integrate (a
    workgroup a block a camera) and marching cubes.
- **Measure first.** Count, per set, the blocks the allocator asks for that
  take no weight, and integrate's device time over them. The stamps give
  the first: `requested` at this tick, `weighted` not.
- **Change.**
  - Dilate a block into a neighbour only where some sample of it lies
    within `trunc_dist` of that neighbour.
  - P3's tile dedup keeps a tile's distinct blocks, not their samples, so
    the test needs each block's sample extent: a per-block min and max in
    shared memory, say.
  - The risk is holes at block edges, where a band the test skipped was
    needed. Judge room0's and the lab rig's meshes against today's with the
    `eval` tier.
- **Expected.** Up to ~2.5x fewer blocks on a static scene, and integrate's
  work with them; an estimate until measured.

### D1 — Report a decoder's fallback to host pictures

- **Problem.** When the device path fails once (out of memory, or refused by
  CUDA or the import), it is dropped without a word, and every later picture
  comes to the host (`hevc_decoder.cpp:541`, `:553`;
  `jpeg_decoder.cpp:434`).
  - On NVIDIA that is 12 MB a camera a frame at 4K over PCIe, plus host
    conversion and staging.
  - `OrbbecCaptureStats` has no field that shows it.
- **Change.** Count host pictures in the stats, say it once on stderr, and
  show it on `rig_viewer`'s Rig panel.
- **Accept.** A forced device-path failure moves the counter.

### D2 — Put `--show-sources`' buffers on the device

- **Problem.** `rig_viewer.cpp:956-986` makes host-visible buffers that gfx
  copies into the atlas on every remesh: ~133 MB over PCIe at 4K on a
  discrete GPU. It is a debug view.
- **Change.** Make them device-local and fill each once with
  `vkCmdFillBuffer`, since the solid colour is one 4-byte word.

### D3 — Keep exported picture buffers out of the BAR

> **Not needed.** Vulkan orders a memory type ahead of any whose flags
> strictly contain its own, so the first `DEVICE_LOCAL` type is already not
> host-visible wherever the buffer allows one. #130 names the rule in
> `core`'s `find_memory_type` and checks it on each CI GPU instead. See
> DECISIONS.md, the 2026-09-28 decoded-frame entry, amended 2026-10-01.

- **Problem.** `create_exported_buffer` takes the first `DEVICE_LOCAL`
  memory type (`external_memory.cpp:62-67`). On a ReBAR system that type can
  be the host-visible BAR heap.
- **Change.** Prefer `DEVICE_LOCAL` without `HOST_VISIBLE`, as
  `vt_pictures.mm`'s `bind_memory` already does.

### Known gaps, not planned

- **Decoded colour stays on the device only on Linux with CUDA
  (`VR_WITH_CUDA`) and on Apple.**
  - `VR_WITH_CUDA` is Linux only (`cmake/vr_cuda.cmake:23`), since the
    import uses an opaque file descriptor.
  - Windows (D3D11VA), VAAPI on Linux, and software decoding hand pictures
    to the host, and `GpuFramePrep` stages them up.
  - They are correct, but not resident.
  - Windows would need `VK_KHR_external_memory_win32` beside the fd path.

### L1 — Shared-vertex, incremental remesh (blocked on gfx)

- **The cost.**
  - The rig's mesh is unshared, because texturing from several views
    chooses a view per triangle (`projective_texturer.cpp:726`).
  - A vertex is 64 B (`mesh.hpp:90`), so 1.5 M triangles take ~290 MB a ring
    slot across 3 slots.
  - Marching cubes and texturing rewrite all of it on every set.
- **The unblock.** A per-triangle tile index in gfx (the `TODO(texture)` at
  `projective_texturer.cpp:722`). It would allow `share_vertices`, 3.4×
  fewer vertices on room0.
- **After that.** Incremental extraction, which also needs support for
  `slot_count > 1` (`TODO(mesh)` on `MarchingCubes`).

### L2 — Pipeline sets

After P6, the GPU could run set N while the host polls and decodes set N+1.

- `GpuFramePrep`'s reuse rule already keeps a frame an in-flight batch
  holds.
- It needs a `CommandBatch` that submits without waiting, its fence polled
  later: a `core` change.
- Do it only if the rows still show host time on the critical path.

### L3 — Read VideoToolbox's plane images directly

- The `TODO(sensor)` at `undistort_color.comp:21`.
- Apple only, and 0.28–0.31 ms of GPU per 4K frame.

### L4 — The atlas copy (kept)

- The 2026-09-29 decision keeps the copy into gfx's image, which gets
  hardware filtering and the sRGB decode.
- Measured 0.23 ms at 720p. At 4K (~133 MB a remesh) it has not been
  measured.
- Measure before revisiting. Sampling the buffers in place needs a gfx
  pipeline variant.
