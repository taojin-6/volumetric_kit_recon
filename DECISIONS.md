# Locked decisions

The dated design record for `volumetric_kit_recon`: each rule's rationale,
measurements, review findings, and verification. Newest context wins.

[AGENTS.md](AGENTS.md) is the concise shared working guide;
[DESIGN.md](DESIGN.md) describes the implementation and detailed contracts.
When a decision changes, update its index entry and dated rationale here in
the same commit, plus the affected contract in DESIGN.md and any essential
shared rule in AGENTS.md. An entry that governs grid's code is grid's
([its DECISIONS.md](https://github.com/taojin-6/volumetric_kit_grid/blob/main/DECISIONS.md#decision-index));
this index keeps a line for each, linking there.

## Decision index

The index and dated entries are in the same chronological order. Read the
entries relevant to your task; later amendments supersede earlier rules.

- [**2026-06-21**](#2026-06-21--single-vulkan-path-moltenvk-on-apple-like-gfx) —
  Single Vulkan path (MoltenVK on Apple), like gfx.
- [**2026-06-21**](#2026-06-21--trivial-interop-same-vulkan-api) —
  Trivial interop (same Vulkan API).
- [**2026-06-21**](#2026-06-21--independent-siblings-gfx-untouched) —
  Independent siblings; gfx untouched.
- [**2026-06-21**](#2026-06-21--vertical-slice-first) —
  Vertical slice first.
- [**2026-06-21**](https://github.com/taojin-6/volumetric_kit_grid/blob/main/DECISIONS.md#2026-06-21--codec-ships-dct-only) —
  Codec ships DCT-only.
- [**2026-07-04**](#2026-07-04--native-cuda-accelerator-under-the-vulkan-baseline) —
  Native CUDA accelerator, under the Vulkan baseline.
- [**2026-07-04**](#2026-07-04--glm-for-hostdevice-math-dropped-the-hand-rolled-pod-types) —
  GLM for host/device math (dropped the hand-rolled POD types).
- [**2026-07-04**](#2026-07-04--zero-copy-interop--one-shared-vkdevice--a-createadopt-seam-refines-trivial-interop-above) —
  Zero-copy interop = one shared `VkDevice` + a create/adopt seam (refines
  "Trivial interop" above).
- [**2026-07-05**](https://github.com/taojin-6/volumetric_kit_grid/blob/main/DECISIONS.md#2026-07-05--shader-buffer-abi-is-scalar-block-layout-not-std430) —
  Shader buffer ABI is scalar block layout, not `std430`.
- [**2026-07-05**](#2026-07-05--compute-core-is-explicit-not-reflected-dispatch-via-submit_single_time) —
  Compute core is explicit, not reflected; dispatch via `submit_single_time`.
- [**2026-07-05**](https://github.com/taojin-6/volumetric_kit_grid/blob/main/DECISIONS.md#2026-07-05--per-voxel-storage-is-a-structure-of-arrays-attribute-store-voxelblockgrid-not-the-prior-engines-aos-voxel-in-the-hashmap) —
  Per-voxel storage is a structure-of-arrays attribute store
  (`VoxelBlockGrid`), not the prior engine's AoS `Voxel`-in-the-hashmap.
- [**2026-07-06**](#2026-07-06--per-kernel-resources-are-bundled-corecompute_kernelhpp-still-not-reflected) —
  Per-kernel resources are bundled (`core/compute_kernel.hpp`), still not
  reflected.
- [**2026-07-06**](#2026-07-06--hybrid-color-renders-through-a-gfx-pipeline-amends-interop-seam-a-needs-zero-gfx-changes) —
  Hybrid color renders through a gfx pipeline (amends "interop seam A needs
  zero gfx changes").
- [**2026-07-06**](#depth-sampling-convention) —
  Depth sampling is texture-centred (pixel centres at i+0.5), a deliberate
  ~½-pixel convention. (Integer-centred since 2026-10-08.)
- [**2026-07-07**](#2026-07-07--the-viewer-example-opts-into-gfx-behind-vr_build_viewer-amends-independent-siblings-gfx-untouched) —
  The viewer example opts into gfx behind `VR_BUILD_VIEWER` (amends
  "Independent siblings; gfx untouched").
- [**2026-07-07**](#2026-07-07--projective-texturing-is-a-new-texture-tier-live-single-camera-first) —
  Projective texturing is a new `texture` tier; live single-camera first.
- [**2026-08-01**](#2026-08-01--perf-instrumentation-starts-in-the-viewer-example-not-a-shared-contract-package) —
  Perf instrumentation starts in the viewer example, not a shared contract
  package.
- [**2026-08-01**](#2026-08-01--ios-is-a-downstream-concern-recon-cross-compiles-to-it-unchanged) —
  iOS is a downstream concern; recon cross-compiles to it unchanged.
- [**2026-08-02**](https://github.com/taojin-6/volumetric_kit_grid/blob/main/DECISIONS.md#2026-08-02--meshvertex-is-the-renderers-vertex-layout) —
  `mesh::Vertex` *is* the renderer's vertex layout.
- [**2026-08-02**](#2026-08-02--the-neutral-shared-vkdevice-bootstrap-lands-in-the-viewer-example-and-prefers-two-families-over-a-shared-queue) —
  The neutral shared-`VkDevice` bootstrap lands in the viewer example, and
  prefers two families over a shared queue. *Amended 2026-10-12:* gfx's
  `gfx_glfw` builds it; recon keeps no copy.
- [**2026-08-02**](#2026-08-02--the-sensor-tier-is-a-contract-not-a-driver-collection-a-capture-driver-lives-here-only-if-this-repo-can-build) —
  The `sensor` tier is a *contract*, not a driver collection: a capture driver
  lives here only if this repo can build (`ICameraCapture` went 2026-10-07)
- [**2026-08-02**](https://github.com/taojin-6/volumetric_kit_grid/blob/main/DECISIONS.md#2026-08-02--the-mesh-arenas-extra-buffer-usage-is-declared-by-the-consumer-not-named-by-this-tier--and-usage-alone-does-not-reach-seam-b) —
  The mesh arena's extra buffer usage is declared by the *consumer*, not named
  by this tier — and usage alone does not reach seam B.
- [**2026-08-02**](#2026-08-02--one-color-space-rule-and-a-named-working-space-8-bit-color-is-encoded-float-color-is-linear-converted-once-at-the-sensor-boundary-and-encoded-once-at-presentation) —
  One color-space rule, and a *named* working space: 8-bit color is encoded,
  float color is linear, converted once at the sensor boundary and encoded once
  at presentation. *Amended 2026-10-28:* the voxel colour attribute's GLSL
  and its host half are `volume`'s, and the PLY writer has a copy of its own.
- [**2026-08-03**](#2026-08-03--a-buffer-names-the-families-that-will-read-it-and-the-dispatch-barrier-widens-only-as-far-as-its-queue-family-may) —
  A buffer names the *families* that will read it, and the dispatch barrier
  widens only as far as its queue family may.
- [**2026-08-03**](https://github.com/taojin-6/volumetric_kit_grid/blob/main/DECISIONS.md#2026-08-03--the-mesh-arena-is-a-ring-of-slots-released-by-the-host-not-a-timeline-semaphore) —
  The mesh arena is a ring of slots released by the *host*, not a timeline
  semaphore.
- [**2026-08-03**](https://github.com/taojin-6/volumetric_kit_grid/blob/main/DECISIONS.md#2026-08-03--the-draw-command-is-written-by-the-kernel-that-counts-it-and-the-counters-unit-changes-from-triangles-to-indices) —
  The draw command is written by the kernel that counts it, and the counter's
  unit changes from triangles to indices.
- [**2026-08-03**](https://github.com/taojin-6/volumetric_kit_grid/blob/main/DECISIONS.md#2026-08-03--nothing-in-the-mesh-extractor-reads-the-current-slot-except-the-code-that-writes-it-the-capacity-plan-is-slot-independent-and-a-slot-is-marked-outstanding-only-where-a-devicemesh-is-handed-out) —
  Nothing in the mesh extractor reads "the current slot" except the code that
  writes it: the capacity plan is slot-independent, and a slot is marked
  outstanding only where a `DeviceMesh` is handed out.
- [**2026-08-04**](#2026-08-04--a-limit-a-lifetime-or-a-staleness-the-caller-cannot-see-is-the-librarys-to-check-not-to-document) —
  A limit, a lifetime, or a staleness the caller cannot see is the library's to
  check, not to document.
- [**2026-08-08**](https://github.com/taojin-6/volumetric_kit_grid/blob/main/DECISIONS.md#2026-08-08--the-sparse-mesh-kernel-resolves-its-own-222-neighbourhood-by-probing-the-hash-table-on-device-and-mesh-therefore-reads-a-volume-buffer--a-coupling-paid-for-not-stumbled-into) —
  The sparse mesh kernel resolves its own 2×2×2 neighbourhood by probing the
  hash table on-device, and `mesh` therefore reads a `volume` buffer — a
  coupling paid for, not stumbled into. *Amended 2026-10-13:* the device
  shows the chained fixture spills; `HashDiagnostics` is gone. *Amended
  2026-10-22:* the extract splits its dispatch at the device's group limit.
- [**2026-08-08**](https://github.com/taojin-6/volumetric_kit_grid/blob/main/DECISIONS.md#2026-08-08--the-overflow-scan-stays-exhaustive-what-gets-bounded-is-its-cost-per-slot-not-its-length) —
  The overflow scan stays exhaustive; what gets bounded is its cost per slot,
  not its length. *Amended 2026-10-13:* `diagnostics()` is gone; the device
  confirms the chain fixtures. *Amended 2026-10-22:* the heap path refuses an
  overflow insert, unscanned, below an eighth of the blocks, and the retry
  loop ends at a capacity limit.
- [**2026-08-08**](#2026-08-08--fuse_viewer-draws-recons-buffers-interop-seam-b-end-to-end-and-the-release-mark-must-be-published-before-the-mesh-is-taken) —
  `fuse_viewer` draws recon's buffers: interop seam B, end to end, and the
  release mark must be published *before* the mesh is taken.
- [**2026-08-08**](https://github.com/taojin-6/volumetric_kit_grid/blob/main/DECISIONS.md#2026-08-08--in-block-vertex-sharing-is-a-second-compiled-kernel-not-a-branch-and-the-two-emitters-must-interpolate-an-edge-in-the-same-direction) —
  In-block vertex sharing is a *second compiled kernel*, not a branch, and the
  two emitters must interpolate an edge in the same direction.
- [**2026-08-09**](https://github.com/taojin-6/volumetric_kit_grid/blob/main/DECISIONS.md#2026-08-09--a-dirty-block-is-one-the-fuse-changed-the-flags-are-anchored-to-a-grid-the-library-checks-and-tracking-them-is-opt-in) —
  A dirty block is one the fuse *changed*, the flags are anchored to a grid the
  library checks, and tracking them is opt-in.
- [**2026-08-09**](https://github.com/taojin-6/volumetric_kit_grid/blob/main/DECISIONS.md#2026-08-09--incremental-mesh-extraction-is-worth-building-at-a-4x-ceiling-not-the-18x-one-window-suggested-and-the-worst-frame-rather-than-the-median-sizes-its-design-amends-the-dirty-block-decision-above) —
  Incremental mesh extraction is worth building at a ~4x ceiling, not the ~18x
  one window suggested, and the worst frame rather than the median sizes its
  design (amends the dirty-block decision above). It was removed on 2026-10-06.
- [**2026-08-09**](#2026-08-09--timings-are-core-vocabulary-and-the-device-half-is-measured-not-inferred-counters-stay-in-the-tier-that-means-them) —
  Timings are `core` vocabulary and the device half is measured, not inferred;
  counters stay in the tier that means them.
- [**2026-08-10**](#2026-08-10--a-breakdown-row-is-one-the-callers-row-already-contains-so-the-host-total-skips-it-and-the-device-total-must-not-and-a-ceiling-the-library-knows-is-the-librarys-to-name) —
  A breakdown row is one the *caller's* row already contains, so the host total
  skips it and the device total must not; and a ceiling the library knows is the
  library's to name. *Reversed 2026-10-23:* device time tiles, so a stage holds
  its breakdowns' and both totals skip them.
- [**2026-08-11**](https://github.com/taojin-6/volumetric_kit_grid/blob/main/DECISIONS.md#2026-08-11--the-per-block-span-table-is-opt-in-is-retired-by-generation-rather-than-described-in-prose-is-anchored-per-block-slot-to-a-globally-unique-topology-token-and-is-one-table-for-the-whole-ring-rather-than-one-per-slot) —
  The per-block span table is opt-in, is retired by generation rather than
  described in prose, is anchored per block slot to a globally unique topology
  token, and is one table for the whole ring rather than one per slot. The table
  went on 2026-10-06.
- [**2026-08-11**](#2026-08-11--projective-texturing-decides-visibility-per-vertex-and-a-negative-uv0-carries-its-atlas-coordinate-rather-than-discarding-it-amends-the-2026-07-07-texture-tier-decision-and-retires-the-share_vertices-refusal-the-2026-08-04-entry-records) —
  Projective texturing decides visibility per *vertex*, and a negative `uv0`
  carries its atlas coordinate rather than discarding it (amends the 2026-07-07
  texture-tier decision, and retires the `share_vertices` refusal the
  2026-08-04 entry records). *Amended 2026-10-16:* the projection is no
  longer a per-tier copy; texture's goes through `core/shaders/pinhole.glsl`
  and keeps its own acceptance.
- [**2026-08-11**](https://github.com/taojin-6/volumetric_kit_grid/blob/main/DECISIONS.md#2026-08-11--an-incremental-extract-trusts-one-struct-cleared-on-every-path-and-re-established-only-where-a-mesh-is-handed-out-every-refusal-is-a-silent-fallback-and-the-fallback-is-reported) —
  An incremental extract trusts one struct, cleared on every path and
  re-established only where a mesh is handed out; every refusal is a silent
  fallback, and the fallback is *reported*. Removed on 2026-10-06.
- [**2026-08-11**](https://github.com/taojin-6/volumetric_kit_grid/blob/main/DECISIONS.md#2026-08-11--incremental-extraction-runs-under-share_vertices-because-that-kernel-owns-its-index-run-and-so-retires-more-cheaply-not-less-reverses-the-share_vertices-clause-of-the-incremental-dispatch-decision-above) —
  Incremental extraction runs under `share_vertices`, because that kernel owns
  its index run and so retires *more* cheaply, not less (reverses the
  `share_vertices` clause of the incremental-dispatch decision above). Removed
  on 2026-10-06.
- [**2026-08-12**](https://github.com/taojin-6/volumetric_kit_grid/blob/main/DECISIONS.md#2026-08-12--meshing-a-cameras-view-is-a-caller-supplied-block-list-not-a-camera-the-mesh-tier-holds-and-it-stays-apart-from-incremental-extraction-rather-than-stacking-with-it) —
  Meshing a camera's view is a caller-supplied block list, not a camera the mesh
  tier holds; and it stays apart from incremental extraction rather than
  stacking with it. Incremental extraction went on 2026-10-06. *Amended
  2026-10-13:* the list is a `DeviceBlockList`; the host `BlockList` is gone.
- [**2026-08-30**](#2026-08-30--a-profiler-label-belongs-to-the-kernel-not-to-the-timed-span-vk_ext_debug_utils-is-requested-independently-of-validation-and-the-instance-extension-is-declared-across-the-adopt-seam) —
  A profiler label belongs to the kernel, not to the timed span;
  `VK_EXT_debug_utils` is requested independently of validation, and the
  *instance* extension is declared across the adopt seam.
- [**2026-08-31**](https://github.com/taojin-6/volumetric_kit_grid/blob/main/DECISIONS.md#2026-08-31--the-dense-extract-goes-extract-becomes-extract_host-so-the-two-workflows-are-named-rather-than-inferred) —
  The dense extract goes; `extract` becomes `extract_host`, so the two
  workflows are named rather than inferred.
- [**2026-08-31**](https://github.com/taojin-6/volumetric_kit_grid/blob/main/DECISIONS.md#2026-08-31--a-triangles-work-unit-is-the-candidate-block-not-the-triangle-and-the-band-it-allocates-is-measured-from-the-surface-not-dilated-from-a-point) —
  A triangle's work unit is the candidate *block*, not the triangle; and the
  band it allocates is measured from the surface, not dilated from a point.
- [**2026-09-14**](#2026-09-14--the-examples-poll-their-frames-through-the-sensor-contract-the-replica-reader-is-an-icameracapture-a-source-says-when-it-is-exhausted-and-a-frame-kept-past-the-next-poll-is-copied--the-last-one-included) —
  The examples poll their frames through the sensor contract: the Replica
  reader is an `ICameraCapture`, a source says when it is exhausted, and a
  frame kept past the next poll is copied — the last one included. (The
  reader became an `IRgbdSensor`, and the copy went, on 2026-10-07.)
- [**2026-09-24**](#2026-09-24--the-orbbec-sdk-is-a-prerequisite-behind-vr_with_orbbec-installed-once-for-the-family-found-and-never-fetched) —
  The Orbbec SDK is a prerequisite behind `VR_WITH_ORBBEC`: installed once for
  the family, found, and never fetched. The floor rose to 2.10.6 on
  2026-10-07.
- [**2026-09-26**](#2026-09-26--the-orbbec-driver-lands-as-sensororbbec-a-target-of-its-own-it-undistorts-colour-and-then-registers-depth-to-it-on-the-host-and-it-reads-the-rigs-sync-roles-without-writing-them) —
  The Orbbec driver lands as `sensor/orbbec`, a target of its own: it
  undistorts colour and then registers depth to it on the host, and it reads
  the rig's sync roles without writing them. The host path went on
  2026-10-06: every frame is handed out as captured.
- [**2026-09-26**](https://github.com/taojin-6/volumetric_kit_grid/blob/main/DECISIONS.md#2026-09-26--the-tsdf-codec-is-one-codec-tier-over-volume-with-separate-encoder-and-decoder-classes-a-geometry-only-intra-frame-of-per-block-dct-coefficients-an-observed-voxel-mask-and-sorted-block-coordinates-entropy-coded-by-chunked-static-table-rans) —
  The TSDF codec is one `codec` tier over `volume`, with separate `Encoder` and
  `Decoder` classes: a geometry-only intra frame of per-block DCT coefficients,
  an observed-voxel mask and sorted block coordinates, entropy-coded by chunked
  static-table rANS. *Amended 2026-10-13:* `check_block_list` is gone.
- [**2026-09-27**](https://github.com/taojin-6/volumetric_kit_grid/blob/main/DECISIONS.md#2026-09-27--the-v1-frame-is-independent-rans-segments-of-sorted-blocks-every-integer-a-class-plus-raw-bits-behind-fixed-per-frame-tables-and-a-section-table-the-decoders-end-check-is-for-consistency-not-integrity-amends-the-2026-09-26-entrys-interleaved-lanes) —
  The v1 frame is independent rANS segments of sorted blocks, every integer a
  class plus raw bits, behind fixed per-frame tables and a section table; the
  decoder's end check is for consistency, not integrity (amends the 2026-09-26
  entry's interleaved lanes). *Amended 2026-10-13:* a segment holds at most
  1,024 entries, a rule every writer and reader holds.
- [**2026-09-27**](#2026-09-27--the-rig-is-orbbecrig-it-checks-the-cameras-against-the-rigs-sync-configuration-and-writes-it-only-when-asked-starts-the-secondaries-before-the-primary-keeps-the-cameras-on-the-hosts-clock-and-builds-each-set-around-a-primary-frame-poses-come-from-the-calibration-file) —
  The rig is `OrbbecRig`: it checks the cameras against the rig's sync
  configuration and writes it only when asked, starts the secondaries before
  the primary, keeps the cameras on the host's clock, and builds each set
  around a primary frame; poses come from the calibration file. The file
  moved to the `camera` tier on 2026-10-06, and the rig's sets became raw
  only the same day. `OrbbecRig` went on 2026-10-07: a rig is a
  `SensorArray` of `OrbbecSensor`s, on the SDK's global timestamps.
- [**2026-09-27**](https://github.com/taojin-6/volumetric_kit_grid/blob/main/DECISIONS.md#2026-09-27--encoder-and-decoder-are-the-codecs-public-api-encoding-drops-never-observed-blocks-and-sorts-the-rest-decoding-makes-a-callers-grid-hold-exactly-the-frame-by-diffing-its-block-set-everything-checkable-is-checked-before-the-grid-is-touched-and-a-grid-too-small-for-the-frame-is-refused-rather-than-grown) —
  `Encoder` and `Decoder` are the codec's public API: encoding drops
  never-observed blocks and sorts the rest, decoding makes a caller's grid hold
  exactly the frame by diffing its block set, everything checkable is checked
  before the grid is touched, and a grid too small for the frame is refused
  rather than grown. *Amended 2026-10-13:* the `Decoder` decodes P-frames
  too, against a reference it keeps. *Amended 2026-10-18:* a player's growth
  sits beside the decoder, as the `Fuser`'s does: `make_decode_grid` and
  `decode_growing`. *Amended 2026-10-21:* with `EncoderConfig::true_sdf`,
  observed blocks without surface are dropped too. *Amended 2026-10-22:*
  `make_decode_grid` and `decode_growing` take the caller's block ceiling,
  and refuse a frame or a grow past it as `InvalidArgument`.
  *Amended 2026-10-26:* the encoder takes a const grid.
- [**2026-09-27**](#2026-09-27--the-video-decoder-is-recon_sensor_video-a-target-of-its-own-over-an-installed-ffmpeg-hevc-in-host-pictures-out-on-the-first-hardware-back-end-that-decodes-a-built-in-clip-nvidia-ahead-of-an-integrated-gpu-each-ci-leg-requires-the-hardware-it-has) —
  The video decoder is `recon_sensor_video`, a target of its own over an
  installed FFmpeg: HEVC in, host pictures out, on the first hardware back end
  that decodes a built-in clip, NVIDIA ahead of an integrated GPU; each CI leg
  requires the hardware it has. The decoders hand out device pictures only
  since 2026-10-06.
- [**2026-09-27**](https://github.com/taojin-6/volumetric_kit_grid/blob/main/DECISIONS.md#2026-09-27--room0-sets-the-codecs-provisional-defaults-k--64-with-one-step-of-02-for-dc-and-ac-alike-the-coefficient-count-sets-the-quality-and-a-coarse-uniform-step-costs-almost-nothing-at-it-and-the-host-rans-coder-fits-a-frame-interval-at-1-cm-so-the-gpu-coder-waits) —
  Room0 sets the codec's provisional defaults, K = 64 with one step of 0.2 for
  DC and AC alike: the coefficient count sets the quality and a coarse uniform
  step costs almost nothing at it, and the host rANS coder fits a frame
  interval at 1 cm, so the GPU coder waits.
- [**2026-09-27**](https://github.com/taojin-6/volumetric_kit_grid/blob/main/DECISIONS.md#2026-09-27--quality-measurement-is-a-tier-of-its-own-eval-branching-off-mesh-mesh-to-mesh-accuracy-coverage-and-f-score-host-side-and-deterministic-and-production-infrastructure-rather-than-an-excluded-research-harness) —
  Quality measurement is a tier of its own, `eval`, branching off `mesh`:
  mesh-to-mesh accuracy, coverage and F-score, host-side and deterministic,
  and production infrastructure rather than an excluded research harness.
- [**2026-09-27**](https://github.com/taojin-6/volumetric_kit_grid/blob/main/DECISIONS.md#2026-09-27--a-mesh-becomes-a-tsdf-two-ways-in-the-tsdf-tier-signed-by-the-closest-triangles-normal-for-a-closed-mesh-or-as-an-unsigned-shell-for-any-mesh--binned-per-block-over-the-allocations-own-candidates-with-no-topology) —
  A mesh becomes a TSDF two ways, in the `tsdf` tier: signed by the closest
  triangle's normal, for a closed mesh, or as an unsigned shell, for any mesh
  — binned per block over the allocation's own candidates, with no topology.
  *Amended 2026-10-28:* in a tier of its own, `voxelize`.
- [**2026-09-28**](#2026-09-28--the-orbbec-driver-streams-colour-as-h265-on-request-every-colour-frame-is-decoded-in-order-on-a-thread-per-camera-ahead-of-the-mailbox-a-lost-frame-is-read-off-the-frame-index-not-the-clock-and-the-femto-megas-stream-is-decoded-as-bt601-full-range-which-it-codes-and-does-not-say) —
  The Orbbec driver streams colour as H.265 on request: every colour frame is
  decoded, in order, on a thread per camera ahead of the mailbox; a lost frame
  is read off the frame index, not the clock; and the Femto Mega's stream is
  decoded as BT.601 full range, which it codes and does not say. Its RGB
  output, and the RGB mode its calibration was read from, went on
  2026-10-06, and its host decoding the same day. *Amended 2026-10-27:*
  H.265 at 3840x2160 and 25 fps is the default.
- [**2026-09-28**](#2026-09-28--projective-texturing-from-several-views-chooses-a-view-per-triangle-on-an-unshared-mesh-into-an-atlas-of-the-views-images-side-by-side-the-single-camera-pass-stays-per-vertex) —
  Projective texturing from several views chooses a view per triangle, on an
  unshared mesh, into an atlas of the views' images side by side, in
  floor(sqrt(n)) rows; the single-camera pass stays per vertex.
- [**2026-09-28**](#2026-09-28--gpu-pre-processing-is-recon_sensor_utils-a-driver-hands-out-the-frame-as-captured-the-device-undistorts-depth-and-undistorts-and-converts-colour-and-fusion-reads-the-buffers-in-place-depth-and-colour-each-with-its-own-camera-rather-than-registered) —
  GPU pre-processing is `recon_sensor_utils`: a driver hands out the frame as
  captured, the device undistorts depth and undistorts and converts colour,
  and fusion reads the buffers in place, depth and colour each with its own
  camera rather than registered.
- [**2026-09-28**](#2026-09-28--memory-the-kernels-use-lives-on-the-device-on-every-platform-and-the-host-only-records-commands-against-it-one-commandbatch-per-call-parameters-inline-bulk-bytes-staged-at-the-edges-small-results-read-back) —
  Memory the kernels use lives on the device on every platform, and the host
  only records commands against it: one `CommandBatch` per call, parameters
  inline, bulk bytes staged at the edges, small results read back.
  *Amended 2026-10-13:* `diagnostics()` is gone; the tests check
  `load_factor()`'s copy of the heap counter by behaviour. *Amended
  2026-10-24:* a frame's results and input go through core's `HostRing`
  slots, in place where device-mapped memory is cached and copied
  elsewhere, chosen by the device, with no switch.
- [**2026-09-28**](#2026-09-28--a-decoded-colour-frame-never-leaves-the-gpu-the-platforms-hardware-decoder-for-h265-and-mjpeg-alike-hands-its-picture-to-vulkan-in-place-and-gpuframeprep-takes-i420-or-nv12-from-the-host-or-the-device) —
  A decoded colour frame never leaves the GPU: the platform's hardware
  decoder, for H.265 and MJPEG alike, hands its picture to Vulkan in place,
  and `GpuFramePrep` takes I420 or NV12 from the host or the device. The
  software decoding and fallbacks it kept went on 2026-10-06.
  *Amended 2026-10-25:* VideoToolbox's plane images are sampled where they
  are, not copied (the plane-image entry).
- [**2026-09-28**](#2026-09-28--a-device-takes-submits-from-several-threads-at-once-each-records-on-a-command-pool-of-its-own-and-only-the-queue-is-locked) —
  A `Device` takes submits from several threads at once: each records on a
  command pool of its own, and only the queue is locked.
- [**2026-09-28**](#2026-09-28--projective-texturing-takes-a-colour-camera-of-its-own-and-depth-on-the-device-the-depth-camera-decides-what-is-visible-its-map-what-the-colour-camera-sees-and-the-colour-camera-gives-the-coordinate-a-views-device-depth-and-coverage-are-copied-on-the-device-rather-than-staged) —
  Projective texturing takes a colour camera of its own and depth on the
  device: the depth camera decides what is visible, its map what the colour
  camera sees, and the colour camera gives the coordinate; a view's device
  depth and coverage are copied on the device rather than staged.
- [**2026-09-29**](#2026-09-29--rig_viewer-draws-a-live-orbbec-rig-raw-sets-prepared-fused-and-textured-on-the-gpu-the-atlas-filled-by-device-copies-recorded-in-gfxs-frame-and-the-frame-preps-colour-shared-with-gfxs-queue-family) —
  `rig_viewer` draws a live Orbbec rig: raw sets prepared, fused and textured
  on the GPU, the atlas filled by device copies recorded in gfx's frame, and
  the frame prep's colour shared with gfx's queue family. It reads a
  `SensorArray` since 2026-10-07. *Amended 2026-10-16:* depth may be shared
  too; the viewer keeps it EXCLUSIVE.
- [**2026-09-30**](#2026-09-30--a-rigs-cameras-fuse-in-one-batch-each-dispatch-binds-a-descriptor-set-of-its-own-over-the-same-kernel-and-the-set-is-compacted-once) —
  A rig's cameras fuse in one batch: each dispatch binds a descriptor set of
  its own over the same kernel, and the set is compacted once. A Dynamic set
  clears with every camera before it fuses with any since 2026-10-08.
- [**2026-09-30**](https://github.com/taojin-6/volumetric_kit_grid/blob/main/DECISIONS.md#2026-09-30--depth-allocation-works-a-16-x-16-pixel-tile-a-workgroup-and-dilates-each-distinct-block-of-the-tile-once-its-band-shared-out-over-the-lanes) —
  Depth allocation works a 16 x 16 pixel tile a workgroup and dilates each
  distinct block of the tile once, its band shared out over the lanes.
- [**2026-09-30**](https://github.com/taojin-6/volumetric_kit_grid/blob/main/DECISIONS.md#2026-09-30--with-the-spans-off-an-extracts-active-list-stays-on-the-device-and-the-map-hands-back-its-last-compaction-while-nothing-has-changed-since) —
  With the spans off, an extract's active list stays on the device, and the
  map hands back its last compaction while nothing has changed since. The spans
  went on 2026-10-06. *Amended 2026-10-24:* an extract records its own
  compaction into its batch when the last list does not hold, so only an
  extract after the encoder reuses one.
- [**2026-10-01**](https://github.com/taojin-6/volumetric_kit_grid/blob/main/DECISIONS.md#2026-10-01--every-block-slot-carries-stamps-ticks-of-one-clock-on-the-map-each-written-by-the-pass-that-knows-its-fact-the-grid-frees-the-blocks-no-allocation-has-asked-for-and-no-voxel-has-weighted-in-max_age-ticks) —
  Every block slot carries stamps, ticks of one clock on the map, each
  written by the pass that knows its fact; the grid frees the blocks no
  allocation has asked for and no voxel has weighted in `max_age` ticks.
- [**2026-10-01**](https://github.com/taojin-6/volumetric_kit_grid/blob/main/DECISIONS.md#2026-10-01--the-dirty-flags-become-the-changed-stamp-every-pass-that-writes-voxels-advances-the-maps-clock-and-stamps-what-it-changed-always-and-an-incremental-extract-keeps-the-tick-it-last-meshed-at-amends-the-2026-08-09-dirty-block-decision-and-the-stamps-entry-above) —
  The dirty flags become the `changed` stamp: every pass that writes voxels
  advances the map's clock and stamps what it changed, always, and an
  incremental extract keeps the tick it last meshed at (amends the
  2026-08-09 dirty-block decision and the stamps entry above). The incremental
  extract went on 2026-10-06; the stamp stays.
- [**2026-10-01**](https://github.com/taojin-6/volumetric_kit_grid/blob/main/DECISIONS.md#2026-10-01--the-codec-review-frame-v2-codes-a-partial-mask-a-plane-and-a-line-at-a-time-against-its-neighbours-and-a-sign-inside-its-mantissas-field-the-transform-finds-every-block-by-its-coordinate-one-commandbatch-a-call-over-device-memory-the-encoder-takes-the-maps-own-list-and-drops-never-observed-blocks-before-the-transform-and-the-decoder-stamps-only-what-it-changes-amends-the-changed-stamp-entry-above) —
  The codec review: frame v2 codes a partial mask a plane and a line at a
  time against its neighbours and a sign inside its mantissa's field; the
  transform finds every block by its coordinate, one `CommandBatch` a call
  over device memory; the encoder takes the map's own list and drops
  never-observed blocks before the transform; and the decoder stamps only
  what it changes (amends the `changed`-stamp entry above). *Amended
  2026-10-13:* the coordinate sort runs on the device too. *Amended
  2026-10-21:* with `EncoderConfig::true_sdf`, only the blocks holding
  surface are kept.
- [**2026-10-02**](#2026-10-02--shared-instructions-live-in-a-concise-agentsmd-claudemd-imports-it-designmd-holds-implementation-detail-and-decisionsmd-holds-the-decision-index-and-rationale) —
  Shared instructions live in a concise AGENTS.md; CLAUDE.md imports it, DESIGN.md holds implementation detail, and DECISIONS.md holds the decision index and rationale.
- [**2026-10-02**](#2026-10-02--gpu-regressions-guard-span-ownership-chroma-placement-descriptor-aliases-invalid-depth-taps-and-zero-near-visibility) —
  GPU regressions guard span ownership, chroma placement, descriptor aliases,
  invalid depth taps, and zero-near visibility; zero holes never occlude a
  colour sight line. The span check went on 2026-10-06.

- [**2026-10-02**](#2026-10-02--device-local-allocation-is-required-and-buffer-residency-is-explicit) —
  Device-local allocation is required and buffer residency is explicit;
  borrowed device inputs refuse unknown or non-local backing memory.

- [**2026-10-03**](https://github.com/taojin-6/volumetric_kit_grid/blob/main/DECISIONS.md#2026-10-03--garbage-collection-deletes-from-the-device-stale-list) —
  Garbage collection passes its device coordinate list directly to deletion,
  retaining host readback only for control counts. *Amended 2026-10-13:* it
  frees through the body `VoxelBlockGrid::remove` uses, which takes a device
  list alone.

- [**2026-10-02**](https://github.com/taojin-6/volumetric_kit_grid/blob/main/DECISIONS.md#2026-10-02--mesh-input-binning-keeps-counts-cursors-and-occupied-bins-on-the-device-retains-scratch-and-reads-only-validation-and-bounded-dispatch-control) —
  Mesh input binning keeps counts, cursors and occupied bins on the device,
  retains scratch, and reads only validation and bounded dispatch control.

- [**2026-10-03**](https://github.com/taojin-6/volumetric_kit_grid/blob/main/DECISIONS.md#2026-10-03--asset-loading-and-export-belong-to-io-with-assimp-as-an-optional-private-backend) —
  Asset loading and export move into the installed `io` tier; Assimp adds
  optional static mesh import, while dataset and normalization policy stay
  with consumers.

- [**2026-10-03**](#2026-10-03--follow-ups-to-the-2026-10-02-merges-zero-depth-is-missing-in-every-reader-only-an-accepted-remove-moves-the-topology-epoch-and-an-adopted-buffer-states-its-memory) —
  Zero depth is missing in every reader; only an accepted remove moves the
  topology epoch; an adopted buffer states its memory. Mesh binning's capacity
  scan and the planar RGB route stay, with measured costs.

- [**2026-10-02**](https://github.com/taojin-6/volumetric_kit_grid/blob/main/DECISIONS.md#2026-10-02--per-basis-quantization-and-a-normalized-mesh-codec-fixture) —
  Per-basis quantization replaces the DC/AC split; frame v3 carries the kept
  bases' weights, and a normalized Rafa2 mesh joins room0 in the codec study.
  *Amended 2026-10-14:* the normalization's arithmetic is `io`'s opt-in
  helpers, and Rafa2's frames 1–150 are coded as a sequence.
- [**2026-10-03**](https://github.com/taojin-6/volumetric_kit_grid/blob/main/DECISIONS.md#2026-10-03--compact-the-codecs-observed-list-on-the-device-and-retain-transform-scratch) —
  Compact the codec's observed list on the device, predict its readback prefix
  to avoid an unconditional extra fence, and retain transform scratch.
  *Amended 2026-10-13:* the list stays on the device; only its count comes
  back.
- [**2026-10-03**](https://github.com/taojin-6/volumetric_kit_grid/blob/main/DECISIONS.md#2026-10-03--codec-mesh-evaluation-consumes-the-shared-io-loader) —
  The codec example uses Assimp-backed asset loading and shared PLY export;
  height normalization and topology checks remain example policy. *Amended
  2026-10-14:* they are opt-in `io` helpers; the example chooses the up
  vector and height, and codes a numbered range under one transform.
- [**2026-10-03**](https://github.com/taojin-6/volumetric_kit_grid/blob/main/DECISIONS.md#2026-10-03--rans-encoding-runs-on-the-device-byte-for-byte-the-hosts-frame-chosen-per-frame-by-segment-count) —
  rANS encoding runs on the device, byte for byte the host's frame, and the
  encoder picks the device per frame from 48 segments. *Amended 2026-10-13:*
  encoding runs on the device alone too, its sort and step layout included;
  the CPU coder is the tests' oracle.
- [**2026-10-03**](https://github.com/taojin-6/volumetric_kit_grid/blob/main/DECISIONS.md#2026-10-03--rans-decoding-runs-on-the-device-block-for-block-the-hosts-chosen-from-80-segments) —
  rANS decoding runs on the device, block for block the host's, in segments
  of at most 1,024 blocks, and the decoder picks the device per frame from 80
  segments. *Amended 2026-10-13:* decoding runs on the device alone, with no
  host fallback; the host reader is the tests' oracle.
- [**2026-10-03**](#2026-10-03--error-handling-comes-from-volumetric_kit_core) —
  Error handling comes from `volumetric_kit_core`: `vr::Status`/`Result` are
  the core's types, `VR_TRY`/`VR_ASSIGN`/`VR_CHECK` its macros under recon's
  names, and recon logs through its one sink with source `"vr"`. The names
  are amended below: recon now writes `core::Status` and `VKC_TRY`.
- [**2026-10-04**](#2026-10-04--the-vulkan-foundation-comes-from-volumetric_kit_core) —
  The Vulkan foundation comes from `volumetric_kit_core`'s vulkan tier: `core/`
  names its types in `vr::`, `device_requirements()` states what recon's
  kernels need, and the viewer builds its device with the core's
  `SharedDevice`. The `vr::` names are amended below.
- [**2026-10-04**](#2026-10-04--recon-writes-the-cores-types-and-macros-under-the-cores-names-amends-the-two-entries-above) —
  recon writes the core's types and macros under the core's names
  (`core::Status`, `VKC_TRY`); the re-export headers and the `VR_*` aliases
  are gone (amends the two entries above).
- [**2026-10-05**](#2026-10-05--the-viewers-build-on-gfxs-core-migration) —
  The viewers pin gfx #106, after gfx moved onto the core: recon and gfx share
  one `Status`, `Device` and `StageRow`, so the device and timing seams
  convert nothing, and a program linking both carries one VMA.
- [**2026-10-06**](#2026-10-06--the-familys-camera-vocabulary-is-recons-camera-tier-a-double-precision-camera-model-and-the-sensor-arrays-calibration-file-version-2) —
  The family's camera vocabulary is recon's `camera` tier: a double-precision
  camera model with one templated projection, rigid transforms, and the
  sensor array's calibration file, version 2. It links the core's base tier
  and GLM, not Vulkan. *Amended 2026-10-11:* `VR_CAMERA_ONLY` configures it
  alone. Cut back the same day, below.
- [**2026-10-06**](#2026-10-06--the-camera-tier-keeps-only-what-a-capture-uses-amends-the-entry-above) —
  The camera tier keeps only what a capture uses: the camera model and its
  check, the lens's forward template, rigid checks, and each sensor's pose
  from the calibration file's version-1 layout. Unprojection, rescaling,
  version 2 and the writer go until a caller needs them. *Amended
  2026-10-12:* version 2 and the writer are back, for calib.
- [**2026-10-06**](#2026-10-06--tests-use-no-real-hardware-amends-the-2026-09-26-orbbec-driver-entry) —
  Tests use no real hardware: the three camera tests are removed, and a
  camera is checked by hand through `fuse_orbbec`. *Amended 2026-10-07:* no
  test looks for a camera either.
- [**2026-10-06**](#2026-10-06--a-sensors-frame-is-rgbdframe-each-cameras-cameramodel-the-poses-in-double-a-sequence-number-and-the-pixels-it-holds-and-the-femto-megas-factory-extrinsic-is-made-a-rotation) —
  A sensor's frame is `RgbdFrame`: each camera's `CameraModel`, the poses in
  double, a sequence number, and the pixels it holds. The Femto Mega's factory
  depth-to-colour rotation is not one, and is made one.
- [**2026-10-06**](#2026-10-06--a-sensor-is-an-irgbdsensor-what-it-is-before-a-frame-sensorinfo-frames-polled-newest-or-drained-oldest-first-and-the-femto-mega-is-orbbecsensor) —
  A sensor is an `IRgbdSensor`: what it is before a frame (`SensorInfo`),
  frames polled newest or drained oldest first. The Femto Mega is
  `OrbbecSensor`. (The Replica source followed on 2026-10-07, and
  `sync_clock_to_host` became the SDK's global timestamps.)
- [**2026-10-06**](#2026-10-06--several-sensors-are-a-sensorarray-vendor-neutral-started-in-their-rigs-order-grouped-by-trigger-or-by-sequence-number-and-posed-by-the-calibration) —
  Several sensors are a `SensorArray`, vendor-neutral: started in their rig's
  order, grouped by trigger or by sequence number, and posed by the
  calibration. (Sequence grouping went later that day; on 2026-10-07
  `TriggerGrouper` became its internals and the tolerance 0.4 of a frame
  period.)
- [**2026-10-06**](#2026-10-06--an-arrays-set-is-prepared-in-one-batch-every-stream-of-every-sensor-one-submit-and-one-wait-host-colour-staged-on-a-thread-per-camera-first) —
  An array's set is prepared in one batch, every stream of every sensor, one
  submit and one wait; host colour is staged on a thread per camera first,
  and `prepare_set` goes. (Host colour and the threads went later that day.)
- [**2026-10-06**](#2026-10-06--every-orbbec-frame-is-handed-out-as-captured-the-sdks-host-path-orbbeccapture-and-fuse_orbbec---gpu-go-amends-the-2026-09-26-2026-09-27-and-2026-09-28-orbbec-entries) —
  Every Orbbec frame is handed out as captured: the SDK's host path,
  `OrbbecCapture` and `fuse_orbbec --gpu` go, and the Orbbec driver needs
  the video decoders.
- [**2026-10-06**](#2026-10-06--the-video-decoders-hand-out-pictures-on-the-device-only-the-hardware-path-is-fixed-per-build-a-stream-the-hardware-refuses-is-an-error-and-off-apple-vr_with_ffmpeg-needs-vr_with_cuda-amends-the-2026-09-27-decoder-2026-09-28-decoded-frame-and-2026-10-06-raw-frames-entries) —
  The video decoders hand out pictures on the device only: the hardware path
  is fixed per build, a stream the hardware refuses is an error, and off
  Apple `VR_WITH_FFMPEG` needs `VR_WITH_CUDA`. *Amended 2026-10-25:* on
  Apple the H.265 decoder uses VideoToolbox directly, with no FFmpeg, and
  `VR_WITH_FFMPEG` there is a configure error.
- [**2026-10-06**](#2026-10-06--gpuframeprep-takes-colour-on-the-device-only-and-a-sets-depth-is-staged-on-one-thread-amends-the-2026-09-28-decoded-frame-and-2026-10-06-one-batch-entries) —
  `GpuFramePrep` takes colour on the device only, and a set's depth is
  staged on one thread. (Packed host colour came back the next day.)
- [**2026-10-06**](#2026-10-06--platforms-linux-android-macos-and-ios-with-gcc-or-clang-windows-and-msvc-are-not-supported-amends-the-2026-06-21-vulkan-path-entry) —
  Platforms: Linux, Android, macOS and iOS, with GCC or Clang; Windows and
  MSVC are not supported.
- [**2026-10-06**](#2026-10-06--public-api-serves-a-caller-entry-points-only-tests-used-go-or-move-into-their-tests-amends-the-2026-07-04-glm-entry) —
  Public API serves a caller: entry points only tests used go, or move into
  their tests. *Amended 2026-10-13:* `VoxelHashMap::diagnostics()` goes too.
- [**2026-10-06**](https://github.com/taojin-6/volumetric_kit_grid/blob/main/DECISIONS.md#2026-10-06--incremental-mesh-extraction-is-removed-the-changed-stamp-stays-and-the-mesher-no-longer-reads-it-amends-the-2026-08-09-2026-08-11-2026-08-12-2026-09-30-2026-10-01-and-2026-10-02-mesh-entries) —
  Incremental mesh extraction is removed; the `changed` stamp stays, and the
  mesher no longer reads it.
- [**2026-10-06**](#2026-10-06--fusion-runs-over-the-blocks-the-scanning-cameras-reach-one-frustum-a-frame-no-near-cut-far-at-max_depth--trunc_dist-their-union-compacted-in-one-scan-amends-the-2026-08-12-view-meshing-and-the-2026-09-30-one-batch-and-device-list-entries) —
  Fusion runs over the blocks the scanning cameras reach: one frustum a
  frame, no near cut, far at `max_depth + trunc_dist`, their union compacted
  in one scan. *Amended 2026-10-22:* in the fuse's own submit.
- [**2026-10-06**](https://github.com/taojin-6/volumetric_kit_grid/blob/main/DECISIONS.md#2026-10-06--marching-cubes-meshes-what-the-viewing-camera-sees-an-extract-over-the-device-frusta-list-the-viewers-re-meshing-on-a-fuse-or-a-view-change-and-a-culled-pass-records-its-density-amends-the-2026-08-12-view-meshing-entry-and-the-2026-10-06-incremental-removal-and-fusion-culling-entries) —
  Marching cubes meshes what the viewing camera sees: an extract over the
  device frusta list, the viewers re-meshing on a fuse or a view change, and
  a culled pass records its density. *Amended 2026-10-13:* the host
  `BlockList` it kept for the codec is gone.
- [**2026-10-06**](#2026-10-06--a-sensor-array-groups-its-frames-by-trigger-only-syncmode-and-sequence-grouping-go-amends-the-2026-10-06-sensorarray-entry) —
  A sensor array groups its frames by trigger only: `SyncMode` and sequence
  grouping go.
- [**2026-10-07**](#2026-10-07--gpuframeprep-stages-a-host-frames-packed-colour-beside-its-depth-amends-the-2026-10-06-device-only-gpuframeprep-entry) —
  `GpuFramePrep` stages a host frame's packed colour beside its depth
  (`RgbdFrame::color_packed`): one copy in the pass's own batch, no host
  conversion, no threads.
- [**2026-10-07**](#2026-10-07--the-replica-source-is-an-irgbdsensor-and-the-examples-fuse-what-gpuframeprep-prepares-icameracapture-capturedframe-and-ownedframe-go-and-io-loads-depth-as-stored-amends-the-2026-08-02-sensor-tier-2026-09-14-examples-and-2026-10-06-irgbdsensor-entries) —
  The Replica source is an `IRgbdSensor` and the examples fuse what
  `GpuFramePrep` prepares: `ICameraCapture`, `CapturedFrame` and
  `OwnedFrame` go, and `io` loads depth as stored. What the iOS scanner
  calls stays until it migrates; the frame gains no float depth. The viewer
  retains its last successful keyframe through a failed preparation or fuse.
- [**2026-10-07**](#2026-10-07--a-rig-is-an-orbbecsensor-per-camera-in-a-sensorarray-on-the-sdks-global-timestamps-sdk-2106-or-later-each-camera-is-checked-against-its-sync-settings-at-open-and-written-only-when-asked-the-tolerance-is-04-of-a-frame-period-and-orbbecrig-its-start-order-and-the-public-triggergrouper-go-amends-the-2026-09-24-sdk-2026-09-27-rig-and-2026-10-06-irgbdsensor-and-sensorarray-entries) —
  A rig is an `OrbbecSensor` per camera in a `SensorArray`, on the SDK's
  global timestamps (SDK 2.10.6 or later): each camera is checked against
  its sync settings at `open`, written only when asked and checked for SDK
  normalization after a write; the tolerance is 0.4 of a frame period, and
  `OrbbecRig`, its start order and the public `TriggerGrouper` go.
  *Amended 2026-10-08:* `open_orbbec_sensors` opens a rig's cameras from its
  sync file, for every caller.
- [**2026-10-08**](https://github.com/taojin-6/volumetric_kit_grid/blob/main/DECISIONS.md#2026-10-08--the-frames-models-and-block-grammar-are-one-glsl-text-that-the-host-compiles-as-c-and-golden-frames-pin-v3-amends-the-2026-10-03-encoding-entry) —
  The frame's models and block grammar are one GLSL text that the kernels
  include and the host compiles as C++, the device's walk runs in emit order,
  and golden frames pin v3's bytes on every machine.
- [**2026-10-08**](#2026-10-08--fusion-samples-at-integer-pixel-centres-and-refuses-a-non-finite-camera-amends-the-2026-07-06-depth-sampling-entry) —
  Fusion samples at integer pixel centres and refuses a non-finite camera.
- [**2026-10-08**](#2026-10-08--the-single-camera-texture-pass-needs-the-vertex-to-face-the-depth-camera-and-both-texture-passes-check-a-view-one-way-amends-the-2026-08-11-per-vertex-and-2026-09-28-colour-camera-entries) —
  The single-camera texture pass needs the vertex to face the depth camera,
  and both texture passes check a view one way.
- [**2026-10-10**](#2026-10-10--fusion-is-library-code-with-strict-defaults-and-opt-in-partial-coverage) —
  `tsdf::Fuser` owns grow/allocate/integrate with strict defaults and opt-in
  partial coverage; `volume::grow_grid` shares stateless growth arithmetic.
  *Amended 2026-10-22:* a partial set on a grid holding a block is one
  submit, and its capacity grow lands a set late.
- [**2026-10-10**](#2026-10-10--only-the-owner-triggers-ci-and-the-runner-tooling-is-the-cores) —
  Only the owner triggers CI, by the repository's Actions policy; no job
  carries a fork guard. The runner tooling is the core's.
- [**2026-10-08**](https://github.com/taojin-6/volumetric_kit_grid/blob/main/DECISIONS.md#2026-10-08--the-fuse-to-render-handoff-is-meshmeshexchange-one-gfx-free-library-type-owns-the-rings-release-a-parked-take-caps-the-mark-and-an-empty-mesh-draws-nothing-amends-the-2026-08-08-seam-b-and-2026-09-29-rig_viewer-entries) —
  The fuse-to-render handoff is `mesh::MeshExchange`: one gfx-free library
  type owns the ring's release, a parked take caps the mark, and an empty
  mesh draws nothing. *Amended 2026-10-19:* an empty mesh goes through
  `commit` with its payload. *Amended 2026-10-26:* `commit` may drop a mesh
  too.
- [**2026-10-08**](#2026-10-08--a-dynamic-set-clears-with-every-camera-before-it-fuses-with-any-amends-the-2026-09-30-one-batch-entry) —
  A Dynamic set clears with every camera before it fuses with any, through a
  clear kernel of its own; a single frame and Classic are unchanged.

- [**2026-10-10**](#2026-10-10--recon-keeps-exception-handlers-shares-frame-preparation-and-leaves-warning-policy-to-consumers) —
  recon keeps exception handlers and the normal CI configuration;
  `GpuFramePrep` shares one preparation path, and `-Werror` defaults on
  only in a top-level build.

- [**2026-10-08**](#2026-10-08--the-examples-parse-their-command-lines-with-one-parser-declare-each-shared-flag-family-once-print-stage-rows-in-one-table-and-keep-their-reports-out-of-their-mains-amends-the-2026-09-27-eval-and-2026-10-03-codec-mesh-entries) —
  The examples parse their command lines with one parser, declare each
  shared flag family once, print stage rows in one table, and keep their
  reports out of their mains.
- [**2026-10-08**](#2026-10-08--the-viewers-draw-with-gfxs-streamedatlas-on-its-frame-timeline-gfx-owns-the-atlas-ring-the-in-frame-copy-and-what-a-frame-holds-and-fuse_viewer-copies-its-keyframe-on-the-device-amends-the-2026-08-08-seam-b-2026-09-29-rig_viewer-2026-10-05-viewers-2026-10-07-replica-and-2026-10-08-exchange-entries) —
  The viewers draw with gfx's `StreamedAtlas` on its frame timeline: gfx
  owns the atlas ring, the in-frame copy and what a frame holds, and
  `fuse_viewer` copies its keyframe on the device.
- [**2026-10-08**](#2026-10-08--recons-gpu-tests-run-on-the-cores-test-policy-the-validation-layer-on-every-ci-leg-fails-a-test-on-any-error-it-reports-and-a-skip-is-a-skip) —
  recon's GPU tests run on the core's test policy: the validation layer on
  every CI leg fails a test on any error it reports, a leaked object's
  included, and a skip is a skip.
- [**2026-10-08**](#2026-10-08--a-decoded-picture-is-its-yuvimage-with-its-pts-and-encoding-the-one-description-of-a-device-picture-which-a-frame-takes-as-it-is-amends-the-2026-09-28-decoded-frame-and-2026-10-02-gpu-regression-entries) —
  A decoded picture is its `YuvImage`, with its pts and encoding: one
  description of a device picture, which a frame takes as it is.
  *Amended 2026-10-26:* NVDEC's picture is two plane images, which CUDA
  wrote; nvJPEG's alone is a buffer.
- [**2026-10-08**](#2026-10-08--apples-pixel-buffer-importer-is-public-pixelbufferimporter-in-recon_sensor_apple-on-every-apple-build-ios-included-with-no-ffmpeg) —
  Apple's pixel-buffer importer is public: `PixelBufferImporter` in
  `recon_sensor_apple`, on every Apple build, iOS included, with no FFmpeg.
  *Amended 2026-10-25:* CI compiles the importer for iOS 26.0.
  *Amended again 2026-10-25:* `recon_sensor_video` is no longer behind
  `VR_WITH_FFMPEG` on Apple, and CI's iOS check links it with the importer.
  *And 2026-10-25 (the plane-image entry):* its planes gain `SAMPLED` usage
  and a view.

- [**2026-10-10**](https://github.com/taojin-6/volumetric_kit_grid/blob/main/DECISIONS.md#2026-10-10--grid-validation-bounds-the-truncation-band-by-the-depth-kernels-integer-representation) —
  Grid validation rejects non-finite metrics and bounds the truncation band by
  the depth kernel's integer representation.
- [**2026-10-11**](#2026-10-11--decoded-pictures-name-the-queue-families-that-read-them-videotoolboxs-images-are-concurrent-across-two-or-more) —
  Decoded pictures name the queue families that read them: VideoToolbox's
  images are CONCURRENT across two or more.
- [**2026-10-11**](#2026-10-11--examples-load-the-cores-scenario-format-and-keep-only-runtime-adapters) —
  Examples load the core scenario format; JSON, defaults and static validation
  have one owner. *Amended 2026-10-12:* the codec's step rules are recon's.
  *Amended 2026-10-14:* so are the mesh conversion's shell bound and a mesh
  path's frame number.
- [**2026-10-11**](#2026-10-11--picture-readback-publishes-buffer-ownership-for-subsequent-stages) —
  Picture readback updates ownership before subsequent reads or preparation.
- [**2026-10-12**](#2026-10-12--the-viewers-window-glue-comes-from-gfxs-gfx_glfw-recon-keeps-no-copy-amends-the-2026-08-02-bootstrap-and-2026-10-04-vulkan-foundation-entries) —
  The viewers' window glue comes from gfx's `gfx_glfw`; recon keeps no copy.
- [**2026-10-12**](#2026-10-12--the-calibration-file-records-lenses-and-is-written-version-2-and-its-writer-return-with-calib-its-first-caller-amends-the-2026-10-06-camera-tier-entries) —
  The calibration file records lenses and is written: version 2 under
  version 1's `device_calibration` key, its writer and
  `rodrigues_from_rotation` return with calib; `SensorArray::open` refuses an
  unposed sensor, and the file I/O is the core's. *Amended 2026-10-18:* the
  world may be a printed AprilTag. *Amended 2026-10-27:* version 3, a
  sensor's depth offset or the colour cameras' exposure, written only for a
  corrected calibration.
- [**2026-10-12**](https://github.com/taojin-6/volumetric_kit_grid/blob/main/DECISIONS.md#2026-10-12--recordings-are-vrst-files-of-our-own-verbatim-records-in-crc-framed-chunks-readable-from-a-killed-writer-in-recon_stream-on-the-cores-base-tier-alone-amends-the-2026-09-27-v1-frame-entry) —
  Recordings are `.vrst` files of our own: verbatim records in CRC-framed,
  8-byte-aligned chunks, readable from a killed writer, in `recon_stream` on
  the core's base tier alone. *Amended 2026-10-14:* a Geometry track's
  params may hold a mesh source's transform. *Amended 2026-10-21:* its
  chunks are made and read in memory too, publicly.
- [**2026-10-12**](https://github.com/taojin-6/volumetric_kit_grid/blob/main/DECISIONS.md#2026-10-12--p-frames-are-frame-type-1-of-v3-each-reference-block-gets-a-mode-in-update-segments-new-blocks-follow-in-intra-segments-and-a-reference-section-names-the-reference-by-xxh64-of-its-bytes-experimental-until-golden-p-frames-freeze-it) —
  P-frames are frame type 1 of v3: a mode per reference block in update
  segments, new blocks in intra segments, the reference named by XXH64 of its
  bytes; a missing reference is `NotFound`. Experimental until golden
  P-frames freeze it. *Amended 2026-10-13:* the `Decoder` decodes them; row
  13 is the device merge's, and the host merge is gone. *Amended
  2026-10-14:* the `Encoder` writes them. *Amended 2026-10-16:* golden
  P-frames pin type 1's bytes, but its freeze waits on a decision about
  interleaved rANS lanes. *Amended 2026-10-17:* no lanes: U defaults to R,
  and the freeze waits on the encode's count and a re-measure. *Amended
  2026-10-17:* frozen, so type 1's bytes change only on purpose.
- [**2026-10-13**](https://github.com/taojin-6/volumetric_kit_grid/blob/main/DECISIONS.md#2026-10-13--the-decoder-decodes-p-frames-against-the-frame-it-decoded-last-kept-on-the-device-as-decoded-in-two-slots-that-swap-only-when-a-decode-succeeds-one-integer-apply-makes-every-decoded-frame-intra-too-and-it-decodes-on-the-device-alone-amends-the-2026-09-27-encoderdecoder-and-2026-10-03-decoding-entries) —
  The `Decoder` decodes P-frames against the frame it decoded last, kept on
  the device as decoded and replaced only when a decode succeeds; one integer
  apply makes every decoded frame; and it decodes on the device alone.
  `FrameType` and `FrameInfo::type` are public. *Amended 2026-10-13:* the
  reference's coordinates live on the device, and the merge runs there.
  *Amended 2026-10-14:* the `Encoder` keeps its frames in the same store
  and writes P-frames. *Amended 2026-10-21:* the deblock, on request, joins
  the decode's last submit, and `rebuild` remakes a frame from the
  reference.
- [**2026-10-13**](https://github.com/taojin-6/volumetric_kit_grid/blob/main/DECISIONS.md#2026-10-13--the-codecs-coordinate-work-runs-on-the-device-the-encoder-sorts-its-blocks-with-a-stable-rank-merge-sort-on-coord_less-and-the-decoder-judges-merges-and-diffs-them-by-count-and-binary-search-never-sorting-only-counts-and-verdicts-cross-the-bus-amends-the-2026-10-13-decoder-2026-10-12-p-frame-2026-10-03-observed-list-and-garbage-collection-and-2026-10-01-codec-review-entries) —
  The codec's coordinate work runs on the device: the encoder sorts with a
  stable rank-merge sort, and the decoder judges, merges and diffs by binary
  search, never sorting; only counts and verdicts cross the bus. *Amended
  2026-10-13:* so does the writer's step layout, which leaves the codec's
  host only I/O and control; and the host `BlockList` it left without a
  caller goes, with the host-array `allocate` and `remove`. *Amended
  2026-10-14:* the encoder's split is its diff's own search, not a
  `BlockLists` pass. *Amended 2026-10-22:* the grid's zeroing rides the
  map's remove's first round, a fence fewer. *Amended 2026-10-24:* the
  verdicts no longer cross before the grid changes; a gate on the device
  judges them, and the counts are read once the frame's batch has run.
- [**2026-10-13**](https://github.com/taojin-6/volumetric_kit_grid/blob/main/DECISIONS.md#2026-10-13--block-lists-reach-the-volume-tier-on-the-device-alone-a-host-list-entry-point-that-only-tests-called-goes-amends-the-2026-08-08-sparse-mesh-and-overflow-scan-2026-08-12-view-meshing-2026-09-26-codec-2026-09-28-device-memory-2026-10-03-garbage-collection-2026-10-06-public-api-and-view-culling-and-2026-10-13-coordinate-entries) —
  Block lists reach the volume tier on the device alone; a host-list entry
  point that only tests called goes: the host `BlockList`, the host-array
  `allocate` and `remove`, and the table readbacks. *Amended 2026-10-17:*
  `HashEntry` and the fail slots are internal, the host sentinels go, and
  `hash_bucket` moves into the tests. *Amended 2026-10-28:* the entries
  accessor is internal too.
- [**2026-10-14**](https://github.com/taojin-6/volumetric_kit_grid/blob/main/DECISIONS.md#2026-10-14--the-encoder-writes-zero-motion-p-frames-against-the-frame-it-returned-last-kept-in-the-referencestore-as-its-forward-made-it-five-rules-make-an-i-frame-and-the-split-the-diff-and-a-static-price-for-zero-run-on-the-device-amends-the-2026-10-13-decoder-and-coordinate-and-2026-10-12-p-frame-entries) —
  The `Encoder` writes zero-motion P-frames at `intra_period` above 1,
  against the frame it returned last, kept in the `ReferenceStore` as its
  forward made it and moved only when `encode` returns one; five rules make
  an I-frame; the split, the diff and a static price for zero run on the
  device. *Amended 2026-10-16:* a sixth rule, an empty grid; U is
  `EncoderConfig::update_size`. *Amended 2026-10-17:* a P-frame is counted
  in the forward's submit.
- [**2026-10-14**](https://github.com/taojin-6/volumetric_kit_grid/blob/main/DECISIONS.md#2026-10-14--mesh-normalization-and-the-topology-audit-are-opt-in-generic-io-helpers-for-a-mesh-sequence-bounds-measured-along-one-up-and-joined-over-frames-and-one-transform-for-every-frame-amends-the-2026-10-02-normalized-mesh-and-2026-10-03-codec-mesh-entries) —
  Mesh normalization and the topology audit are opt-in, generic `io`
  helpers for a mesh sequence: bounds measured along one up and joined over
  frames, and one transform applied to every frame. `codec_mesh` uses them,
  and its single frame keeps its bytes.
- [**2026-10-14**](https://github.com/taojin-6/volumetric_kit_grid/blob/main/DECISIONS.md#2026-10-14--codec_mesh-codes-a-numbered-mesh-sequence-under-one-transform-a-frame-number-in-sourcepath-an-exact-range-one-transform-fitted-to-every-frame-and-stored-in-the-stream-a-cleared-grid-and-one-record-per-frame-and-sourceallow_holes-amends-the-2026-10-02-normalized-mesh-2026-10-03-codec-mesh-2026-10-11-examples-and-2026-10-12-container-entries) —
  `codec_mesh` codes a numbered mesh sequence: a printf-style frame number in
  `source.path`, `run.first` and an exact `run.frames`, one transform fitted
  to every frame and stored in the Geometry track's params, a cleared grid
  and one record per frame, `source.allow_holes`, and its stream made before
  any work. Frame 150's conversion outlier is mostly the closest-face sign
  at a sliver fold. *Amended 2026-10-15:* require core 0.8.0's directory
  escaping, and refuse example intra periods other than 1 until C5b.
  *Amended 2026-10-16:* the examples code P-frames, on core 0.9.0.
  *Amended 2026-10-21:* it codes a true SDF and refuses a band too narrow
  for one; the templates convert in 16 voxels.
- [**2026-10-16**](https://github.com/taojin-6/volumetric_kit_grid/blob/main/DECISIONS.md#2026-10-16--the-codec-examples-code-p-frames-codecintra_period-and-codecupdate_size-reach-the-encoder-and-a-streams-rows-report-and-keys-go-by-frame-type-an-empty-grid-is-an-i-frame-golden-p-frames-pin-type-1s-bytes-and-its-freeze-waits-since-at-r--16-its-p-frames-code-slower-than-intra-amends-the-2026-10-12-p-frame-2026-10-14-encoder-and-codec-mesh-entries) —
  The codec examples code P-frames: `codec.intra_period` and
  `codec.update_size` reach the `Encoder`, a stream's rows, report and keys
  go by frame type, and a failure after the encode forces an I-frame. An
  empty grid is an I-frame, U is `EncoderConfig::update_size` (64,
  measured), and golden P-frames pin type 1's bytes. Type 1 stays
  experimental and the templates at period 1: at R = 16 its P-frames code
  slower than intra, and the freeze waits on a decision about interleaved
  rANS lanes. *Amended 2026-10-17:* U defaults to the segment size; no lanes,
  and the freeze waits on the encode's count and a re-measure. *Amended
  2026-10-17:* type 1 is frozen, though P encode at R = 16 stays slower than
  intra on the host, and the templates code at period 30.
- [**2026-10-16**](#2026-10-16--a-shared-glsl-pinhole-for-recon-and-its-consumers-coreshaderspinholeglsl-exported-under-the-include-root-recon_core-names-amends-the-2026-08-11-texturing-entrys-per-tier-copies) —
  A shared GLSL pinhole for recon and its consumers,
  `core/shaders/pinhole.glsl` (amends the 2026-08-11 texturing entry's
  per-tier copies): values, not structs, so scalar-layout and std430 kernels
  both call it; recon's kernels project through it, keeping only their own
  acceptance rules; and `recon_core`'s `VR_SHADER_INCLUDE_DIR` names the root
  a consumer includes it under, fetched or installed. *Amended 2026-10-22:*
  fetched only; recon installs nothing. *Amended 2026-10-28:* volume
  unprojects through a copy of its own, held to it bit for bit.
- [**2026-10-16**](#2026-10-16--a-brew-upgrade-needs-no-cache-clearing-the-mac-jobs-install-nothing-and-the-dependency-cache-keys-on-the-cores-homebrew-key) —
  A brew upgrade needs no cache clearing: the Mac jobs install nothing, and
  the dependency cache keys on the core's Homebrew key.
- [**2026-10-16**](#2026-10-16--a-prepared-frames-depth-is-shared-like-its-colour-gpuframeprepconfigdepth_queue_families-recon-builds-no-point-clouds-and-makes-no-calibration-measurements-the-same-k-depth-crop-and-nearest-resampling-are-accepted-on-a-stand-in-lens-pending-a-fold-check-of-the-rigs-own-amends-the-2026-09-29-rig_viewer-entry) —
  A prepared frame's depth may be shared with another queue family, as its
  colour is: `GpuFramePrepConfig::depth_queue_families`, with no vertex
  usage. recon builds no point clouds and makes no calibration
  measurements. The same-K depth crop and nearest resampling are accepted on
  a stand-in lens, pending a fold check of each rig camera's own (by hand,
  not yet run).
- [**2026-10-17**](https://github.com/taojin-6/volumetric_kit_grid/blob/main/DECISIONS.md#2026-10-17--a-p-frames-update-segments-default-to-its-segment-size-encoderconfigupdate_size-0-the-default-takes-segment_size-resolved-once-in-encodercreate-the-format-and-the-goldens-are-unchanged-amends-the-2026-10-16-codec-examples-2026-10-14-encoder-and-2026-10-12-p-frame-entries) —
  A P-frame's update segments default to its segment size:
  `EncoderConfig::update_size` 0, the default, takes `segment_size`
  (`resolved_update_size()`, resolved once in `Encoder::create`); the format
  and the goldens are unchanged. No lanes: the freeze waits on the encode's
  count moving into the forward's submit, and a re-measure. *Amended
  2026-10-17:* the re-measure ran, and type 1 is frozen (the freeze entry).
- [**2026-10-17**](https://github.com/taojin-6/volumetric_kit_grid/blob/main/DECISIONS.md#2026-10-17--a-p-frame-is-counted-in-the-forwards-submit-its-count-and-step-layout-run-over-m-plus-the-frames-blocks-the-device-takes-off-the-diffs-survivors-and-an-encode-waits-on-one-fence-fewer-amends-the-2026-10-14-encoder-entry) —
  A P-frame is counted in the forward's submit: its count and step layout
  run over M plus the frame's blocks, the device takes off the diff's
  survivors, and an encode waits on one fence fewer (amends the 2026-10-14
  encoder entry).
- [**2026-10-17**](https://github.com/taojin-6/volumetric_kit_grid/blob/main/DECISIONS.md#2026-10-17--frame-type-1-is-frozen-so-its-bytes-change-only-on-purpose-and-the-codec-templates-code-at-period-30-p-frames-code-no-slower-than-intra-on-room0-at-1-cm-and-on-rafa2-at-r--64-and-at-r--16-decode-no-slower-but-encode-up-to-4-slower-on-the-host-alone-amends-the-2026-10-12-p-frame-2026-10-16-codec-examples-and-2026-10-17-update-size-and-count-entries) —
  Frame type 1 is frozen, so its bytes change only on purpose, and
  `codec_replica.json` and `codec_mesh_sequence.json` code at period 30.
  The re-measure missed one comparison, Rafa2's encode at R = 16, where
  P-frames take up to 4% longer than intra on the host alone; the user
  froze type 1 anyway, and trimming that overhead is next.
- [**2026-10-18**](#2026-10-18--the-calibration-files-world-may-be-a-printed-apriltag-in-opencvs-marker-axes-and-the-file-stays-version-2-amends-the-2026-10-12-calibration-file-entry) —
  The calibration file's world may be a printed AprilTag, in OpenCV's
  marker axes rather than libapriltag's, so a floor tag's z is up. A world
  is a sensor or a tag, never both, and a tag puts no sensor at the origin.
  The file stays version 2: an older reader already refuses a tag world as
  `Unsupported`.
- [**2026-10-20**](#2026-10-20--mesh-sequences-clear-the-grid-per-frame-live-rigs-fuse-dynamic-and-free-stale-blocks-and-fuse_orbbecs-stream-stays-in-the-calibrations-world-uncropped) —
  Mesh sequences clear the grid per frame (`codec_mesh`); live rigs fuse
  Dynamic and free stale blocks (`rig_viewer` by default, `fuse_orbbec`
  with `fusion.dynamic: true`, as its stream template sets), in the world
  the calibration defines: no upright transform, crop, raw recording or
  colour yet. *Amended 2026-10-20:* `fuse_orbbec` fuses Dynamic by default
  too (core #45).
- [**2026-10-21**](https://github.com/taojin-6/volumetric_kit_grid/blob/main/DECISIONS.md#2026-10-21--the-vrst-chunks-in-memory-are-public-chunkencoder-frames-what-streamwriter-writes-by-the-record-rules-it-now-holds-and-chunkparser-reads-bytes-as-they-arrive-through-streamreaders-checks-amends-the-2026-10-12-container-entry) —
  The `.vrst` chunks in memory are public (`stream/chunks.hpp`), so a second
  tier, networking next, writes and reads the format without stream's
  detail: `ChunkEncoder` frames what `StreamWriter` writes and holds the
  record rules; `ChunkParser` copies bytes as they arrive and checks each
  chunk with the functions `StreamReader` uses, its header before waiting
  for its body; a failure is final.
- [**2026-10-21**](https://github.com/taojin-6/volumetric_kit_grid/blob/main/DECISIONS.md#2026-10-21--a-session-is-served-live-over-tcp-by-a-tier-of-its-own-recon_networking-the-wire-is-the-file-newest-key-wins-and-a-viewer-joins-at-the-cached-gop) —
  A session is served live over TCP by a tier of its own,
  `recon_networking` (the owner's: sockets apart from the format, testable
  alone). The wire is the `.vrst` file's bytes; a viewer behind skips to a
  newer key, only at a chunk boundary, so no P-frame goes without its
  references; a viewer joins at the cached GOP; `publish` never waits; a
  viewer types the sender's address. *Amended 2026-10-25:* a live sender
  serves keys on demand instead (`replay_gop` false), and the cached GOP
  stays the default.
- [**2026-10-21**](https://github.com/taojin-6/volumetric_kit_grid/blob/main/DECISIONS.md#2026-10-21--meshintegrator-finds-each-voxels-nearest-triangle-with-an-exact-early-exit-a-triangle-is-ruled-out-only-by-a-lower-bound-less-a-rounding-margin-of-its-own-strictly-above-the-voxels-limit) —
  `MeshIntegrator` finds each voxel's nearest triangle with an exact early
  exit: a workgroup per block sorts its bin by a lower bound, and a triangle
  is ruled out only by a bound, less a rounding margin of its own, strictly
  above the voxel's limit, so every voxel writes what its whole bin writes.
  Blocks past 16^3 voxels are refused.
- [**2026-10-21**](https://github.com/taojin-6/volumetric_kit_grid/blob/main/DECISIONS.md#2026-10-21--a-true-signed-distance-codes-without-the-observed-mask-encoderconfigtrue_sdf-cuts-each-corner-farther-than-3-voxels-from-zero-of-a-cell-whose-sign-changes-and-codes-only-the-blocks-that-hold-surface-whole-codec_mesh-alone-sets-it-in-a-band-of-at-least-15-voxels-so-fused-depth-keeps-its-mask-and-the-mesh-templates-convert-in-16-voxels-and-code-at-k-56-amends-the-2026-09-27-encoder-2026-10-01-codec-review-and-2026-10-14-codec-mesh-entries) —
  A true signed distance codes without the observed mask:
  `EncoderConfig::true_sdf` cuts each corner farther than √3 voxels from
  zero of a cell whose sign changes, and codes only the blocks that hold
  surface. `codec_mesh` alone sets it, in code, so multi-view RGB-D capture
  keeps its mask, byte for byte. The mesh templates convert in 16 voxels
  and code at K 56, about 2 Mbit/s on Rafa2; the format, the decoder and
  the goldens are unchanged.
- [**2026-10-21**](https://github.com/taojin-6/volumetric_kit_grid/blob/main/DECISIONS.md#2026-10-21--the-decoder-deblocks-on-request-a-runtime-switch-off-by-default-runs-the-studys-step-keyed-line-fit-and-gaussian-on-the-device-in-the-decodes-last-submit-keeping-each-block-with-an-unobserved-voxel-as-decoded-and-a-paused-player-rebuilds-its-frame-from-the-reference-amends-the-2026-10-13-decoder-entry) —
  The decoder deblocks on request: `Decoder::set_deblock`, off by default
  and read at each decode, runs the study's step-keyed line fit and
  Gaussian on the device in the decode's last submit, its filter picked
  from each frame's header; `Decoder::rebuild` remakes the last frame from
  the reference with the switch as it is, so a paused player shows a
  toggle. Streams are unchanged. *Amended 2026-10-22:* a frame with an
  unobserved voxel was skipped: on a live room capture the filter erodes
  surface at mask edges and in thin parts. *Amended 2026-10-23:* no frame
  is skipped; a block with an unobserved voxel keeps its exact decode, and
  a face is fitted only between two fully observed blocks, so a true-SDF
  frame is smoothed but for its cut blocks and a live frame, masked in
  nearly every block, decodes nearly exactly; `DeblockInfo` counts the
  blocks kept.
- [**2026-10-22**](#2026-10-22--recon-is-fetched-by-commit-never-installed-no-install-rules-package-config-or-package-tests-amends-the-2026-10-16-glsl-pinhole-entry-and-the-2026-10-17-amendment-of-the-2026-10-13-block-list-entry) —
  recon is fetched by commit, never installed (the owner's rule for the
  family): no install rules, package config or package tests. The
  add_subdirectory checks stay, the public headers' one among them, and the
  camera-only check runs in the camera-only build alone.
- [**2026-10-22**](https://github.com/taojin-6/volumetric_kit_grid/blob/main/DECISIONS.md#2026-10-22--streamclient-reports-progress-before-a-whole-record-is-ready) —
  `StreamClient::next` can report partial input and connection progress,
  allowing a consumer to drain it before waiting on its own controls.
- [**2026-10-23**](#2026-10-23--a-stages-device-time-holds-every-command-recorded-inside-it-so-both-totals-skip-breakdowns-the-extract-and-the-decodes-grid-edits-are-timed-and-a-grid-builds-its-block-kernels-at-create-amends-the-2026-08-10-breakdown-entry) —
  A stage's device time holds every command recorded inside it (core
  0.20.0's tiles), so both totals skip breakdowns; a fused set's allocation
  is `"integrate"`'s `"  ..allocate"`; the extract and the decode's grid
  edits are timed inside today's submits; a grid builds its block kernels at
  create, and an extractor or a grid its timer at the first timed call.
- [**2026-10-24**](https://github.com/taojin-6/volumetric_kit_grid/blob/main/DECISIONS.md#2026-10-24--a-frame-decodes-and-meshes-in-the-callers-batch-every-count-stays-on-the-device-the-maps-rounds-k--3-the-extract-and-the-decoder-gather-their-results-into-ring-slots-and-the-blocking-calls-are-one-submit-amends-the-2026-09-28-device-memory-2026-10-13-coordinate-and-2026-10-23-breakdown-entries) —
  A frame decodes and meshes in the caller's batch: the map's removal and
  allocation run K = 3 device rounds, the extract sizes itself on the
  device, and the decoder judges rows 11 to 13 there; each gathers its
  results into a ring slot read after the batch. The blocking calls are
  record, one submit and finish.
- [**2026-10-25**](#2026-10-25--hevcdecoder-decodes-through-videotoolbox-directly-on-apple-no-ffmpeg-the-hardware-decoder-or-none-pictures-in-display-order-by-their-order-count-and-vr_with_ffmpeg-on-apple-a-configure-error-amends-the-2026-10-06-device-only-decoder-and-2026-10-08-pixel-buffer-importer-entries) —
  `HevcDecoder` decodes through VideoToolbox directly on Apple: no FFmpeg,
  the hardware decoder or none, pictures put in display order by their
  order count and written at the range the stream declares;
  `recon_sensor_video` builds on every Apple platform, and `VR_WITH_FFMPEG`
  on Apple is a configure error.
- [**2026-10-25**](#2026-10-25--a-sender-records-each-set-into-two-batches-every-count-on-the-device-and-its-figures-are-per-set-amends-the-2026-09-27-encoder-2026-09-28-device-memory-2026-10-10-fusion-and-2026-10-21-chunk-entries) —
  A sender records each set into two batches: its prep, one allocation
  round, its fuse (gated on the whole band where strict, a refused set fused
  again at its own tick), the free pass and the frame's front half in one,
  the frame's rANS half in the next, the payload in a slot and written and
  served under one CRC: 2 submits and no staging buffer a warm coded set,
  from 6 and 4. A sensor may notify. fuse_orbbec and codec_replica print
  each row's mean, p50, p95 and max over the sets, the device's counters per
  set and the latency to publish, P-frame, I-frame and uncoded sets apart;
  the encoder's and the free pass's compactions are `"  ..encode set"` and
  `"  ..stale set"`, apart from a fuse's `"  ..active set"`.
- [**2026-10-25**](#2026-10-25--the-h265-syntax-reader-is-public-in-a-target-of-its-own-on-every-platform-amends-the-2026-10-25-videotoolbox-decoder-entry) —
  The H.265 syntax reader is public (`sensor/video/hevc_syntax.hpp`,
  `recon_sensor_video_syntax`): host code on the core's base tier alone,
  built on every platform, its parsers returning `Result` and its
  `BitReader` public with them.
- [**2026-10-25**](#2026-10-25--frame-prep-samples-nv12-plane-images-where-they-are-in-a-second-entry-of-the-colour-pass-and-the-plane-copies-go-amends-the-2026-09-28-decoded-frame-and-2026-10-08-pixel-buffer-importer-entries) —
  Frame prep samples NV12 plane images where they are, through combined
  image samplers in a second entry over the colour pass's body, to the
  buffer form's words bit for bit; the two plane copies and their scratch
  buffer go. Plane images need `SAMPLED` usage and a view for frame prep,
  `TRANSFER_SRC` for readback, and the importer's carry both.
  *Amended 2026-10-26:* NVDEC's pictures are plane images too, which both
  readers take over from outside Vulkan (the NVDEC plane-image entry).
- [**2026-10-25**](#2026-10-25--a-runs-sets-the-fusion-grid-and-the-block-layout-are-library-calls-rigsets-in-the-arrays-tier-create_fusion_grid-in-tsdf-and-block_grid_params-in-volume-opening-a-rig-stays-the-callers) —
  A run's sets, the fusion grid and the block layout move from the examples
  into the library: `sensor::RigSets` in `recon_sensor_array`,
  `tsdf::create_fusion_grid` and `fusion_buckets` in `recon_tsdf`, and
  `volume::block_grid_params`, which `codec::make_decode_grid` builds with
  too. Opening a rig stays the three public calls, and a set's prep and all
  printing stay the caller's.
- [**2026-10-25**](https://github.com/taojin-6/volumetric_kit_grid/blob/main/DECISIONS.md#2026-10-25--a-live-sender-makes-keys-on-demand-a-joiner-waits-for-the-next-key-the-server-keeps-what-its-viewers-still-need-and-a-viewer-that-falls-max_gop_bytes-behind-waits-for-a-key-alone-amends-the-2026-10-21-networking-entry) —
  Keys on demand: without `ServerOptions::replay_gop`, a joiner waits for
  the next key, the server keeps only what its viewers still need, and a
  viewer that falls `max_gop_bytes` behind waits for a key, alone; newest
  key wins still. `StreamServer::key_wanted()` counts both, and the codec
  examples' `CodecStream` codes an I-frame at its next front half when it
  rises. fuse_orbbec serves so; `serve_stream`, which cannot make a key,
  replays the GOP. *Amended 2026-10-26:* that stream is
  `sender::CodecStream`.
- [**2026-10-26**](#2026-10-26--a-senders-set-step-and-its-codec-stream-are-library-calls-in-recon_sender-after-networking-the-frames-halves-stay-internal-with-no-friend-amends-the-2026-10-25-two-batch-sender-and-keys-on-demand-entries) —
  A sender's set step and its codec stream move from the examples into
  `recon_sender`, a tier after `networking`: `SetSender`, `CodecStream`,
  `SentFrame` and `fuse_inputs`, printing nothing. The frame's halves are an
  internal class that `CodecStream` hands out only incomplete, so no
  `friend` is needed. The examples keep their reports, built from what the
  calls return. *Amended 2026-10-28:* the stream is a tier of its own,
  `codec_stream`, and the halves are its public calls; `coder()` is gone.
- [**2026-10-26**](https://github.com/taojin-6/volumetric_kit_grid/blob/main/DECISIONS.md#2026-10-26--meshexchanges-consumer-may-drop-a-mesh-it-took-a-third-outcome-rather-than-a-deadline-in-the-exchange-and-a-ring-slot-more-for-each-mesh-dropped-before-the-next-commit-amends-the-2026-10-08-meshexchange-entry) —
  `MeshExchange`'s commit may drop a mesh it took (`CommitDecision::kDrop`):
  it never becomes live, the next frame takes the newest mesh, and its slot
  goes with the generations below the next commit, so a ring needs a slot
  more for each mesh dropped before a commit. A third outcome, with no
  deadline or policy in the exchange. The ring's size counts one frame in
  flight as two.
- [**2026-10-26**](#2026-10-26--nvdecs-pictures-land-in-two-plane-images-videotoolboxs-form-cudas-one-copy-writes-them-a-reader-takes-each-over-from-outside-vulkan-and-nothing-hands-it-back-amends-the-2026-10-08-decoded-picture-and-2026-10-25-plane-image-entries) —
  NVDEC's pictures land in two plane images, an R8 luma and an R8G8 chroma,
  VideoToolbox's form: CUDA's one copy writes them, handed to it once in
  `GENERAL`; a reader takes each picture over from outside Vulkan in the
  layout it is in, and nothing hands it back. The images take the
  decoder's queue families; nvJPEG keeps its I420 buffer.
- [**2026-10-26**](https://github.com/taojin-6/volumetric_kit_grid/blob/main/DECISIONS.md#2026-10-26--a-compaction-records-from-a-const-map-and-the-encoder-takes-a-const-grid-the-maps-compaction-output-is-scratch-any-compaction-rewrites-amends-the-2026-09-27-encoder-and-decoder-entry) —
  `VoxelHashMap::record_compaction` is `const`, so a caller holding a grid
  read-only records the active set's compaction, or a frustum's, into its
  own batch. It moves no block, stamp, heap count, epoch or tick; the list,
  its count, its kind's serial and the frusta are the map's compaction
  output, `mutable`, and a held list of that kind goes stale as before.
  `Encoder::encode` and `record_front` take a const grid. One call on a map
  at a time, const or not, and no resize or frusta grow until the batch has
  resolved; broken, a resize refuses the batch and a frusta grow is
  undefined behaviour.
- [**2026-10-27**](#2026-10-27--a-sensors-depth-offset-lives-in-the-calibration-file-version-3-and-frame-prep-subtracts-it-the-calibration-its-one-source-amends-the-2026-10-12-calibration-file-entry) —
  A sensor's depth offset (`SensorCalibration::depth_offset_m`, + where it
  reads too far) lives in the calibration file, version 3, which the writer
  writes only for a corrected calibration; frame prep's depth pass
  subtracts it, 0 staying no return for either sign, before the
  within-colour mask. `SensorArray` sets each pass's offset from the
  calibration, its one source, and refuses one in `Options::prep`.
  *Amended the same day:* the colour cameras' exposure joins version 3.
- [**2026-10-27**](#2026-10-27--the-orbbec-driver-defaults-to-h265-at-3840x2160-and-25-fps-and-opens-each-colour-camera-at-the-calibrations-exposure-written-where-the-camera-differs-auto-white-balance-on-and-read-back-an-array-refuses-a-sensor-opened-at-another-amends-the-2026-09-28-h265-entry-and-the-depth-offset-entry-above) —
  The Orbbec driver defaults to H.265 at 3840x2160 and 25 fps, as core
  0.25.0's scenarios do; MJPEG stays for calibration captures and
  diagnostics. Every colour camera opens at the calibration's exposure
  (`ArrayCalibration::color_controls`, a version-3 field; empty is auto
  exposure), written at each open where the camera differs, auto white
  balance always on, checked before the first write and read back after the
  last; an array refuses a sensor opened at another.
- [**2026-10-28**](https://github.com/taojin-6/volumetric_kit_grid/blob/main/DECISIONS.md#2026-10-28--the-voxel-layer-is-camera-free-volume-owns-its-depth-camera-depthpinhole-and-the-one-line-of-unprojection-it-needs-tsdf-copies-its-camera-in-amends-the-2026-10-16-glsl-pinhole-entry) —
  The voxel layer is camera-free: `volume::DepthPinhole`, the fields and
  96 bytes of `DepthCameraParams`, is what depth allocation takes, and its
  kernel unprojects through a copy of the pinhole's one line; tsdf's
  `FrameInput` keeps `DepthCameraParams` and `to_depth_pinhole` copies it
  field by field. Allocation is main's, bit for bit.
- [**2026-10-28**](#2026-10-28--io-is-two-targets-images-on-the-family-cores-base-tier-alone-mesh-files-on-mesh-amends-the-2026-10-03-asset-io-entry) —
  `io` is two targets: `recon_image_io` (images, stb) links the family
  core's base tier alone; `recon_mesh_io` (PLY, the triangle containers,
  normalization, tinyply) links `mesh`, and `recon_mesh_io_assimp` replaces
  `recon_io_assimp`. Headers and namespace are unchanged; `recon_io` is
  gone.
- [**2026-10-28**](https://github.com/taojin-6/volumetric_kit_grid/blob/main/DECISIONS.md#2026-10-28--a-triangle-meshs-distance-field-is-a-tier-of-its-own-voxelize-on-volume-alone-amends-the-2026-09-27-mesh-to-tsdf-entry) —
  A triangle mesh's distance field is a tier of its own, `voxelize`
  (`recon_voxelize`), on `volume` alone: `MeshIntegrator` and its kernels
  leave `tsdf`, and the closest-point GLSL moves to `volume`.
- [**2026-10-28**](#2026-10-28--recon-takes-the-cores-glsl-export-sibling-floor-test-support-and-forward-declarations-and-keeps-no-copy-amends-the-2026-10-16-glsl-pinhole-entry-and-the-2026-10-08-test-policy-entry) —
  recon keeps no copy of what the core holds for every sibling:
  `recon_core` exports its GLSL with `vkc_export_glsl`
  (`VR_SHADER_INCLUDE_DIR` is gone), the umbrella carries
  `VKC_PACKAGE_VERSION` for a consumer's `vkc_require_sibling`, the headers
  include the core's `vulkan/fwd.hpp`, and the tests run on the core's test
  support. The core floor is 0.29.0.
- [**2026-10-28**](https://github.com/taojin-6/volumetric_kit_grid/blob/main/DECISIONS.md#2026-10-28--the-codec-stream-is-a-tier-of-its-own-codec_stream-on-codec-and-networking-and-its-frame-in-halves-is-public-amends-the-2026-10-26-sender-entry) —
  The codec stream is a tier of its own, `codec_stream`
  (`recon_codec_stream`), on `codec` and `networking`: `CodecStream`,
  `CodecOutputs` and `SentFrame` leave `recon_sender`, which keeps
  `SetSender` and `fuse_inputs`. The frame in halves is the stream's public
  calls (`record_front`, `front_ran`, `redo`, `finish_frame`), `SetSender`
  codes through them, and `coder()` is gone.
- [**2026-10-28**](https://github.com/taojin-6/volumetric_kit_grid/blob/main/DECISIONS.md#2026-10-28--the-voxel-grids-gpu-contract-has-one-home-volume-exports-grid_typesglsl-and-voxel_colorglsl-names-its-shared-attributes-and-keeps-the-hash-entries-internal-amends-the-2026-08-02-color-space-and-2026-10-13-block-list-entries) —
  The voxel grid's GPU contract has one home: `recon_volume` exports
  `grid_types.glsl` (the grid's structs, the in-block index, `vrTickAfter`)
  and `voxel_color.glsl` (the colour attribute's format), tsdf includes
  only those, `kTsdfAttribute` / `kWeightAttribute` / `kColorAttribute`
  name the shared attributes, and the hash entries' accessor is internal.
  Bit-identical to main.
- [**2026-10-28**](https://github.com/taojin-6/volumetric_kit_grid/blob/main/DECISIONS.md#2026-10-28--grid-is-a-package-of-its-own-the-voxel-grid-and-what-reads-it-under-grids-names-on-the-core-alone) —
  grid is a package of its own (`volumetric_kit_grid`, now its own
  repository, which recon fetches by commit): the voxel grid and what reads
  it moved there under grid's names, and recon builds on it (recon 0.1.0).

## Decision record

### 2026-06-21 — Single Vulkan path (MoltenVK on Apple), like gfx.

*Amended 2026-10-06 (the platforms entry, below):* Windows is not a target.

Compute is
Vulkan compute (GLSL → SPIR-V), one path across Linux / Android / macOS / iOS /
Windows — chosen over a Metal + CUDA split for cross-platform reach and a
single shader source. *Supersedes* the earlier "Metal-first / CUDA-later"
decision. Native Metal is not pursued (MoltenVK runs our GLSL); native CUDA is
now a planned NVIDIA accelerator, *not* the baseline — see 2026-07-04.

### 2026-06-21 — Trivial interop (same Vulkan API).

Because the renderer is
also Vulkan, recon and gfx share **one** `VkDevice` and pass `VkBuffer`/`VkImage`
directly (zero-copy needs a *single* shared device — see the 2026-07-04 interop
refinement below; UUID-matched *separate* devices would still need an import).
The cross-API external-memory
machinery (CUDA↔Vulkan UUID import, Metal-objects, the MoltenVK shared-event
export) is **not needed** for the recon→gfx path. This is the main reason for
the Vulkan choice. (Exception: when the native-CUDA accelerator [2026-07-04]
produces the geometry, the CUDA→gfx handoff *does* need the CUDA↔Vulkan
external-memory import — or an extra copy back into a `VkBuffer` — a cost borne
only on the CUDA path.)

### 2026-06-21 — Independent siblings; gfx untouched.

recon and gfx stay
standalone. Now that both are Vulkan, a shared `volumetric_kit_core` is
attractive and may be revisited — but for now each mirrors the Vulkan core
independently so neither repo's release is coupled to the other. We keep the
`Status`/`Result` shape close to gfx's so a later extraction stays cheap.

### 2026-06-21 — Vertical slice first.

v1 = `core` → `volume` → `tsdf` →
`mesh` → `interop`. Compression, sensor capture, tracking, codecs are later
tiers, added once the spine renders end-to-end.

### 2026-07-04 — Native CUDA accelerator, under the Vulkan baseline.

Vulkan
compute (GLSL → SPIR-V) stays the cross-platform baseline — it is the only path
that reaches every target (Linux, Android, macOS, iOS/iPadOS via MoltenVK);
CUDA reaches none of Android/Apple and only NVIDIA on Linux/Windows. On top of
that baseline we add a **native CUDA backend as an NVIDIA performance
accelerator**, layered under the Vulkan path and ported per-kernel where
profiling justifies it — *prove the gain on a hot kernel (e.g. TSDF integrate)
before porting the rest*, since these fusion kernels are bandwidth/atomic-bound
and the CUDA-vs-Vulkan gap is often small. Cost carried knowingly: two kernel
sources (GLSL + CUDA) kept numerically in lockstep, a backend seam in the
compute tiers, and the CUDA↔Vulkan interop exception noted above. *Amends* the
2026-06-21 single-Vulkan-path decision (CUDA: "possible" → "planned"); it does
**not** demote Vulkan.

### 2026-07-04 — GLM for host/device math (dropped the hand-rolled POD types).

*Amended 2026-10-06 (the test-only API entry, below):* `vr::normalize` and
`Vec3u8` went with `device_macros.hpp`; the `vr::` aliases and the `dot`,
`cross` and `length` re-exports stay.

The `vr::Vec3f/Vec3i/Vec4f/Mat4f` vocabulary aliases GLM instead of hand-rolled
structs. GLM gives tested math, byte-for-byte packed layouts for the Vulkan
buffer ABI (via scalar block layout, since `std430` 16-byte-aligns a `vec3` --
see 2026-07-05), `__host__ __device__` operators for the CUDA accelerator, and
parity with gfx (which also uses GLM) so the interop seam needs no vector
conversions. `core` therefore takes one header-only external
dependency (GLM); Eigen was rejected (its alignment + expression templates fight
a GPU-upload POD contract). The `vr::` names stay so the backing type is
swappable, and `vr::normalize` keeps a zero-length guard GLM lacks.

### 2026-07-04 — Zero-copy interop = one shared `VkDevice` + a create/adopt seam (refines "Trivial interop" above).

Being both-Vulkan removes the *cross-API*
machinery, but not all of it: a `VkBuffer` is valid only on the `VkDevice` that
created it, so zero-copy requires a **single shared `VkDevice`** (one process) —
*not* "UUID-matched compatible devices," which would still need external-memory
FD import (the same cost the native-CUDA→gfx handoff pays; see the CUDA
accelerator decision above). Each `core` therefore exposes `Device::create`
(owns) *and* `Device::adopt` (borrows, verifying against a published
`DeviceRequirements`); a neutral app-side bootstrap builds one device from the
union of both libraries' requirements and hands it to each, so both stay
standalone *and* compose. The live textured mesh is the target: variable
topology via `vkCmdDrawIndexedIndirect`, a ring of mesh/atlas slots, an
intra-device timeline-semaphore handoff (the MoltenVK external-semaphore caveat
does not apply on one device). Authoritative detail: DESIGN.md → "The interop
seam".

### 2026-07-05 — Compute core is explicit, not reflected; dispatch via `submit_single_time`.

The Vulkan compute foundation mirrors gfx's core (VMA
`Allocator`, RAII `Buffer`, `ShaderModule`, descriptor + `ComputePipeline`
wrappers, a `UniqueHandle` owner for pure-Vulkan handles) with two deliberate,
lean divergences. (1) **No SPIR-V reflection** — `ComputePipeline::create`
takes explicit descriptor-set layouts + push-constant ranges rather than
reflecting them from the shader, so the tier vendors only VMA (gfx pulls in
spirv-cross); the caller declares bindings matching the shader's
`layout(set=, binding=)`. (2) **No standalone `CommandBuffer` / `Fence` /
`CommandPool` types yet** — a one-shot `Device::submit_single_time(record_fn)`,
shared-queue-safe via `Device::queue_submit` + `submit_mutex`, is the dispatch
primitive; reusable command buffers and timeline-semaphore sync land when a
fusion tier actually batches dispatches. VMA is a private dependency (only
`allocator.cpp` and the one `VMA_IMPLEMENTATION` TU, `vma_impl.cpp`, include
`<vk_mem_alloc.h>`; `SYSTEM` include). Revisit reflection if the volume/tsdf
binding boilerplate grows painful.

### 2026-07-06 — Per-kernel resources are bundled (`core/compute_kernel.hpp`), still not reflected.

The volume tier had grown parallel `*_layout_` /
`*_pipeline_` / `*_set_` members (21 fields for 7 kernels) plus a hand-summed
descriptor-pool size — the boilerplate the 2026-07-05 decision flagged.
Resolved *without* SPIR-V reflection, in `core` because every compute tier
repeats the shape: a `ComputeKernel` struct bundles one kernel's
`{layout, pipeline, set}`; a `KernelSetBuilder` registers each kernel (its
SPIR-V + storage-buffer binding count + optional push range), auto-sizes the
shared pool from the exact descriptor total, and allocates every set (resolving
the pool chicken-and-egg the tier used to hand-sum); and a free
`dispatch(Device&, const ComputeKernel&, push, push_size, groups, max)` carries
the workgroup-limit guard + the COMPUTE→COMPUTE/HOST barrier. Seven kernels'
setup drops from ~80 lines to seven `kb.add(...)` calls. The **mechanism**
(kernel bundle / pool builder / dispatch) is generic and lives in `core`; the
**policy** — which buffers bind to which slot — stays in `volume`
(`write_persistent_bindings`). A reusable buffer-slot table is deliberately
*not* built yet: it waits until `tsdf` is a second consumer whose buffer shape
can be generalized against, rather than over-fitting one now. The caller still
declares each shader's binding count by hand, so this answers the 2026-07-05
"revisit reflection" note with a builder, not reflection — the tier still
vendors only VMA. (2026-07-08: the other per-tier duplicates the same decision
flagged — the dispatch group-count ceil-divide and the host-visible
storage-buffer create/upload helpers, copied verbatim into every tier — moved
to `core/compute_util.hpp` (`group_count` / `storage_buffer` /
`upload_storage_buffer`) once the `texture` tier would have been the fourth
copy; `texture` and `mesh` consume them now — `mesh` migrated on 2026-08-02
rather than fork its copy to carry the `MarchingCubesConfig` usage bits, which
is why `storage_buffer` grew its `extra_usage` parameter — and the `volume` /
`tsdf` copies migrated on 2026-08-04 — forced rather than chosen: `volume` and
`tsdf` had to include `compute_util.hpp` for the new range guard, and their
private `storage_buffer` copies then made every unqualified call **ambiguous**
by ADL against the `core` one, since `Allocator` names `recon` as an
associated namespace. Each tier keeps only its one-argument `group_count`
wrapper, which bakes in that tier's `local_size` and cannot collide with
`core`'s two-argument overload.)

### 2026-07-06 — Hybrid color renders through a gfx pipeline (amends "interop seam A needs zero gfx changes").

The renderer's shipped PBR path is
texture-only and ignores per-vertex `color`, so `volumetric_kit_gfx` grew a
**`HybridMeshPipeline`** (its long-anticipated "mesh pipeline"): per fragment
it samples the projective-texturing atlas where a triangle carries a valid
`uv0`, else uses the per-vertex `color` (the `uv0 = (-1,-1)` sentinel). A
general vertex-color/atlas pipeline is a broadly-useful renderer feature, so
the siblings stay independent — gfx gained a capability, not a dependency on
recon. Recon's `mesh::Vertex` accordingly carries `{position, normal, color,
uv0}`; marching cubes interpolates per-voxel `color` (the RGB the `tsdf` tier
fuses into the volume) to `Vertex::color` and leaves `uv0` at the sentinel, and
the projective-texturing pass (a later slice) fills real atlas UVs. The
interop-seam converter still maps recon's vertex to gfx's `{…, tangent, uv0,
color}` shape (synthesizing `tangent`). *Amends* the 2026-06-21 "zero gfx
changes" stance in the interop seam below. (*Superseded in part on 2026-08-02*:
`mesh::Vertex` now **is** gfx's layout, so there is no shape to map — see that
decision.)

<a id="depth-sampling-convention"></a>

### 2026-07-06 — Depth sampling is texture-centred (pixel centres at i+0.5), a deliberate ~½-pixel convention.

*Amended 2026-10-08 (below):* the condition below for revisiting it has
been met, and the sampler is integer-centred: no −0.5 shift, and the fallback
and the colour sample rounded.

The `tsdf` bilinear depth sampler
(`tsdf_integrate.comp::sample_depth`) shifts its 2×2 taps by −0.5 and takes the
containing pixel (`floor(u)`, `floor(v)`) as its nearest-neighbour fallback —
the GPU-texture convention, self-consistent across interpolation *and* fallback.
recon's projection (`u = fx·x/z + cx`) and the `volume` unprojection
(`hash_allocate_depth.comp`: `x = (u − cx)·d/fx` on integer pixels) are instead
integer-centred, so an on-axis voxel (`u = cx`) samples the 50/50 blend of the
two pixels straddling `cx` rather than the principal-point pixel. We keep the
offset knowingly: it faithfully ports the prior engine's `sampleDepthBilinear`
taps, and we further make the *fallback* texture-centred (`floor`) where the
prior engine inconsistently *rounded* to an integer centre while shifting its
taps. Revisit only if the depth intrinsics are ever calibrated integer-centred —
then drop the −0.5 shift and round the fallback so the sampler matches the
projection.

### 2026-07-07 — The viewer example opts into gfx behind `VR_BUILD_VIEWER` (amends "Independent siblings; gfx untouched").

The gfx-linked reconstruction
viewer examples (`examples/viewer/`: `fuse_render`, a headless colour-PNG
render, and `fuse_viewer`, a live window) pull `volumetric_kit_gfx` via
FetchContent + system GLFW — the **only** place the recon tree touches the
renderer, behind an **off-by-default** `VR_BUILD_VIEWER`. The library tiers and
the default build stay renderer-independent (verified: a default configure
fetches no gfx and builds the gfx-free `fuse_replica`). *Amended 2026-08-02:*
**one** CI leg does opt in — `.github/workflows/viewer.yml`, build-only, macOS,
scoped to `fuse_viewer` + `fuse_render`. Leaving it unbuilt was worse: this
directory is the single place recon's code meets gfx's API, and it silently
drifted out of build (`fuse_viewer.cpp` declared `Result<Mesh>` against
`extract_device`'s `Result<DeviceMesh>`, so `VR_BUILD_VIEWER=ON` failed at
`8a439fc` with nothing reporting it). Every *other* leg stays gfx-free, so the
renderer is still absent from everything that gates the library itself. This
amends the independence stance *for an opt-in example only* — recon's
**release** is still never coupled to gfx; a developer who wants the live
viewer opts in and pays the gfx fetch. The alternative (a standalone neutral
repo) was weighed and
set aside for discoverability: the example lives with the pipeline it demos.
*Amended 2026-08-02 (below):* the device-adoption proof that used to live in
that standalone repo now lives here too — `fuse_viewer` runs on **one shared
`VkDevice`**. The mesh **handoff** is still a host mesh (interop seam A); the
shared device is the precondition seam B needs, not seam B itself.
*Amended 2026-08-08 (below):* `fuse_viewer` now draws recon's buffers
directly — seam B — so the precondition has been collected on.
`fuse_render` deliberately stays two-device seam A.
*Amended 2026-10-22:* the leg is no longer build-only. It runs the viewers'
headless tests, which need no window or dataset: `recon_example_viewer_atlas`
and `recon_example_viewer_pass` on a headless device,
`recon_example_viewer_device`, and each viewer's config entry, on the core's
test policy with validation.

### 2026-07-07 — Projective texturing is a new `texture` tier; live single-camera first.

*Amended 2026-08-11 (below):* the verdict is taken per **vertex**, not per
triangle, and a negative `uv0` carries its coordinate as `-uv - 1` instead of
collapsing to the bare sentinel. The tier, its placement, the live
single-camera scope and the occlusion sampler below all stand; read "one thread
per triangle … keeps the triangle only when all three" and "the rest get the
`(-1,-1)` sentinel" as the shape this decision shipped with and the later entry
replaced.

*Amended 2026-10-08 (below):* the tsdf sampler is integer-centred too, so
both samplers follow one convention.

Filling the mesh `Vertex::uv0` (the atlas coordinate the
2026-07-06 hybrid-colour decision reserved) lands as a **new `texture` tier**
between `mesh` and `interop`, not a `mesh` add-on: it is a distinct pass over a
mesh + a posed camera that depends on `volume` (`DepthCameraParams` + the
`project_to_image` ABI) as well as `mesh`, mirroring the prior engine's
separate `projective_texturing` module. The first slice is **live,
single-camera** (chosen over a post-scan multi-keyframe atlas): one thread per
triangle projects its three vertices into the current frame's camera and keeps
the triangle only when all three are in front, in-frame, and **unoccluded** —
their projected depth agrees with the frame's depth map within an occlusion
threshold (default 0.02 m), the *line-of-sight* test. A kept triangle's
vertices get `uv0 = (pixel + 0.5)/size` (the prior engine's half-texel atlas
UV, no y-flip); the rest get the `(-1,-1)` sentinel, so gfx's
`HybridMeshPipeline` textures where a camera saw the surface and falls back to
the fused voxel colour elsewhere — and every call **overwrites** `uv0`, so a
triangle that leaves the view reverts to the fallback. The atlas is just the
camera image the caller binds (single-camera: no packing, no per-texel baking,
full sensor resolution), so **keyframe retention is unneeded** — the current
frame *is* the atlas. The prior engine's multi-camera score + winner-take-all
vertex atomic are **dropped here**: one camera needs no ranking, and the mesh
tier's independent triangles (no shared vertices) let each thread own its three
`uv0` writes. A later slice restores both for a post-scan multi-keyframe atlas
(best-of-N view, packed atlas, the Metal two-step 32-bit atomic once a dedup
slice shares vertices). Ported faithfully from
`implicit_world_reconstruction`'s `projective_texturing` (the Metal kernel
shape); `texture_common.glsl`'s projection mirrors `tsdf_common.glsl`'s
`project_pinhole`. The occlusion depth sampler shares the tsdf sampler's
*intent* — bilinear with hole renormalisation, plus a **depth-discontinuity
fallback** to the nearest tap (keyed to `occlusion_threshold`, since this tier
carries no `trunc_dist`) so a foreground vertex on a silhouette is not rejected
by a blended foreground/background depth — but is **integer-centred** (no −0.5
tap shift), matching this tier's integer-centred projection and half-texel
atlas UV rather than the tsdf sampler's texture-centred convention (the
2026-07-06 depth-sampling decision); the two are self-consistent within their
own tier. The kernel bounds its `indices → vertices` addressing with a
`num_vertices` push constant (a malformed/loaded mesh whose index is out of
range is skipped, not an out-of-bounds SSBO write), and the host rejects a
vertex/index/depth buffer past `maxStorageBufferRange` with a clean `Status`
(the mesh tier's arena guard).

### 2026-08-01 — Perf instrumentation starts in the viewer example, not a shared contract package.

The question left open when a private
`volumetric_kit_interop` repo was stood up to hold a shared `FrameMetrics`
(and judged "too thin") is settled the lean way: **the viewer example owns the
instrumentation**. `fuse_viewer` already links both siblings, so it fills gfx's
plain-data `gfx::FrameMetrics` directly from recon-side timings and draws it
with gfx's shipped `ui::ImGuiOverlay` + `ui::draw_metrics_panel` — no shared
package, no metrics contract in recon, and *no recon→gfx dependency* (the
coupling stays inside the already-opt-in `VR_BUILD_VIEWER` example, per the
2026-07-07 decision). The compute tiers stay profiler-free. `core` gains only
`Allocator::memory_stats()` (per-heap usage/budget over `vmaGetHeapBudgets`,
mirroring gfx's identically-named API) — a genuine library capability, not a
metrics framework. Promoting a shared contract waits for a **second** consumer
(a headless recon benchmark, or production telemetry), the same
wait-for-the-second-consumer rule the 2026-07-06 kernel-registry decision
applied. Two consequences carried knowingly: (1) recon's stage rows are
**wall-clock CPU**, not a CPU/GPU split — every recon dispatch goes through
`submit_single_time`, which blocks on a fence, so the span around a call is
that stage's true end-to-end cost (host record *plus* device execution) and is
reported with `has_gpu` false rather than as a fabricated device measurement;
the renderer's own rows *do* carry real GPU spans, since gfx's `Profiler`
writes timestamp queries and this GPU reports 64 `timestampValidBits` through
MoltenVK. Splitting recon's host from device time needs a query pool plumbed
through `submit_single_time` — the greppable follow-up. (2) VMA's `usage`
counts allocated *blocks*, not live sub-allocations, and VMA pools an emptied
block, so the reported figure is a **high-water mark that does not fall when a
buffer is freed** (`tests/core_memory_stats_test.cpp` pins exactly that); the
numbers are VMA estimates until `VK_EXT_memory_budget` is enabled, a
`TODO(core)` on the device seam.
*Amended the same day:* "the compute tiers stay profiler-free" holds for
*frameworks*, not for measurement — a tier may report its own phase breakdown
through an **explicit, opt-in out-param**, and `mesh::MarchingCubes::extract`
is the first (`ExtractTimings*`, defaulting to `nullptr`). The bar it must
clear: no global sink, no timing state retained between calls, no dependency
added, and nothing measured when the caller passes `nullptr`. That is a
by-value struct the caller owns, not a profiler in the tier — the aggregation,
display, and history stay in the consumer (the viewer's overlay). It earns its
place because the phases are *invisible from outside*: whole-volume meshing,
a neighbour-resolution pass, and a worst-case arena allocation all hide
inside one call, and picking between interop seam B and the incremental
block-mesh pool needs their split. (The neighbour pass was a host table when
this was written; the 2026-08-08 neighbour-probe decision moved it into the
kernel on the
strength of exactly this breakdown, and `neighbour_lut_ms` went with it.) Other tiers follow the same shape only
when they have the same problem.

### 2026-08-01 — iOS is a downstream concern; recon cross-compiles to it unchanged.

The iOS app shell lives in a **separate sibling repo**,
`volumetric_kit_ios` — an Xcode project, a bundle identifier, a signing team,
and a provisioning profile have no business in a library that also ships on
Linux and Windows. This is the *stronger* form of the 2026-07-07 viewer
decision (which admitted an opt-in gfx example into this tree for
discoverability): an app target is heavier than an example, so it goes fully
outboard and recon keeps the "independent siblings" rule intact. Verified: the
five library tiers build for `ios-arm64` (`platform 2`, minos 16.0) with **zero
source changes here**. The whole platform port is one toolchain file
downstream, because iOS ships **no ICD loader and no `libvulkan`** — MoltenVK's
static library *is* the Vulkan implementation, linked directly (so: no
validation layers on device), and seeding `Vulkan_LIBRARY` /
`Vulkan_INCLUDE_DIR` with MoltenVK's iOS xcframework is enough for CMake's
`FindVulkan` to build a `Vulkan::Vulkan` that satisfies every
`find_package(Vulkan)` in this repo. This is the 2026-06-21 single-Vulkan-path
decision paying off exactly as argued — MoltenVK carries recon to iOS with no
second code path. Proven on an iPad Pro M5 (Apple M5 GPU, MoltenVK 1.4.2 /
Vulkan 1.4.357) by an on-device four-stage smoke: device capabilities, a
compute dispatch, a **scalar-block-layout ABI round-trip** (`Vec3i` block
coords survive host → GLSL → host, so the 2026-07-05 ABI decision holds through
MoltenVK's SPIR-V → MSL translation — the one genuinely at-risk assumption),
and the full `allocate_from_depth` → TSDF integrate → marching-cubes spine. Two
findings that shape the later tiers: the GPU reports a **host-visible
`DEVICE_LOCAL` heap** (unified memory), so an ARKit `CVPixelBuffer` upload
copies straight into GPU-visible memory with no staging blit; and
`scalarBlockLayout` / `timelineSemaphore` / `dynamicRendering` are all present,
so the shared-`VkDevice` interop seam (2026-07-04) is reachable on iOS. One
packaging wrinkle stays downstream by design: Homebrew's glm 1.0.3 defines
`glm::glm` as a *macOS* dylib, and `core` links that canonical name
deliberately (older packagings lack `glm::glm-header-only`), so the iOS
consumer overrides glm with a header-only copy rather than this repo changing a
correct choice. The **`sensor` tier is where iOS actually lands in this tree**:
an ARKit `ICameraCapture` source feeding `sceneDepth` (256×192 float metres),
`capturedImage`, and `camera.transform` — note ARKit is +Y up / −Z forward
while recon projects +Z forward, so poses convert as
`T_world_cv = T_world_arkit · diag(1, −1, −1, 1)`, and ARKit's differing depth
and colour resolutions are already modelled by the separate
`DepthCameraParams` / `ColorCameraParams` the 2026-07-06 decision introduced.

### 2026-08-02 — The neutral shared-`VkDevice` bootstrap lands in the viewer example, and prefers two families over a shared queue.

*Amended 2026-10-12 (below):* gfx's `gfx_glfw` builds it; recon keeps no copy.

The 2026-07-04
interop decision's "neutral app-side bootstrap" is now real and in-tree:
`examples/viewer/shared_device.hpp` merges what each library publishes through
`Device::requirements` (API floor, extension union, core-feature union, the
`timelineSemaphore`/`scalarBlockLayout`/`dynamicRendering` chain, plus gfx's
opaque `feature_chain`), creates one instance + device + surface, and hands the
same handles to `vr::Device::adopt` and `vg::app::WindowedApp::adopt`. Neither
library owns it; the bootstrap outlives both wrappers and destroys the device
last. It lives beside the example it serves rather than in the standalone repo,
for the same discoverability reason as the 2026-07-07 viewer decision.
**Taking the merged API floor is load-bearing**: MoltenVK caps a physical
device's advertised `apiVersion` to whatever the instance requested, so an
instance at recon's 1.2 floor makes a 1.3-capable device fail gfx's check —
the gotcha below, hit for real here.
**The queue plan is a three-tier preference, and the order is the decision.**
(1) One family with ≥ 2 queues: concurrent submission *and* no queue-family
ownership transfer on a buffer recon writes and gfx reads — what the seam-B
plan assumes. (2) Two families, one queue each: still concurrent, but a shared
buffer will need `VK_SHARING_MODE_CONCURRENT` or an explicit release/acquire
pair. (3) One family, one queue behind a mutex: fusion and rendering
*serialize*. Measured on an Apple M5 Max (MoltenVK 1.4.2): the driver reports
**four graphics+compute+present families of exactly one queue each**, so tier 1
is **unreachable on Apple** and tier 2 is what actually runs
(`family 0 (gfx) + family 1 (recon)`). Preferring tier 2 over tier 3 is the
whole point — the pre-shared-device viewer got concurrency for free from two
separate `VkDevice`s, and dropping to one shared queue would have handed that
back to buy an ownership-transfer saving that **nothing yet collects**, since
the mesh handoff is still a host copy. The cost is booked, not avoided: when
seam B lands, its buffer is cross-family here and pays one of the two sharing
mechanisms. Tier 3 remains for hardware that offers no second compute family,
and says so on stdout when taken.
Both `adopt` calls are fed from what `vkCreateDevice` actually saw — the
enabled extension list, the merged `VkPhysicalDeviceFeatures`, and the three
feature booleans are *recorded* by the bootstrap, never restated by hand in the
payload builders: Vulkan cannot be asked what a device enabled, so those
declarations are the only thing the `adopt` verification has to work with, and
a hardcoded `true` would quietly turn the check into a no-op. Extensions and
features are checked against the device before creation so a shortfall names
itself instead of collapsing into "vkCreateDevice failed"; instance creation
retries without `VK_KHR_portability_enumeration` (the iOS direct-MoltenVK case,
mirroring `Instance::create`); and `--validation` is the embedder's own knob,
since `WindowedApp::adopt` ignores gfx's `enable_validation` once the embedder
owns the instance. Verified on room0: two families, 30/30 frames fused while
the window renders, and **zero validation errors** with the layer on.

### 2026-08-02 — The `sensor` tier is a *contract*, not a driver collection: a capture driver lives here only if this repo can build

and test** it.** The
tier's first slice is platform-neutral C++ — an `ICameraCapture` interface, a
`CapturedFrame` POD, and the coordinate/intrinsics conversion helpers — with
**no** driver implementations bundled in. (*Amended 2026-10-07 (below):* the
interface is `IRgbdSensor`, handing out `RgbdFrame`s; `ICameraCapture`,
`CapturedFrame` and the conventions test's `FakeCapture` are gone.) The
first real source, **ARKit,
lives in `volumetric_kit_ios`**, not here. The rule that places it: a driver
belongs in this repo when this repo's CI can compile and exercise it (a
cross-platform C++ SDK — an Orbbec driver would qualify); otherwise it belongs
with the platform that can. ARKit is iOS-only Objective-C, buildable only in
an iOS cross-compile recon's CI does not run and verifiable only on LiDAR
hardware, so hosting it here would mean an Objective-C++ TU that **no CI
compiles and no test exercises** — unbuilt code that rots silently. *Refines*
the provenance policy's "its sensor driver framework is rebuilt behind a clean
`ICameraCapture`": the **interface** is rebuilt here; drivers are placed by the
buildable-and-testable rule. This mirrors the sibling precedent exactly —
gfx's windowing tier is *window-system* agnostic (`Surface` adopts a raw
`VkSurfaceKHR`, `WindowedApp` takes a consumer-supplied `SurfaceFactory`),
which is precisely why gfx ported to iOS with zero source changes and why the
app, not the library, owns the `CAMetalLayer`. The `sensor` tier is
*capture-system* agnostic for the same reason and buys the same portability.
What deliberately **does** stay here is the conversion math, because it is the
part most likely to silently ruin a reconstruction and it needs no hardware to
test: (1) ARKit's camera is +Y **up** / −Z **forward** while recon projects +Z
forward, so poses convert as `T_world_cv = T_world_arkit · diag(1, −1, −1, 1)`
— get this wrong and the mesh is smeared or doubled, never an error; and (2)
ARKit's depth (`sceneDepth`, 256×192 float metres) and colour
(`capturedImage`, 1920×1440) come from **one physical camera**, so depth is
*registered* to colour — the two share a pose and differ only by an intrinsics
scale, which lands fusion in the registered case and avoids the
occlusion/partial-colouring caveats `TsdfIntegrator::integrate` documents for
an unregistered colour camera. (`sceneDepth.confidenceMap` gates low-confidence
pixels to depth 0, so the integrator skips them.) Both are pinned by plain
host-side unit tests before any Objective-C touches them. Revisit the
placement if ARKit capture ever gains a second consumer — a headless capture
tool, a visionOS target — since that is reuse across consumers rather than one
app's platform glue; moving it down stays cheap precisely because the contract
already lives here. (*2026-09-26:* the Orbbec driver that "would qualify" has
landed, as a target of its own beside the contract — see that entry.)
**Two consequences of the implementer being out of tree, both found by review
and fixed on the same PR.** (1) *The camera-parameter structs are `core`
vocabulary, and the contract depends on `core` alone.* (*2026-10-06:* and on
`camera`, which links no Vulkan; see that entry.) They had been placed by
which tier first needed one — `DepthCameraParams` in `volume` (block
allocation unprojects a depth frame), `ColorCameraParams` in `tsdf` (the first
tier that fuses colour) — which split a matched pair across two tiers and, far
worse, buried both inside `voxel_hash_map.hpp` / `tsdf_integrator.hpp`, which
reach `core/vulkan.hpp` through the VMA allocator and the pipeline wrappers.
Including the sensor contract therefore preprocessed **105 k lines with 1 412
Vulkan handle references**, all of it paid by an ARKit Objective-C++ TU that
only wants to fill in an intrinsics struct. Both now live in
**`core/camera_params.hpp`** as `vr::DepthCameraParams` / `vr::ColorCameraParams`
— a posed pinhole camera is pure math over `Mat4f`, the same kind of vocabulary
as `Status` and the GLM aliases, and four tiers take one. `recon_sensor`
consequently links **`recon_core` and nothing else**: 0 Vulkan references,
77 k lines, the remainder GLM, which `Mat4f` requires. They stay two *types*
(88 vs 96 bytes — colour carries no depth range), and each still pins its host
packing against the GLSL mirrors in `volume/`, `tsdf/` and `texture/`
shaders.
(2) *`Result<std::optional<T>>` has no natural spelling*, and a contract whose
only implementers are out of tree cannot afford that. `Result` converts
implicitly from exactly its value type, so `return std::nullopt;` and
`return frame;` are both two user-defined conversions and neither compiles;
`return {};` is ambiguous between the value and `Status` constructors; and
`return Status{};` compiles and then **aborts**, since an OK `Status` is not a
failure. `ICameraCapture` therefore ships `no_frame()` and `some_frame(f)` and
documents why. That trap was invisible until something implemented the
interface: the tier had shipped an interface **nothing in the repo
implemented**, so `tests/sensor_conventions_test.cpp` now carries a
`FakeCapture` driven through a base-class reference — it caught the
success-path half of this on its first compile.

### 2026-08-02 — One color-space rule, and a *named* working space: 8-bit color is encoded, float color is linear, converted once at the sensor boundary and encoded once at presentation.

*Amended 2026-10-28 (the grid-contract entry):* the GLSL the kernels decode
and encode with is no longer `core`'s. The voxel colour attribute's format,
the curve included, is `volume/shaders/voxel_color.glsl`, which `volume`
exports, with its host half `volume/voxel_color.hpp`; the PLY writer encodes
with a copy of its own. `core/color_space.hpp` stays the sensor boundary's.
The volume test holds the voxel format to the published curve and the PLY
test holds the PLY copy to the voxel format, so the grid tiers' tests take
no colour vocabulary from `core`; `recon_tsdf_voxel_color_space`, in tsdf,
which links both, holds the voxel format to `color_space.hpp`. The rule is
unchanged.

Color arrives from a sensor encoded for a
display and is then **averaged** four times — the TSDF running mean
(`tsdf_integrate.comp`), the marching-cubes edge lerp
(`marching_cubes_common.glsl`), texture filtering, and gfx's shading multiply
— before a `_SRGB` swapchain encodes it a second time. Averaging is linear and
a display encoding deliberately is not, so each is wrong by construction, and
wrong *quietly*: linear `0.0` and `1.0` fuse to `0.214` rather than `0.5` (a
plausibly darker surface, not a visible error), a few percent between similar
samples, worst across high-contrast pairs — so it concentrates on silhouettes
and depth discontinuities, exactly where a reconstruction gets inspected.
Surfaced by the iOS scanner, where only the double-encode is visible.
The rule decides the integrator, marching cubes, the atlas format, the vertex
color and the render targets with no judgment left over, and it is not
arbitrary: eight bits suffice for color *because* a display curve spends them
perceptually, so 8-bit storage stays encoded (naively linearizing trades a
blending error for banding in the darks); a float has the range to be linear,
so it is. It says *color* deliberately — a future 8-bit confidence or
occupancy channel is a number and carries no curve.
**"Linear" alone does not name a space, so one is named**: the working space
is **linear BT.709/D65** (the linear half of sRGB), its canonical encoded form
is that space through the *exact piecewise* sRGB transfer, full range, and
every 8-bit color below the sensor boundary is in that one form. Without this
the rule is under-determined — a driver can declare four transfers and three
primaries, so "encoded" would name twelve different things — and, worse,
`Primaries` would be a label nothing consumes: two sensors with different
primaries produce linear values in *different RGB bases*, and averaging those
is wrong the same way. The boundary conversion is therefore two steps, decode
the transfer then 3×3 into the working basis, both identity for a BT.709
source (so the ARKit path is a genuine no-op). Cost booked: BT.709 is the
narrowest declarable gamut, so a P3/BT.2020 source clips — widening the
working space means widening the storage with it, both named constants rather
than assumptions spread through the kernels. Presentation is the symmetric
half: the surface declares its space (`VkFormat` + `VkColorSpaceKHR`) exactly
as a driver does, so a P3/HDR display is reached by that declaration changing
and the presentation step gaining the primaries half — nothing else moves.
**Placement follows the 2026-08-02 camera-params precedent, not the sensor
tier.** `ColorEncoding` and the curve live in `core` (`core/color_space.hpp` +
a `core/shaders/color_common.glsl` mirror), because `sensor` branches off
`core` *beside* the fusion tiers — so `tsdf` and `mesh`, which do the decoding
in GLSL, cannot include from it — and because a curve four tiers need is
vocabulary. `sensor/color_conventions.hpp` keeps what is genuinely the
boundary's — `to_canonical`, which walks a frame. *Implementing this moved
the line*: the `ColorEncoding` type **and** `is_canonical` went to `core` too,
because `TsdfIntegrator::integrate` **refuses** a non-canonical colour frame
rather than fusing it through the wrong curve, and `tsdf` cannot include from
`sensor` — a property of the type belongs with the type. That refusal is what
makes "convert once at the sensor boundary" a contract rather than a hope.
The cross-tier GLSL include (the repo's first) is spelled like the C++ header
path so its provenance is visible at the include site; `vr_compile_shaders`
passes `src/` as the shader include root.
The declaration rides **beside** the camera
(a field on `sensor::CapturedFrame` / `tsdf::ColorFrame`), never *inside*
`ColorCameraParams`: that struct is uploaded verbatim under scalar block
layout, pinned at 88 bytes with GLSL mirrors in `tsdf/` and `texture/`, so a
field there would spend a cross-tier shader ABI change to carry what the
kernel needs at most as a push constant. No `Range` field — the contract
requires full-range packed R'G'B', so the YCbCr matrix and range expansion are
the driver's, applied before the contract; `Primaries` earns its place because
the working space gives it a consumer, and a field nothing consumes is a label
free to drift. ARKit declares `{Bt709, Bt709}` (its YCbCr→R'G'B' uses a BT.601
*matrix*, independent of the transfer function — conflating the two is its own
silent error), and `Transfer::Bt709` 8-bit content is **accepted as sRGB
rather than converted**: they differ by a couple of codes in the toe, which is
what display pipelines assume anyway (BT.1886), and converting would cost a
per-frame pass over 1920×1440 to buy nothing.
Two consequences worth stating. (1) **Presentation is three sites, not one** —
the swapchain (already `_SRGB`), `fuse_render`'s offscreen target (`_UNORM`
today; it renders straight to PNG, so linear values would simply come out
dark, and it is the CI-visible leg), and host export, where PLY `uchar` colors
are read as sRGB by every external viewer and must be encoded on write while
glTF `COLOR_0` is linear and passes through — the exporters cannot share one
path. (2) **`volumetric_kit_gfx` needs no change at all**: every format is
caller-set, the swapchain already prefers `_SRGB`, and `hybrid_mesh.frag`'s
`albedo *= ambient + diffuse` was never the bug, only its operands. Both the
host curve and its GLSL mirror must be the *exact piecewise* sRGB function,
not `pow(x, 2.2)` — hardware `_SRGB` sampling uses the exact curve and that
shader selects atlas-vs-vertex-color per triangle across one surface, so an
approximation shows as a seam exactly where texturing stops. Storage stays
`uint32` (convert in the shader) and is measured; the escalation trigger is
**not** banding but the running mean *latching* — re-quantized to 8 bits it
stops moving once the per-frame delta falls under half a code. A historical
**host arithmetic experiment**, with `max_weight = 5` and a 2 m observation
(weight 0.25), repeatedly unpacked the current and target codes to linear,
averaged them with weights 5 and 0.25, and packed the result back to sRGB: the
mean stops **~10 codes short, uniformly across the range**
(0→64 settles at 55, 0→255 at 245), and a gap narrower than ~10 codes never
moves the voxel at all — the residual is range-independent because the sRGB
curve turns a fixed fraction of the linear gap into a roughly fixed number of
codes. So fused colour accuracy is ceilinged at ~4%, a *convergence* limit
rather than a precision one, which is precisely why banding was the wrong
thing to watch. The 2026-10-10 test cleanup keeps this experiment here: it
models the arithmetic and does not exercise the production integrator, so it
cannot detect a change to fusion or storage. The color-space tests retain the
actual conversion and host/shader parity checks. Authoritative detail:
DESIGN.md → "Color space".

### 2026-08-03 — A buffer names the *families* that will read it, and the dispatch barrier widens only as far as its queue family may.

Seam-B
blockers (2) sharing and (3) visibility from the `MarchingCubesConfig`
decision above, both the same shape: the buffers were already correct for
recon reading its own results and **silently** wrong for anyone else.
(2) `BufferDesc` grows `queue_families` / `queue_family_count`, and
`create_buffer` picks the mode from their *distinct* count — two or more give
`CONCURRENT`, one or zero give `EXCLUSIVE`. Naming the families rather than
the mode is what lets a consumer pass its compute and render indices
**unconditionally**: Vulkan requires `CONCURRENT` to name at least two and
requires them unique, so a caller restating the mode itself would be
malformed on every platform where the two collapse to one family — the common
case off Apple, and where `CONCURRENT` would cost access performance for
nothing. Over `kMaxQueueFamilies` (4) distinct is an **error**, not a
truncation: a partial list names fewer families than actually touch the
buffer, which is the original bug in a different hat.
(3) The shared `dispatch()` barrier's destination scope adds
`VERTEX_INPUT`/`VERTEX_ATTRIBUTE_READ`/`INDEX_READ` and
`DRAW_INDIRECT`/`INDIRECT_COMMAND_READ`. Unconditional in the *stage* sense —
a destination mask costs an execution dependency the driver already owed the
host and compute cases, and a per-dispatch knob would have to be threaded
through every kernel this helper exists to keep uniform — but **not** in the
*capability* sense, which is the correction the first cut needed:
`VK_PIPELINE_STAGE_VERTEX_INPUT_BIT` requires `VK_QUEUE_GRAPHICS_BIT`, and
recon requires only *compute* of the family it is handed, so it can sit on a
compute-only one. That is not hypothetical — the bootstrap's two-families
plan matches on `queue_flags` alone (`VK_QUEUE_COMPUTE_BIT`), so on a discrete
GPU it picks the async-compute family, and naming `VERTEX_INPUT` there is
invalid usage in *every* dispatch in *every* tier. Invisible on Apple, where
all four MoltenVK families are graphics+compute — the same
correct-on-the-machine-that-ran-it hazard this decision exists to close, hit
while closing it. `Device` therefore records its family's `queueFlags` at
create/adopt (**read from the driver**, unlike the enabled extensions and
features, because capabilities *are* queryable and so nothing here is a
hand-written declaration that can drift), and `dispatch` consults it.
`DRAW_INDIRECT` needs graphics *or* compute, so it is always available. A
cross-queue handoff needs its semaphore regardless, and a semaphore's
signal/wait carries availability and visibility on its own — so nothing is
lost on a compute-only family.
**Verification is what makes both a seam rather than a hope**, and the first
cut got it wrong in an instructive way. Vulkan cannot be asked what a
`VkBuffer` was created with, so `Buffer` records its `sharing_mode()` beside
its `usage()` — with more at stake, since reading an `EXCLUSIVE` buffer from a
family that does not own it is *undefined* where a missing usage bit is at
least a validation diagnostic. The mode alone, not the family list: the mode
is what decides whether an ownership transfer is needed. That accessor is also
what gives the *test* teeth. The first cut asserted the sharing rule by
creating deliberately-malformed buffers and checking the call **succeeded**,
reasoning that a validation layer would catch the rest — but recon's debug
callback returns `VK_FALSE` (it must; aborting is a debugger's job),
`vmaCreateBuffer` still returns `VK_SUCCESS`, and nothing turned the
diagnostic into a failure, so those cases asserted **nothing on any machine**,
layer or no layer. The suite now pins three independent layers: the reduction
through the internal header (pure, fails everywhere), the mode read back off
`Buffer::sharing_mode`, and an Error-count delta through `set_log_handler`
around the device cases. Sabotaging the mode selection fails layer 2;
sabotaging the dedup fails layers 2 *and* 3 (the layer is installed on the dev
machine and in CI, and the error now reaches the exit code). The test also
reads the device's real family count instead of hardcoding `{0, 1}`, which on
a single-family driver — lavapipe, which both Linux CI legs run — is itself
the malformed shape it is trying to detect.
`distinct_queue_families` lives in a **non-installed** `src/` header
(`core/queue_family_set.hpp`, mirroring `vk_physical_device.hpp`), not in a
`detail` namespace in the public one: its location is the access control, so
the tier commits nothing to the exported ABI for a helper that exists to be
host-testable, and the test reaches it by include path. Still open at the time
of writing: seam-B blockers (1) lifetime and (4) indirect draw — both settled
by the two decisions below, which close the set.

### 2026-08-04 — A limit, a lifetime, or a staleness the caller cannot see is the library's to check, not to document.

A whole-repo review found fifteen
defects and thirteen were one shape: *correct for recon reading its own
results on the machine CI runs, silently wrong elsewhere* — the same family
the seam-B decisions above kept turning up, now swept across every tier. What
unifies the fixes is **where** the check went, so the rule is stated once
here rather than re-argued per site: when a caller cannot obtain the fact it
is required to honour, an obligation in prose is not a contract, and the
library asks the question itself.
**(1) `maxStorageBufferRange` is a real ceiling, and it is lowest exactly
where nothing tests it.** Vulkan guarantees only 2^27 (128 MiB) — what
Android-class drivers report — while MoltenVK and desktop drivers report far
more, so an over-large binding is invisible on every machine this repo runs
on and undefined on a stated target. `mesh` and `texture` guarded their
buffers; `volume` and `tsdf` did not, and their `VoxelBlockGrid` attribute
arrays are the **largest allocations in the repo** — 256 MiB *per attribute*
at every example's default grid, twice the floor before a single resize, and
bound `VK_WHOLE_SIZE` with `robustBufferAccess` enabled nowhere. The guard is
now `core`'s (`compute_util.hpp`: `max_storage_buffer_range` /
`check_storage_buffer_range`), which is what forced the `volume`/`tsdf`
helper migration noted above, and it is applied where the size is *chosen*
(attribute create + every grow, the depth/colour frame uploads) rather than
at each bind, since a consumer could only re-derive the same number.
**(2) A lifetime rule that cannot describe move-assignment is not a rule.**
`Allocator` documented "destroy Buffers before their Allocator", which a
reader satisfies by keeping the object alive — and `a = std::move(b)` then
ends the resource's life while the wrapper visibly lives on, so every
outstanding `Buffer` freed through a destroyed `VmaAllocator`. Made
structural instead: the pImpl is a `shared_ptr` and each `Buffer`'s deleter
holds a reference, so the ordering is unconstrained and the `@warning`
becomes a `@note`. `Allocator` stays move-only; the cost is one atomic per
buffer create/destroy, against a device allocation on the same path.
**(3) Staleness must be *askable* by whoever acts on it.** `DeviceMesh`
carried a `generation`, but only `MarchingCubes::download` could compare it —
so the *reading* path was guarded and the *writing* path,
`ProjectiveTexturer::texture`, was not, while its header stated the
obligation and handed the texturer no means to check it. Binding a superseded
view is a use-after-free when the intervening extract grew the arena
(`Buffer::operator=` runs `vmaDestroyBuffer` with no fence), and silently
wrong when it did not. `DeviceMesh` now carries a non-owning pointer to its
producer's live counter and answers `is_current()`; the texturer refuses.
Handle comparison cannot substitute — a grow-only arena reused in place makes
a superseded view name the very same `VkBuffer`. The pointer borrows the
producer's address as the handles already borrow its storage, so a view still
must not outlive or be held across a move of its producer.
**(4) A predicted quantity is clamped, never rejected.** The mesh arena's
range check tested `plan_capacity`'s *guess* — last call's density scaled by
this call's active set — and `tris_per_block_` is written only on success
while a scan's active set only grows, so a single over-estimate failed the
extractor **permanently**, quoting a limit the real surface fitted inside. It
clamps now, and the rejection moved onto the *measured* count the refit
protocol already produces, which is what the error message always claimed.
**(5) A validator must not be defeated by its own arithmetic.**
`VoxelGridParams::validate` compared `num_blocks` against a *narrow* product
that wrapped exactly as the `uint32` multiply in `VoxelHashMap::resize` had,
so both sides agreed on the wrapped value and the guard could never fire —
and cubed `block_size` unbounded, which is signed-overflow UB inside the
function whose job is rejecting bad input (2048³ wraps to exactly 0, so
`voxels_per_block == 0` passed and every block aliased pointer 0). Bounded
before the cube, widened for the product. `bucket_size == 1` is now rejected
outright: the last entry of each bucket is its chain anchor, so at one entry
per bucket *every* slot is an anchor and `allocate_in_overflow` — which skips
anchors — is a guaranteed no-op, failing the first colliding pair forever,
which a caller reads as capacity pressure and answers by growing until it is
out of memory.
**(6) A retry loop may only erase a failure it can actually resolve.**
`dispatch_with_retry` re-dispatches while the failure count drops, which is
right for allocation (the element is still unprocessed) and wrong for
deletion: the heap append runs *after* the table entry is cleared, so a
coord whose block could not be returned to the free heap is absent next round
and counts nothing — `remove()` reported a clean delete over a permanently
leaked block index. The tally gains a **`kFailTerminal` slot** the host
accumulates across rounds instead of reading from the last one, and the slot
layout moved to `hash_common.glsl` because it is a host/device ABI two
kernels share, not one header's private business.
**(7) The aggregate the API returned could not answer the question the caller
had to ask.** Every allocate kernel already split its failures by reason and
nothing exposed them, so `fuse_replica` read a residue of lost bucket-lock
races — transient contention, worst for depth, whose adjacent pixels dilate
into the same block — as map overflow and **doubled the volume**: at the
example defaults 768 MiB → 1536 MiB per attribute, with both old and new
buffers live across the swap, for a table that was never full. `AllocFailures`
is an opt-in `nullptr`-defaulted out-param (the `ExtractTimings` shape the
2026-08-01 amendment blessed, and the same bar: no global sink, no retained
state, nothing measured when null) carrying `capacity_limited()` — the
predicate `resize` actually answers. All three examples consult it.
**(8) The tier that owns both halves does the clearing.** The delete kernel
justified not clearing a freed block's per-voxel data with "there is no voxel
data to clear yet", false since the `tsdf` tier landed. It genuinely cannot:
under the SoA decision the block index does not know what attributes a
consumer declared. But the heap is **LIFO**, so the next allocation re-draws
the freed index onto the identical range and the removed surface's
`tsdf`/`weight` resurrect at full fused weight — bypassing the integrator's
`color_attr == 0` first-observation gate, which exists to stop exactly a
wrong first colour. So `VoxelBlockGrid` grew `remove` / `clear`, which zero
the ranges first (resolved from the compacted active set — no new kernel, and
the buffers are host-mapped); `map().remove()` remains the raw path and says
so. Its sibling hazard, `map().resize()` desyncing the attribute arrays, was
documented as safe because "a downstream bounds check stays sound" via
`AttributeView::element_count` — which **nothing read**. `attribute()` is the
funnel every consumer passes through, so it now refuses an array that no
longer covers the live grid, and the promise is true.
**Verification followed the repo's own standard, and it was measured.** Each
fix that could be pinned host-side or on MoltenVK is, and each new assertion
was confirmed to **fail against the pre-fix code** by mutation: reverting the
`validate` widening, dropping the attribute clearing, dropping the
`attribute()` desync check, and dropping the texturer's currency check each
fail a named line. `fuse_replica` on Replica room0 produces a mesh identical
to `main` triangle-for-triangle (832 518 vertices / 277 506 triangles over
120 frames), which is the point: none of this changes fusion numerics on the
hardware that already worked — it changes what happens on the hardware that
did not. Two example-level fixes carry no test because they are in the
gfx-linked examples, which only the build-only viewer CI leg covers:
`fuse_render` / `fuse_viewer` discriminating a decode failure from
end-of-sequence (both swallowed it, and the CI-visible render leg wrote a
partial-room PNG and exited 0), and `StageTimes::total_ms` skipping the
`"  .."`-prefixed breakdown rows, which re-state time already counted by the
stage they decompose and so reported every extract twice. Both viewers also
gained `--trunc`, defaulted to `4 * voxel` as `fuse_replica` does, since a
band hardcoded in metres made `--voxel` silently change the band's width *in
voxels* — 1.6 voxels at `--voxel 0.05`, 16 at `0.005` — and made the same
flag value reconstruct differently across the examples the A/B exists for.
**Second pass, the nine the review cut for its report cap.** Same rule, and
three were the same *shape* as the thirteen above. (a) `pack_linear_to_srgb`
clamped with `e < 0 ? 0 : (e > 1 ? 1 : e)`, which **NaN slips through** —
it compares false to both bounds — into a float-to-unsigned conversion that
is undefined for a value it cannot represent, in a header this library
*installs*. Rewritten as "keep only what is provably in range"; confirmed by
reverting it under the UBSan leg, which reports `nan is outside the range of
representable values of type 'unsigned int'` at that exact line. (b) The
frustum cull's block AABB was the occupied volume shifted **+½ voxel** on
every axis (it took the origin voxel's *centre* as `bmin` and added a whole
edge, where node-centred voxels extend a half-voxel past each end). The
~10% widening does not absorb it: `make_frustum_planes` scales `fx`/`fy`, so
near and far are exact, and the shift lands on them asymmetrically —
the near test compares `bmax` and is merely loosened, while the far test
compares `bmin` and is *tightened*, culling blocks whose voxels are within
`far_z` while their surface is in view. (c) An overflow-chain insert that
failed because the **table** was full reported `kFailHeap` — a reason that is
*provably impossible* on the rehash path, which presets each pointer and
never touches the heap. Now `kFailTable`, and the two allocate helpers return
a reason rather than a bool, which is what makes the distinction expressible;
`AllocFailures` grows the field and `capacity_limited()` covers it.
The rest were smaller. `ExtractTimings::arena_bytes` reported 0 on the
empty-extract path — "the extractor released its memory", the opposite of
true, on the instrument the ring runaway was diagnosed with.
`VoxelBlockGrid::resize` now validates the grown grid before allocating,
though honestly: it *overlaps* the `maxStorageBufferRange` guard added above,
which fires first on any real device, and is kept only because it states the
grow is illegal arithmetically rather than too large for this hardware. Three
GLSL headers still documented the superseded 48-byte vertex (`color@24` —
the offset `tangent` has occupied since the 2026-08-02 layout decision), a
stale comment that read as current because `uv0@40` and the number 48 both
still appear in the real layout. `fuse_render`'s `--follow` computed its FOV
as `2*atan(360/600)`, wrong twice — 340 is the half-height of a 680-tall
sensor, and the hardcoded `fy` ignored `--cam-params` — so the sensor's vfov
is now carried out of `fuse()` from its own intrinsics, split about the
principal point. `--fuse-per-tick` was parsed, clamped and documented in the
usage string while **nothing read it**; removed rather than given an invented
meaning.
**Three assertions that could not fail** are the last group, and they are the
reason this pass exists at all — a test that cannot fail is worse than no
test, because it is counted. `marching_cubes_config_test` asserted that a
stale `release_through(0)` still *refused* an extract, which held with or
without the monotonic guard because the ring had no free slot either way; it
now releases the whole ring first, so a dropped `std::max` turns the mark to
0 and stalls the ring — a failure. `compute_raii_test` seeded the
move-assignment's destination from the source (`c.set = b.set`), so
`c.set.handle() == raw` was true *before* the assignment ran; the destination
is now a second real pool + set with its own handle to lose. Both were
confirmed to fail against the mutation they exist to catch.
`sensor_conventions_test`'s was vacuous for a third reason — `dst` still held
the previous call's output — and is now zeroed and checked in full, but it is
recorded here that it *still* cannot discriminate the `is_canonical` mutation
the review named: for 8-bit input the decode/encode round trip is the
identity, which is precisely why `Transfer::Bt709` is accepted as sRGB rather
than converted. That predicate is pinned directly where it lives, in
`core_color_space_test`, and the mutation fails there.

### 2026-08-08 — `fuse_viewer` draws recon's buffers: interop seam B, end to end, and the release mark must be published *before* the mesh is taken.

*Amended 2026-10-08 (the streamed-atlas entry, below):* the atlas is gfx's
`StreamedAtlas`, and what a frame's copy reads is held until the frame loop's
timeline reaches the frame, so the per-slot atlas and colour-buffer holds are
gone.
*Amended 2026-10-08 (the exchange entry, below):* the protocol is
`mesh::MeshExchange`'s, and the mark also stops below a parked take, which
obligation (3) requires and the fallback below broke before anything was
committed. The panel snapshot is no longer read in the take's section:
its extract rows come in the mesh's payload (see the exchange entry), and
`AtlasVersion` is `viewer_atlas.hpp`'s `Atlas`.

*Amends* the 2026-07-07 viewer decision ("the mesh **handoff** is still a host
mesh (interop seam A)") and closes the interop seam's "what is left is a
consumer that draws it". recon's marching-cubes kernel writes the vertex
arena, index run and `VkDrawIndexedIndirectCommand`; gfx binds those very
handles as a `pipelines::LiveMesh` and issues `vkCmdDrawIndexedIndirect`, so
the index count is read GPU-side out of the command recon wrote and never
crosses the CPU either. What goes away is `download` → `to_gfx_mesh` →
`upload_mesh` — a full readback plus a full re-upload of geometry that never
conceptually left the device, every remesh.
**`fuse_viewer` first, not `fuse_render`, and that order is the decision.**
`fuse_render` builds *two* devices on purpose, and a `VkBuffer` is valid only
on its creating device, so zero-copy is structurally impossible there without
first adopting the shared-device bootstrap — and migrating it would delete the
only coverage the two-device path has. `fuse_viewer` already had the shared
device, the device extract and the device texturing; the host round trip was
the one thing left.
**The verification half is the point, as everywhere else on this seam.** The
consumer checks `DeviceMesh::vertex_usage` / `index_usage` / `indirect_usage`
**and `sharing_mode`** before binding, rather than trusting that the flags it
published in `MarchingCubesConfig` reached the producer: Vulkan cannot be
asked what a `VkBuffer` was created with, and the sharing mode is the term
that actually varies — this machine runs the two-families plan
(`family 0 (gfx) + family 1 (recon)`), where reading an `EXCLUSIVE` buffer
from a non-owning family is undefined with nothing to report it, and on Apple
undefined in the way that appears to work. A failure **latches**: the usage
bits and families come from two constants in this file, so a mesh that is
unusable once is unusable every time, and collecting the ones that follow
would only walk the ring to exhaustion one undrawable generation at a time.
No barrier is recorded here, and **the reason is the fence, not the barrier
scope** — a distinction the first cut got wrong. recon's extract blocks on its
own fence inside `submit_single_time` before anything is published (an
availability operation over every device write it made) and the renderer's
later `vkQueueSubmit` makes those writes visible to the draw; that chain is
what orders the two. recon's shared `dispatch()` barrier *does* also name
`DRAW_INDIRECT`, but it names `VERTEX_INPUT` only where its queue family
advertises graphics — Vulkan forbids that stage on a compute-only family, and
the bootstrap matches recon's family on `VK_QUEUE_COMPUTE_BIT` alone, so off
Apple recon can sit on exactly such a family (the 2026-08-03 decision says so
and this seam must not restate it as unconditional).
**The ordering fault this turned up is the finding worth keeping.** The
renderer must publish its release mark **in the same critical section as the
take, and compute it first**. Taking is what frees the fuse thread to extract
again, and the fuse thread reads the mark in the same breath as it tests for
an uncollected mesh — so a take published *ahead* of the mark for the same
frame lets it run on a mark one iteration stale, ask for a slot beyond the
ring's depth, and be refused. Measured, not reasoned: **25 refused extracts**
on a preloaded 200-frame room0 run, against **zero** after the fix. The
refusal count is the discriminator and the publish rate is not — under the
fault ~40 of those 200 frames published a mesh, which is indistinguishable
from the *healthy* rate the uncollected-mesh gate produces (80 of 400,
below), so anyone re-measuring a regression has to read the refusals. It is
invisible whenever fusion is *slower* than the render loop, which is every
streaming run — it took `--preload` at `-O3` to surface it at all. The mark
itself is `oldest_in_flight − 1` over a `frame_generations[frames_in_flight]`
array whose entry for the current slot is cleared first (`begin_frame`
fence-waited it), floored by what this frame is about to draw so the fallback
cannot retire a live generation. The panel snapshot is read in that same
section, for a smaller version of the same reason: `begin_frame`'s per-slot
fence wait can span a whole frame and the fuse thread routinely publishes
inside it, so a snapshot taken before it printed one generation's vertex
counts above another's arena and dispatch count.
**Consumer obligations that have no analogue in the host path**, and they are
all consequences of the mark being a single monotone high-water value. (1)
*The producer must not publish over an uncollected mesh* — it still holds a
ring slot, and the producer cannot free it (`release_through` is the
consumer's mark, so releasing that generation retires every older one with it,
including ones in-flight frames are drawing). It skips the extract instead,
whose result would have been discarded anyway; at 400 frames that is **~80–90
remeshes instead of 400** (87 on the run below), so it is also a large saving.
(2) *An empty extract
must still be published, and must still be **committed**.* It claims and
stamps a slot like any other, so a mesh that never reaches a consumer is a
slot nothing can ever release — `slot_count` of those and every later extract
is refused, permanently. On the consumer side that means committing it as the
live view (which is what parks its generation for release) even though it
draws nothing, and *not* running it through the bindable check: recon names an
empty extract's buffers as it found them, so a slot that was never sized
carries **null handles** beside `empty()` being true, and a check that folds
`valid()` in with the usage bits reads recon's own legal path as a
configuration fault. Since that fault latches (below), doing so stopped the
viewer drawing *and* extracting for good — reachable from `--max-frames 0`, a
frame-0 load failure, or a first depth frame with nothing in range. (3) *A
taken generation that is not committed must be bounded.* The mark cannot skip
one, so a generation the consumer takes and then drops keeps its slot until
some *newer committed* generation sweeps past it — and the commit that would
produce one is exactly what failed. Two dropped takes fill a three-slot ring
and every later extract is refused, permanently, with no way back. So a failed
atlas upload **parks** its pair and retries it on the next frame instead of
dropping it, and the take is gated on nothing being parked, which bounds the
uncommitted set at one. (4) *The release is applied by the **fuse** thread*,
at the top of its next remesh: `MarchingCubes::release_through` is not atomic
and its header makes serializing it against the extracting thread the caller's
job, so the render thread records the mark and the fuse thread applies it —
which also puts the release *before* the extract it makes room for.
**The mesh and its atlas stay one value**, which is what this example has that
the iOS scanner does not (its `FusionConfig::texture` is off precisely for
want of the atlas ring). `uv0` is a coordinate into the image of the camera
that textured it and every `texture()` call rewrites every vertex's `uv0`
against the *current* frame, so they are published, taken and committed
together; a failed atlas upload commits neither and the previous coherent pair
keeps drawing while the new one is retried (obligation 3). The `AtlasVersion`
`shared_ptr` ring is unchanged — it owns gfx-side storage and is correctly
ref-counted per frame slot; only the *mesh* ring moved to generation tracking.
**The final full-volume extract is not gated on collection.** It supersedes
whatever the last in-loop remesh published, and recon refuses cleanly if the
ring really is full, so skipping it on an uncollected mesh only threw away the
complete surface — silently, and on the very path that produces one: a
minimized window makes `begin_frame` return no frame, so the render loop never
reaches its take, the bounded wait times out, and the run ends replaying a
partial mesh beside "fused 400/400". `quit` is re-checked *after* that wait
too; the guard and the extract used to be adjacent statements, and a window
closed inside the (up to one second) wait fell straight through into a full
marching-cubes pass the guard exists to skip.
**Cost, booked rather than waved at.** `slot_count` is `frames_in_flight + 1`
(3), and each slot is a full vertex arena that never shrinks: **~205–220 MiB
resident across the ring** on 400-frame room0 (it varies run to run, since
each slot is fitted independently to whatever surface it was handed), against
~70 MiB for the single arena a recon-only consumer keeps. Against that it
deletes the `GpuMesh`
(~67 MB device), two host mesh copies (~67 MB each — the `rmesh::Mesh` and the
`vg::assets::Mesh`), and the per-remesh readback *and* upload.
**The residency class changes with it, and that is the part that does not
travel.** The `GpuMesh` this replaces was `DeviceLocal`; recon's arena and
index run are host-visible mapped memory, because `core::storage_buffer`
allocates every recon buffer that way. So the vertex-input stage now fetches
the whole mesh out of system RAM on every *presented* frame — ~64 MiB at these
counts — where the old path paid ~67 MB per *remesh* into device-local
memory. (`share_vertices` would cut that ~4x, and this example deliberately
does not take it: it runs the `texture` tier, which the flag is incompatible
with — see the 2026-08-08 decision.) On Apple's unified
memory, which is the only hardware this was measured on, that is free and the
seam is a clear win; on a discrete GPU it would invert. Booked as a greppable
`TODO(mesh)` beside the host-visible indirect command's, waiting on the same
discrete-GPU consumer to measure a device-local arena plus staging.
Verified on room0: 400/400 frames fused, **991 167 vertices / 330 389
triangles** — identical to the figure this file records for the host path —
rendering a coherent, projectively textured room at **~100–120 fps** (the
panel's figure moves with what is on screen; 96.5, ~106 and 102–119 across
three runs), with zero validation errors, zero refused extracts and one
dispatch per extract.

### 2026-08-09 — Timings are `core` vocabulary and the device half is measured, not inferred; counters stay in the tier that means them.

*Promotes* the 2026-08-01 decision's parked contract, on the trigger that
decision named. It held that the viewer example would own the instrumentation
and that a shared contract waits for a **second consumer**. There are now three
— `fuse_viewer`, the iOS scanner, and `fuse_replica --device-extract` — and the
cost of waiting had become visible rather than theoretical: `fuse_viewer` grew
`StageTimes`/`StageScope` in `examples/viewer/stage_metrics.hpp`, and the
scanner solved the same problem again and worse, as a 2048-byte `snprintf` into
a `UILabel`. Two implementations of one idea, neither reusable, and the platform
with the hardest problems got the weaker one.

**The split is timings uniform, counters local**, which is the same
mechanism-in-`core` / policy-in-the-tier line the 2026-07-06 kernel-bundle
decision drew. `vr::StageMetrics` carries `{name, cpu_ms, gpu_ms, has_gpu}`
rows, accumulating by name, with `kBreakdownPrefix` marking a row as a
decomposition of the one above it so a total does not count that stage twice
(not hypothetical: it reported every extract twice in the viewer before the skip
existed). `active_blocks`, `dispatches`, `triangle_capacity`, `AllocFailures`,
`load_factor` stay exactly where they are — a millisecond does not care which
tier produced it and those counters mean nothing outside theirs.

**It is recon's own type rather than the renderer's, and that is the fifth
application of one idiom.** `gfx::FrameMetrics::Section` is structurally
identical and nothing about it is renderer-specific — but recon cannot include
from a sibling it does not depend on, and a shared package for four fields was
already weighed and judged too thin. So each side declares in its own vocabulary
and the neutral app that knows both converts in a loop, exactly as
`DeviceRequirements` does at the device seam and
`MarchingCubesConfig::extra_vertex_usage` at the buffer seam.
`examples/viewer/stage_metrics.hpp` survives as *only* that mapping, which is
what stops the promotion leaving two copies behind.

**`vr::GpuTimer` is the half that was actually missing.** Every recon timing
before it was wall clock around `Device::submit_single_time`, which blocks on a
fence — so host record, submit, the stall and device execution collapsed into
one number, and "the kernel is slow" could not be told from "we are waiting".
That is not an abstract gap: the whole current optimisation roadmap —
incremental extraction, the vertex-layout narrowing, the `max_buckets`
decision — rests on numbers that cannot make the distinction, including a
`meshing` stage measured at 373 ms on an M5 iPad Pro.

**recon resolves more cheaply than a renderer can, and the blocking design is
why.** A render loop runs ahead of the GPU, so gfx's profiler buffers per
in-flight slot and publishes a snapshot that lags. Every recon dispatch is
already fence-blocked, so the timestamps are readable the instant the submit
returns: no ring, no deferred publish, no lag. The `submit_single_time(record,
GpuTimer*, label)` overload exists for exactly that reason rather than leaving a
caller to bracket the work — reading a timestamp before its submit completes
returns nothing useful and nothing at the call site makes that visible, so it is
a staleness the library sequences (the 2026-08-04 rule). It resolves **only** on
the path where the fence signalled; on the wait-failure path the submit may
still be pending, and the span stays unresolved rather than publishing a zero
that reads as a fast dispatch.

**Availability is not an error.** A queue family may report
`timestampValidBits == 0`, and the two-family bootstrap can hand recon a
compute-only family on a discrete GPU. `create` then succeeds with `available()`
false: `begin` returns `kNoSpan`, `end` does nothing, `report_into` contributes
no rows. A caller writes one code path and gets host timings. Failing instead
would let an optional diagnostic break a scan.

**Measured, and the number is the point.** A 128 MB device-local
`vkCmdFillBuffer` through the timed overload: **0.817 ms of device time inside a
14.256 ms blocking submit — 5.7%.** That is one cold measurement and the fixed
cost includes first-use residency, so it is not a claim about the steady state;
what it establishes is that the two quantities are nothing like each other,
which is the entire premise. The test asserts the inequality rather than a
magnitude — `gpu_ms < wall_ms` is what catches a span that is the wall clock
relabelled or resolved against the wrong period, and a fill sized so the work
cannot round to zero is what keeps `gpu_ms > 0` from passing vacuously. The
host half is pinned separately and runs on the GPU-less CI legs: accumulation,
content-matching (two pointers, equal strings, one row — literals are not
guaranteed to be pooled and across TUs routinely are not), the breakdown skip
(dropping it reports 30 where 20 is right), and the wrap case where masking the
endpoints before subtracting reports several thousand years.

**Review: the window needed an owner, and `report_into` became it.** The first
cut had `GpuTimer::spans_` as a host-side index into device query state with
nothing keeping the two in step, and five separate findings were that one gap
seen from different sides. `submit_single_time` sequenced begin/end/resolve but
never ended a *window*, so a timer created once — the shape the class's own
example shows — silently stopped timing after `max_spans` (measured: 40 submits
left `count() == 32`, every call returning OK) and every publish re-emitted the
whole window, which `StageMetrics` accumulates by name, so frame N reported
frames 1..N summed under this frame's label (measured: one row at the sum of 32
spans). The fix is that **publishing ends the window**: a caller creates one
timer, submits as often as it likes, and reports per frame, with `reset` left
for discarding instead of publishing. `max_spans` is then a per-frame bound
rather than a lifetime one, and exhausting it warns through the log handler
instead of going quiet. Verified as the realistic loop: 40 frames × 2 submits
through one timer now publish two measured rows every frame, with frame 39's
device total at 0.95× frame 0's rather than 40× it.

The same gap had two device-side edges. A span is recorded by commands *inside*
a command buffer, so a buffer abandoned before submission left its query pair
never reset and never written — undefined to read
(VUID-vkGetQueryPoolResults-None-09401), and worse once the pool had been used,
since the pair still holds an earlier window's value with its availability bit
set and resolves as a plausible duration under the abandoned label. Every early
return in `submit_single_time` now drops the span. And `resolve` no longer fails
the dispatch: `VR_TRY` on a query readback turned an optional profiler into
something that could fail compute work that had already completed, on the one
primitive every tier goes through — the exact outcome this entry's availability
rule exists to prevent. It logs and leaves the span unmeasured, which
`report_into` already skips.

Three more, each small and each a rule this repo had already written down. The
labels are now **borrowed on `StageRow::name`'s terms** rather than copied into
the timer: copies made the published rows point into a `std::string` the timer
would free, which the one consumer that exists reads from another thread, and
one lifetime rule across the whole vocabulary is simpler than two. `create`
bounds `max_spans` above as well as below, because `max_spans * 2` is the pool's
`uint32_t` query count and `0x80000000` built a **zero**-query pool that `begin`
believed had room for two billion spans. And the moves are hand-written per the
RAII rules — the defaulted pair left `device_` live in the source, so a
moved-from timer reported `valid() == true`, a failure that hides especially
well here because a hollow timer is byte-indistinguishable from the supported
`timestampValidBits == 0` degradation and reports itself as a missing GPU
capability. `tests/compute_raii_test.cpp` gained the three mandated move tests;
the first two fail on the pre-review code.

On the host side `StageMetrics` grew `merge`, because the viewer's remesh→fuse
fold was written as `add_cpu(row.name, row.cpu_ms)` and dropped `gpu_ms` and
`has_gpu` at the only place rows are combined — silently, since an absent
`has_gpu` is indistinguishable from a genuinely host-only stage. Carrying both
halves belongs on the vocabulary type rather than in each consumer.
`OptionalStageScope` folded back into `StageScope` as a second constructor,
which deletes ~35 duplicated lines and makes that class's null check
load-bearing instead of dead code beside a copy of itself.

Scope stopped at `core` initially, because the mesh tier was held by in-flight
incremental-extraction work.

**Extended the same day to `volume`, `tsdf` and `texture`**, and the plumbing
point is that it went through `dispatch()` rather than through each tier. Every
compute tier already routes there for the workgroup guard and the barrier, so a
device span costs a tier one argument and no submit path of its own; each holds
one `GpuTimer`, created in its `create()` because a query pool of a few
timestamps costs nothing and a diagnostic that can fail on first use is worse.
The host scope opens **before** the validity check, so a refused call still
costs its row — a stage silent on failure reads on an overlay as a stage that
did not run, which is the reading a frozen pipeline most needs not to give. In
`volume` the timer threads through the retry loop, so a contended frame's rounds
accumulate under one label: that is the honest total, and reporting only the
last would hide exactly the cost that makes contention worth seeing.

**That one argument is `GpuStageScope`, and it is the review's main fix.** The
first cut hand-copied a three-part idiom at four sites — open a `StageScope`,
pass `metrics != nullptr ? &gpu_timer_ : nullptr` to `dispatch`, then call
`report_into` as a plain statement afterwards — and the third part is skipped by
*every early return between them*. `volume`'s retry loop makes that concrete: a
round that fails returns through `VR_ASSIGN` with the successful rounds' spans
still in a timer that now outlives the call, so the next frame publishes a
failed frame's device time under its own label, and after `max_spans` such
failures `begin` refuses every span and that tier's GPU column freezes for the
map's lifetime. A destructor cannot be skipped by a return, so the scope owns
all three parts, and taking *it* rather than a `(GpuTimer*, const char*)` pair
also closes both halves of the argument mistake: a timer with no label published
an anonymous `"gpu"` row that `StageMetrics` merges across unrelated stages, and
a label with no timer read as instrumented while measuring nothing. Both
compiled. The same scope inoculates the deferred `mesh` sites.

**A stage that runs two kernels and times one misattributes the difference.**
`integrate`'s host row wraps the active-set compaction as well as the fusion
dispatch, so that kernel's device time fell into the gap between the halves and
read as submit overhead — the fourth instance of the mistake the *Measured
lessons* section below exists to record, and this one shipped in the entry's own
prose. `compact_active_blocks` now reports itself as a `kBreakdownPrefix`
sub-row, so the gap is overhead and nothing else. It is named `"  ..active set"`
rather than `"  ..compact"` because rows merge by content and `mesh` already
publishes a `"  ..compact"` phase for the same call inside its extract; two
stages' compactions summed into one row is the collision the prefix exists to
avoid. On the test fixture it is *half* of `integrate`'s host row (0.22 of
0.45 ms), so this was not a rounding-error correction.

**Availability is not an error — including when the pool will not allocate.**
Creating each tier's timer with `VR_ASSIGN` made a failed `vkCreateQueryPool`
refuse to construct `VoxelHashMap`, `TsdfIntegrator` and `ProjectiveTexturer`,
i.e. fail the whole reconstruction spine for a caller who never asked for
timing. `GpuTimer::create` now degrades that to `available() == false` plus a
logged warning, exactly as it already did for `timestampValidBits == 0`; the
class had been carrying two opposite policies. `GpuTimer::abandon` is the same
rule mid-run: when `submit_single_time`'s fence wait fails it deliberately leaks
the command buffer because the GPU may still execute it, and that buffer still
carries the span's `vkCmdResetQueryPool` and both timestamp writes — so the two
queries retire with it rather than being recycled into the next `begin`
(VUID-vkCmdResetQueryPool-None-02841). Long-lived per-tier timers are what made
that reachable; a throwaway timer died with the failed dispatch.

**The consumer was wired in the same change, because leaving it was a trap.**
`fuse_viewer` already wrapped `allocate`, `integrate` and `texture` in
`StageScope`s under exactly the literals the tiers now publish, and rows
accumulate by name — so passing the out-param without removing them would have
inflated `integrate` and `texture` ~2x and `allocate` up to ~6x (the tier's
scope fires once per retry round inside the outer one), silently, with
`shared_fuse_ms` inflating along with it. The tiers' rows replace the wrappers.
What a wrapper legitimately covered and the tier's row does not — the `resize`
between retries, by far an overflowing frame's largest cost — gets its own row
in both examples rather than being folded into `allocate`, where it would sink
that stage's device share on precisely the frames whose host cost is not the
kernel. Untimed, as it was in `fuse_replica`, the most expensive frames
contributed nothing at all to the table.

**The bug this shipped with for an hour is the one worth recording.** Each tier
gained a `GpuTimer` member and none of them *created* it, so every tier silently
reported host-only — a default-constructed timer is unavailable by design, which
is indistinguishable from the supported no-timestamps device. The first cut of
the test printed `timestamps unavailable` for all three rows and **exited 0**.
The fix is not just creating the timers: the test now asks the device
independently, by creating a `GpuTimer` of its own, and *fails* when a tier
reports no span on hardware that a probe says can time. A capability a test can
establish for itself must never be inferred from the thing under test.
Confirmed by mutation — deleting one tier's `GpuTimer::create` fails the suite
by name.

**And the test that pinned it was itself the review's flakiest finding.** Its
drain assertion compared two ~16 µs spans within a 1.8x band and failed 5 runs
in 60 on unmodified correct code — a ~1-in-12 false red on every CI leg — and
degenerated to `0.0 < 0.0` whenever a span resolved inside one timestamp tick,
which is a healthy device. Both window properties are now asserted on *counts*
instead: a window holds `max_spans` and then silently stops recording, so
running more calls than one window holds and requiring the last to still report
a device span catches "an untimed call recorded a span anyway" and "publishing
did not end the window" alike, with no timing ratio anywhere. Its companion,
`CHECK(untouched.empty())`, was asserting that a `std::vector` nothing was
passed to is empty — it held against exactly the global-sink tier it claimed to
be the only guard against. Both mutations now fail the suite by name, the second
via `gpu_ms >= cpu_ms` (32 accumulated spans against one call's wall clock is a
factor, not a coin flip); 80 consecutive runs of the replacement pass.

First numbers, **Release** (`-DCMAKE_BUILD_TYPE=Release`; at `-O0` these are
worthless — see the build-type gotcha in DESIGN.md), on the deliberately tiny
fixture the test runs (64x64 depth, three triangles), so they are
fixed-overhead-dominated and not a workload claim:

| stage | host | device | device share |
|---|---|---|---|
| `allocate` | 0.368 ms | 0.144 ms | 39.0% |
| `integrate` | 0.455 ms | 0.018 ms | 3.9% |
| `  ..active set` | 0.224 ms | 0.009 ms | 4.1% |
| `texture` | 0.246 ms | 0.018 ms | 7.3% |

**Read the device shares knowing what the instrument costs.** A timed submit is
~0.13 ms more expensive than an untimed one on MoltenVK — measured on an M5 Max
at 0.020 ms for a barrier-only submit with no timestamps against 0.145 ms with
one and 0.153 ms with two, so the whole penalty is the *first*
`vkCmdWriteTimestamp` in a command buffer and it is per submit, not per query.
On a real workload that vanishes (a 300-frame room0 A/B shows no regression),
but on this fixture it is a third of `integrate`'s host row, so these
percentages are ratios the instrument moved and not only fixed overhead. Quote
them as what they are: a demonstration that the halves separate, not a
measurement of the pipeline.

What is *not* yet wired: `mesh`'s GPU column — `ExtractTimings` keeps its six
host phases and its counters, and giving each phase a device half means
threading the timer through several dispatch sites inside one extract. The
number above says it must not be done one timed submit per phase: this run's
real phases are compact 0.15, inputs 0.00, arena 0.00, descriptors 0.00,
dispatch 0.63, readback 3.73 ms, so four of six would gain more than they
measure. Bracket several dispatches inside **one** timed submit with
`GpuTimer::begin`/`end` plus a single `resolve()` — which `submit_single_time`'s
own doc already points at for exactly this — and it is `TODO(mesh)` in
`marching_cubes.cpp`. Debug-utils labels are also deferred (`TODO(core)` in
`gpu_timer.hpp`): recon enables `VK_EXT_debug_utils` only when validation is on,
so a label needs `Device` to record whether it was enabled — the same
declare/verify shape as the enabled extension list, and its own change.

> **Superseded on 2026-08-30**, in the entry of that date. The labels landed,
> and the paragraph above is wrong in every clause: the extension is no longer
> gated on validation, the `TODO(core)` in `gpu_timer.hpp` is gone, and the
> label is **not** paired to a span — that pairing is the specific design this
> entry proposed and the later one rejects, since a span is opt-in and costs a
> timestamp while a label is free and unconditional. `Device` does now record
> whether the extension was enabled, which is the one clause that survived.

### 2026-08-10 — A breakdown row is one the *caller's* row already contains, so the host total skips it and the device total must not; and a ceiling the library knows is the library's to name.

*Reversed 2026-10-23 (below):* device time tiles, so a stage's device half
holds its breakdowns' and `total_gpu_ms` skips them again.

Refines the entry above, on three things the first cut of `kBreakdownPrefix` got
wrong once a sub-row carried a *device* half.

**The two halves do not nest the same way, so they cannot obey one skip.** A
host scope spans a whole call, including everything that call invokes — so a
sub-row's host time is already inside its parent's row and summing both
double-counts, which is the observation `kBreakdownPrefix` was built on. A
device span covers **one dispatch**. `integrate`'s span is its fusion kernel and
nothing else; the compaction it runs first is a separate submit that no row
above it contains. Skipping breakdown rows in `total_gpu_ms` therefore did not
deduplicate that time, it *deleted* it: the host half stayed counted through
`integrate` while the device half was counted nowhere at all, so the two totals
silently stopped describing the same work — in the one release where a
`..active set` row had just been added to make that kernel visible. `total_cpu_ms`
still skips; `total_gpu_ms` no longer does. The asymmetry is not an exception to
the rule, it *is* the rule: skip what a parent already contains, and a device
span never has a parent.

**Whether a row is a breakdown is a property of where it was called, not of what
it does.** `compact_active_blocks` hard-coded `"  ..active set"`, which is right
under `tsdf`'s open stage and wrong everywhere else: a caller compacting at top
level got a row `total_cpu_ms` skips — their only stage, missing from the total
they read it from, while still sitting in `rows()` looking accounted for. Naming
it plainly instead would just move the double-count back. So `StageMetrics`
answers `in_stage()` — maintained by `StageScope` itself, not threaded down by
hand — and the callee picks between two literals. This is the 2026-08-04 rule
applied to the one thing left that the caller could not see: it is the library
that knows whether a scope is open, so it is the library that checks rather than
documents. The same reasoning gave the frustum compaction the reporting
parameter its sibling had; it is the same round trip over a smaller set, and a
caller who switches to it to make that trip cheaper has to be able to read what
that bought rather than watch the row disappear.

**A tuning constant belongs beside the reading it qualifies.** `load_factor`'s
own header tells callers linear probing degrades sharply past ~0.7 and to grow
well under 1.0. The viewer's new gauge warned at 0.85, twice — once for the bar
colour and once for the text, two literals three lines apart that could drift —
and `volumetric_kit_ios` carried a third copy. So the number a reader saw
disagreed with the number the library documented, and the band between them is
precisely where every insert is slowest: an M5 iPad Pro lost the GPU inside
`allocate_from_depth` at 96% (see the 2026-08-08 overflow-scan entry), after
sitting healthy-looking well past 0.7 for half a scan. `VoxelHashMap::kGrowThreshold`
now names it once, at the library's own number, and the gauge reads it for both
the colour and the words. A UI drawing its own ceiling is a second source of
truth for a property of the table.

**And a gauge cannot answer an unknown with the last good value.** The viewer's
`load_factor()` read was `if (result) { store }` — on failure the bar kept
rendering the previous frame's fraction as current, which is the calm-green-over-
a-stopped-scan reading the gauge exists to prevent, arrived at by a different
route. It now reports on stderr like every other call in that loop and publishes
a negative the panel draws as `unavailable` rather than as a low number. The
read also moved out of the `share_mtx` critical section, beside the
`memory_stats()` call that was hoisted out for the same reason.

### 2026-08-11 — Projective texturing decides visibility per *vertex*, and a negative `uv0` carries its atlas coordinate rather than discarding it (amends the 2026-07-07 texture-tier decision, and retires the `share_vertices` refusal the 2026-08-04 entry records).

*Amended 2026-10-08 (below):* a vertex is textured only where its normal
points toward the depth camera, and a view whose depth range admits no sample
is refused.

*Amended 2026-10-16 (below):* the tiers no longer keep self-contained copies
of the projection. texture's goes through `core/shaders/pinhole.glsl`, as
tsdf's does, and keeps its deliberate divergence as its own acceptance rule.

`ProjectiveTexturer` refused any mesh built with
`MarchingCubesConfig::share_vertices`. It decided visibility per **triangle**
while writing `Vertex::uv0` per **vertex**, so an interior vertex referenced by
up to six triangles that disagreed was written by whichever thread ran last —
`Status::ok`, no validation diagnostic, visible only as flicker along a
silhouette. The refusal was the honest response to that, and the 2026-08-04
entry books it as an example of the rule it follows.

It also cost the one configuration a memory-bound consumer wants. The iOS
scanner runs sharing because the vertex arena is the term that binds there;
unsharing to satisfy the refusal hands back ~2 GB, and `fuse_viewer` — the only
seam-B consumer — could not take the flag at all, because it runs this tier.

**Every input to the verdict is a property of the vertex alone.**
`project_to_image` reads the position and the camera; `occluded_ok` reads the
resulting pixel and the depth map. Deciding it per triangle and writing the
answer to three vertices was a choice, not a requirement. One thread per vertex
gives one writer per vertex — nothing to race, no index buffer bound, and each
vertex projected once instead of once per referencing triangle (~3x less work on
a shared mesh). The descriptor set drops 4 bindings to 3 and the push block 12 B
to 8.

Measured on Replica room0, 120 frames, frame 60, via `fuse_render`: a shared and
an unshared render are **byte-identical** (same SHA-256), at 3.00 → 0.88
vertices per triangle. `uv0` is a function of position, so an unshared duplicate
receives the same value as the shared original and every triangle's provoking
vertex resolves the same class either way. **Sharing became a pure memory trade
with no effect on appearance.**

**What it costs is the all-three-vertices gate**, and that is the part that
needed designing rather than deleting. A triangle straddling the visibility
boundary is no longer refused whole; the renderer resolves albedo from the
provoking vertex, so such a triangle takes one class for its whole face and the
textured region grows by up to one triangle at an occlusion silhouette — 1.02%
of pixels differing by more than 8 levels, the difference map black everywhere
else.

Bounding it is what the encoding is for. `uv0` has **three** outcomes now:

- **visible** → `uv`.
- **in frame, occluded** → `-uv - 1`. Still negative, so gfx's `uv0.x < 0` class
  test is unchanged, but the coordinate is **carried** and gfx decodes it rather
  than substituting `(0, 0)`.
- **behind the camera, or a position that is not a number** → `(-1, -1)`, which
  decodes to `(0, 0)` — the old behaviour, so a producer that knows nothing of
  this is unaffected.

Without the middle one a mixed triangle interpolates from a real coordinate
toward the atlas origin and smears the image's corner across its face, which is
exactly how a shared mesh rendered before. **A consumer must test the sign and
never `== (-1, -1)`** — the rule now states so on `mesh::Vertex` itself, since
that struct is the renderer's vertex layout (2026-08-02) and a mesh-only
consumer has no reason to read the texture tier's header. A writer that must
emit a plain coordinate, such as the planned glTF `TEXCOORD_0` export, has to
decode rather than pass through.

**The frustum boundary was found on device, and is the same bug one step out.**
The first cut wrote the bare sentinel for anything `project_to_image` refused,
conflating *behind the camera* (no projection exists) with *in front but outside
the image* (the projection is fine, just past the border). A boundary triangle
therefore interpolated to the atlas origin and drew the **entire camera image
inside one triangle**, repeated the length of the frustum edge. So
`project_to_image` now projects unconditionally when the point is in front and
lets `pixel_to_atlas_uv` clamp; such a vertex carries the nearest **edge**
coordinate and the boundary renders as edge stretching. The bounds test is not
lost — `occluded_ok` already refuses a pixel within a texel of the border, so
nothing outside the image is ever classed visible. This is a deliberate
divergence from `tsdf_common.glsl`'s identically-signatured `project_pinhole`,
which rejects an out-of-image pixel; both files now say so, because the standing
convention is that each tier keeps a self-contained copy and the next person to
reconcile them would otherwise copy one into the other.

**A computed sentinel is not an absorbing one**, which is the review finding
that shaped the guards. The per-triangle kernel wrote the literal `(-1, -1)` on
its reject path; `-uv - 1` carries whatever it is given, and a NaN is not
negative — so a non-finite vertex position would produce a `uv0` the renderer
classifies as *textured* and samples the atlas at NaN, after passing an
`occluded_ok` bounds test that NaN also passes and indexing `depth[]` off a NaN
pixel. Every guard on that path is therefore written **negated**, so a
non-finite value fails it rather than slipping through, and `project_to_image`
rejects a NaN pixel as well as a non-positive depth: a finite `zc` does not make
the pixel finite. An infinity is still allowed through, since the clamp handles
it like any far-outside projection.

That guard is load-bearing for a second reason the dispatch introduced. The
domain widened from "vertices the index buffer names" to "every slot in
`[0, vertex_count)`", and on the sharing kernel the published count is a sum of
each block's *reservation* while claims are bounded separately — so a block that
claims fewer than it reserved leaves slots inside the count that no triangle
references and nothing zeroes. Reading one is in-range and harmless; decoding
arbitrary bytes as a position is what needed a bound, and the NaN rejection is
it.

**What did not change.** The packed multi-camera atlas still needs a
per-**primitive** camera id, and that is a genuinely different problem: a
triangle whose vertices index different sub-rects of a pack cannot be expressed
per vertex under any encoding, including this one. `DeviceMesh::shares_vertices`
therefore stays published — for **sizing**, since `v = 3t` no longer holds, not
as an incompatibility.

**The renderer half is a floor, not an option.** The decode lives in gfx #93,
and against an earlier pin the carried vertices forward `(0, 0)` — which is not
the pre-change behaviour, because the gate that made mixed triangles impossible
is gone in the same commit. Both viewer examples would render the smear on
default flags, and the viewer CI leg builds them without rendering, so nothing
would catch it. The `examples/viewer/CMakeLists.txt` pin carries a `gfx #93`
floor comment beside its two existing ones; it is the only one of the three that
fails silently rather than at compile time.

**Also settled here, from the same review.** The host overload's early-out read
`mesh::Mesh::empty()`, which reports `indices.empty()` — the one axis this pass
stopped reading — so a mesh with vertices and no indices returned OK having
written nothing, leaving every `uv0` holding a previous frame's coordinate. It
keys on `vertices.empty()` now, and the index count is not validated at all,
because it is not read. And the cached device limits moved with the dispatch: the
1-D workgroup ceiling now trips on the vertex count, three times sooner in
primitives on an unshared mesh, which is not worth working around — 16.7 M
vertices is 1 GiB at 64 B each, the common `maxStorageBufferRange` floor, so the
binding range runs out at the same point and does so naming the mesh.
### 2026-08-30 — A profiler label belongs to the kernel, not to the timed span; `VK_EXT_debug_utils` is requested independently of validation, and the *instance* extension is declared across the adopt seam.

Motivated by a plain question — how do we find out whether this pipeline is
memory-bandwidth bound on mobile — and by the answer that recon could not be
profiled by the tools that would say. The instrumentation recon had was good
and entirely about *time*: `StageMetrics` rows, `GpuTimer` spans, the
`ExtractTimings` phases. What it had none of was a way for **Nsight Graphics**
(Vulkan on NVIDIA) or **Xcode's Metal debugger** (MoltenVK on Apple) to name
what they were looking at, so a capture was a wall of anonymous dispatches over
buffers called `0x…`. One change fixes it on both, because MoltenVK maps
debug-utils labels onto Metal debug groups and `MTLResource` labels: the same
call that gives Nsight a trace range gives Xcode a named encoder.

**A label is not a span, and pairing them would have been the wrong seam.** The
`TODO(core)` this replaces read "pair a span with a `VK_EXT_debug_utils` label",
and implementing it literally would have produced labels *only where the caller
passed `StageMetrics*`* — `GpuStageScope::timer()` is null otherwise, and
`dispatch()` took its label from `stage->name()`. Two things are wrong with
that. A profiling run does not want recon's timings at all; the profiler is the
instrument, and recon's job is only to say what each dispatch *is*. And asking
for timings costs a timestamp — ~0.13 ms per submit on MoltenVK, measured in
the 2026-08-09 entry — so tying the two would perturb the very workload being
captured, and would leave every uninstrumented call anonymous. So the name
moved onto `ComputeKernel`, which every dispatch already has: labels are free
and unconditional, spans stay opt-in and measured. `dispatch()` opens the region
around the whole recording — bind, push, dispatch, barrier — because that is
the work a capture should charge to the kernel.

**The extension is requested on its own, because a Release build is the only
one worth profiling.** `Instance::create` enabled `VK_EXT_debug_utils` only
when validation was on, since the only consumer was the validation messenger.
That left exactly the build a profiler attaches to with no labels, and it is
the build whose numbers mean anything (the `-O0` gotcha in DESIGN.md is the
same lesson from the other side). `InstanceConfig::request_debug_utils`
defaults **on**: the label entry points are driver stubs when nothing is
capturing, so the cost is a predictable branch, and the alternative — a build
flag — means profiling a binary you already have requires rebuilding it, which
is how a profiling session turns into a different workload.

**Debug utils is an *instance* extension, which is what makes the adopt seam
its own decision.** It cannot ride `AdoptedDevice::enabled_device_extensions`:
that array carries what was enabled on the *device*, and this was not. So the
embedder declares it as `enabled_debug_utils`, the same declare/verify shape
`enabled_timeline_semaphore` and `enabled_scalar_block_layout` already use for
the same underlying reason — Vulkan will not say what was enabled. It is
declared rather than probed because the loader's result for a command of an
extension that was *not* enabled is not portable: a conformant loader returns
null, a directly-linked MoltenVK (the iOS case) does not. Asking
unconditionally would hand one platform a pointer it must not call. Unlike the
two feature flags, this one is never *required* — its absence costs the
capture's names and nothing else. The declaration exists on **both** seams:
`AdoptedDevice::enabled_debug_utils` on adopt, and
`DeviceConfig::instance_debug_utils_enabled` on create, with a
`Device::create(const Instance&, …)` overload that fills the latter in so no
recon call site sets it by hand. All three label commands are device-level, so
they resolve through `vkGetDeviceProcAddr` — as gfx's `DebugUtilsTable` does —
and a `Device` therefore keeps nothing of its `VkInstance`.

**Naming is hooked where handles change, not where they are first made.** A
debug-utils name lives on the handle, so every path that replaces a buffer
un-names it. Three of those exist and all three were live bugs waiting: the
grid's attribute arrays are all replaced by `VoxelBlockGrid::resize`, the hash
table's by a rehash commit, and the mesh arena's by a grow. The hash map hangs
its naming off `write_persistent_bindings()`, which already runs on create *and*
on every resize commit, so the names cannot drift from the handles by
construction; the grid and the mesh name at their own commit points, the mesh
only where a handle actually changed (per-frame re-statement of an unchanged
name is pure hot-path overhead). The mesh slot index is *in* the name —
`mesh.arena[2]` — because the ring hands a consumer one slot while the next
extract writes another, and a capture showing three identical `mesh.arena`
entries would not say which generation it caught.

`VoxelBlockGrid` grew a borrowed `Device*` for this, purely to re-name after a
grow, and the header's own warning caught the trap on the way in: its
move-assignment is hand-written and "must name EVERY member, and a forgotten
one is silent" — the same note left behind by the topology-epoch member that
was once dropped there.

**What this does *not* do**, and both are deliberate. It does not add
bytes-moved accounting, so the bandwidth question is still unanswered — the
labels make an external profiler readable, and the roofline that would make its
numbers comparable across NVIDIA's DRAM counters and Apple's UMA counters is
its own change. And it does not batch dispatches. One `vkCmdDispatch` still
means one command buffer, one submit and one fence wait (the 2026-07-05
decision, which named batching as the trigger for reusable command buffers and
timeline sync), so a capture will show serialized bursts with idle between
them, and both profilers will report that idle rather than the kernels. The
value of fixing it is unmeasured: the one phase breakdown that exists has
`readback` at 3.73 ms against `dispatch` at 0.63 ms, and batching does nothing
for a readback. Count the consecutive dispatches with no host dependency
between them before building the machinery.

**Review: the labels were dead in the one consumer that motivated them, and
that was a missing channel rather than a typo.** A max-effort pass over the
first cut returned fifteen findings, and the shape of them is one story. The
feature was off in `fuse_viewer`: `recon_adopt_payload` never forwarded
`enabled_debug_utils` (its `gfx_adopt_payload` neighbour did), and the shared
bootstrap still gated the extension on the validation layer — so the Release
build with no Vulkan SDK, which is precisely the profiling setup on Apple since
Xcode's Metal debugger needs no layer, got nothing at either end. Both are
fixed, but the *cause* was that `DeviceRequirements` had no debug-utils channel
at all, so an embedder following the documented merge could not learn recon
wanted it. It now carries `debug_utils` — the one optional entry, and the one
instance extension, both stated on the field.

Three findings were live defects rather than dead features. `end_debug_label`
guarded on the entry-point pointers while `begin_debug_label` also skipped a
null `name`, so a kernel registered with a null name popped a region that was
never pushed; `end_debug_label` now takes the name and tests every condition
its partner does. Resolution was not collapsed, so a driver returning two of
three pointers left a region openable and unclosable — which carries the buffer
to `vkEndCommandBuffer` with a label open and fails *every* submit; it is now
all-or-nothing, which is also what makes `debug_labels_available()` honest
reading one pointer. And `Device::create` resolved ungated, violating the gate
its own helper's comment declares mandatory — the create path had no channel
to gate on, which is the same missing-channel finding from the other side.

**The label was inside the timed span, which silently redefined every published
`gpu_ms`.** `dispatch()` recorded begin/end inside the lambda `submit_single_time`
brackets with timestamps, so every device figure in this file, in DESIGN.md and
on the `fuse_viewer` overlay began measuring two marker commands as well as the
work. Measured as noise on MoltenVK (allocate 0.470 vs 0.476 ms, interleaved),
but a debug group can force an encoder boundary and encoder boundaries are
exactly what costs there. The label is now passed to `submit_single_time` as
`debug_label` and recorded **outside** the timestamp pair: the capture's region
also covers the two timestamp writes, which is where they belong, being work
that submit does.

Verified on MoltenVK (M5 Max, Release): 28/28 ctest, the viewer target built,
and the new positive assertion shown to be a real detector rather than a
vacuous one — deleting the `resolve_debug_label_fns` call from `Device::create`
turns `recon_device` red (`instance=1 device=0`), where every assertion the
first cut added stayed green with the feature fully inert. The rest of the
findings were coverage and honesty: `%u` against a `std::size_t` slot index
(invisible to `-Werror` only because the format string reached `snprintf` as a
struct member); `debug_object_handle(VK_NULL_HANDLE)` ill-formed on 64-bit and
fine on 32-bit, the inverse of the portability it exists for; the span table
and the dirty-flag array left anonymous on their grow paths while their
mutually-exclusive dummies were named; no pipeline named at all, though
`VkPipeline` is what Nsight groups a capture by and what MoltenVK maps to
`MTLComputePipelineState`; and a doc claiming every buffer carried a name while
`tsdf` and `texture` named none — including the per-frame depth and colour
uploads, the largest transfers in the pipeline and the literal subject of the
bandwidth question. `KernelSetBuilder` took a `const Device&` rather than a bare
`VkDevice` to reach any of the naming, and the kernel name now also prefixes a
build failure, which is what its own comment had claimed all along.

### 2026-09-14 — The examples poll their frames through the sensor contract: the Replica reader is an `ICameraCapture`, a source says when it is exhausted, and a frame kept past the next poll is copied — the last one included.

*Amended 2026-10-07 (below):* the Replica reader is `ReplicaSensor`, an
`IRgbdSensor`, and the examples fuse what `GpuFramePrep` prepares from its
frames. A frame holds its pixels, so nothing is copied to be kept:
`OwnedFrame` is gone, and the viewers keep the prepared `DeviceFrame`. The
move constructor is defaulted, the frames to play moving with their vector,
so a moved-from sensor plays nothing; assignment is deleted.

The sensor tier had shipped a contract with no producer and no consumer in
this tree. Its only implementer was the ARKit driver in `volumetric_kit_ios`,
its only in-repo exercise the test fake, and the three fuse examples — the
code that actually drives the pipeline — read frames through a dataset API of
their own (`ReplicaDataset::frame(i)` returning a `FrameView`, plus
`make_depth_camera` to build the camera structs the loop then patched per
frame). With an external RGB-D source (Orbbec) next, that left a live driver
with nothing to plug into: every consumer was written against the dataset,
not the contract, and the contract's ergonomics on a real frame path — a
frame that borrows its pixels, a range stamped by the driver, a pose that
rides on the frame — had never been paid for by anything that ran.

**What changed.** `ReplicaDataset` / `FrameView` / `example_camera.hpp` are
gone, replaced by `examples/common/replica_capture.hpp`: a `ReplicaCapture
final : sensor::ICameraCapture` that plays the sequence back through
`poll()`. All three examples take an `ICameraCapture&` for the fuse loop and
touch the concrete type only where it is opened (options, `preload`,
`frame_count`, the colour camera the viewers size their render camera from).
No compatibility layer: the repo is under active development, and a shim
would have kept the loop's old shape alive beside the new one. Beside it, two
header-only helpers in `examples/common` that the review of the first cut
asked for: `rgbd_frame.hpp` (the one frame type, below)
and `fuse_frame.hpp` (one frame's allocate-and-integrate, which three loops
had each carried a drifting copy of — one printed the overflow and timed the
grow, one did neither, and the encoding hand-off into `ColorFrame` was the
kind of line a fourth copy drops with no error, since a `ColorFrame` left
defaulted *declares* canonical). The contract itself gained one member, and
`ReplicaCapture` gained a host-only test. Three points of the design carry
the decision; the first and third are corrections the review made to the
first cut, recorded as such.

1. **A source says when it is exhausted; the loop does not infer it from an
   empty poll.** The contract's `no_frame()` means "nothing this tick" for a
   live device, and a replay has no such state — every poll yields the next
   frame until there is none — so the first cut treated the first empty poll
   as the end of the sequence, rejected an `exhausted()` accessor as "a branch
   nothing reaches today", and deferred a wait-and-retry to "the day a live
   source sits beside the replay". The review showed that made the headers'
   own claim false as merged: a live driver constructed in place of
   `ReplicaCapture` returns `no_frame()` on its first idle tick and all three
   loops `break` — `fuse_replica` writes a header-only PLY and exits 0 with
   "done: fused 0 frames" — and the deferral could not be executed at the
   loop at all, because replay-exhausted and live-idle were one signal. So
   `ICameraCapture::exhausted()` exists: non-pure, defaulting to `false` (a
   live device is never exhausted, only stopped), so the test fake and the
   out-of-tree ARKit driver compile untouched, and overridden by the replay
   to `next_ >= end_`. Every loop now waits a millisecond and polls again on
   an empty poll from a source that is not exhausted, and ends on one that
   is; nothing on that branch is deferred any more. A decode failure on a
   frame that is present stays a real error and stops the run, which is what
   `fuse_render` already required after a partial-room PNG once exited 0.

2. **The frame cap, the stride and the depth gate are the capture's options,
   stamped on the frame — and the probe looks only at the frames those options
   select.** The ARKit driver stamps its own range constants on every frame's
   depth camera; the replay does the same with the CLI's `--min-depth` /
   `--max-depth` (now exposed by all three examples, where the viewers had
   offered only the far end and so refused `--max-depth 0.1` with a message
   naming a knob they never took — the refusal is also re-labelled as this
   capture's, with both values in it), deriving the depth camera from the
   colour one through `depth_from_registered_color` at `open`. That
   validates the range before a frame exists, keeps the two poses from a
   single trajectory entry so they cannot drift, and is the only copy of the
   intrinsics: the first cut kept a third `CameraModel` beside the two camera
   structs and size-checked the decode against *it*, so a later depth
   downscale could have stamped one extent and held another. `frame_count()`
   is probed from disk at `open` rather than read off the trajectory: room0
   lists 2000 poses against 400 images, which is what `fuse_replica`'s banner
   printed ("dataset: 2000 poses") and what the viewer's "fused N / M" showed
   under a `--max-frames` above 400 (its default of 400 clamped it; the first
   cut of this entry claimed the default over-promised, and it did not), while
   the old `preload_bytes_projected` probed for itself but only under
   `--preload`. The first cut probed every contiguous index from 0 and only
   afterwards applied the limit: the review measured 801 file opens to learn
   `end_ = 5` at `--max-frames 5`, on `fuse_viewer`'s main thread after the
   window was up, and a sequence thinned on disk to every N-th frame — which
   `main` fused completely under `--stride N` — played one frame with
   nothing said. The probe now visits index 0, `stride`, `2·stride`, … below
   the limit and stops at the first absent one, through a `stat` rather than
   an `ifstream` open (~10x cheaper for the same answer), so it scales with
   the frames the run will play and an index the stride never visits cannot
   end the sequence. One probe, one count, and a missing image at poll time
   becomes the error it is.

3. **A frame kept past the next poll is copied — every one, the last
   included.** `CapturedFrame` is a view the capture recycles on the next
   `poll()`. `fuse_render` used to re-read its keyframe from the dataset after
   fusion — random access a live source cannot offer — and now retains a copy
   of that frame as it goes by (the `--follow` frame, else the middle of the
   sequence, chosen against `frame_count()`). The first cut let `fuse_viewer`
   hold the *last* polled frame as a borrowed view for its final texture
   pass, "valid precisely because nothing polls after the loop" — and that
   named the wrong boundary. The retention sat at the bottom of a successful
   iteration, so every failure exit (a map that would not allocate after
   five rounds, a resize that failed, an integrate error) left the loop after
   the poll that had already move-assigned the next decoded frame over the
   one the view pointed at, and the final `publish` uploaded ~3 MB of freed
   depth and packed a freed colour plane — heap-use-after-free under ASan,
   a corrupted atlas or a SIGSEGV in Release, on the default streaming path.
   The ordinary exit survived only because `ReplicaCapture`'s exhausted
   branch happens to return before touching its frame, a detail no document
   promised and no test pinned; the terminating empty poll *is* a poll under
   the contract, and the ARKit driver rotates its buffers on one. So both
   viewers keep a keyframe the same way, in an `RgbdFrame` of their own —
   the type `ReplicaCapture` decodes into, made public and given the whole
   header, so there is one frame type in the examples and `CapturedFrame` is
   its view (`assign` from the view, capacity reused; `view()` back to the
   contract type, built on access so the defaulted moves stay correct), and `fuse_viewer` copies
   the newest fused frame at the bottom of every iteration: the review
   measured the 6.5 MB retain at 0.08–0.09 ms at `-O2` on an M5 Max — ~5% of
   a preloaded fuse iteration, ~1% of a streaming one. `ReplicaCapture`'s
   header no longer promises anything about the empty poll, and the in-loop
   publish still consumes the frame synchronously, as it may.

**What else the review corrected.** `ReplicaCapture`'s defaulted move pair
emptied the vectors but copied `end_`, `running_` and `next_` — the shape the
2026-08-03 entry records for the extractor — so a moved-from capture still
claimed frames and, polled, indexed an emptied pose table; the pair is
hand-written to leave the source default-constructed (nothing on disk, not
running, exhausted), and `load()` guards the pose table as well as `end_`.
The `current_borrowed_` member and its resolve-on-access `current()` had one
caller, in the same `poll()` that set both members under an "owned wins"
precedence rule; a `poll()`-local pointer does the same job with one member,
and the comment that said the decode was "swapped in only on success" now
says what the code does — move-assigned in place on success, releasing the
previous frame's storage, which is exactly the property point 3 tripped
over. `pack_color_rgba8` re-derived, byte for byte, the identity branch of
`sensor::to_canonical` and never read the frame's encoding declaration, so
the atlas leg assumed canonical bytes where the fuse leg checks them; both
viewers now run the keyframe through `to_canonical` (the identity plus an
opaque alpha for Replica's sRGB JPEGs) into the packed words an RGBA8 upload
reads on the little-endian hosts every Vulkan platform here is, and the
function is gone.

**What the review asked for and did not get.** A host-only test of
`ReplicaCapture`, since the examples had come to depend on three of its
behaviours — the empty poll and a failed decode leaving the previous frame
intact, and `frame_count()` agreeing with `preload()` and `poll()` — that no
test pinned and no CI leg ran (`viewer.yml` is build-only; nothing in `tests/`
links `vr_example_common`). Declined. The first two dependencies are gone
with the copy, and a test that reaches `poll()` at all needs decodable
images — an encoder in the test, or a JPEG as a byte array in source — which
is a fixture out of all proportion to an example's file reader. The replay
is verified the way the examples are, against the dataset (below); the
contract member it implements is pinned where the contract is tested.

**Verified.** `fuse_replica room0 --max-frames 60 --stride 3 --device-extract`
after the review's fixes: 20 frames played, 6 057 blocks, 628 833 vertices /
209 611 triangles — the same figures the first cut matched against the
pre-refactor Release binary (where the two PLYs were identical as triangle
multisets, the byte diff being the atomics' arrival-order permutation).
`fuse_render --follow 30 --max-frames 100` textures the retained frame-30
keyframe in register through the `to_canonical` atlas (PNG inspected);
`fuse_viewer --max-frames 12 --remesh-every 4`, *streaming*, fuses 12/12 and
publishes the final mesh textured with the copied last frame, nothing on
stderr. 27/27 tests pass; the examples build under `-Werror` on MoltenVK; an
ASan/UBSan build of the whole tree, viewer included, runs the suite and both
of those example runs clean.

### 2026-09-24 — The Orbbec SDK is a prerequisite behind `VR_WITH_ORBBEC`: installed once for the family, found, and never fetched.

*Amended 2026-10-07 (the rig entry, below):* the floor and CI's pin are
2.10.6, for its global timestamps, and CI installs it through one composite
action, `.github/actions/install-orbbec-sdk`.

The next capture source is the Orbbec rig — Femto Mega units, calibrated by
`volumetric_kit_calib` for their poses — and its driver needs the Orbbec SDK
v2. The SDK is a prebuilt binary (a universal dylib plus plugin libraries in
`lib/extensions/` that it loads from beside itself), and recon is not its only
consumer: calib's planned `capture/desktop` tier drives the same cameras.

**The rule.** `VR_WITH_ORBBEC` (OFF by default) requires an installed SDK
≥ 2.9.3 and exposes `ob::OrbbecSDK`; `cmake/vr_orbbec.cmake` does the finding.
The SDK is installed **once, outside every repo** — the family convention is
`<workspace>/third_party/OrbbecSDK_v<version>` — and every build that needs it
points at that copy with `-DOrbbecSDK_ROOT=<sdk>`, or the `OrbbecSDK_ROOT`
environment variable set once for every repo. It is linked in place, so no
build tree or worktree carries a copy of its own, and nothing is copied beside
a binary for it to run. That is the Vulkan SDK's treatment, not VMA's.

**Why not a pinned download.** A FetchContent of the release tarball, hashed
per platform, with a local-copy override, was the first proposal: it would
have let a fresh clone and CI build with no setup. The owner's call was to
make the SDK a requirement instead — one copy the whole family links, the
version stated as a floor this repo checks, not a download each build tree
repeats. Vendoring it into git (≈12 MB per platform) was ruled out, as was
assuming a system install (`/usr/local`: sudo, and a machine-wide version that
drifts under every repo); a system install still works, it is just not
assumed.

**Why off by default.** The SDK ships macOS, Linux (x86_64, arm64) and Windows
builds and no iOS one, and recon cross-compiles to iOS unchanged (the
2026-08-01 decision); a required SDK would break that build, and make every
consumer that never touches a camera install it. The iOS half is checked, not
just stated (the 2026-08-04 rule): with the option on, an Apple target other
than macOS stops the configure and says to turn it off, where it would
otherwise find the macOS SDK and fail at link time on the wrong platform's
dylib.

**Two quirks of the SDK's package files**, handled once in the module rather
than rediscovered by each caller:
- `OrbbecSDKConfig.cmake` sits in `<sdk>/lib`, which none of `find_package`'s
  standard layouts search under a prefix. Without `PATH_SUFFIXES lib`,
  `OrbbecSDK_ROOT` and `CMAKE_PREFIX_PATH` both miss it; only
  `OrbbecSDK_DIR=<sdk>/lib` finds it.
- The version file is `OrbbecSDKVersion.cmake`, not
  `OrbbecSDKConfigVersion.cmake`, so `find_package` never reads it and a
  versioned `find_package(OrbbecSDK 2.9.3)` rejects even a matching SDK. The
  module reads `PACKAGE_VERSION` out of that file and checks it itself: the
  floor *and* the same major version — the rule the file encodes — since the
  first cut's bare `VERSION_LESS` let a 3.0.0 through.

**A named root is authoritative.** `find_package` caches the directory it
found in `OrbbecSDK_DIR` for the life of the build tree, so review found that
re-pointing `OrbbecSDK_ROOT` at an upgrade was ignored without a word — the
smoke test still passing, since it compared the runtime against the same
stale configured version — and that after an "older than" error, re-pointing
it at a newer SDK repeated the error (the 2026-08-04 rule: a staleness the
caller cannot see is the library's to check). So whenever `OrbbecSDK_ROOT` is
set, as a variable or in the environment, the module drops the cached
directory and finds again. With no root named the cache stands, so a tree
configured with a one-off `OrbbecSDK_ROOT=<sdk> cmake …`, or reopened by an
IDE that never read the shell profile, still re-runs cleanly. A fuller
version — walking every candidate on the search path through its own version
file, skipping the incompatible ones, keeping a record to tell a
caller-supplied `OrbbecSDK_DIR` from the cache's — was written and dropped as
not worth its ~170 lines: the root is how the family points at the SDK, the
cases it added (two SDKs on one search path, a 32-bit build) do not occur
here, and an SDK the check rejects stops the configure with the fix named.

**CI installs it the way a developer does.** `_build.yml` takes an
`orbbec_sdk` input, on for the ubuntu-24.04 and macos-26 legs and off for the
other two, so both configurations build on every PR: a step downloads the
pinned 2.9.3 release for the runner's platform, checks its SHA-256, unpacks it
under `RUNNER_TEMP` and exports `OrbbecSDK_ROOT`, and the configure passes
`-DVR_WITH_ORBBEC=ON`. That is not the rejected pinned download: the *build*
still only finds, and the step is the one-time install a developer does,
repeated per job because a CI job starts with nothing. The pin is the
module's floor, so CI builds against exactly the oldest SDK it accepts; the
two move together.

**The smoke test's own fixes.** The SDK's `setLoggerSeverity` sets *every*
sink, the file one included, so calling it after `setLoggerToFile(OFF, "")`
switched file logging back on and the test wrote `./Log/OrbbecSDK.log.txt`
into `build/tests` — the thing its comment said it prevented. It now sets each
sink separately (`setLoggerToConsole(WARN)`), and both calls sit inside the
`try`, since they throw `ob::Error` like the rest. The device-list strings go
through a null guard before `printf("%s")`. And the listing is described as
what it is: `queryDeviceList()` blocks about a second probing the network, and
still an Ethernet camera can miss it from cold — polled every 250 ms on the
rig, the three Femto Megas answered the first query in 12 of 14 runs; in one
of the other two none answered for ~2.5 s before all three did, and in the
last none answered in the whole ~4 s window. So "0 device(s)" is a
log line, not proof that none is attached, and the driver's open path will
need to wait for discovery rather than trust one query. On Windows, where a
DLL is found on `PATH` rather than through an rpath and the SDK is not copied
beside the test (it loads `extensions/` from beside its own DLL), the test
prepends the SDK's DLL directory through `ENVIRONMENT_MODIFICATION` (CMake ≥
3.22).

**Verified** on macOS (Apple silicon) against the 2.9.3 install at
`~/ws/volumetric_kit/third_party/OrbbecSDK_v2.9.3`. `recon_orbbec_sdk_smoke`
asserts the runtime version equals the one configured against (the failure a
linked-in-place install invites is a different copy answering at runtime, not
a build error), then opens an SDK context and enumerates devices: it lists the
rig's three Femto Megas, all over Ethernet, passes with none attached, and
leaves no `Log/` behind. The module was run against scratch SDKs (2.7.6,
2.9.3, 2.10.0, 3.0.0) under both CMake 3.21.4 — the declared minimum — and
4.4.3, with identical results: re-pointing `OrbbecSDK_ROOT` (by `-D` and by
environment) in an existing tree switches SDKs, including out of an "older
than" error; 2.7.6 and 3.0.0 are each refused with the fix named; dropping the
environment variable in a later configure keeps the SDK already found, a
mistyped `-DOrbbecSDK_ROOT` stops the configure, and a found SDK deleted from
disk is not kept; and an iOS target stops at configure.
With the option on, all 28 tests pass under `-Werror` in Release; with it off,
the build is unchanged — 27 tests, the SDK never looked for.

**Open.** The Windows `PATH` has never run — no CI leg builds on Windows at
all (a `TODO(ci)` beside it). The rig's driver has not landed; under the
2026-08-02 rule it may, now that CI builds and tests against the SDK, and its
open path owns the discovery wait above. (*2026-09-26:* it has landed, discovery
wait included — see that entry.)

### 2026-09-26 — The Orbbec driver lands as `sensor/orbbec`, a target of its own: it undistorts colour and then registers depth to it on the host, and it reads the rig's sync roles without writing them.

*Amended 2026-10-06 (below):* the host path is gone. `OrbbecCapture`, the
SDK's undistortion and registration, and the host conversion are removed;
every frame is handed out as captured and prepared on the GPU.
*Amended 2026-10-07 (the rig entry, below):* `OrbbecSensor` writes a
camera's sync settings when asked (`apply_sync`), and only where they
differ; software triggering is refused after any write.

The 2026-08-02 rule admits a driver this repo can build and test, and CI builds
against the Orbbec SDK (the 2026-09-24 decision), so the Femto Mega driver lives
here: `OrbbecCapture final : sensor::ICameraCapture`, target
`volumetric_kit::recon_sensor_orbbec`, built only with `VR_WITH_ORBBEC`.

**Placement.** A target of its own, so `recon_sensor` stays what an out-of-tree
driver compiles against — no vendor SDK, no Vulkan. The SDK is `PRIVATE` behind
a pointer to an implementation, so the public header names no SDK type; the
driver installs its own headers, and the package config re-finds the SDK, only
when the package was built with it. The config hints the SDK the build checked
and does not re-check a version itself: a consumer that points
`OrbbecSDK_ROOT` elsewhere chose that SDK. The folder names the driver rather than a
category (`sensor/drivers/`): the rule keeps platform-bound drivers out, so a
second may never come. calib's planned capture tier is the known second
consumer and links this target through the package; if the reuse outgrows
linking, moving the folder to a shared repo is the whole change.

**The frame path: undistort colour, then register depth to it, then convert.**
Measured on the rig's primary (CL2A141000N, 1280x720 colour, 640x576 depth):
- The SDK's depth-to-colour alignment **ignores the colour lens.** Aligning a
  raw frame set and one whose colour was undistorted first gives bit-identical
  depth, and the registered depth matches an independent *pinhole* projection
  to 0.90 mm mean / 1.36 mm RMS over 1,755 samples; predicting the distorted
  colour location instead misses by 28.6 mm RMS. The colour lens moves pixels
  5.25 px on average (10.5 px at most), so without undistorting first every
  colour sample would sit that far from its depth. Undistorting first puts both
  images on one pinhole camera — the registered case `tsdf` and `texture` are
  written for.
- The SDK's `UnDistortionFilter` runs in the right direction: 0.25/255 per
  channel from an independent forward remap, against 17.6 for the opposite
  direction and 14.3 for none.
- The depth lens is the large correction (31.5 px mean, 220.9 px at the edge)
  and the SDK's registration makes it; nothing here re-implements it.
- Both filters keep the intrinsics and zero the distortion, so both of a
  frame's cameras are the colour camera, the depth one derived through
  `depth_from_registered_color`. The first pair of every start is held to that
  — both images reporting the stamped intrinsics, colour reporting no
  distortion, a NaN failing — and a mismatch is an `IoError` rather than a
  misprojected reconstruction (the 2026-08-04 rule). Registration is asked for
  at the colour image's full size (`setMatchTargetResolution(true)`) rather
  than left to the SDK's default.
- Cost on the polling thread (M5 Max, -O2): undistort 1.33 ms + register
  1.76 ms median; a whole `poll()` that hands out a frame, 3.56 ms median /
  4.20 ms worst. The SDK's frame thread only swaps the newest pair into a slot,
  so a pair nobody polls costs nothing past the SDK's own decode.

Depth to colour, not colour to depth: the texture tier's atlas must be
registered to the frame's camera, and colour sampled through depth would be a
640x576 atlas with a hole wherever depth has none. The camera's **hardware**
registration (`ALIGN_D2C_HW_MODE`) is deferred — a `TODO(sensor)` — because it
ships 2.5x the depth pixels over Ethernet, which three cameras on one link will
feel, and how it treats the colour lens is unmeasured.

**Colour is requested as RGB; the wire carries MJPG either way.** At 1280x720 an
RGB and an MJPG request move the same traffic (31 and 27 Mbit/s averaged over a
run), since the camera compresses and the SDK decodes on its own thread. The
camera also encodes H.265, which the SDK passes through undecoded: 21 Mbit/s
against MJPG's 185 at 4K, where MJPG reaches only 16.7 of 25 fps. A decoder of
our own is the step to 4K on three cameras, not this one.

**Sync roles are read and never written.** The rig is hardware-synced:
CL2A141000N is the primary, CL2A141000G and CL2A14100A4 are `SECONDARY_SYNCED`.
A secondary opened alone produced one frame set in ~6 s — to a consumer, a
camera that is merely slow. That contradicts the SDK's description of the mode
(capture at once, re-time to a signal when one arrives), so `waits_for_primary`
follows the measurement. The role persists on the camera, and a driver that
"fixed" it for a one-camera run would break the rig for the next one, so
`OrbbecDeviceInfo::sync_mode` reports it and nothing writes it; starting a rig
in its wiring's order belongs to the multi-camera source. Software triggering
is refused at `open`: it waits for a host trigger this driver never sends. A
camera reporting its image mirrored, flipped or rotated is refused too — its
intrinsics describe the unmirrored image, and nothing downstream could tell.

**Discovery waits.** `open` re-queries every 250 ms until the camera answers or
`discovery_timeout_ms` (8 s) runs out, since one query is no proof of absence
for an Ethernet camera (the 2026-09-24 measurement) — and, by the same
measurement, a list of one is no proof a camera is alone. So an empty serial
waits out the whole window and is refused as soon as a second camera answers;
a named camera opens the moment it answers, which is why the examples and the
hardware test name one.

**The SDK's callbacks hold a shared mailbox, not the capture.** The SDK calls a
*copy* of the device-changed callback after releasing its lock, so
unregistering does not wait for a call in flight, and a pipeline stop that
throws may keep the frame callback. Both capture the pending pair, the fault and
the counters by `shared_ptr`, so whichever of the capture and the SDK lets go
last frees them.

**Failures are graded.** A disconnect is permanent — the handle is gone, and a
camera that comes back is a new device — so the capture becomes `exhausted()`
and `start()` refuses it. A pair the SDK fails on is skipped and counted
(`stats().failed`), and only 30 in a row (a second at 30 fps) are an `IoError`,
since a camera whose every pair fails would otherwise look merely slow. A pair
that contradicts the negotiated stream (size, format, intrinsics) is an error
at once.

**The hardware test opens only the camera it is told to.**
`recon_sensor_orbbec_capture` skips unless `VR_ORBBEC_TEST_SERIAL` names a
camera — discovery is a broadcast, the cameras are exclusive, the Debug and
Release legs run at once, and a person may be using the rig — and skips on a
secondary. `recon_sensor_orbbec_conversion` pins the arithmetic with no camera:
units (0 kept exact), byte order, the camera struct (a negative `int16_t` size
refused, not wrapped), the sync-mode mapping, and the option checks `open`
makes before touching the SDK.

**A live camera is its own example, `fuse_orbbec`,** not a flag on
`fuse_replica`, whose preload, stride, dirty-block survey and phase breakdown a
camera takes none of. It fuses through the shared
`examples/common/fuse_frame.hpp`, stops after `--frames` frames (300 by
default; a camera is never exhausted), and gives up after 10 s without a frame,
naming a sync secondary as the likely reason.

**Verified** on macOS (Apple silicon) against the 2.9.3 SDK and the rig. The
capture test against CL2A141000N: 30 frames, 0 dropped, 0 failed, 63.6% of
registered pixels carrying depth, increasing timestamps, restart and moves; on
CL2A141000G it skips as a secondary. `fuse_orbbec --serial CL2A141000N`:
29.6 fps (the camera's rate), none dropped, a planar floor with its marker's
texture on its own geometry; on the secondary it stops after 10 s naming the
role, and with no serial and three cameras it refuses with all three serials.
With the option on, 30 tests pass in Release under `-Werror` and clean under
ASan/UBSan (the capture test and a `fuse_orbbec` run included), and a
`BUILD_SHARED_LIBS=ON` build links; with it off the build is unchanged — 27
tests, no `orbbec` file installed, no SDK asked for. A consumer project against
an install links the target through `find_package`, and `fuse_replica` on 60
room0 frames produces the same 636,624-vertex mesh as before.

**Open.** The disconnect and skip-and-count paths are implemented but
unexercised: no run has unplugged a camera, and no healthy run fails a pair.
Each camera stamps frames with its own hardware clock, so a rig's frames are not
yet on one clock. That, HEVC decoding, GPU pre-processing and the hardware
registration are `TODO(sensor)`s in `orbbec_capture.cpp`; Windows is unbuilt.

### 2026-09-27 — The rig is `OrbbecRig`: it checks the cameras against the rig's sync configuration and writes it only when asked, starts the secondaries before the primary, keeps the cameras on the host's clock, and builds each set around a primary frame; poses come from the calibration file.

*Amended 2026-10-06 (below):* a set's frames are as captured, read through
`poll_set`; the rig processes nothing on the host and is no longer an
`ICameraCapture`.
*Amended 2026-10-07 (below):* `OrbbecRig` is gone. Each camera is an
`OrbbecSensor`, checked against its entry of the sync file at `open` and
written only with `apply_sync`; a `SensorArray` starts and groups them, on
the SDK's global timestamps rather than `enableDeviceClockSync`, within 0.4
of a frame period rather than 5 ms. A read after a write comes from the
SDK's cache, so it does not prove the write, as this entry says it does.

**The rule.** `OrbbecRig` (`sensor/orbbec/orbbec_rig.hpp`) is opened from the
rig's sync configuration (`orbbec_sync_config.hpp`: the SDK's
`MultiDeviceSyncConfig.json` layout, as in `femto_mega_sync.json`), which
names the cameras, the primary, and each one's sync settings. `open` compares
every camera's stored settings with it and refuses a difference, naming each
field, unless `apply_sync_config` writes the configuration to the cameras that
differ. It then requires one primary and its secondaries. `start` has the SDK
sync the cameras' clocks to the host's (`enableDeviceClockSync`) and starts
the secondaries first, since the primary's first trigger is what they wait
for. Each set is a primary frame plus each secondary's frame within
`sync_tolerance_us` of it on that clock, and only a set's frames are
processed, through the `CameraStream` that `OrbbecCapture` now wraps.
`poll_set()` hands out one set per trigger, with an empty slot for a missing
secondary; `poll()` hands out the same frames one at a time, so the fuse loop
reads a rig as it reads one camera. A caller reads with one of the two per
`start`.

**Which sync fields are compared.** On the Femto Mega, a written `SECONDARY`
reads back as `SECONDARY_SYNCED`, `trigger2ImageDelayUs` reads back as the
depth delay, and `framesPerTrigger` as 0; writes persist (a 200 µs delay read
back from a fresh process). So the comparison covers the role (primary or
secondary), the depth and colour delays and the trigger output, and
`framesPerTrigger` only in a triggering mode; anything more would refuse
every open. The camera has one secondary mode, and it waits for the trigger:
started alone for 4 s, the three secondaries delivered one frame between
them. The SDK answers a read in the same process with what was written, so
the check after a write proves the write, and the next `open` proves the
camera. The lab rig's file is committed as `config/femto_mega_sync.json`,
which a test keeps valid; the library never reads `config/` itself.

**Why these numbers.** On the four-camera rig over the 2.5 Gbit/s cable,
720p colour, 640x576 depth, 30 fps:
- With clock sync, a secondary's frame lands +0.4 to +1.7 ms from the
  primary's, stragglers to 3.4 ms. Without it the cameras' clocks are ~30 s
  apart and host arrival times jitter by ±20 ms. The 5 ms tolerance clears
  every straggler; half a frame period (16.7 ms), where one frame could match
  two triggers, is refused.
- Each secondary drops 1-6% of its frames and the primary none, so 90-99% of
  sets are complete. A set goes out once each missing secondary has sent a
  later frame or stayed silent for 1.5 frame periods, rather than costing the
  other cameras their frames.
- The primary's frames name the triggers, so its first triggers, which reach
  the secondaries ~525 ms before its own first frame reaches the host, a late
  secondary frame, and a camera whose clock is off by more than the tolerance
  all cost frames, never an extra set.
- A set's four frames take ~11 ms to process. A rig camera's mailbox keeps
  its four newest frames, so a poll slower than the cameras still finds one
  trigger in all of them: at 58 ms a set (ASan), one-frame mailboxes left
  75.6% of sets complete, four-frame ones 98.9%.
- Over Wi-Fi the rig ran at 13-28 fps. Depth crosses the network uncompressed
  at ~170 Mbit/s per camera, so the rig needs the wired link.

**The calibration file** (`sensor/rig_calibration.hpp`, in `recon_sensor`;
*amended 2026-10-06 (below):* now `camera/array_calibration.hpp`, in
`recon_camera`, which `recon_sensor` links, and it reads only the poses) is
the family's config
layout: `device_calibration.<serial>` with `intrinsics`, `distortion`,
`optimal_intrinsics` and `pose {rvec, tvec}`; other sections are ignored.
`pose` is the colour camera's OpenCV extrinsic
(world to camera; Rodrigues radians; metres), the one required field, turned
into this repo's camera-to-world on read. The lens fields are parsed and kept
but not used: they are meant to equal the factory calibration the capture
undistorts with. The inverse Rodrigues takes its angle from `atan2`, since
`acos` near pi turned float round-off into an 8e-5 pose error; every pose now
round-trips to 1.2e-7. It is in `recon_sensor` because calib writes it and
nothing in it is Orbbec's. Both files are parsed with nlohmann/json 3.12.0
(hash-pinned in `third_party/CMakeLists.txt`), a private `SYSTEM` include like
VMA's.

**Verified** on the rig. `recon_sensor_orbbec_rig` (`VR_ORBBEC_TEST_RIG`
names the sync file) opens against `femto_mega_sync.json` with nothing
written, refuses a copy with one delay changed, naming the camera, and gets
90 sets, 98.9% complete, worst skew 1.59 ms, each frame posed from a written
calibration file. `fuse_orbbec --rig --apply-sync` wrote only the one camera
that differed; applying the real file put it back, matching a backup taken
first. `recon_sensor_rig_calibration`, `recon_sensor_orbbec_sync_config` and
`recon_sensor_orbbec_grouping` need no camera; 36 tests pass with the driver
built, under `-Werror` in Release.

**Open.** Why the secondaries drop frames with bandwidth to spare is
unmeasured. Discovery once found no camera for 8 s right after another
process released one. A set's frames are processed one camera after another
(a `TODO(sensor)`). No real rig poses exist yet; that waits on calib.

### 2026-09-27 — The video decoder is `recon_sensor_video`, a target of its own over an installed FFmpeg: HEVC in, host pictures out, on the first hardware back end that decodes a built-in clip, NVIDIA ahead of an integrated GPU; each CI leg requires the hardware it has.

*Amended 2026-10-06 (below):* host pictures, `Auto`, the probe clip and
software decoding are gone. The decoders hand out device pictures only, on
NVDEC through CUDA or on VideoToolbox, and only the 24.04 and macOS legs
build them.

**The rule.** `HevcDecoder` (`sensor/video/hevc_decoder.hpp`) takes an Annex B
H.265 stream one access unit at a time and hands out host pictures, as
`Rgb24` or as the decoded `Yuv420` planes with the matrix and range to convert
them by. It knows nothing of cameras, so the Orbbec driver will link it, not
the other way round. FFmpeg is a prerequisite behind `VR_WITH_FFMPEG`, found
through pkg-config and never fetched, like the Orbbec SDK; the floor is
Ubuntu 22.04's FFmpeg 4.4. No FFmpeg type is public.

**Back ends.** `Auto` tries VideoToolbox on Apple; CUDA (NVDEC), then VAAPI
on Linux; CUDA, then D3D11VA on Windows. A machine with an NVIDIA and an
Intel GPU therefore decodes on the NVIDIA one. A back end counts only if it
decodes a built-in 157-byte clip, because an open device says nothing about
HEVC: an older Intel GPU opens under VAAPI without decoding HEVC. VideoToolbox is asked instead, through
`VTIsHardwareDecodeSupported`, the one back end with a single call for it.
The others each have a query of their own (NVDEC's `cuvidGetDecoderCaps`,
VAAPI's profiles, D3D11's decoder profiles),
each with its own dependency and two with no CI leg, while the clip tests
the path FFmpeg will decode on. Each is probed once per process, with
FFmpeg's log silenced (a missing back end says so at ERROR: "Cannot load
libcuda.so.1"), and `Auto` probes down the list only until one decodes and
opens, so an NVIDIA machine never probes VAAPI. A named back end
that is not there is refused, and the message lists those that are. Only
`Auto` falls back to software, including mid-stream when the hardware refuses
a stream (4:4:4 or 4:0:0, say); `backend()` reports the switch. Whether a
picture needs copying off the GPU is asked of the picture (`hw_frames_ctx`),
not of the back end, because the pictures the hardware decoded before the
switch are still waiting for display. A named back end meeting such a stream
returns `Unsupported`, after the pictures decoded before it.

**No Vulkan.** FFmpeg can also decode HEVC through Vulkan video, on NVIDIA's
driver and Mesa's, and it is left out. At a format change mid-stream (the
fallback clip's 4:0:0), FFmpeg tears its Vulkan decoder down before asking
for the new format, and freeing the decoder afterwards segfaults in NVIDIA's
Vulkan driver: `avcodec_free_context`, then libavcodec, then
`libnvidia-eglcore.so`, on FFmpeg 6.1 and 8.0 with driver 615.71.09. CUDA
passes the same clip. Nothing outside FFmpeg avoids it, and CUDA and VAAPI
already cover NVIDIA, Intel and AMD. Its draw comes with the GPU module:
pictures decoded straight into `VkImage`s, taken without a copy.

**Cropping.** A stream's display window can start right of or below the
coded picture's corner (an SPS conformance window). FFmpeg crops a hardware
picture only at the right and bottom, so the decoder crops the host copy at
the left and top. VideoToolbox cannot be fixed that way: FFmpeg sizes its
output at the display size, which VideoToolbox fills from the coded corner,
and then clears the frame's crop, so the right pixels never reach the host.
The decoder therefore reads each SPS itself on VideoToolbox (the conformance
window, H.265 7.3.2.2; only the NAL units ahead of a slice) and treats a left
or top offset like a stream the hardware refuses. Software decoding sets
`AV_CODEC_FLAG_UNALIGNED`, without which FFmpeg keeps up to 64 columns of a
left crop to keep the planes aligned.

**Colour.** `matrix` and `full_range` describe the planes handed out, which
is what a `Yuv420` consumer converts by. So a converted `Yuv420` keeps its
source's range (swscale alone takes a YUVJ source to limited range), and an
RGB-coded stream is converted to `Yuv420` by the matrix it is labelled with.
SMPTE 240M and FCC have their own matrices, since swscale has tables for
both. `encoding` carries the stream's transfer function and primaries as the
`ColorEncoding` a capture source declares, and is empty for anything that
type cannot name (HLG, BT.601's own primaries), so a driver refuses rather
than guesses.

**Modules.** Only `hevc_decoder.cpp` is HEVC-specific. `ffmpeg.hpp` is the one
FFmpeg include (owning pointers, errors), `hw_backend` holds the platform
order and opens devices, and `picture_converter` is the swscale step and the
matrix choice. An H.264 decoder, or a GPU path that skips the host copy,
reuses the three; the converter reports errors under the name its owner
gives it.

**Packaging.** A static `recon_sensor_video` is code compiled against one
FFmpeg's headers, so the package config holds a consumer to that FFmpeg: its
major versions, no older than its minors. The floor applies only to the
build. A consumer whose pkg-config finds another FFmpeg is told which to
point `PKG_CONFIG_PATH` at, rather than linking one that lays `AVFrame` out
differently.

**Why these numbers.** M-series Mac, FFmpeg 9.0.2, 120-frame IPPP clips,
ms per frame:

| | 720p YUV | 720p RGB | 4K YUV | 4K RGB |
|---|---|---|---|---|
| VideoToolbox | 0.93 | 2.90 | 4.21 | 22.70 |
| software, FFmpeg's threads | 0.45 | 2.05 | 2.07 | 18.54 |
| software, one thread | 1.50 | 3.46 | 13.38 | 32.09 |

- Converting to RGB on the host costs ~18 ms at 4K, and nearly all of that
  is the flags. An earlier note here said full chroma interpolation and
  accurate rounding cost nothing measurable; that is wrong on FFmpeg 9.0.2.
  `SWS_ACCURATE_RND | SWS_FULL_CHR_H_INT` take swscale off its unscaled SIMD
  path, and plain `SWS_BILINEAR` converts 4K NV12 or YUV420P to RGB24 in
  0.96 or 0.90 ms instead of 18.9 or 17.9. On solid colours both are within 1
  code of the standards: 99.7% exact with the flags, 77.7% without. What the
  fast path gives up is chroma interpolation, since it repeats each chroma
  sample. `SWS_ACCURATE_RND` alone is the case that came out 3 codes off
  (10 ms). swscale is CPU-only, and its own threads did not speed up either
  path. 4K colour for a rig is to be converted on the GPU, from `Yuv420`
  (the next step), so the host path's flags are left as they are for now.
- VideoToolbox's times include copying each picture out of GPU memory.
- Software with FFmpeg's thread count held back 15 pictures, half a second at
  30 fps, so a live stream sets `threads = 1`. Hardware held back none.
- A hardware decoder opens with the caller's `threads` as *slice* threads, for
  a stream `Auto` moves to software. They hold nothing back, since frame
  threads would hold pictures back on the hardware too, but they help only a
  wavefront (WPP) stream: on 4K x265, 12.3 ms a frame on one thread against
  3.7 on slice threads (frame threads: 4.2), and 10.8 either way without WPP.

**CI.** Every build leg builds the decoder against its own FFmpeg and sets
`VR_TEST_HEVC_BACKEND`: `cuda` in the Linux GPU containers, `videotoolbox` on
the Mac. The decoder test fails unless that back end decodes and `Auto` picks
it, so a container that lost NVDEC, or one that picked an integrated GPU,
fails instead of passing on software. The sanitizer job decodes in software.

**Verified** on the Mac, and on NVDEC in CI (FFmpeg 4.4, 6.1 and 8.0).
`recon_sensor_video_hevc` decodes the committed clip
(`tools/make_hevc_fixtures.sh`: 256x144, the height NVDEC's minimum allows,
8 frames of solid patches at qp 4). In software each patch is within 2 codes
of the pattern in YUV and 3 in RGB, BT.709 limited. On VideoToolbox and
NVDEC the YUV is bit-identical to software and the RGB within 1 code.
`recon_sensor_video_converter` checks each matrix and range on hand-built
frames against the standards' constants. A wrong matrix, wrong coefficients or
a dropped pts each fails a test, and `VR_TEST_HEVC_BACKEND=cuda` fails on the
Mac.

Two more clips came out of review. `cropped_240x128.h265` is the same
frames with a window 16 right and 16 down; it must equal the uncropped
decode's pixels there, exactly, on every back end that crops, and named
VideoToolbox must refuse it. `fallback_256x144.h265` has B-frames and then
4:0:0 grey, for which FFmpeg offers no hardware format in any version from
4.4 to 9.0.2. `Auto` must hand out all 10 pictures, in display order and
each pts sent once, and end on software; a named back end must refuse it.
Taking back each of these fixes fails a test: deciding the GPU copy by the
back end, the SPS read, `AV_CODEC_FLAG_UNALIGNED`, a YUVJ source's range, the
target matrix of an RGB-coded stream, and SMPTE 240M. The SPS reader was
also checked against an x265 stream with three temporal sub-layers.

**Review (PR #92).** Fixed as described above: the lost pictures after a
fallback, the left and top crop on hardware and in software, the range of a
converted YUVJ source, the transfer and primaries, the matrices, a failed
`sws_scale` (it returned OK), single-threaded decoding after a fallback, the
unpinned FFmpeg in the package config, `Auto` giving up after the first
hardware back end, the probe's cost and noise, and the converter's
hard-coded name. Measured and left as it was: FFmpeg allocates a fresh host
picture for each hardware picture, and keeping one buffer measured no faster
(3.70 against 3.63 ms a 4K picture on VideoToolbox), so the simpler code
stays. Kept on purpose: a decoder opens its own device after the probe
closes its own, since sharing one would tie every decoder to a device the
process holds.

**Open.** VAAPI decoding is untested, and D3D11VA has no leg. The
left and top crop on hardware is tested only on the back ends CI has
(NVDEC). Pictures pass through host memory (a `TODO(sensor)`).

### 2026-09-28 — The Orbbec driver streams colour as H.265 on request: every colour frame is decoded, in order, on a thread per camera ahead of the mailbox; a lost frame is read off the frame index, not the clock; and the Femto Mega's stream is decoded as BT.601 full range, which it codes and does not say.

*Amended 2026-10-06 (below):* the decoder hands on the planes, or the picture
the hardware left on the device, never an RGB frame: the `Rgb24` frame
stamped with the RGB profile described here fed the host path, which is gone.
A frame's colour camera is read off the H.265 mode's own profile, so the
byte comparison under "The calibration" went with the RGB mode.
*Amended again the same day (the device-only decoder entry, below):* it
decodes on NVDEC or VideoToolbox only, hands on the picture on the device
only, and a stream the hardware refuses stops it.
*Amended 2026-10-27 (the colour-controls entry, below):* `Hevc` at
3840x2160 and 25 fps is the default; `Mjpeg` stays selectable.

**The rule.** `OrbbecStreamOptions::color_codec` is `Mjpeg` (the default)
or `Hevc`. With `Hevc` the camera sends H.265, and each camera's
`HevcColorDecoder` (`sensor/orbbec/hevc_color.hpp`, built only with
`VR_WITH_FFMPEG`) decodes every colour frame, in order, on a thread of its
own, before the mailbox. It uses `HevcDecoder` (`Auto`, one thread, `Rgb24`),
turns each picture into an RGB frame stamped with the RGB mode's profile, and
posts it with its depth. From the mailbox on, nothing differs from MJPEG.
Pairs the decoder cannot hand on are counted in `OrbbecCaptureStats::lost`.
Without FFmpeg, `open` refuses `Hevc` before it touches the SDK.

**Why ahead of the mailbox.** The mailbox keeps the newest pairs and drops
the rest. An H.265 frame dropped before decoding corrupts the frames predicted
from it, up to the next key frame, which the camera sends every 30 frames.

**The colour.** The camera writes no VUI (`vui_parameters_present_flag = 0`),
so its stream would decode as BT.709 limited range, the default for an
unlabelled 720p stream. Its luma spans 0-255 in every frame, though. Fitting
the decoded colour to the camera's own MJPEG output of the same still view,
per channel (`a = gain * b + offset`, which absorbs the exposure change
between the two captures), stable over two runs:

| decoded as | residual RMS | on saturated pixels | gains (R, G, B) |
|---|---|---|---|
| BT.601 full | 4.16 | 3.50 | 0.998, 0.999, 0.996 |
| BT.709 full | 4.54 | 5.18 | 0.977, 1.007, 0.983 |
| BT.601 limited | 5.12 | 4.52 | 0.886, 0.884, 0.879 |
| BT.709 limited | 5.31 | 5.83 | 0.878, 0.890, 0.868 |

So the driver decodes it as BT.601 full range, through
`HevcDecoder::Options::unlabelled_color`: the matrix and range for a stream
that declares no matrix. A stream that declares one is decoded as it says, so
a camera or firmware that labels its stream is taken at its word. The range
goes with the matrix because FFmpeg reads a stream with no VUI as limited
range (ffprobe's `color_range=tv`), just as one that declares limited, so
only the missing matrix tells the unlabelled stream apart. BT.601 full is
also how JPEG codes the MJPEG stream, so a frame's colour is the same sRGB
either way.

**The calibration.** The decoded frames carry the RGB mode's profile. Its
intrinsics, distortion and depth extrinsic are the H.265 mode's byte for byte,
measured at 720p, 1080p and 4K, so undistortion and registration run
unchanged. `open` compares those bytes on every camera and refuses one where
they differ (`Unsupported`): the first-pair check compares the frames with
the RGB profile they were stamped with, so it could never catch it.

**What loses a pair.** Two things measured on the rig shaped this:
- **The SDK's pairing.** Handed only complete pairs, the SDK dropped a
  secondary's colour frame whenever its depth frame was missing, under 2% of
  triggers. With MJPEG that costs the one pair; with H.265 each gap cost the
  stream up to its next key frame, 8-30% of a secondary's pairs over two
  runs. So for H.265
  the SDK hands over every colour frame (`COLOR_FRAME_REQUIRE`), each is
  decoded, and a pair without depth is dropped after. Colour index gaps over
  a run went from 22 to 0.
- **The clock.** A sync secondary's first frame comes as soon as it starts,
  and the rest once the primary triggers it, 0.3-4.2 s later, with no frame
  lost between. A rule that dated a gap from the timestamps threw away 29
  good frames per start. The camera's frame index, one up per encoded frame,
  is what shows a loss (`ColorStreamGate`). A skipped or repeated index, an
  empty frame (which, sent, would end the stream), a decode error, or a queue
  two seconds behind waits for the next key frame; so does the start.

Where decoding picks up again, and what is counted:
- **The restart.** The key frame that ends a wait resets the decoder
  (`HevcDecoder::reset`, FFmpeg's flush). What it still holds belongs to pairs
  from before the loss and is counted lost, and a CRA's leading pictures are
  skipped rather than decoded from a reference that may be the lost frame;
  FFmpeg 6.1 conceals a missing reference instead of refusing it. The reset
  also clears a decoder left holding pictures after a failed send or a
  thrown call, which would otherwise refuse every send after it.
- **Display order.** Pictures come out in display order, so a picture settles
  its own pair and no other. A pair sent 32 access units ago with no picture
  is one the decoder skipped, and is counted then: H.265 holds at most 16
  pictures back, one thread adds none, and an FFmpeg before 7.1 (6.1 on
  Ubuntu 24.04) hands out one picture per access unit, so the pictures a
  sequence holds back can wait behind the last one's too. A window of 16,
  the first cut's, would have dropped such a pair as lost. The Femto Mega
  sends no B-frames; a stream that did would otherwise have lost about half
  its pairs.

**Why these numbers.** The four-camera rig at 720p30 over the cable,
1200 frames, Release:

| | MJPEG | H.265 |
|---|---|---|
| complete sets | 291 of 304 | 292 of 304 |
| lost per camera | 0 | 1-4 (colour without its depth) |
| fused | 115.6 fps | 114.3 fps |
| CPU, user + sys | 9.37 + 2.58 s | 9.49 + 2.88 s |

Colour on the wire is 37-38 Mbit/s per camera as MJPEG and 21.6 as H.265
at 720p30, and 185 against 21 at 4K25. Decoding costs about what the SDK's
JPEG decoding does. One camera at 4K25 fuses at about 24 fps either way,
using about 55 ms of CPU a frame, nearly all of it undistorting, registering
and converting at 4K. So a 4K rig waits on the GPU module, not on the
decoder.

**Verified.** `recon_sensor_orbbec_hevc` runs the decoder on committed clips
wrapped in SDK frames, with no camera. It checks order, timestamps, depth
and colour; that a lost frame costs the frames to the next key frame, an
empty one too, and a pause costs nothing; that colour without depth is
decoded and dropped, and the frames after it still decode; that a labelled
stream keeps its label; that B-frames lose nothing; that a restart in open
GOPs skips the CRA's leading pictures and counts them 32 frames on; and it
tests `ColorStreamGate` on its own. The clips are the patch clip unlabelled
(`unlabelled_256x144.h265`, its SPS's matrix, primaries and transfer set to
unspecified, which is what FFmpeg reads off the camera), labelled, with
B-frames, and in open GOPs (`open_gop_256x144.h265`).
A test cannot set an SDK frame's index, so it numbers its frames through
`Options::frame_index`. It first relied on the decoder noticing the loss,
which FFmpeg 9 does and FFmpeg 6.1 does not: 6.1 conceals the missing
reference and decodes the next frames wrongly, without an error. Removing the gate's index check,
the decode of a colour frame without depth, or the unlabelled colour each
fails it. So does taking back each review fix below: settling earlier pairs
with a picture, the reset at a restart, the count 32 frames on, applying the
unlabelled colour to a labelled stream, a reset that does not flush or does
not undo the end, and the early refusal without FFmpeg. Sending an empty
frame alone does not, since the reset at the next key frame now undoes the
end of stream it caused; sending it without that reset does.
`recon_sensor_video_hevc` tests the reset and the unlabelled colour on every
back end. The first cut of the B-frame test failed on CI's Ubuntu 24.04 leg:
it waited for every picture, and FFmpeg 6.1, handing out one picture per
access unit, still held two when the stream ended, where FFmpeg 9 lets them
all out at the next clip's key frame. Replayed against libavcodec 6.1, the
open-GOP run hands out 41 of its 43 pictures, none later than 4 access
units after its own. So the tests take every picture in order but the last
three, and the window above is 32, not 16. On the rig, `recon_sensor_orbbec_capture` holds H.265 to the MJPEG
contract, and its colour to MJPEG's within a 5% gain (0.997, 0.984, 0.974
measured). `recon_sensor_orbbec_rig` streams 90 sets with each codec (H.265:
89-96% complete, worst skew 1.6-2.2 ms, 0-3 lost per camera). Both passed
again after the review fixes, the calibration check included (gains 1.000,
0.987, 0.974; H.265 sets 98.9% complete, 0-1 lost per camera).

**Review (PR #97).** Fixed as described above: an empty frame ending the
stream, the calibration assumed rather than checked, the colour forced on a
labelled stream, pairs lost to display order, a CRA's leading pictures
decoded after a loss, and a decoder wedged by pictures left waiting. Also
fixed: a queue allocation that throws inside `noexcept push` loses the pair
rather than terminating; the decoding thread failing to start is `IoError`,
not an exception; `start()` documents the decoder's `Unsupported` and
`IoError`; the refusal without FFmpeg moved ahead of discovery;
`configure_sdk_logging` says it sets FFmpeg's level, and does so at the
first start only; the stats say what `received` and `lost` count under
H.265; and `stop()` releases the decoder and the RGB profile rather than
leaving them to the last reference, which the SDK's callback may hold past
the context. Measured and left as it was, from a 21 Mbit/s 4K x265 clip on
an M5 Max, Release:
- Each decoded picture is copied into a freshly allocated SDK frame: 0.5 ms
  at 4K, allocation included, against 24.4 ms a picture on VideoToolbox. A
  pool, or swscale writing into the SDK frame, would save 2%.
- Software decoding stays on one thread: 32.5 ms a picture, about 13 of it
  decoding and the rest the RGB conversion, inside the 40 ms of 4K25. Frame
  threads would shorten only the decode, and hold pictures back. A host
  slower than this one with no hardware decoder would fall behind, which
  the queue's overflow would show in `lost` (a `TODO(sensor)`).

**Open.** A 4K rig waits on the GPU module, and so does keeping the decoded
picture off the host. The camera's own H.265 settings (key-frame interval,
bitrate) are left at its defaults (a `TODO(sensor)`). The SDK's "Stream have
not been started!" at every close and its RTP "Metadata size is too large!"
come with MJPEG too.

### 2026-09-28 — Projective texturing from several views chooses a view per triangle, on an unshared mesh, into an atlas of the views' images side by side; the single-camera pass stays per vertex.

**The rule.** `ProjectiveTexturer::texture(mesh, views, layout)` textures a
mesh from N posed `TextureView`s. A view is a depth map, the depth camera
that took it, and the size of the colour image registered to it, which may be
larger. The views' images sit side by side in one atlas
(`texture_atlas.hpp`): `side_by_side_atlas` lays them out in view order,
wrapping into a new row past the device's `maxImageDimension2D`, and
`pack_atlas` copies the images in. Each triangle takes one view, and its
three vertices point into that view's tile. A triangle no view sees whole
from the front gets `(-1, -1)`, the vertex colour. gfx needs no change:
`HybridMeshPipeline` samples the atlas through `uv0`, as it samples the
single camera's image.

**Why per triangle, and so unshared.** If a triangle's vertices took
different views, `uv0` would interpolate across the atlas between two tiles
and smear whatever lies between them over the face. No per-vertex encoding
avoids that. So the pass runs one thread per triangle, which needs each
triangle to own its three vertices: the default marching-cubes kernel's
meshes (`v = 3t`, identity indices). A `share_vertices` mesh is refused.
Texturing a shared mesh from several views would need a tile per primitive,
as the prior engine's Metal path kept (a per-triangle UV table the fragment
shader reads by primitive id), which is a gfx change. The single-camera pass
keeps its per-vertex verdict (2026-08-11), since one image has no tiles to
cross.

**How a view is chosen.** A view qualifies when it sees the triangle's front
and all three vertices are in front of it, inside its image and unoccluded,
by the single-camera pass's test. The front is the counter-clockwise side,
which marching cubes' outward normal leaves by and gfx draws. The score is the
cosine of the angle between the triangle's normal and the view's ray back
from its centroid, less 0.01 per metre of the smallest depth disagreement;
the highest wins, and the first view wins a tie. This is
implicit_surface_compression's `triangles_to_uv_multicam_kernel`
(`texture_mapping.cu`), which the prior engine's texture mapper also uses,
with the same default weight, except for the front. Both take `|cos|`, so a
view behind a triangle scores as well as one in front of it. On thin
geometry the back of a sheet sits within the 2 cm threshold of the front its
camera saw, passes the depth test, and took the front's image. Our winding is
consistent, so a view behind the triangle does not qualify. The facing test
runs first, since it needs no depth. A triangle with no area faces no view.

**The coordinates.** The vertex's pixel centre as a fraction of the depth
map, times the tile's size, clamped half a texel inside the tile, plus the
tile's origin, over the atlas size. Filtering never reaches a neighbouring
view. The fraction is the same fraction of a colour image registered to the
depth camera, when the depth intrinsics are a pixel-centre-preserving rescale
of the colour ones (`sensor::depth_from_registered_color`), so the tile is
the colour image at its own resolution. That is the single-camera pass's
argument, and the reference's separate low-resolution depth camera is the
same idea. A phone's 256x192 depth under a 1920x1440 capture textures at the
capture's resolution, with no upsampled depth. A `TextureView` whose image
size is zero takes its depth map's. With one view at the depth map's size the
coordinates match the single-camera pass to 1e-6.

**What is refused.** Before anything is written, as `InvalidArgument`: no
views, a null depth, an empty depth map or image, an image with one side
given and not the other, a depth range with `min_depth >= max_depth`, and a
layout that does not match the views, lies past the device's extent, or
has tiles outside the atlas or overlapping one another. The depth range
check exists because a camera converted from a `ColorCameraParams`, which has
no range, arrives with both zero, and then no sample counts: every triangle
took the vertex colour and the call returned OK. Overlapping tiles would let
`pack_atlas` overwrite one view's pixels with another's, so a triangle
assigned the first view sampled the second's image. `pack_atlas` checks the
same, plus a null output. `side_by_side_atlas` gives an atlas past the extent
`InvalidArgument` too, as every device limit this tier checks does.

**Its buffers.** The views' depth and the view table are grow-only members,
rewritten each call like the single-camera pass's camera buffer. The dispatch
is fence-waited, so nothing still reads them when the next call writes. A
four-camera 1080p rig texturing at 30 fps otherwise allocated and freed about
33 MB a frame.

**Verified.** `recon_texture_multiview` builds a wall seen by three cameras,
with depth ray-cast per camera. Each triangle takes the view worked out by
hand: head on, the squarer of two, and the other view when the best one is
occluded. A triangle behind every camera, with a vertex off every image, or
wound away from the cameras, gets the vertex colour, though camera 0's depth
agrees with the last one. The same triangle wound toward them is textured.
Every coordinate lands at its projection, inside its tile, in one row and
wrapped into two. A half-resolution depth map under a full-resolution tile
puts every vertex where the full-resolution depth does. It also checks one
view against the single-camera pass, `side_by_side_atlas` and `pack_atlas`,
each refusal, and a moved-from texturer's extent. Inverting the score,
testing one vertex instead of three, dropping the occlusion test, ignoring
the tile offset, taking `|cos|`, scaling to the depth map instead of the
tile, dropping the image size, the depth-range check, the overlap check or
the null check, never growing the depth buffer, or reading a moved-from
texturer's extent each fails it. `recon_texture_device_mesh` checks that the
device pass equals the host pass vertex for vertex on a sphere seen by two
views, and that it reports its `"texture"` row. It also checks that a shared
mesh and a superseded one are refused, the latter both over a live buffer
and over a freed one. Dropping the device overload's `is_current` check or
its stage scope fails it.

**Amended: floor(sqrt(n)) rows.** One row of a rig's four 4K views is a
15360 x 2160 atlas, near the 16384 `maxImageDimension2D` many devices allow,
and long and thin. So `side_by_side_atlas` now lays the views out in
floor(sqrt(n)) rows of ceil(n / rows): up to three stay in one row, four make
two rows of two (7680 x 4320 for four 4K views, the same pixels), up to eight
make two rows, and nine three. Not ceil(sqrt(n)) columns, which is squarer
but leaves a quarter of a three-view atlas empty, a third more pixels than
one row that fits. A row still ends early where the next image would pass
the extent. The texturer reads each tile's rectangle, so nothing else
changes. A layout left in one row, or one of ceil(sqrt(n)) columns, fails
the multi-view test.

**Open**, each a `TODO(texture)` at the code it names. The atlas is packed on
the host and uploaded by gfx; packing it on the GPU into an image gfx samples
directly needs `core` images. Each call copies every view's depth into the
buffer, so a keyframe set that does not change pays for it every call. Views
are not blended, so exposure differences show as seams between triangles
textured from different views. A shared mesh waits on a per-triangle tile
index in gfx.
### 2026-09-28 — GPU pre-processing is `recon_sensor_utils`: a driver hands out the frame as captured, the device undistorts depth and undistorts and converts colour, and fusion reads the buffers in place, depth and colour each with its own camera rather than registered.

*Amended 2026-10-06 (below):* raw frames no longer come through
`ICameraCapture`: `OrbbecSensor::poll` and `OrbbecRig::poll_set` hand them
out, and the examples fuse them with `fuse_set` alone.

**The rule.** `GpuFramePrep` (`sensor/utils/gpu_frame_prep.hpp`) takes a
`RawFrame` (`sensor/raw_frame.hpp`): raw depth, the decoded Y'CbCr 4:2:0
planes, and each camera's `LensCamera` (`sensor/lens.hpp`) and pose. Two
compute passes undistort depth, and undistort and convert colour; the result
is a `DeviceFrame` of storage buffers that `VoxelHashMap::allocate_from_depth`
and `TsdfIntegrator::integrate` read in place (their `Buffer` overloads, and
`ColorFrame::buffer`). Nothing registers depth to colour: each keeps its own
intrinsics and pose, and `tsdf` fuses a separate colour camera as it always
could. The `DeviceFrame` holds its buffers. A source hands raw frames out
through the capture contract, `ICameraCapture::poll_raw`, and says with
`raw_frames` that it does. The Orbbec driver is one when opened with
`OrbbecStreamOptions::raw` (H.265 colour only), and `fuse_orbbec --gpu` fuses
its frames.

**Why not registered.** The SDK can register depth to colour two ways, and
neither suits a 4K rig. Its software `Align` runs on the CPU, and at the
colour image's full size it made each fuse allocate over 8.3 M depth samples
where the sensor has 0.37 M (9.1 ms of host allocation at 4K against 1.3 ms
here). The camera's hardware registration sends depth at the colour size:
about 8.3 M x 2 B x 25 fps, 415 MB/s a camera at 4K, past gigabit Ethernet.
Fusing the two cameras as they are removes the step, and the depth resampling
with it. What it gives up is the registered case's colour visibility: a voxel
hidden from the colour camera but seen by depth takes the occluder's colour
(`tsdf`'s documented caveat), and the texture tier still wants a frame
registered to one camera.

**Placement.** A target of its own under `sensor/utils/`, because it carries
Vulkan and shaders and `recon_sensor` must not (the 2026-09-26 decision: an
out-of-tree driver compiles against it). `RawFrame` and `LensCamera` stay in
`recon_sensor`, so a driver produces them without a GPU API.
`LensDistortion` moved from `rig_calibration.hpp` to `lens.hpp`, beside the
host model the GLSL mirrors.

**The kernels.** Undistorting keeps each camera's intrinsics and drops its
lens, as the SDK's filter does: each pinhole pixel is sampled where OpenCV's
rational model (`distort_normalized`) images it, so only the forward model is
needed. Depth is sampled at the nearest pixel, since between a foreground and
a background return there is no surface to interpolate. Colour is sampled
bilinearly, luma and chroma each at its own siting (chroma with the even luma
columns, between two luma rows, H.265's default), and converted by the
picture's `kr`, `kb` and range in the same pass, so no 4K R'G'B' intermediate
is written. Only an encoding `is_canonical` accepts is converted. Where the
lens maps a pixel outside the captured image, depth is 0, "no return", and
colour a 0 word. Every other colour word carries 0xFF in its high byte, so
the high byte is the pixel's coverage, and `ColorFrame::coverage_in_alpha`
has fusion skip a pixel without it.

**The calibration** is the camera's, read at open: each stream profile's
intrinsics and lens, and `depth_profile->getExtrinsicTo(color_profile)` for
the depth camera's pose (`p_colour = R p_depth + t`, `rot` row-major,
millimetres). The calibration file's `pose` is still the colour camera's, so
it needs no new fields. On CL2A141000N the depth lens is the rational model
with all six radial terms (k1 29.29 ... k6 3.77), the colour lens plain
Brown-Conrady (k1 0.073, k2 -0.10, k3 0.04), and the depth camera sits at
(-32.5, -0.5, 2.2) mm in the colour camera's frame. The SDK's modified,
inverse and Kannala-Brandt models are refused.

**The Orbbec raw path.** The decoder thread decodes to `Yuv420` and posts the
planes as an SDK I420 frame through the same mailbox. It must not stamp the RGB
mode's stream profile on it, as the RGB path does: the profile restamps the
frame's format as RGB, which the first rig run caught. The frame carries the
matrix and range the decoder resolved in its metadata (`PlanesColor`, since
an SDK frame has no field for them), with the transfer and primaries the
stream declares. A labelled stream is therefore converted by its own labels
on both paths, and an unlabelled one as the Femto Mega's BT.601 full range. A
stream declaring a transfer or primaries `ColorEncoding` cannot name is
refused. `OrbbecRig` refuses `raw` for now.

**Why these numbers.** `fuse_orbbec` on CL2A141000N at 4K25, 300 frames,
Release, M-series Mac, each path's own run:

| per fused frame | host path (`--hevc`) | GPU path (`--gpu`) |
|---|---|---|
| fps | 24.4 | 24.9 (the camera's 25) |
| poll, host | 15.5 ms | 0.01 ms |
| frame prep | -- | 2.6 ms host, 0.89 ms device |
| allocate | 9.1 ms host, 5.5 ms device | 1.3 ms host, 0.76 ms device |
| integrate | 3.4 ms host, 0.46 ms device | 1.7 ms host, 0.58 ms device |
| CPU for the run, user + sys | 15.96 + 1.82 s | 0.94 + 1.23 s |
| peak resident | 1.43 GB | 1.05 GB |

The CPU saved is more than the poll row shows: the host path's decode thread
also converted every 4K picture to RGB through swscale. The frame prep's host
time is mostly copying 12 MB of planes into the upload buffer. (Since step 5a
of the residency decision below, into a batch's staging, and the copy up is
timed on the device.)

**Verified.** `recon_tsdf_device_input` fuses one frame from host arrays and
from buffers and compares every observed voxel. `recon_sensor_gpu_frame_prep`
holds both passes to a host reference (depth exact, colour within a code),
checks the lens is undone rather than applied (a checkerboard imaged through
an independently inverted lens comes back where it was drawn: 0 of 49 920
pixels off), anchors the matrix to a forward conversion, and fuses the
output. Breaking the lens direction, the matrix weights, the range, the
depth rounding or the chroma siting each fails it. On the camera,
`recon_sensor_orbbec_gpu_prep` captures a still scene through both paths.
At 720p every box of a 5 x 3 grid, corners included, matches the SDK's
undistorted colour best unshifted, 1.4-3.0 RMS apart. At 4K every textured
box does (1.9-8.2 RMS, a 2 px shift raising it two to four times). The pass's
depth, moved into the colour camera by the extrinsic, lies a median 5.1-6.0
mm from the SDK's registration, best within a pixel of unshifted. A reversed
extrinsic would shift it by about f * baseline / z, some 30 px there. Two 4K
meshes, one from each path at 2 cm voxels, sit a median 10 mm apart
vertex to vertex (half a voxel). 7-16% of vertices have no counterpart within
2 cm, unexplained: the scene changed between the runs.

**Review.** The PR's review changed eight things, each where the first cut
would have been silently wrong.
- **Colour labels.** The raw path converted every picture as unlabelled
  BT.601 full range, so a labelled stream fused other colours than the host
  path. The labels now travel with the planes, as above.
- **Coverage.** A pixel outside the picture was black and fused as colour.
  Since a voxel's first colour is assigned rather than blended, the view's
  edges fused black patches. The flag is opt-in because a host image's high
  byte is undefined.
- **The depth gate.** An unset range (`RawFrame`'s zeros) fused nothing, and
  a range from 0 fused every "no return" as a surface at the camera. The pass
  refuses anything but a finite `0 < min < max`.
- **Binding ranges.** The device-input overloads bound a caller's buffer at
  `VK_WHOLE_SIZE`, past `maxStorageBufferRange` for a large enough buffer.
  They bind the image's exact range now, through one `core` type,
  `StorageInput` (a host array or a device buffer, never both). Both tiers
  and the colour image bind through it, and its O(1) check runs before the
  compaction dispatch, so an empty grid refuses what a full one does.
- **Order.** `prepare` checks both halves of a frame before either is
  uploaded, so a refused colour picture costs no depth dispatch.
- **Lifetime.** A `DeviceFrame` was a borrowed view "valid until the next
  prepare". A kept one then silently fused the next frame's buffers with its
  own cameras. A generation stamp would need checking where the frame is
  consumed, and the fusion tiers take a `Buffer`, not a `DeviceFrame`.
  Ownership removes the staleness instead: the frame shares its buffers, and
  `prepare` reuses one only when no frame still holds it (the 2026-08-04
  rule, met by construction).
- **Device-local outputs.** They were host-visible, so on a discrete GPU the
  integrate kernel's reads would cross the bus. They are now `DeviceLocal`
  and `TRANSFER_SRC` (a test copies them out).
- **The contract.** `fuse_orbbec --gpu` reached into the concrete
  `OrbbecCapture`, against the 2026-09-14 rule. Raw frames now come through
  `ICameraCapture`, non-pure like `exhausted()`. The `DeviceFrame` overload
  of `fuse_frame` lives in `fuse_device_frame.hpp`, so the dataset examples
  do not link `recon_sensor_utils`.

The plain Brown-Conrady model also stopped reading k4..k6, which only the K6
model has. One suggestion was not taken: recording both passes in one submit
would save a fence wait a frame. It needed a several-kernel dispatch in
`core`, which `ExtractTimings`' device half wants too, so it waited for that.
`CommandBatch` is that dispatch, and step 5a of the residency decision below
has since put both passes in one submit. None of this has run on the camera
yet: the hardware test was not re-run after the review.

**Open.** The colour-occlusion fringe of an unregistered camera is
unmeasured, and the texture tier's separate colour camera is a later slice.
The planes are copied from the decoder. Zero-copy input from VideoToolbox or
NVDEC is a `TODO(sensor)`, and so are the rig's raw sets. Nothing has run on
an NVIDIA host with a camera. This PR's first test, 320 x 240 pixels of one flat wall, contended on
the hash map's bucket locks past the driver's 7 s watchdog in CI (Xid 109, and
a runner lost), while they were host-visible; PR #81 has since moved them into
device memory, the fix the fleet's first watchdog report called for.

*Amended 2026-09-29:* **the depth pass can keep depth to the colour camera's
view.** With `GpuFramePrepConfig::depth_within_color` set, a frame with colour
has a depth pixel zeroed ("no return") when its point, moved into the colour
camera by the two poses, lies behind that camera or lands on a pixel the colour
pass marks uncovered. The test uses the colour pass's own rounding and lens
test, so depth survives exactly where `DeviceFrame::color`'s coverage byte is
set. The option is off by default, and a frame without colour keeps all its
depth.

- **Why.** A Femto Mega's depth covers about 65 degrees vertically and its
  16:9 colour about 51. The depth outside the colour view fuses into surfaces
  no camera can colour or texture, and the mesher draws them white. On the
  lab rig that was about half the viewer's untextured triangles (the
  `rig_viewer` entry below).
- **Why here.** The frame prep is the one place that holds both cameras,
  their poses and the colour's coverage on the device. A zero is the "no
  return" that `allocate_from_depth` and `integrate` already skip, so neither
  fusion tier changes. The host Orbbec path needs no option: it registers
  depth to colour, so its depth already lies inside the colour view.
- **Not registration.** Depth keeps its own resolution and camera, as this
  entry chose. The mask costs each depth pixel one rigid transform and one
  lens evaluation, plus a 120-byte inline upload per frame. That device cost
  has not been measured apart from the pass.
- **The view, not the line of sight.** A point inside the colour view that a
  nearer surface hides from the colour camera is kept. That is `tsdf`'s
  documented colour-occlusion caveat, unchanged.

`recon_sensor_gpu_frame_prep` checks three cases:

- a pinhole depth camera behind a pincushion colour lens, with the same
  intrinsics and pose: depth is zeroed exactly where the colour's coverage
  byte is 0 (63 807 pixels kept, 12 993 zeroed, 0 wrong);
- a colour camera 1.5 times narrower and 10 cm to the side: the depth kept
  is the region a host projection puts inside its image;
- the option off, or a frame without colour: nothing is zeroed.

Each of three mutations fails the test: ignoring the pose, skipping the lens
coverage test, and ignoring the flag. The test passes under the Khronos
validation layer with no messages.

### 2026-09-28 — Memory the kernels use lives on the device on every platform, and the host only records commands against it: one `CommandBatch` per call, parameters inline, bulk bytes staged at the edges, small results read back.

*Amended 2026-10-13 (the volume block-list entry, below):* `diagnostics()`
is gone. Every allocate and remove round reads the device's heap counter
into the copy `load_factor()` returns, and `recon_volume_block_grid` and
`recon_volume_resize` check that copy against the blocks the map holds after
the calls that move it, a rolled-back `resize` included.

**The rule.** A buffer the kernels read or write is device-local
(`device_storage_buffer`: VRAM, never mapped), on Apple as on NVIDIA.
Host-visible memory (`storage_buffer`) is for what the host produces or
consumes: staging, readback, small parameters.

The host reaches device memory through a `CommandBatch`. A batch records
one call's `upload`, `fill`, `copy`, `dispatch`, `dispatch_indirect` and
`readback` into one command buffer, and submits it once with one fence wait.
In the loop the host sends only a frame's inputs and reads back only small
results: block counts, failure tallies.

**Why: the RTX 5090 measurement.** Every buffer except the bucket locks was
host-visible on main, so every kernel access crossed PCIe. A throwaway
allocator patch moved them into device-local host-visible memory (resizable
BAR, memory type 4 on the 5090, the whole 31.8 GiB). `fuse_replica` ran room0
with `--preload --device-extract`, 400 frames, Release, twice per
configuration, on an idle GPU. Per fused frame:

| | main | hash + voxels in VRAM | + every buffer ≥ 64 KB | M5 Max |
|---|---|---|---|---|
| `integrate`, device | 14.6 ms | 5.3 ms | **0.067 ms** | 0.12 ms |
| `allocate`, device | 2.6 ms | 0.35 ms | 0.32 ms | 0.43 ms |
| extract dispatch | 54 ms | 62 ms | **4.7 ms** | 0.67 ms |
| fused fps | 43 | 84 | — | 612–640 |

Four findings shaped the design:

- **Size is what matters.** Moving only the buffers under 64 KB changed
  nothing: `integrate` stayed at 5.4 ms and the extract at 72 ms. Those
  buffers are the parameters, the marching-cubes tables, the draw command
  with its per-block atomics, and the counters. The large ones carry the
  cost: the voxel arrays, the hash table, the depth and colour frames, the
  arena and the block lists. So small parameters stay host-visible.
- **The CPU must not read VRAM directly.** With the arena in BAR memory,
  `extract_host`'s download took 6.6 s, and the per-frame compaction readback
  rose from 1.4 to 5–9 ms. So device memory is never mapped, and a readback
  always copies into cached host memory. The last column above has no fps
  for that reason.
- **Apple is unaffected by placement.** On the M5 Max every configuration
  read within noise of main: `integrate` at 0.12 ms of device time in all
  four. So residency costs Apple nothing but the staging copies, which is
  what the next section weighs.
- **Host overhead remains.** Once resident, `integrate`'s host row on the
  5090 is still about 9 ms against 0.07 ms on the device, where the Mac's is
  0.59 ms. That points at per-call allocations, the active-list round trip
  and the fence-waited submits, which is what the batch and the tiers moving
  onto it remove.

**The batch.**

- **Order.** Commands run in the order recorded. A barrier makes shader and
  transfer writes visible to the next command's reads and writes, indirect
  reads included, wherever a command could see an earlier one's writes:
  around every dispatch, and between two transfers only when they share a
  buffer one writes. A fill or inline upload that starts past the end of the
  one before it in the same buffer needs none either: the run's first write
  there was checked against the rest, so zeroing thousands of scattered
  blocks is one run, with no scan of it per fill. Two readbacks never need
  one, since each lands in its own slice of the batch's buffer; a pipeline
  drain between them would be a wait-for-idle on NVIDIA. The last barrier
  also reaches the host and a
  renderer drawing the result, as far as the queue family allows.
  `dispatch()` is a batch of one dispatch, so that scope, the dispatch
  checks, the label and the span each have one definition. Each dispatch
  keeps its debug-utils region, outside its `GpuStageScope` span, and every
  timer a batch used is resolved after the one wait.
- **Bound at submit.** A kernel's descriptor set is bound when `submit`
  records, so a set rewritten after its dispatch was recorded would run that
  dispatch on the later binding. `DescriptorSet` counts its writes, and
  `submit` refuses a batch whose kernel's count has moved. A push is checked
  against the kernel's range (`ComputeKernel::push_bytes`, which
  `KernelSetBuilder::add` records and holds to offset 0).
- **A failed wait leaks the staging.** `submit_single_time` leaks its command
  buffer when the fence wait fails, since the device may still run it, and
  says so through `in_flight`. The batch then leaks its staging too and
  retires the spans' timers (`GpuTimer::abandon`). A submit that fails before
  the device has the buffer only drops its spans (`GpuTimer::discard`), so a
  transient failure does not turn a tier's timing off for good. On success
  the staging is freed at the end of `submit`, not when the batch goes.
- **One path each way.**
  - **Uploads.** One of up to 64 KiB, 4-byte aligned (a frame's parameters)
    is written inline in the command buffer with `vkCmdUpdateBuffer`, so no
    buffer is mapped for it. A larger one, such as a depth frame, is staged
    through a host-visible buffer the batch allocates and frees.
  - **Readbacks** are copied into one small host buffer allocated at
    `submit`.

  Nothing is read or written through a mapping of the destination, so a
  buffer's memory type never changes what a batch does.
- **No staging arena.** Staging happens only at the edges. VMA carves small
  buffers out of blocks it keeps, so a staging buffer per call costs
  microseconds. The first cut kept a persistent, grow-only arena per tier
  object and dropped it before merge. Its saving was unmeasured, and its
  cost was an object every tier had to own and a one-batch-at-a-time rule
  its own test tripped over. Revisit if a tier's host rows show the
  allocations.
- **Usage is checked on every buffer**, host-visible ones included:
  `TRANSFER_DST` for an upload, `TRANSFER_SRC` for a readback. Every path
  copies, so every buffer needs the bits, and moving one into device memory
  later cannot turn up a missing bit.
- **A refused call poisons the batch.** `submit` returns the first refusal and
  runs nothing.
- **`dispatch_indirect`** is here now because the tiers need it next. A count
  a kernel writes on the device can size the next dispatch without reaching
  the host, which is how the active-list round trip goes.

**No mappable device memory, deliberately.** The first cut had a third
kind, device memory mapped where it is also CPU-cached, i.e. unified memory,
so that Apple could skip staging. It was dropped before merge:

- On NVIDIA it was plain VRAM, identical to `DeviceLocal`.
- On Apple it saved only the staging copies. Those are a few bytes for a
  counter and a few MB for an input frame. The largest is an export such as
  the mesh download, which can be streamed through a bounded readback in
  chunks. None of it is measured.
- What it cost was a Mac taking a different path from NVIDIA. The staged path
  NVIDIA runs would then have gone unexercised where the code is developed,
  the same invisible-where-tested class as the host-visible locks.

With plain device memory, a Mac run is the NVIDIA path. Revisit it with a
measurement of staging on the memory-bound iPad. *Revisited 2026-10-24* (the
one-batch frame entry): a frame's ring slots run in place where
device-mapped memory is cached. The first cut's shortcuts
through a mapped destination went with it, for the same reason and because
nothing in the loop is mapped: uploads target device memory, and readbacks
are small counts.

`device_storage_buffer` now adds `TRANSFER_SRC | TRANSFER_DST` usage and takes
extra usage and queue families, so a batch can fill, copy and stage through
anything it makes, and the mesh arena can use it. #99's
`tests/buffer_readback.hpp` reads back through a batch, so the sensor tests
exercise it on every CI leg.

**Verified.** `recon_core_command_batch` runs every call over a device-local
and a host-visible buffer with the same expectations. It runs under the
Khronos validation layer wherever that is installed (this Mac and the Linux CI
images), and fails on any error the layer reports.

- **Orders that would expose a command run out of place:** an upload recorded
  after a dispatch into the same buffer, and a readback recorded before one.
- **The rest:** inline, staged (past 64 KiB) and unaligned staged uploads;
  several readbacks of odd sizes in one batch; fills and copies at offsets;
  an indirect dispatch sized by an uploaded command; transfers on different
  buffers without a barrier between them; two timed dispatches in one submit,
  each publishing its own row; each refusal, with its batch poisoned; a set
  rewritten before submit, a push off 4 bytes or past the kernel's range, a
  moved-from device and a batch with no allocator refused; and the moves.

The whole suite passes on both machines, 41 of 41, the 5090's build with GCC
13 at `-O3 -Werror` in the CI image. Each of these fails the test:

- an inline upload past 64 KiB or off 4-byte alignment. Only the validation
  layer sees this one; MoltenVK runs it anyway.
- dropping an inline upload's offset, or a staged copy's source offset;
- readbacks sharing one slice;
- skipping either usage check;
- submitting a poisoned batch;
- not resolving the timers;
- dropping the overlap check;
- ignoring the indirect command;
- skipping the rewritten-set check or the push check, not recording a span,
  or a move that leaves its source live (each checked on the Mac).

Dropping the barriers between commands fails it too, 10 runs of 10, but only
on the 5090: MoltenVK orders the same work without them. So CI's NVIDIA legs
are what hold the barriers, the ones now left out between independent
transfers included.

**Open.** The review behind this entry ranked what moves, by what the
measurement says it costs on the 5090:

1. `volume`: the hash table, its counters and the voxel arrays resident, with
   GPU fill and copy for create, clear, resize and remove, and the compacted
   list kept on the device.
2. `tsdf`: `integrate` on that list by indirect dispatch in one submit, the
   dirty flags resident, and `MeshIntegrator`'s slot counts, whose
   host-visible atomics cost 115 + 143 ms (the measured lesson below).
3. `mesh`: the arena and index run resident, with `download` staged.
4. `texture`: depth read from a device buffer.
5. `sensor`: the frame-prep outputs on a ring, NV12 decoded straight into
   staging, then NVDEC to Vulkan device to device.
6. The examples.
7. `codec`: coefficients read back as filtered int16.

**Step 1, `volume`, has landed** but for the compacted list, which still
reaches the host and goes with step 2's indirect dispatch. Each allocating or
removing call is one batch per round: its inputs uploaded to device buffers in
the first, the tally and the heap counter read back in every one. A compaction
reads back its count and, in the same submit, the list as far as a quarter
past its last count; only a set that outgrew that takes a second submit.
`load_factor()` reads a host copy of the heap counter, so it still costs no
dispatch, and `diagnostics()` reads the device's. `resize` rebuilds the heap
on the host and uploads it, a `TODO(volume)`. On the 5090, room0 as above,
per fused frame against main:

| | main | resident |
|---|---|---|
| `integrate`, host / device | 17.6 / 14.6 ms | 7.8 / 5.4 ms |
| `allocate`, host / device | 4.1 / 2.6 ms | 1.3 / 0.35 ms |
| extract dispatch | 54 ms | 69 ms |
| fused fps | 43 | 91 |

That is the A/B's "hash + voxels in VRAM" column. The mesh is the same 330 389
triangles. The extract dispatch is slower, as it was in the A/B, and waits on
step 3's arena.

**The review of step 1** measured three of its findings on a grid of 16 384
blocks, Release:

| | 5090 | M5 Max |
|---|---|---|
| `VoxelBlockGrid::remove`, 2 048 scattered blocks | 7.6 → 4.3 ms | 17.4 → 5.2 ms |
| `compact_active_blocks`, again on one grid | 1.11 → 0.66 ms | 0.30 → 0.17 ms |
| a round's 28 B readback buffer | < 1 µs | 2 µs |

- `remove` zeroed each run of freed blocks with a fill and a barrier after
  it, and the barriers were the cost. It now zeroes attribute by attribute,
  so each array's fills rise through it as one run, and `CommandBatch::zero`
  takes the unaligned edges the grid used to handle.
- A compaction took two submits, one for the count and one for the list, at
  0.38 ms each on the 5090 for an empty one.
- The buffer each batch allocates for its readbacks stays. A persistent one
  would save under a microsecond a round.
- `clear` freed the indices before zeroing, so a failed zeroing left the heap
  handing them back over the old surface. It zeroes first now, as `remove`
  does.
- `diagnostics` read the host copy of the heap counter, so it could not catch
  that copy stale. It reads the device's now, and the tests compare the two
  after every call that moves the counter, a rolled-back `resize` included.
- The per-call inputs (coordinates, a mesh, the rehash snapshot) were still
  host-visible. They are device buffers the first round uploads into.

**Step 2, `tsdf`, has landed, without the indirect dispatch.** The compaction
leaves its list on the device (`compact_active_blocks_on_device`) and reads
back only the count, which sizes the dispatch on the host, as the plan's
"small results read back" allows. So a fuse is two submits: the compaction,
then one batch that stages the frames, writes the cameras inline and
dispatches. `StorageInput` stages every host frame this way, `allocate`'s
too. The dirty flags, which the kernel ORs per changed voxel, and
`MeshIntegrator`'s per-slot counters are device-local; the counters are read
back once, between the count and fill passes. Indirect dispatch would save
the one fence wait between the submits. Revisit it if a host row shows that
wait.

| room0, per fused frame | RTX 5090, step 1 | RTX 5090, step 2 | M5 Max, main | M5 Max, step 1 | M5 Max, step 2 |
|---|---|---|---|---|---|
| `integrate`, host / device | 7.8 / 5.4 ms | 1.9 / 0.033 ms | 0.64 / 0.124 ms | 0.81 / 0.123 ms | 0.68 / 0.118 ms |
| `allocate`, host / device | 1.3 / 0.35 ms | 1.3 / 0.29 ms | 0.78 / 0.43 ms | 0.79 / 0.42 ms | 0.80 / 0.43 ms |
| fused fps | 91 | 185–199 | ~592 | ~535 | ~573 |

The 5090's device `integrate` is under the A/B's 0.067 ms. The sheet test
(`recon_tsdf_mesh_integrate`) ran 1.29 s on the 5090 at step 1 and 0.87 s here.
Apple is not quite unaffected: step 1's extra submits cost the Mac 10% of its
frame rate, and this step wins most of that back, leaving it 3% under main.
The rest sits in the host rows, 0.04 ms on `integrate` and 0.02 ms on
`allocate`: the two calls that now stage a host depth frame, which a device
frame from `GpuFramePrep` skips. Not yet split further.

**The review of step 2** measured its `MeshIntegrator` findings on the
81 920-triangle sphere (Release, medians of ten), and the counter it tried on
room0:

| | 5090 | M5 Max |
|---|---|---|
| one submission per split, 16 384 slots | 7.7 → 8.4 ms | 12.4 → 12.6 ms |
| counts read back without coordinates, 393 216 slots | 11.6 → 10.1 ms | 14.4 → 14.4 ms |
| a dirty counter in the kernel, `integrate` device | | 0.12 → 0.15 ms |

- `MeshIntegrator` had folded its split write into one submission, so
  `kMaxDispatchBinEntries` no longer bounded one. Each dispatch is submitted
  on its own again, the fill and uploads riding the first, and `dispatches`
  is counted at the submit. The first row is its price.
- Its count pass read back each slot's coordinate beside its count. The
  integrate pass reads them on the device, so 4 B a slot comes back, not 16.
- `reset_dirty` un-anchored before it cleared, so the next fuse cleared the
  flags again, and a failed clear left them un-anchored but set. It clears
  first, and a fuse clears only flags anchored to another grid. It is
  `[[nodiscard]]`, since it used to return nothing.
- `DeviceBlockList` was a bare handle. It carries the epoch and a compaction
  serial, and `check_device_block_list` refuses it after a compaction,
  resize, remove, clear or move (the 2026-08-04 rule).
- Not changed: `dirty_block_count` reads the flags back. A counter the kernel
  bumps per newly set flag cost every tracked fuse the last row, the
  incremental extract's included, which never reads it. A `TODO(tsdf)` holds
  an on-demand count.
- Not changed: a host frame is staged per call. Staging it once for both
  `allocate` and `integrate` takes a submit of its own, which costs more than
  the 0.06 ms both stagings take on the Mac.
- The deferred work is greppable again: `TODO(volume)` for the extract's list,
  `TODO(tsdf)` for the indirect dispatch.

**Step 3, `mesh`, has landed.** The arena, the index run, the draw command
and the tables are device-local. Each extract attempt is one batch: the
active list staged, the command reset inline, the dispatch, then the 32-byte
command and its scratch words read back. A refit is a second batch. The
identity index run of the unshared kernel is filled once per grow, in a
submit of its own before the run is committed. `download` copies back through
a batch per buffer. The span table stays host-visible:
the host is its reader (`block_spans()` hands out a pointer), and the kernel
writes it once per block.

| room0 | RTX 5090, step 2 | RTX 5090, step 3 | M5 Max, step 2 | M5 Max, step 3 |
|---|---|---|---|---|
| extract dispatch | 76 ms | 0.97 ms | 0.70 ms | 0.73 ms |
| remesh, whole extract | 78.9 ms | 2.6 ms | 1.8 ms | 1.8 ms |
| `download` (991 k vertices) | 21 ms | 31 ms | 6.5 ms | 9.8 ms |
| fused fps (8 remeshes) | 179–190 | 268 | ~627 | ~618 |

The dispatch is the A/B's 4.7 ms and better: the arena and the index run were
the buffers it was waiting on. `download` is the cost. It copies twice now,
into a readback buffer the batch allocates and then into the `Mesh`, and it is
paid once per host extract, and it is not optimized: the host copy is for
export (PLY, the codec's mesh-to-mesh eval, tests), never for online capture,
where gfx draws these buffers in place (seam B). The same mesh, 330 389
triangles, comes out.

The rows moved with the batch. Step 3's `dispatch_ms` also carries the active
list's copy, the command reset and the command readback, which ride its
submit, so the table understates the gain. `readback_ms` on `extract_device`
now reads near zero.

**The review of step 3** changed:

- The unshared identity is submitted on its own, before the run is
  committed. Recorded into the attempt's batch, a batch that failed after the
  commit left the run uninitialized, and no later extract refilled it.
- Every failed attempt disarms the command, a refused dispatch or readback
  included, since the reset was only recorded.
- An empty extract resets the command only when the slot's is not already
  empty, so a run of them costs one submit rather than one each.
- `download` reads back a batch per buffer, so no staging buffer is larger
  than a device buffer already allocated.
- `dispatch()` has closed with the batch's barrier, transfers included, since
  the `CommandBatch` review, so `download`'s copy is ordered after texturing.
- Tests: the unshared run read off the device after a grow, the empty
  command read back across empty, real and empty extracts, and an
  `extra_indirect_usage` bit no buffer carries otherwise.

**Step 4, `texture`, has landed.** The single-camera `DeviceMesh` pass takes
its depth as a device `Buffer` too, bound in place. That does not yet keep
the online chain on the device: the atlas must be registered to the depth
camera, and a `GpuFramePrep` frame's colour keeps its own camera, so its
colour is textured only once the pass projects into a colour camera (a
`TODO(texture)`). Every pass is one batch: a host depth frame staged, the
camera inline, the dispatch, and for a host `Mesh` the vertices staged up and
read back. The camera and the several-view buffers are device-local; a view's
depth is still a host array, staged per call (a `TODO(texture)`). Measured
on room0's final mesh (991 k vertices) and last frame, Release:

| texture pass, host / device | RTX 5090, step 3 | RTX 5090, step 4 | M5 Max, step 3 | M5 Max, step 4 |
|---|---|---|---|---|
| `DeviceMesh`, host depth | 1.31 / 0.50 ms | 0.82 / 0.017 ms | 0.51 / 0.25 ms | 0.49 / 0.23 ms |
| `DeviceMesh`, device depth | — | 0.58 / 0.017 ms | — | 0.39 / 0.22 ms |
| host `Mesh` (export) | 28.2 / 10.8 ms | 33.9 / 0.019 ms | 4.2 / 0.39 ms | 9.8 / 0.20 ms |

The host `Mesh` pass is the export path, `fuse_render`'s, and it costs more
host time for the same reason `download` does: the vertices now go up and
come back through staging copies. The live pass, `fuse_viewer`'s, is the
`DeviceMesh` one.

**The review of step 4** changed:

- The docs no longer offer the device-depth pass for a `GpuFramePrep` frame,
  whose colour is not registered to its depth.
- A host `Mesh` reads its vertices back in a batch of their own, as
  `download` does, so the upload's staging is freed first: two copies of the
  vertices at a time, not three. Reading back only `uv0` would need a pass
  that packs it; the export path does not pay for one.
- The depth copy is a grow-only member, as the views' buffers are, so the
  live pass allocates only its staging once the frame fits.
  `StorageInput::buffer` reuses the buffer it is handed when it fits.
- Staged uploads rising through one buffer share a barrier, as fills and
  inline uploads do, so a rig's views no longer take one between each pair.
- The three single-camera overloads share one recording helper, as the
  several-view pair already did, which names the depth copy for all of them;
  the export paths name their vertex copy again.
- The test's claim that it pins the depth binding's exact range was dropped:
  the kernel reads within the image either way, and only a buffer past
  `maxStorageBufferRange` would tell the two apart.
- Tests: staged uploads at rising offsets and one going back over them, and a
  larger frame after smaller ones, which fails when the depth copy is not
  regrown.
- Unchanged: a caller-owned buffer a batch records still dies at the call's
  return when the fence wait fails, as `CommandBatch` documents; the device is
  then treated as lost, as it already is in `tsdf`.

**Step 5a, `GpuFramePrep`, has landed.** The raw depth and colour planes go
up through one batch into device-local inputs, and the depth and colour
passes share its submit. The three planes are packed into one staging buffer
(`CommandBatch::reserve_upload`), whatever the decoder's strides, and copied
up as one command. The copies are timed with the passes, so the
`"frame prep"` row's device half counts moving the frame. Measured on a
640x576 depth frame and a 3840x2160 colour frame, Release, per frame (host /
device):

| | RTX 5090 | M5 Max |
|---|---|---|
| step 4 | 2.18 / 0.495 ms | 0.87 / 0.32 ms |
| step 5a | 1.52 / 0.52 ms | 0.77 / 0.33 ms |

The device column compares like with like: in step 4 the kernels read the
planes across the bus inside their spans, and in 5a the copy's span carries
that transfer, 0.47 ms of the 5090's 0.52 and about 0.04 ms of the Mac's. So
the device time did not move. The win is the host's, one fence wait a frame.

On unified memory the input is now held twice while the batch runs: the
device-local input and the staging, about 13 MB each at 4K, the staging freed
when the submit returns. The M5 Max's time did not move. The iPad, which is
memory-bound, is not measured.

**The review of step 5a** changed:

- The first cut timed only the dispatches, so its 0.055 ms against step 4's
  0.495 set the passes alone against passes that read across the bus.
  `upload` and `reserve_upload` now take the stage's scope. The spans cost
  nothing measurable on either machine.
- A padded plane was packed into a vector and then copied again into the
  staging, and each plane was an upload of its own: three staging buffers,
  and a barrier between copies that never overlap. The planes now go
  straight into one staging buffer. A tight plane is one `memcpy`, since
  copying it row by row into write-combined memory cost the 5090 0.2 ms.
- The raw inputs are named for a capture (`sensor.raw_depth`,
  `sensor.raw_color`).
- The reference test also runs at 321x241 with padded rows full of junk, so
  the rounded plane offsets and the packing are both held to the reference.

One suggestion was not taken. The two passes share no buffer, yet a barrier
still separates them, because the batch cannot see a kernel's bindings. The
depth pass it waits on takes 0.003 ms on the 5090 and 0.008 ms on the Mac.

**Step 5b, the rig's raw sets, has landed.** `OrbbecRig::poll_raw_set`
hands out a set of `RawFrame`s, views into the SDK's frames, and
`prepare_set` prepares a set with one `GpuFramePrep` per camera, each on a
thread of its own and the last on the caller's. Receiving and decoding were
already a thread per camera. This needed `Device` to take submits from
several threads, which it now does (the entry below).

The first cut was slower in parallel than in turn on the 5090. Each pass
had its batch allocate a 13 MB staging buffer per frame, and four at once
made VMA allocate and free a block for every set, under its lock: 39 ms a
frame, against 0.003 ms one pass at a time. This is the revisit the "no
staging arena" bullet above asked for, and it is kept to the one caller that
needs it: `GpuFramePrep` keeps a staging buffer, grown to the largest frame,
writes the frame into it, and copies it up in its batch (`copy` now takes the
stage's scope, as `upload` does). On unified memory the frame is therefore
held twice for good, not only while the batch runs: about 13 MB more a
camera at 4K. Measured on four 640x576 depth and 3840x2160 colour frames,
Release, per set:

| | RTX 5090 | M5 Max |
|---|---|---|
| one camera at a time, per-call staging | 7.6 ms | 2.5 ms |
| `prepare_set`, per-call staging | 46 ms | 1.78 ms |
| one camera at a time, kept staging | 7.6 ms | 2.4 ms |
| `prepare_set`, kept staging | 4.8 ms | 1.73 ms |

What is left of the 5090's set is the frame's copy into staging (1.1 ms a
camera with four at once, against 0.6 ms alone) and `submit_single_time`'s
fence and command buffer, made and freed per submit under the device's one
lock (about 0.7 ms a submit). Reusing those is the `TODO(core)` below, now
measured. The live rig fused 403 frames at 73.7 fps on the Mac, the
`"frame prep"` row 0.55 ms of host time a frame.

The frame prep's outputs stay off a ring. A ring would have spared a frame
kept past the next its new 35 MB output at 4K, but measured on the 5090 with
one camera, keeping the previous frame prepared in 0.91 ms against 1.04 ms
dropped, since VMA carves the new buffer from a block it holds. The M5 Max
was within its noise. There is nothing for a ring to win, so its
`TODO(sensor)` is gone.

Next is 5c, the decoder's planes straight to the device. It needs its own
design: CUDA or VideoToolbox memory shared with Vulkan.

`dispatch()` is unchanged. `submit_single_time` still allocates a command
buffer and a fence per submit; reusing them is a `TODO(core)`, measured by
step 5b at about 0.7 ms a submit on the 5090.

**The review of step 5b** changed:

- A rig opened raw kept `raw_frames()` false and refused `poll()`, so a loop
  written to the contract got no frames at all. It now says `raw_frames()`
  and hands out `poll_raw()`, the raw sets one frame at a time, as `poll()`
  does the host ones. The two set types are one template,
  `OrbbecRigSet<Frame>`.
- `prepare_set` joins its threads on every way out, and a thread's
  `bad_alloc` becomes a `Status`: either exception ended the process.
- A failed submit lets go of the pass's staging rather than rewriting or
  freeing it under a copy that may still run, as the batch does its own.
  The staging is named (`sensor.raw_staging`), made by the inputs' helper.
- `fuse_orbbec --rig --gpu` stops at `--frames` mid-set, reports progress,
  and a set with no frame no longer resets the silence limit.
- The rig's hardware test holds each raw depth frame to its own camera's
  pose through a fixed extrinsic, and each frame to the trigger's clock. It
  prepares every set, incomplete ones too, since the lab rig's rarely are.

The adopted device that submitted unlocked is fixed by the entry below.
Two suggestions were not taken. A staging pool in `CommandBatch` for every
caller: only `prepare_set` stages from several threads at once, so the "no
staging arena" bullet stands for the rest. A persistent worker per pass:
creating and joining three threads costs 0.027 ms a set on the M5 Max, 1.6%
of the set.

**Verified.** The Mac suite passes. On the lab rig, the raw section passes,
the MJPEG and H.265 sections skipped: their completeness and loss checks
fail on this rig, on the base too. `fuse_orbbec --rig --gpu --frames 400`
fused 400 frames at 47.2 fps (Release).

### 2026-09-28 — A `Device` takes submits from several threads at once: each records on a command pool of its own, and only the queue is locked.

Step 5b prepares the rig's cameras on the GPU in parallel, a thread per
camera, and two threads batching on one `Device` were undefined. Every submit
allocated, recorded and freed its command buffer on the device's one
`VkCommandPool`, which Vulkan requires be externally synchronized, and a
created device called `vkQueueSubmit` with no lock. Four threads of fifty
batches on one device failed 3 of 3 runs under the validation layer:
`THREADING ERROR` on the pool, and one segfault.

**Each submit takes a command buffer no other submit holds**, on a pool of
its own, from a free list the device keeps. When every one is in use it makes
another, so the list grows to the most submits ever in flight at once, and
recording takes no lock. The buffer goes back once its wait is done and is
begun again by the next submit, which the pool's `RESET_COMMAND_BUFFER` flag
allows. A failed wait leaves it to the device, as before, and `destroy()`
frees every pool made, that one included.

**The fence is kept with its buffer**, reset after its wait, which
closes the `TODO(core)` the entry above left. On the 5090 the objects were the
cost, not the submit: `vkCreateFence` took 0.29 ms and destroying the fence
with the buffer's free 0.39 ms, against 0.005 ms for `vkQueueSubmit`. On the
M5 Max each was about a microsecond. Measured on room0 with
`fuse_replica --preload --device-extract`, Release, four runs each, the same
330 389 triangles either way:

| RTX 5090, per fused frame | fence per submit | fence kept |
|---|---|---|
| `allocate`, host | 1.27–1.42 ms | 0.83–0.89 ms |
| `integrate`, host | 1.82–2.06 ms | 0.95–1.06 ms |
| 400 frames | 258–286 fps | 431–463 fps |

Its review settled the failure paths, where a kept fence can outlive a
submit the device may still run:

- `destroy()` waits for a command a failed wait left to the device before
  freeing it, and leaks it if the device may still run it. The old code
  leaked the fence, but freed the pool, whose buffer might still be running.
- A submit that loses the device is handled as a failed wait. `vkQueueSubmit`
  promises that the fence and buffer are untouched only when it fails for
  memory.
- The fence is reset after its wait, the one place it is known signalled, so
  every free fence is unsignalled. One that will not reset is not given back.
- A `record` that throws gives its buffer back reset, since the next begin
  may not find it recording. `take_command` grows both lists before making
  anything, under the lock, so a throw there leaks nothing and `give_back`
  never allocates.

**Only the queue is locked**: under the embedder's mutex on a queue shared
with another library, else under the device's own, so `submit_mutex()` is
never null. It is not held across the wait. This amends the 2026-07-05
dispatch entry: "shared-queue-safe" now holds for every queue.

**What a caller records stays its own**: a kernel's descriptor set, a buffer,
a `GpuTimer`. The `Allocator` a batch stages through may be shared, because
VMA locks its own state; it must stay created without
`VMA_ALLOCATOR_CREATE_EXTERNALLY_SYNCHRONIZED_BIT`.

`Device::command_pool()` is gone. There is no one pool to hand out, and a
caller recording on it would race the device's submits.

**The review of the first cut** changed:

- The first cut locked the one pool from allocate to end of recording, and a
  created device used that mutex for its queue too. An adopted device given
  no embedder mutex still submitted unlocked. That is `fuse_viewer`'s layout
  whenever its queue is not shared, so the race remained on the path the
  viewer runs. The test now runs on such a device too, and fails 3 of 3 when
  only its queue is left unlocked.
- `record` ran under the pool lock, so a callback that submitted on the same
  device deadlocked, and one thread's submit held up every other thread's
  recording. Neither can happen now, since no lock is held while recording.
- The test also stages an upload past the inline limit, times it on a timer
  per thread, and prints each failure's thread, round and first refusal.

Sharing one pool across threads, as a mutant, fails the test 3 of 3 with
`THREADING ERROR` on the pool and a segfault; dropping the queue lock fails it
3 of 3 on the queue. With neither, it passes 20 of 20 on the M5 Max.

### 2026-09-28 — A decoded colour frame never leaves the GPU: the platform's hardware decoder, for H.265 and MJPEG alike, hands its picture to Vulkan in place, and `GpuFramePrep` takes I420 or NV12 from the host or the device.

*Amended 2026-10-06 (below):* the MJPG mode's calibration is no longer
compared with the RGB mode's; a frame's colour camera is read off the mode
streamed.
*Amended 2026-10-08 (below):* `place_device_color` goes: a decoded picture
carries its `YuvImage`, which a frame takes as it is.
*Amended 2026-10-06 (below):* the software decoding and fallbacks this entry
kept are gone, with nvJPEG's `GPU_HYBRID` back end: every picture is a
device picture, and a device path that fails is an error.
*Amended again the same day (below):* `GpuFramePrep` takes colour from the
device only.

**The rule.** The camera stream arrives on the host, the RGB-D frame goes to
the GPU once, and undistortion, colour conversion and fusion all run there.
Depth already did: raw 16-bit samples, one upload. Colour did not. A
hardware-decoded picture came back to the host (`av_hwframe_transfer_data`),
swscale converted it from NV12 to I420, the Orbbec driver copied it into a
new 12 MB buffer, `GpuFramePrep` copied it into staging, and the batch
uploaded it again. Per 4K picture per camera, Release, that was:

| | M5 Max | RTX 5090 box |
|---|---|---|
| copy to host | 0.20-0.34 ms | 1.1 ms (3.9 with a fresh driver buffer) |
| NV12 to I420 | 0.31-0.46 ms | 0.6 ms |
| driver copy (`i420_frame`) | 0.18-0.28 ms | 0.57 ms (3.8 fresh) |
| copy into staging | 0.2 ms | 0.6 ms |
| upload, device time | 0.04 ms | 0.47 ms |

On Linux a fresh 12 MB buffer a frame page-faults, which is what the driver
does today; the bench that measured it decoded a 3840x2160 clip through
`HevcDecoder` and repeated the driver's copy.

Instead, each platform's hardware decoder hands its picture to Vulkan where it
lies:

- **NVIDIA.** NVDEC, for H.265, copies its surface device to device
  (`cuMemcpy2DAsync`) into a ring of Vulkan buffers imported into CUDA
  (`VK_KHR_external_memory_fd`). Its pool is small, so the surface is copied
  out at once rather than held. nvJPEG's hardware back end decodes MJPEG
  straight into the same buffers, and `GPU_HYBRID` stands in on a GPU without
  the JPEG engine. Both sit behind `VR_WITH_CUDA`, on the CUDA 13 toolkit.
- **Apple.** VideoToolbox, for H.265 through FFmpeg and for JPEG directly,
  gives an IOSurface-backed NV12 `CVPixelBuffer`. Each plane becomes a Metal
  texture over the IOSurface, imported with `VK_EXT_metal_objects`, and the
  pass copies the planes into its input in its own batch.
- **`GpuFramePrep`** takes I420 or NV12, as host planes or as a device
  buffer with per-plane offsets and strides (`YuvImage::device`), and binds
  device planes where they are. The frame holds that buffer, so a decoder's
  ring cannot reuse it while the frame is prepared.

**Why each piece**, measured on throwaway spikes:

- **MoltenVK 1.4.2 refuses an IOSurface as one two-plane image**: `IOSurface
  bytes per element 1 does not match VkImage bytes per element 6`. A Metal
  texture per plane imports in 0.16-0.24 ms. VideoToolbox cycled through 5
  surfaces for 110 pictures, so the import can be cached per surface. The
  planes read back identical to FFmpeg's own host copy: 0 of 8.3 M luma and
  4.1 M chroma bytes differ. Copying both planes image to buffer takes
  0.24-0.30 ms of device time, which the kernel reading the images directly
  would save.
- **MJPEG needs a hardware decoder.** The Femto Mega's JPEGs are 4:2:0, full
  range, BT.601, like its H.265, about 1.4 MB at 4K (its limit is 25 fps),
  and carry no restart markers, so each frame's entropy decode is serial.
  CPU per 4K frame, with CUDA waiting on blocking sync, since its default
  spin bills every wait as CPU:

  | decoder | wall | CPU |
  |---|---|---|
  | FFmpeg software, M5 Max | 13.2 ms | 14.4 ms |
  | VideoToolbox JPEG, M5 Max | 5.2 ms | 0.25-0.29 ms |
  | FFmpeg software, 5090 | 14.7 ms | 16.2 ms |
  | FFmpeg through NVDEC, 5090 | 12.9 ms | 14.3 ms |
  | nvJPEG 13.2, CPU Huffman, 5090 | 10.9 ms | 10.0 ms |
  | nvJPEG 13.2, `GPU_HYBRID`, 5090 | 4.6 ms | 4.2 ms |
  | nvJPEG 13.2, `HARDWARE`, 5090 | 3.8 ms | 0.38 ms |

  The hardware engine's picture differs from the others only by IDCT
  rounding (a luma sum 350 apart over 8.3 M pixels), so its tests compare
  with a tolerance. FFmpeg's NVDEC MJPEG path keeps the picture on the GPU
  but not the work off the CPU. An MJPEG camera sends about 280 Mbit/s at 4K
  against about 30 for H.265, so H.265 stays the rig's default.
- **Vulkan Video decode was set aside.** FFmpeg 6.1 decodes H.265 through
  it on the 5090 at 12.4x real time, against 14.4x for CUDA. It needs Vulkan
  1.3 (recon's instance asks for 1.2), a decode queue in both device seams
  (gfx's bootstrap too), and FFmpeg's `AVVkFrame` protocol, and MoltenVK has
  none of it. It becomes the better choice if AMD or Intel on Linux, or
  Windows, become targets.
- **Host-side trims were set aside**: pooled driver buffers, and NV12
  through the host. They polish the path this removes. The page-fault
  finding stays true for the paths that keep host frames.

**Landed.** `GpuFramePrep` takes I420 or NV12, from the host or the device.
The colour kernel reads each plane at an offset and row stride of its own,
with a chroma step of 1 or 2, so its push constants grow from 76 to 96 bytes.
A test prepares one odd-sized, lensed picture every way it can come and
requires the outputs to be identical: host I420, host NV12 with tight and
padded rows, and device I420 and NV12 at odd offsets with padded rows, 4 KiB
into their buffer, taken over from outside Vulkan or not, the I420 with Cb
and Cr rows of different lengths. It also refuses:

- planes on both the host and the device, a stale third one included, and
  an NV12 host picture with a third plane;
- a plane past its buffer, or its last word past it;
- planes that overlap, as I420's do with their offsets left at zero;
- a row shorter than its picture;
- a buffer that is empty or without storage usage;
- a queue family the device lacks, before any work.

A kernel that ignores the chroma step, as a mutant, fails the NV12 case; one
that ignores the luma offset fails the device case.

Its review settled the device path, which a picture another queue or API
wrote reaches:

- **Ownership.** Reading an EXCLUSIVE buffer another queue family wrote, or
  memory CUDA wrote through an import, is undefined without a queue-family
  ownership transfer, and appears to work on Apple, where Metal has no such
  thing. `YuvImage::queue_family` names the writer: another family of the
  device, `kQueueFamilyExternal` for CUDA, or `kQueueFamilyIgnored`, the
  default, for the pass's own. The pass opens its batch with
  `CommandBatch::acquire`, new in `core`: the receiving half, from
  `VK_QUEUE_FAMILY_EXTERNAL` for an EXCLUSIVE or a CONCURRENT buffer, and
  from another family for an EXCLUSIVE one, whose writer records the
  release. The batch waits on no semaphore, so the writer must have
  finished, a fence waited or the stream synchronized, which
  `YuvImage::device` now says. An acquire naming this family for a
  CONCURRENT buffer, rather than every family, fails the batch test under
  the validation layer (VUID-09051); a pass that skips the acquire fails its
  refusal of an unknown family.
- **Lifetime.** A failed wait already leaked the staging the device may
  still read. It now leaks a reference to the device planes' buffer too,
  which the caller could otherwise drop, or a decoder's ring reuse, as soon
  as `prepare` returned.
- **Binding.** Device planes are bound from the first plane, rounded down to
  `minStorageBufferOffsetAlignment`, not from byte 0, so
  `maxStorageBufferRange` (2^27 on Mali and many Intel drivers) and the
  kernel's 32-bit addressing weigh the picture, not where it sits: a
  decoder ring's slot 128 MB into one buffer would have been refused on
  those drivers. A kernel reading each
  plane where it sits in the buffer rather than the binding fails the test;
  binding from byte 0 does not, since no device CI runs has a limit that
  low.
- **Checks.** Overlapping planes are refused, which catches I420's offsets
  left at zero, and so is a host pointer beside device planes, whichever
  plane it is. The kernel takes Cb's and Cr's strides separately, so the
  device path accepts Cb and Cr rows of different lengths, as FFmpeg's
  linesizes allow, rather than refusing them. The borrowed buffer is checked
  through `StorageInput::check`, so an empty one is called empty. A mutant
  dropping any of the new checks fails the test.

**NVDEC has landed.** `HevcDecoder::Options::device` hands a CUDA decode's
pictures out on the device:

- The decoder opens NVDEC on the CUDA device whose UUID is the Vulkan
  device's.
- It keeps a ring of buffers from `core`'s `create_exported_buffer`, each
  imported into CUDA once.
- For each picture it runs two `cuMemcpy2DAsync`, luma and chroma, and a
  stream sync on FFmpeg's stream.
- A picture takes a buffer no earlier picture holds, so a kept one is still
  itself.
- CUDA wrote it, so a reader acquires it from `VK_QUEUE_FAMILY_EXTERNAL`
  first, and a `YuvImage` of it carries `kQueueFamilyExternal`.

Anything else comes to the host as before: another back end, a 10-bit
stream, a left or top crop by an odd count, or a build without
`VR_WITH_CUDA`. FFmpeg has already taken the right and bottom crop off a
hardware picture's size, so the device path offsets its pointers by the
left and top crop and copies the rest.

`VR_WITH_CUDA` needs the CUDA 13 toolkit, and `nvcc` with it, since CMake
3.28's FindCUDAToolkit will not find the toolkit without it, though recon
compiles no CUDA code. CI's Ubuntu 24.04 leg installs the packages from
NVIDIA's repository. Measured on the RTX 5090, `receive` per 4K picture:

| | wall | CPU |
|---|---|---|
| host (copy to host, NV12 to I420) | 1.73-1.75 ms | 1.34-1.35 ms |
| device | 0.17-0.19 ms | 0.03-0.05 ms |

The host path then pays the 0.6 ms staging copy and the 0.47 ms upload; the
device path neither. On the 5090 and on CI's CUDA leg, the device pictures
of the plain and the cropped clips equal software's, byte for byte. Without
the left-crop offset, as a mutant, the cropped clip fails. A build without
CUDA, or any other back end, hands the same device option's pictures to the
host, which the Mac checks.

Its review changed six things:

- **libcuda is loaded, not linked.** Linking `CUDA::cuda_driver` stopped
  every program built with `VR_WITH_CUDA` from starting without the NVIDIA
  driver. It is `dlopen`ed now, as FFmpeg does, each entry point named
  through `cuda.h`'s macros so it is the versioned symbol its prototype
  declares. Only `cuda.h` comes from the toolkit, so the installed package
  finds no CUDA.
- **A failure falls back.** A slot that would not allocate, or a CUDA call
  that failed, failed the picture, and the next one the same way. Now the
  device path is let go, and that picture and every later one come to the
  host. Retrying each picture was not taken: a lasting failure would pay an
  allocation a picture.
- **Export is asked, not assumed.** The extension does not promise that a
  storage buffer exports, so `create_exported_buffer` asks
  `vkGetPhysicalDeviceExternalBufferProperties`.
- **A shared device hears of it.** `DeviceRequirements::external_memory`
  names the extension as optional, and `fuse_viewer`'s bootstrap enables it
  where offered. Before, an adopted device never exported memory.
- **The matrix guess reads the displayed height.** The device path passed
  the hardware frame's, which still counts the top crop, so a stream near
  576 rows could take BT.709 on one path and BT.601 on the other.
- **A free slot too small is let go**, so a change of picture size strands
  no memory. Slots added while pictures were held stay, a high-water mark
  like the mesh arena's.

`VR_WITH_CUDA` is Linux only, since CUDA imports the memory as a file
descriptor. The device must outlive every picture on it, documented rather
than checked, as for every `Buffer`. The test expects device pictures only
on an NVIDIA Vulkan device, one CUDA sees.

**nvJPEG has landed.** `JpegDecoder` decodes the JPEGs an MJPEG camera
sends, to I420, BT.601 full range as JFIF defines it:

- Given a device, in a `VR_WITH_CUDA` build, it opens nvJPEG in the primary
  context of the CUDA device whose UUID is the Vulkan device's, with a
  decoder on the GPU's hardware JPEG engine where it has one and one on its
  cores (`GPU_HYBRID`) for what the engine refuses; `backend()` names the
  first.
- An 8-bit 4:2:0 JPEG that one of them supports decodes into a buffer from the
  same ring as NVDEC's, Y, Cb and Cr each packed and starting on 256 bytes,
  so `DecodedPicture::offset` grows a third plane. The decoder waits on an
  event made to block, since CUDA's default wait spins and bills the wait as
  CPU. The picture carries `kQueueFamilyExternal` like NVDEC's.
- Anything else decodes in software through FFmpeg, 4:2:2 converted to
  4:2:0 on the host: another subsampling or precision, a JPEG nvJPEG cannot
  parse or decode, a device on another GPU, no device.
- libnvjpeg is loaded at run time, like libcuda, so a program built with
  CUDA starts where neither is. CI's CUDA leg installs it again.
- A JPEG nvJPEG refuses (`BAD_JPEG`, `JPEG_NOT_SUPPORTED`,
  `INCOMPLETE_BITSTREAM`) goes to software alone, since one corrupt frame
  from a camera must not cost every later one 14 ms of CPU. Any other
  failure, nvJPEG's or CUDA's, lets the device path go, as NVDEC's does.

Measured on the RTX 5090, per 4K frame of the Femto Mega's MJPEG, Release:

| | wall | CPU |
|---|---|---|
| software | 14.27 ms | 14.34 ms |
| nvJPEG, `HARDWARE` | 3.83 ms | 0.38 ms |

The software picture then pays the staging copy and the upload; the nvJPEG
one is already on the device. The committed JPEGs
(`tools/make_jpeg_fixtures.sh`: the HEVC clips' pattern at 4:2:0, at an odd
size and at 4:2:2) decode to the pattern in software, and to within 2 codes
of software on the device, the two inverse DCTs rounding apart. A buffer is
reused only once no picture holds it. Swapping the Cb and Cr offsets, as a
mutant, fails the test. Both CI hosts are Blackwell and have the JPEG
engine, so `GPU_HYBRID` runs there only on a fixture 16400 pixels wide.

Its review changed five things, measured on the RTX 5090 against 4K JPEGs
spoiled the ways a camera link might spoil them: entropy data flipped or
zeroed, and the file cut short.

- **A status is read, not only its failure.** Every nvJPEG failure sent the
  JPEG to software, so a lost GPU or a device out of memory cost every later
  frame a parse and 14 ms of CPU while `backend()` still named nvJPEG. Only
  the three statuses about the JPEG are refusals now, and a decode that fails
  still waits for what it queued before the buffers are reused. No spoiled
  frame returned another status: a flipped one fails to parse as
  `JPEG_NOT_SUPPORTED`, and a zeroed or cut one decodes, CUDA sound. The test
  holds a garbage and a cut frame to keeping the device path.
- **The cores take what the engine refuses**, a JPEG past 16384 pixels a
  side, which `GPU_HYBRID` decodes; nothing from 2x2 to 8192x8192 was
  refused. Both decoders share one handle, made on the engine.
- **A device path let go releases what it held**: the primary context, the
  stream, nvJPEG's handle, states and staging buffers. So does one that
  never finished opening. Both kept them for the decoder's life.
- **libnvjpeg is the major version `nvjpeg.h` names**, and configure refuses
  a toolkit without `nvjpeg.h`, which is a package of its own.
- **The test checks what it reads**: each picture against its fixture's
  size, and every plane whole before it indexes one. Dropping the cores'
  decoder, or refusing no status, as mutants, fails it on the 5090.

Not taken:

- Remembering a refusal. A full parse of a 350 KB 4K JPEG is 0.05 ms, under
  0.4% of the software decode that follows it. A header-only parse is not
  enough: the engine's support check answered differently after one.
- One loader for libcuda and libnvjpeg. Each table would still need its own
  macro list, so sharing saved no lines.
- Releasing a reused buffer back to CUDA. CUDA overwrites the picture, a
  family may take a buffer without a transfer when its contents need not
  survive, and the reader that let the picture go had finished with it.

**VideoToolbox's H.265 has landed.** Given `HevcDecoder::Options::device`
on a device that imports Metal textures, a VideoToolbox decode hands its
picture out as two images:

- `Device::create` enables `VK_EXT_metal_objects` where MoltenVK offers it
  (`imports_metal_textures()`), and `DeviceRequirements::metal_objects` asks
  a shared device for it.
- Each plane of the picture's IOSurface becomes a Metal texture on the
  device's own `MTLDevice`, imported as an image that `core`'s new `Image`
  holds. The `MTLDevice` is read off the exported texture of a 1x1 probe
  image: exporting the device itself (`vkExportMetalObjectsEXT`) needs the
  instance to have asked for it when it was made, which an embedder's may
  not have, where an image asks for its own texture. VideoToolbox cycles its pictures through
  a few surfaces (five for the lab's 4K clip), so each surface's images are
  made the first time a picture arrives on it and kept until none has for 64
  pictures. A picture's images hold its pixel buffer, so VideoToolbox does
  not reuse the surface while a frame reads it.
- `DecodedPicture::image` carries the two, and `YuvImage::image` takes them.
  `GpuFramePrep` copies both into its input in its batch
  (`CommandBatch::copy` from an `Image`, new), then reads them as it reads
  packed host planes.
- An image stays in one layout, one a copy reads, which its maker puts it
  in: `CommandBatch::copy` refuses any but GENERAL and TRANSFER_SRC_OPTIMAL.
  A surface's images go from UNDEFINED to GENERAL once, when they are made.
  Vulkan does not promise that keeps their contents; MoltenVK keeps them,
  Metal having no layouts, so the assumption lives with the Metal import,
  not in `core`. The images the tests fill by hand are GENERAL too, so the
  copy is exercised on every GPU CI has.
- A picture that is not 8-bit NV12 on an IOSurface comes to the host, and a
  failed import lets the device path go, as for NVDEC. A device path that
  cannot be set up, on either, leaves every picture to the host rather than
  failing the decoder. VideoToolbox has cut
  the right and bottom already, and a stream cropped at the left or top never
  reaches it.

Measured on the M5 Max, Release, per picture of the lab's 4K clip, with a
640x576 depth frame beside it:

| | receive, CPU | prepare, CPU | prepare, GPU |
|---|---|---|---|
| host planes | 0.57-0.75 ms | 0.43-0.58 ms | 0.94-1.01 ms |
| plane images | 0.003-0.005 ms | 0.31-0.45 ms | 1.09-1.17 ms |

Unified memory makes the host path cheap here, so the saving is modest: the
copy to the host, the conversion and the staging go, and the device copies
the planes instead, 0.28-0.31 ms alone and some 0.15 ms more than the staged
copy it replaces. Receiving is only handing the images over: made for every
picture, two textures, two images and two allocations cost 0.16-0.33 ms of
it. The pictures read back
identical to software's, byte for byte, the cropped clip going to software.
Colour copied from images larger than the picture, read from their corner,
prepares identically to host planes, and a mutant copying the chroma a
column short at an odd width fails that test.

**VideoToolbox's JPEG has landed.** Given a device that imports Metal
textures, `JpegDecoder` opens VideoToolbox's hardware JPEG decoder
(`JpegDecodeBackend::VideoToolbox`):

- It takes a baseline or extended 8-bit 4:2:0 JPEG in three components, the
  frame header read off the bytes, as nvJPEG takes it. Anything else goes to
  software, as does one past the device's `maxImageDimension2D`, before a
  session starts: Metal aborts the process on a texture past it, which the
  16400-wide fixture is on the M4 runner (16384).
- A session holds for one size, asked for full-range NV12 on an IOSurface
  and the hardware only. A size no session opens for is not asked again each
  frame.
- The decode is synchronous. Its picture goes through the same import as
  H.265's, into `DecodedPicture::image`.
- A JPEG the session refuses goes to software alone, and the session is
  started afresh for the next; only a failed import lets the device path go.

Measured on the M5 Max, Release, per frame of the Femto Mega's 4K MJPEG, CPU
the whole process's:

| | decode, wall | decode, CPU | prepare, CPU | prepare, GPU |
|---|---|---|---|---|
| software | 13.1-13.3 ms | 13.1-13.3 ms | 0.38-0.39 ms | 0.98-1.00 ms |
| VideoToolbox | 6.0-6.1 ms | 0.92-0.94 ms | 0.64-0.67 ms | 1.14-1.18 ms |

A frame costs 1.6 ms of CPU where it cost 13.7. The images of every 4:2:0
fixture, the 16400-wide one included where the extent takes it, are within 2
codes of software's; 4:2:2 goes to software.

**The Orbbec raw path carries H.265's device picture.** Given
`OrbbecStreamOptions::device` with `raw`, each camera's colour decoder
decodes onto that device, and a raw frame's colour is the picture NVDEC or
VideoToolbox left there (`YuvImage::device`, taken over from outside Vulkan,
or `YuvImage::image`). Any other picture comes as the I420 planes it came as
before.

The picture crosses the mailbox with an SDK colour frame
(`picture_frames.hpp`). The decode thread, the mailbox, a rig's
grouping and the held pair all handle frame sets, as before. The picture then
lives exactly as long as its frame, and is dropped, grouped and held with it;
changing the mailbox's element type instead would have touched every one of
them.

The frame's bytes are a tag and a serial, and the picture waits in a table
under that serial until the frame's deleter erases it. The first cut stored
the picture itself as the bytes, `shared_ptr`s and all, and review found that
a byte copy of the frame (`copyFrameInfo` then `updateData`, or an SDK filter)
would duplicate them without a reference, and outlive what they point at. Now
such a copy owns nothing, and once the frame is freed it carries no picture.
The frame is a 0x0 video frame of `OB_FORMAT_H264`, where the first cut's
NV12 invited a reader to take its bytes for pixels. The SDK refuses
`OB_FORMAT_UNKNOWN` for a colour frame, and a compressed format is sized by
its bytes. No stream here carries H.264, so nothing decodes them either.

No CI leg has a camera, so `process_raw` itself never runs there. What it
does with a device picture is `place_device_color`, which the offline test
runs on NVDEC's buffers on the CUDA leg and on VideoToolbox's images on the
Mac. Both tests require the pictures on the device only on the legs whose
hardware leaves them there: VideoToolbox, and NVDEC in a `VR_WITH_CUDA`
build. The three decoder tests read a device picture back through one
helper, `tests/device_picture_readback.hpp`.
`fuse_orbbec --gpu` now creates the GPU before it opens the cameras, and
hands the capture that device.

The committed clip, fed through the decoder with a device on the M5 Max,
comes out as VideoToolbox images holding each patch's value within 2 codes.
They are described as the Femto Mega codes its stream, and released with
their frames. On the live camera, raw frames arrive with their colour as
images, and the pass still matches the SDK's own undistortion and
registration at 720p and at 4K (depth 5 mm median).

Measured on the lab rig, four Femto Megas at 4K and 25 fps over the wired
link, `fuse_orbbec --rig --gpu --hevc`, 600 frames, Release, two runs each:

| colour | user CPU | sys CPU | fused |
|---|---|---|---|
| on the device | 0.56-0.57 s | 1.61-1.64 s | 94.8-97.4 fps |
| host planes | 1.37 s | 1.60-1.65 s | 92.7-97.4 fps |

That is about 1.4 ms less CPU per camera frame: the copy to the host, the
conversion, the I420 frame and the staging copy. The cameras set the rate
either way.

**MJPEG raw frames have landed.** A raw stream no longer needs H.265. For
MJPEG it streams the camera's MJPG mode, the JPEGs themselves, where the
host path takes the SDK's RGB mode. Each camera's `JpegColorDecoder` decodes
every pair on a thread of its own, ahead of the mailbox as H.265's decoder
does, through `JpegDecoder` onto the device where nvJPEG or VideoToolbox
takes the JPEG and in software to I420 otherwise. It hands the picture on in
the same frame (`picture_frames.hpp`, which now holds both kinds of colour
frame, and `PlanesColor`).

- On its own thread, so a rig's four cameras decode at once rather than one
  after another on the polling thread, where VideoToolbox's 6 ms a 4K frame
  would have cost a set 24.
- A JPEG depends on no other frame, so there is no gate: a pair missing a
  frame, an empty colour frame, a JPEG that does not decode, or a pair a
  newer one replaced while the decoder was busy each cost only themselves,
  all counted in `stats().lost`. The queue holds as many pairs as the
  mailbox after it, newest kept, so a decoder slower than the camera skips
  pairs rather than handing them out seconds late.
- The MJPG mode's calibration must be the RGB mode's, byte for byte, as
  H.265's must.
- `VR_ORBBEC_WITH_HEVC` becomes `VR_ORBBEC_WITH_VIDEO`, since it now gates
  both decoders, and raw frames without them are refused at open.

Measured on the lab rig, four Femto Megas at 4K and 25 fps over the wired
link, `fuse_orbbec --rig`, 600 frames, M5 Max, Release:

| path | user CPU | sys CPU | poll a frame |
|---|---|---|---|
| host MJPEG (the SDK decodes, the host undistorts and registers) | 60.0 s | 11.5 s | 18.6 ms |
| raw MJPEG, `--gpu --mjpeg` | 0.83-1.09 s | 3.66-4.37 s | 0.02 ms |
| raw H.265, `--gpu --hevc` | 0.61 s | 1.71-1.75 s | 0.02 ms |

Raw MJPEG takes about a sixteenth of the host path's CPU, and a little more
than H.265: its system time is mostly the kernel receiving the JPEGs, about
185 Mbit/s a camera against H.265's 21. H.265 stays the rig's default for
that reason and for the link's headroom: `fuse_orbbec --gpu` still streams
it, and `--mjpeg` asks for MJPEG by name. Before, `--gpu` forced H.265, and
the first MJPEG runs measured H.265 for that reason; the table is the rerun.
The offline test holds the decoder to the committed JPEG. It checks I420
frames in software, the device's pictures given one (VideoToolbox's images,
or nvJPEG's buffer on a CUDA leg), each released with its frame, each loss
counted once, and a decoder held back skipping to the newest pair. On the
live camera, raw MJPEG frames arrive as images and match the SDK's
undistortion and registration at 720p and 4K, as H.265's do, and the rig's
raw pass runs over both codecs.

**Next**: measured first, the colour kernel reading Apple's plane images
directly, which saves the copy's 0.28-0.31 ms of GPU.
*Done 2026-10-25 (the plane-image entry):* the colour pass samples them
where they are, and the copy is gone.

*Amended 2026-10-01 (PERF.md's D1 and D3):* the fallback is no longer
silent, and an exported picture buffer was already out of the BAR.

- **The fallback.** Every way a decoder given a device leaves its pictures
  on the host is said once, as a warning through `core`'s log handler: a
  device path that does not open (the back end has none, or its setup
  failed), one that fails mid-stream, and Auto moving a stream to software.
  `Options::label` puts the camera ahead of it, since a rig runs a decoder a
  camera. `OrbbecCaptureStats::host_pictures` counts, of the raw frames
  handed out, those with host colour although the stream decodes onto a
  device, and a frame the rig withdraws from a failed set comes back out of
  it. `fuse_orbbec` and `rig_viewer`'s Rig panel show it when it is not 0.
  On a discrete GPU each such 4K frame costs 12 MB across the bus.
- **Pinned by the tests.** Each decoder's test opens it on a device adopted
  with none of its optional extensions, where no device path opens, and
  holds it to one warning that names the label. The camera's GPU
  pre-processing test holds the count to the host-colour frames it sees (0
  for H.265 and MJPEG on the M5 Max, where a mutant counting every frame
  failed it), then decodes H.265 onto that bare device, where every frame
  must come to the host and be counted, so a count that never moves fails
  too.
- **Already out of the BAR.** Vulkan orders a memory type ahead of any whose
  flags strictly contain its own, so the first device-local type a buffer
  allows is one the host cannot map wherever the buffer allows one:
  `create_exported_buffer` never took the BAR heap over plain VRAM, and D3
  had nothing to fix. What it leaves is `core`'s `find_memory_type`, which
  says so, replaces the three hand-written pickers (`create_exported_buffer`,
  `vt_pictures.mm`'s, the tests' image), and is checked against each CI
  GPU's types by the external-memory test. `vt_pictures.mm` still excludes
  host-visible types, since MoltenVK would back that memory with a buffer.

### 2026-09-28 — Projective texturing takes a colour camera of its own and depth on the device: the depth camera decides what is visible, its map what the colour camera sees, and the colour camera gives the coordinate; a view's device depth and coverage are copied on the device rather than staged.

*Amended 2026-10-08 (below):* per vertex, the normal must point toward both
cameras, not only put them on one side.

**The rule.** A `TextureView` may carry a `color_camera`, the camera its
tile's image was taken with; its depth as a `depth_buffer` on the device in
place of the host `depth` array, exactly one of the two; and a `coverage`, the
image on the device with a zero high byte where it recorded nothing. The
buffers are held by `shared_ptr`. The single-camera pass takes the same view,
in `texture(const DeviceMesh&, const TextureView&, ...)` and
`texture(Mesh&, const TextureView&, ...)`. In both passes the depth camera
decides visibility through the same projection and occlusion test, and its
map decides too whether the colour camera's line of sight is clear. A
vertex's coordinate is its pixel in the colour camera. A registered image is
textured as one whose colour camera is its depth camera: one path, the same
arithmetic, and every colour-camera test then passes wherever the depth
camera's does, so the result is what it was.

**Why.** It closes the two `TODO(texture)`s that kept a `GpuFramePrep` frame
off the texture tier. The GPU pre-processing decision above keeps depth and
colour unregistered, each with its own camera, and step 4 of the residency
decision left the single-camera device-depth pass textured only for a
registered atlas and the several-view pass staging every view's depth from
the host. A rig of such frames could therefore be fused on the device but not
textured there. Now its views texture from the frames `prepare_set` returns,
and nothing visits the host on the way.

The prior repos never met the offset. Both set the Femto Mega to
`ALIGN_D2C_HW_MODE`, so depth arrived registered to colour and occlusion was
tested in the colour camera itself (`implicit_surface_compression` against a
downsampled copy of the registered depth). `GpuFramePrep` keeps the two
apart, and the pass asks the same question of the depth map as it stands,
with both cameras' intrinsics and poses, rather than building a registered
map for every view.

**What the colour camera must see.** Cheapest test first, the depth reads
last:

- **Its image, where the image recorded it.** Behind the colour camera a
  vertex gets the `(-1, -1)` sentinel, whatever the depth camera makes of it.
  In front, it must land inside the colour image, not on an edge clamp, and
  on a pixel `coverage` marks: `GpuFramePrep` writes 0 where the lens saw
  nothing, and without the test a view whose undistorted corner is black won
  a triangle another view saw and textured it black.
- **The side the depth camera saw.** A colour camera across the surface's
  plane from the depth camera images the other side of it. Per triangle that
  is the colour camera's own facing test; per vertex, where the pass has only
  the vertex, it is the sign of the vertex normal against each camera's
  centre, which must agree. A zero normal passes.
- **A clear line of sight.** The depth camera proves its own line to the
  vertex. The colour camera, a few centimetres away, looks along another,
  and past an occluding edge it sees the occluder where the depth camera sees
  the vertex: a sliver about the cameras' parallax wide,
  `f·b·(1/z_occluder - 1/z_surface)` pixels. The line from the vertex back to
  the colour camera projects into the depth image along the vertex's
  epipolar line, on which 1/z is linear, so the pass walks it and knows the
  line's depth at every sample. A surface the map puts more than
  `occlusion_threshold` nearer than the line blocks it. The walk takes at most
  64 samples, a pixel apart until the line is longer, one nearest read each,
  and ends at the near end of the depth range or the image's edge. For a
  registered image the line is a point and nothing is sampled.
- **The tile is the colour image.** `side_by_side_atlas` sizes a view with a
  colour camera from it, and a given `image_width` x `image_height` other than
  its size is refused, since every coordinate would be misplaced.

The score stays the depth camera's: the baseline is a few centimetres, and the
depth camera is the one that proved the view.

**Device inputs are copied, not bound in place.** The several-view kernel
reads every view's depth from one buffer, and every marked image's coverage
from another, at each view's offset. A device view's are copied into them
with `CommandBatch::copy`, a host view's depth staged, every depth before any
coverage. `CommandBatch` now lets a copy join a run of writes rising through
one buffer, as fills and uploads did, unless a command in the run writes the
copy's source, so the copies take no barrier between them. They are timed in
the call's row. Binding each view's buffer in place would take a descriptor
array indexed per view -- `shaderStorageBufferArrayDynamicIndexing`, a core
1.0 feature most GPUs have but `Device::create` does not enable, so every
embedder's device would have to, or the copy would stay beside it as the
fallback -- or a binding per view, which caps the views. The copy is one
device-local pass: 1.5 MB of depth at 640 x 576 and 3.7 MB of coverage at
1280 x 720 a view. A copied input need only be a copy source, which the batch
checks; the single-camera pass binds its inputs, which must be storage
buffers.

**Held, not borrowed.** `GpuFramePrep` reuses a frame's buffer once its
`shared_ptr` is the pass's alone. A raw pointer in a kept view could not be
seen: the next same-size frame overwrote the depth the view read against its
old camera, and a larger one freed it under the view.

**Measured**, on the M5 Max, Release, a throwaway harness (not committed),
medians of 30 over two interleaved passes, shared machine. A wall 3 m out of
180 000 unshared triangles (540 000 vertices), 640 x 576 depth, 1280 x 720
colour cameras 3.2 cm beside their depth cameras, through the host-mesh
overloads; the device span, which is the dispatch and, for several views,
the inputs' transfers:

| | one camera | four views |
|---|---|---|
| registered | 0.24-0.27 ms | 0.27-0.29 ms |
| colour camera on the depth camera | 0.23 ms | 0.26-0.28 ms |
| colour camera 3.2 cm aside | 0.52-0.74 ms | 0.85-1.08 ms |
| the same, with coverage | 0.58-0.74 ms | 0.80-1.23 ms |

The walk is the difference: about 0.3-0.5 ms for one camera and 0.6-0.8 ms
for four. With bilinear samples
the aside rows read 0.83-0.88 and 1.30-1.39 ms, which is why each sample is
one read: on a plane its error passes the threshold only at slopes where the
bilinear sampler falls back to its nearest tap too. An earlier harness, four
views of 3 000 triangles, put the host row at 0.54-0.55 ms with the depth
staged and 0.42-0.48 ms with it on the device; its triangles faced away from
every camera, so it measured the transfers and not the tests. Not measured on
the RTX 5090.

**Open.**

- **What the depth camera cannot see.** The walk tests the line against the
  map, so an occluder the depth camera does not see, or one thinner than a
  sample step on a line longer than 64 pixels, does not block it.
- **Inputs kept between calls.** Every call stages or copies every view's
  depth and coverage afresh; a static keyframe set could keep them (a
  `TODO(texture)` at the buffers).
- **The atlas and gfx.** `pack_atlas` still packs on the host, and nothing yet
  hands a device frame's colour to gfx as an image; the texture pass's side is
  what landed here.

**The review.** The first cut documented the fringe and the black corners as
open, took the device depth by raw pointer, gave the single-camera colour
path no side test, a branch of its own beside the registered one and no host
overloads, required a copied depth to be a storage buffer, left its copies
untimed with a barrier between each, and dropped the `TODO(texture)` for
keeping inputs between calls. Each is fixed above. Its "same camera" test
passed on a mesh an earlier case had already textured; it now follows a pass
against a frame that sees nothing.

**Verified.** The whole suite passes on the M5 Max, 42 of 42, and the batch
and texture tests report nothing under the Khronos validation layer with
synchronization validation on. The new cases check:

- device depth, every view's, one beside two host views in a buffer the pass
  can copy from but not bind, giving the host run's coordinates exactly, each
  after a run against empty depth maps;
- colour cameras beside the depth cameras winning the same triangles with the
  colour cameras' coordinates, and a colour camera that sees a triangle's
  back, nothing of it, or it past its image refusing it;
- a rig of device depth, colour cameras and coverage matching the host run
  exactly after a run whose images recorded nothing, and a view whose image is
  blank over a triangle giving it to another;
- a triangle beside an occluder in the depth map given to a clear view when
  its colour camera looks through the occluder, and kept with the colour
  camera on the other side or none, in both passes;
- vertices whose normals put the colour camera behind their plane carried,
  and textured from the depth camera's side;
- one camera: the registered result from a colour camera on the depth camera
  after a blind pass, one aside, its image half blank, one too narrow and one
  facing away;
- the refusals: both depths or neither, a device depth or coverage short of
  its image, empty, or not a copy source, a single-camera coverage that is not
  a storage buffer, and a colour camera with no image or a size it does not
  have;
- `CommandBatch`: copies into one buffer at rising offsets land, and one whose
  source the run filled copies the fill.

Each of these mutations fails a test: coverage always recorded, ignored in
either kernel, or not copied; the walk never blocking, or skipped in either
kernel; no side test; the depth pixels as the coordinates; and, under
synchronization validation, a copy joining the run whatever wrote its source.

### 2026-09-29 — `rig_viewer` draws a live Orbbec rig: raw sets prepared, fused and textured on the GPU, the atlas filled by device copies recorded in gfx's frame, and the frame prep's colour shared with gfx's queue family.

*Amended 2026-10-07 (the rig entry, below):* it opens an `OrbbecSensor` per
camera of the sync file and reads them as a `SensorArray`, each set prepared
by `SensorArray::process`; CI's viewer leg compiles it.
*Amended 2026-10-08 (the exchange entry, below):* both viewers hand meshes
over through `mesh::MeshExchange`; `viewer_common.hpp` keeps the teardown
guards and the render camera they mesh.
*Amended 2026-10-08 (the streamed-atlas entry, below):* the atlas is gfx's
`StreamedAtlas`, and what a frame's copy reads is held until the frame loop's
timeline reaches the frame, so the per-slot atlas and colour-buffer holds are
gone.

**The rule.** `examples/viewer/rig_viewer` is `fuse_viewer`'s sibling for a
live, synced rig of Orbbec cameras. It opens `OrbbecRig` raw, onto the device
it shares with gfx, and keeps everything a camera produces on the GPU after
it arrives:

- each set is prepared by `prepare_set` (one `GpuFramePrep` per camera) and
  fused through the device-input overloads;
- every `--remesh-every` sets, the unshared marching-cubes mesh is textured
  from all the set's cameras (`ProjectiveTexturer`, each view's depth, colour
  camera and coverage its frame's);
- the atlas is a gfx image, filled by `vkCmdCopyBufferToImage` from each
  camera's colour buffer into its tile, recorded in the renderer's frame
  before the draw; `HybridMeshPipeline` then draws recon's buffers in place,
  as in `fuse_viewer`.

The atlas is laid out once, from the rig's colour cameras, with
`side_by_side_atlas`, so four cameras make a 2 x 2 grid of their images and it
never changes size. A set missing a camera textures from the cameras present,
into their own tiles, and a version samples only the tiles it wrote. It is a
separate executable rather than a mode of `fuse_viewer`, which stays the
dataset replay. The two share the bootstrap, the stage table, and
`viewer_common.hpp`: the teardown guards and the render side of the ring (the
release mark and the check that a mesh can be bound). It builds only where
the Orbbec driver and its colour decoders do, since it opens the rig raw. It
reads the rig's raw sets directly (`poll_raw_set`), as `fuse_orbbec --gpu
--rig` does: the sensor contract of 2026-09-14 has frames, not sets.

`GpuFramePrep::create` takes a `GpuFramePrepConfig` whose
`color_queue_families` its colour output is shared with, the twin of
`MarchingCubesConfig::queue_families`; both `create`s bound the count with
`core`'s `check_queue_family_count`. The viewer names recon's and gfx's
families, since MoltenVK hands the two libraries different ones, where
copying from an EXCLUSIVE buffer is undefined in the way that appears to
work. Depth stays EXCLUSIVE whatever the config says, since only recon reads
it. *Amended 2026-10-16 (below):* depth may be shared too
(`GpuFramePrepConfig::depth_queue_families`); the viewer keeps it EXCLUSIVE.
Left empty, both outputs are EXCLUSIVE, as they always were.

**Why a copy, not zero-copy sampling.** gfx's hybrid pipeline samples an
image, which buys filtering and the sRGB decode in hardware. Reading the
cameras' storage buffers directly would need a pipeline variant that decodes
and filters in the shader, as gfx's pending patch pipeline does, to save a
device-to-device copy of about 0.23 ms per remesh at 1 cm (measured below).
Nothing crosses the host either way.

**Lifetimes and visibility, as `fuse_viewer`'s mesh.**

- A copy's colour buffers are held by the frame slot that recorded it until
  `begin_frame` fence-waits that slot again, so a `GpuFramePrep` reuses one
  only after gfx has read it. The pass's reuse test is `use_count() == 1`, a
  relaxed load, so an acquire fence follows it: without one nothing orders
  the render thread's fence wait before the pass's next write.
- An atlas image is reused only when nothing but the viewer's pool holds it:
  not the committed version, and no frame slot that bound it.
- The frame prep fence-waits its batch before `prepare_set` returns, and
  gfx's later `vkQueueSubmit` makes those writes visible to the copy.
- Each colour buffer's usage, size and sharing mode are checked on the fuse
  thread, before its camera textures the mesh. Once uv0 point into a tile the
  tile must be filled, so a buffer that fails the check costs its camera's
  triangles, which other cameras or the fused colour take. Checked at the
  copy instead, the only fallback was the white atlas, which draws every
  textured triangle white.

**Defaults.**

- 1 cm voxels: a Femto Mega's depth pixel covers about 4 mm at 1.5 m.
- Unlit shading: the cameras' colour as they saw it.
- H.265 colour: raw MJPEG takes about nine times the bandwidth.
- Remesh every set (every 5 before the amendment below).

*Amended 2026-09-29:* the viewer remeshes on **every** set by default, so the
drawn mesh follows the cameras rather than a sixth of their rate. Every 5
sets, the first default, changed the mesh about six times a second, which
read as a mesh that was not live. A set with its remesh fits the rig's frame
period with room to spare, and the Rig panel and status line now report the
measured rate (meshes committed per second, over the last second). On the lab
rig, calibrated, 1 cm, 1.5 M triangles, M5 Max, Release:

| per set | fuse | remesh (extract + texture) | mesh updates |
|---|---|---|---|
| every set | 9.3-11.1 ms | 6.1-7.8 ms | 25.9-28.8 /s |

That is the rig's own set rate, 30 fps less the sets the SDK drops on the
wired link, so the viewer is no longer what limits it. What still delays a
moving surface is the fusion, not the mesh: at `--max-weight 20` a voxel
settled over 20 frames takes a new observation at a twentieth of its weight,
and `Classic` integration keeps what a receded surface left. The `Dynamic`
mode clears it, and is not yet the viewer's.

*Amended 2026-09-29, again:* the viewer now fuses **`Dynamic`** by default
(`--static` for `Classic`) and textures at a **5 cm** occlusion threshold
(`--occlusion`), and the View panel tunes both live, with the max weight.
Two reports from the lab rig drove it.

- **A moving surface faded out and in.** Under `Classic` a surface that moved
  away was only averaged down, over up to `--max-weight` frames, and one that
  arrived where free space had been fused at full weight took as long to
  appear. `Dynamic` clears a voxel a camera sees as free space past the band,
  so the old surface is gone on the next set and the new one forms from
  weight 0. `fuse_frame` (the `DeviceFrame` overload in `examples/common`)
  takes the mode, `Classic` by default, so `fuse_orbbec` is unchanged.
- **Much of the scene looked textured by one camera, or not at all.** Two
  causes, measured with the new `--texture-stats`, which reads the mesh back
  every 30 remeshes and counts each camera's triangles by the tile their
  `uv0` land in:
  - **The April calibration.** Converted from that rig's bundle adjustment, it
    placed three of the four cameras 35-50 cm from where the 2026-09-29
    calibration puts them. The rig had been rearranged since, so a camera's
    depth disagreed with the fused surface far past any threshold, and the
    camera never qualified.
  - **The 2 cm threshold.** With today's calibration, 1 cm voxels, a 1.4 M
    triangle mesh:

    | occlusion | untextured | each camera |
    |---|---|---|
    | 2 cm | 27-29% | 16-20% |
    | 5 cm | 16-18% | 19-23% |

    ToF noise and what calibration error is left both grow with range, so the
    background failed the tighter test first. 5 cm is the viewer's default,
    and the texture tier's own default stays 2 cm.

  Most of what is left untextured lies outside every colour camera's view.
  The Femto Mega's depth covers about 65 degrees vertically and its
  1280 x 720 colour about 51, so the floor near each camera is fused from
  depth alone. The mesher gives such a vertex opaque white, having no colour
  to give it.

`--show-sources` (a View panel toggle too) fills each camera's tile with a
colour of its own rather than its image, copied from device-local buffers
gfx makes and fills on the device the first time it is switched on
(host-visible until 2026-10-01, which crossed the bus on every remesh on a
discrete GPU: PERF.md's D2; every buffer is made before any fill is
recorded, so a failed allocation frees none a fill names), so the window
shows which camera
textured each triangle and what none did. On today's run it shows each
camera owning the surfaces it faces, and the overlap between them choosing
per triangle, which speckles. That speckle is where the cameras' auto
exposure and white balance, both on in every camera's firmware, show as
seams.

Still open:

- A set missing a camera textures that remesh without it, so its region
  falls to fused colour for a frame, a flicker now that every set remeshes.
  Holding each camera's last frame for texturing would cover it, at the cost
  of the frame prep's output reuse. (Done: the fourth amendment below.)
- Blending the views where they meet, or locking every camera to one
  exposure and white balance (a write to the cameras, like `--apply-sync`),
  would take the seams out of the speckle.

*Amended 2026-09-29, a third time:* the viewer fuses **depth only inside each
camera's colour view** by default, through the frame prep's new
`depth_within_color` (the GPU pre-processing entry's amendment). `--all-depth`
fuses all of it, as before. The white floor near each camera, which no colour
camera sees, is no longer fused. On the lab rig, with the defaults above and
today's calibration, over 2400 frames per run:

| depth fused | triangles | untextured | each camera |
|---|---|---|---|
| all (`--all-depth`) | 1.39 M | 16.8-16.9% | 19.0-22.5% |
| inside the colour view | 1.26 M | 8.5-8.7% | 20.8-24.8% |

The triangles removed were almost all untextured ones: the untextured share
roughly halves, and each camera's share grows only because the total shrinks.
A screenshot of each run shows the white walls and floor of the first gone
from the second. The 8.6% left has not been broken down by cause.

*Amended 2026-09-29, a fourth time:* a camera missing from a set **textures
from its last frame** for up to `--hold-ms` on the rig's clock (2000 by
default; 0 turns it off). The viewer keeps each camera's newest frame with
colour, and `texture_sources` gives the texture pass, per camera, this set's
frame or, within the limit, the one kept from an earlier set. A held frame
goes to the pass whole: its colour into the camera's tile, and its depth and
colour camera as the view. It is a **fallback view**
(`TextureView::fallback`, new here): it takes only the triangles no present
camera qualifies for, and of those only where its own depth still agrees
with the mesh within the occlusion threshold. That test is geometric: a
surface that moved toward or away from the camera is refused, but one that
slid along itself, or changed its look, passes and takes the old image.

- **Why.** With every set remeshed, each set missing a camera was a remesh
  textured without it. For one frame, every triangle that camera held fell
  to fused colour, which read as flicker whenever the Rig panel's `dropped`
  count went up. The SDK drops those frames on the wired link, more often
  from the secondaries and more often at 4K, and a lost H.265 frame costs
  its camera every frame to the next key frame. The viewer cannot prevent
  either.
- **Texture only, not fusion.** The missing camera's depth is not re-fused
  from the held frame. Fusing a frame twice would double its weight, and
  under `Dynamic` stale depth would clear surfaces that have since arrived.
- **The limit.** 2 s on the rig's clock, so a stall does not keep a frame
  fresh. It spans the wait for the next key frame (30 frames at the camera's
  default, 1.2 s at 25 fps) with room for the decoder's reset. A held view
  gives way to every present one, so a longer hold costs only surfaces no
  present camera sees.
- **The cost.** A kept frame keeps its buffers, so its camera's frame prep
  writes the next frame to new colour and depth buffers rather than reusing
  them (the reuse rule of the GPU pre-processing entry). Frames are kept
  only while a hold can be used: not at `--hold-ms 0`, nor with texturing
  off. `prepare_set` on four 3840 x 2160 colour and 640 x 576 depth frames,
  the previous set's kept, measured 0.1 ms a set slower on the RTX 5090
  (2.82 against 2.71 ms mean, and 2.94 against 2.85 on a second run; Release,
  570 sets each, interleaved), its 95th percentile about 1 ms higher.
  Keeping two sets measured no slower than keeping none. On the M5 Max the
  rig's fuse time did not move (below).

The Rig panel and status line count the views given to the texture pass from
a held frame, and the remeshes given fewer cameras than the rig has. Both
count views given, not triangles taken, so the second at zero shows that
every remesh had a view for every camera, not that nothing flickered: a held
view whose depth no longer agrees with the mesh takes nothing.
`--texture-stats` marks the cameras a remesh textured from a held frame
beside their share of the triangles, which is where that is read.

On the lab rig (M5 Max, Release), defaults otherwise, today's calibration,
runs of 20-40 s, each row's counts from that run's last status line, so the
`sets` column gives the run's length. The hold is the first cut's, 30 sets
and no fallback rank:

| colour | hold | sets | remeshes short of a camera | views held | fuse per set |
|---|---|---|---|---|---|
| 1280 x 720, 30 fps | off | 903 | 17 of 877 (1.9%) | 0 | 6.0 ms |
| 1280 x 720, 30 fps | 30 sets | 637 | 0 of 636 | 13 | 6.6 ms |
| 3840 x 2160, 25 fps | off | 909 | 71 of 823 (8.6%) | 0 | 7.6 ms |
| 3840 x 2160, 25 fps | 30 sets | 946 | 0 of 888 | 101 | 7.5 ms |

Watching the 4K pair, the person at the window saw the first flicker and
the second stay stable. A third 4K run with the validation layer on (626
sets, 64 views held) reported nothing. The reviewed version, 2 s and the
fallback rank, has not yet been run on the rig.

These are also the viewer's first 4K runs (M5 Max, Release): a 7680 x 4320
atlas, 6.7-7.1 ms a remesh, and 24.7-25.8 mesh updates a second at the
cameras' 25 fps.

**The review of the fourth amendment** changed:

- A held view competed with the present ones on score alone, so an image a
  second old could take a triangle a present camera also saw, by facing it
  more squarely. `TextureView::fallback` ranks it below every unmarked view,
  and the several-view test pins both halves: the marked view gives a
  triangle up, and still takes one no other view can.
- The limit counted sets fused, so a rig stall kept a frame of any age
  fresh, and at 30 it equalled the camera's key-frame interval, leaving
  nothing for the decoder's reset. It is now `--hold-ms`, 2 s on the rig's
  clock.
- "A stale image lands only on surfaces that have not moved" was wrong: the
  depth test is geometric, as above.
- Frames were kept with texturing off, costing the reuse for nothing.
- A raw frame missing its colour was described as a case the hold covers.
  The Orbbec driver skips such a pair, so the set lacks the camera instead.

Not taken: a two-slot output ring in `GpuFramePrep`. The review expected
four passes allocating at once to repeat step 5b's 46 ms. Measured, the hold
costs 0.1 ms a set (above), the step 5b entry's conclusion again, now for
four cameras.

Ctrl+C closes the window rather than ending the process, so the rig is
stopped either way. The viewer's gfx pin moves to #98 for `kHybridMeshNormals`,
the View panel's third shading mode.

**Measured** on the lab rig: four Femto Megas at 1280 x 720 colour over the
wired link, H.265, the M5 Max, Release. The run was **uncalibrated** (no
calibration file for these cameras is on the machine), so the four cameras
sit at the origin and the surfaces fuse over one another; the geometry and
texture are not yet judged.

| voxel | fuse per set | mesh | render | atlas copy, GPU | map | recon heap |
|---|---|---|---|---|---|---|
| 2 cm | 9.9 ms | 245 k triangles | 60 fps | 0.03 ms | 7.8% | 1.3 GB |
| 1 cm | 12.7-20 ms | 1.0 M triangles | 60 fps | 0.23 ms | 37% | 2.3 GB |

"Fuse per set" includes the newest remesh's extract and texture, as the
panel then summed them into every set. The panel now reports the two apart,
fuse per set and the newest remesh, and the rig has not been re-run with it.

The map's share is of its initial 131 072 blocks, which grow on overflow.
The cameras deliver a set every 40 ms, so 1 cm fuses with room to spare.

About 2% of sets miss a camera. The SDK drops those frames on the wired link:
"Metadata size is too large!" on the H.265 RTP stream, and depth frames that
arrive short of 640 x 576 x 2 bytes. That is 22 and 9 in a minute. The rig
hands out the set without the camera, and the viewer textures from the rest.
The eight "Stream have not been started!" lines at exit are the SDK stopping
the H.265 streams; `fuse_orbbec` prints the same eight.

**Failures.**

- The last set of a `--sets` run is meshed even when the renderer has not
  collected the previous mesh, as `fuse_viewer`'s final extract is, and is
  skipped once the window has closed.
- A failure that ends fusion (the rig's start, a poll, the frame prep or a
  fuse) makes the run exit 1.
- The rig is stopped on every exit from the fuse thread, an exception
  included.
- A mesh gfx cannot bind is said once and dropped, not re-said every frame,
  and so is an atlas image that cannot be made, until one is made.

A mesh that goes out without an atlas draws in fused colour until the next
remesh. The Rig panel counts each by its reason: an empty extract, no camera
with colour, texturing switched off, or a failed texture pass. The first runs
showed such versions in bursts, eight in one and six in another. Both were the
View panel's texturing checkbox, switched off for about a second and back on,
which the counter named and the person at the window confirmed.

**Verified.**

- The suite passes on the M5 Max, 55 of 55.
- `rig_viewer`, `fuse_viewer` and `fuse_render` build at the new pin under
  `-Werror`.
- On the rig with the Khronos validation layer loaded, the viewer reports
  nothing. That run ended at 60 sets of 237 frames, 12 remeshes, and 12 atlas
  copies from four cameras. A screenshot shows the cameras' images on the
  mesh.
- A SIGINT mid-run takes the clean exit, the rig stopped and rc 0.
- `recon_sensor_gpu_frame_prep` checks the new config:
  - a second family makes the colour output CONCURRENT;
  - the pass's own family named twice, or no config, leaves it EXCLUSIVE;
  - depth is EXCLUSIVE throughout;
  - a count past the array is refused.

  Ignoring the config in `ensure_output` fails it, and so does sharing depth
  with the colour families.
- After the review fixes, `fuse_viewer` on room0 (150 frames, preloaded,
  validation on, across two queue families) committed 39 mesh versions with
  no refused extract and no validation message, through the shared ring
  helpers. `rig_viewer`'s fixes have not been run on the rig.

**Open.**

- A calibration file for these cameras, without which the rig's view means
  nothing yet.
- 4K colour on the rig, which makes a 7680 x 4320 atlas and about 133 MB of
  copies per remesh. (Since run: the fourth amendment above.)
- A CI leg that builds `rig_viewer`: the viewer leg had no Orbbec SDK, so it
  was compiled only where the SDK was installed. (Since done: the viewer leg
  installs FFmpeg and the SDK; the 2026-10-07 rig entry.)

### 2026-09-30 — A rig's cameras fuse in one batch: each dispatch binds a descriptor set of its own over the same kernel, and the set is compacted once.

*Amended 2026-10-06 (below):* the one compaction is now of the blocks the
set's frusta reach, their union, rather than of the whole active set.
*Amended 2026-10-08 (below):* a Dynamic set no longer reproduces its frames
fused one at a time: every frame's clear runs before any frame's fuse.
Classic still does.

**The rule.** `VoxelHashMap::allocate_from_depth` and
`TsdfIntegrator::integrate` take a list of frames (`volume::DepthInput`,
`tsdf::FrameInput`), and record every frame's dispatch, in order, into one
`CommandBatch`: allocation one submit a round, fusion one compaction and one
submit for the whole set. A batch binds a set when it submits and refuses one
rewritten after its dispatch was recorded, so each frame's dispatch binds a set
of its own from `core`'s `KernelSets`, through the set overload of
`CommandBatch::dispatch`. The camera buffers stay shared and are rewritten
inline before each dispatch, which the batch's barrier around every dispatch
orders after the read before it. The one-frame overloads are lists of one.
`fuse_set` (`examples/common/fuse_device_frame.hpp`) allocates and grows for a
set, then integrates it, and `rig_viewer` and `fuse_orbbec --rig --gpu` fuse
through it (PERF.md's P1).

**Why this shape.** Fusion ran one set of kernels per camera, three or more
fence-waited submits each: twelve a set for four cameras. Each voxel belongs to
one thread, so dispatching the frames in order in one batch computes exactly
what separate submits did, Dynamic's clearing included.
`recon_tsdf_integrate_set` fuses three cameras both ways, over the same
blocks, and compares every voxel's bits. The one difference is the compaction:
it runs once, after every frame's allocation, so a block first allocated for a
later camera is fused from the earlier ones too.

PERF.md proposed descriptor arrays and a kernel that loops over the cameras,
reading and writing each voxel once rather than once a camera. That needs
`shaderStorageBufferArrayDynamicIndexing` and a second shader, for a gain in
device time that no one has measured. A set per camera needs neither and
removes the submits.

**Measured.** room0, sets of four 1200 x 680 frames held in device-local
buffers (as `GpuFramePrep` hands them out), 1 cm, Dynamic, Release, metrics
off, the two modes interleaved over three runs of 30 sets, median per set:

| per set | one frame at a time | `fuse_set` |
|---|---|---|
| M5 Max | 6.87 ms | 5.93 ms |
| RTX 5090 | 4.00 ms | 3.16 ms |

Summed over a run of 30 sets with metrics on:

| | M5 Max, one / set | RTX 5090, one / set |
|---|---|---|
| integrate, host | 91.3 / 55.6 ms | 41.7 / 21.6 ms |
| its compaction, host | 24.9 / 6.7 ms | 16.8 / 4.9 ms |
| allocate, host | 127 / 125 ms | 89.3 / 83.7 ms |
| allocate, device | 92.0 / 110.9 ms | 72.1 / 74.2 ms |

Allocation gains least, for two reasons:

- **Its device time is most of it.** It is about 3.7 ms a set on the Mac and
  2.5 on the 5090.
- **A retry round now re-dispatches every frame**, not only the one that lost
  a race. On the Mac 63% of sets need a second round, against 31% of single
  frames, and the device time rose 20%.

Per-frame tallies would re-dispatch only the frames that failed, at the cost of
a second retry loop. The contention's cause is the 27 blocks each pixel probes,
which PERF.md's P3 removes, so that comes first.

The first 5090 run put integrate at 23 ms of device time a set. The bench had
made its frames with `upload_storage_buffer`, which is host-visible, so every
voxel read them across PCIe. The library's own frames are device-local; the
bench was wrong.

**Not taken.**
- Per-frame retry tallies (above).
- The kernel that loops over the cameras: measure integrate's device share
  after P3 first. It is 0.34 ms a set on the 5090 and 1.4 ms on the Mac.

Both are `TODO`s at the code they would change.

**Review.** The PR's review changed five things:

- **A set must outlive the batch.** The batch binds and checks the set object
  it was given at `submit`, and `DescriptorSet` is copyable. So the set
  overload deletes its rvalue twin, as `block_list` does.
- **`KernelSets` grows itself.** It `reserve`s the sets a call needs, one a
  frame, rather than the kernel's own set plus extras. The binding count it
  sizes its pool by is on `ComputeKernel`, recorded by `KernelSetBuilder::add`,
  not repeated at the call site.
- **The timers fit a set.** A set records a device span a frame a round, past
  the timer's 32 at four cameras and nine rounds. `GpuTimer::reserve` raises
  the bound between windows.
- **An empty frame is skipped by both calls.** `allocate_from_depth` skipped
  one and `integrate` refused it, so one camera's empty frame lost a whole set.
- **The retry budget is the call's.** A set gets the ten rounds one frame
  gets, so the doc no longer claims the frames' blocks match one-at-a-time
  allocation unconditionally: they match once every frame's blocks are in.

**Verified.**
- The suite passes on the M5 Max and the RTX 5090 (in the CI image).
- `recon_tsdf_integrate_set` also passes under synchronization validation, and
  fails when only the first frame's camera is uploaded.
- `recon_core_command_batch` dispatches one kernel over two sets in one batch,
  and refuses a set rewritten after its dispatch was recorded.
- `recon_tsdf_integrate_set`'s set carries an empty frame, grows the map's
  sets past a one-frame call, and counts the same dirty blocks as one frame
  at a time. That last check fails when only the first frame's set binds the
  dirty flags.

### 2026-10-02 — Shared instructions live in a concise AGENTS.md; CLAUDE.md imports it, DESIGN.md holds implementation detail, and DECISIONS.md holds the decision index and rationale.

Codex and Claude Code work on the same repository and need the same rules.
`AGENTS.md` is the canonical entry point for shared instructions; `CLAUDE.md`
contains `@AGENTS.md`, with no second copy to drift and no pointer back to it.

The previous `CLAUDE.md` was 1,317 lines and 97,275 bytes. Its architecture,
Vulkan gotchas, tier contracts, examples, and remaining-work notes now live in
`DESIGN.md`; its complete dated index now lives above this record. Existing
decision entries and measured lessons are retained. The short entry point
routes each task to the relevant sections rather than requiring the whole
implementation history before every edit.

When a contract changes, update its current description in `DESIGN.md` and
its index entry and dated rationale here in the same commit. Update
`AGENTS.md` only when an essential shared rule changes. `PERF.md` continues
to hold the live rig work plan. Shared guidance is committed with the project
so either agent can resume from it; concurrent tasks use separate worktrees.

This reorganizes documentation without changing implementation contracts.
The migrated tier descriptions replace the older overview's superseded
incremental-meshing, queue-selection, and timeline-ring status. The existing
color-space rationale remains, with `fuse_render`'s already-shipped sRGB
target described as current behavior.

### 2026-10-02 — GPU regressions guard span ownership, chroma placement, descriptor aliases, invalid depth taps, and zero-near visibility.

*Amended 2026-10-06 (below):* the span-ownership item no longer applies;
the span table is removed with incremental mesh extraction.
*Amended 2026-10-08 (below):* chroma location travels in one place, the
picture's `YuvImage`, which `DecodedPicture` carries.

Five correctness reproductions exposed gaps in otherwise passing tests.

- **Incremental spans are checked before the GPU reuses them.** Removing a
  block, running a full fallback, then allocating a replacement in the freed
  slot left an old span pointing at the survivor's newly compacted triangles.
  Before an incremental dispatch, zero each active slot whose host stamp is
  not the previous extract's serial, including clean newly allocated slots.
  Sum all current active spans afterward, including newly emitted blocks.
  This refines the 2026-08-11 span and incremental-dispatch decisions without
  restoring a whole-table clear or changing the 2026-10-01 changed stamps.
  Both sharing modes now retain all 196 fixture triangles instead of 98.
- **Chroma location travels with the planes.** `DecodedPicture` and
  `YuvImage` carry `ChromaLocation`, left by default for existing callers.
  JPEG decoders declare centre; HEVC preserves the frame's tag, defaulting to
  left when unspecified. The Orbbec handoff preserves it for host and device
  pictures. GPU preparation uses the corresponding luma-grid offsets for
  I420 and NV12 in every storage form. Host resampling specifies the source
  and destination positions and includes the tag in its conversion cache;
  axes without subsampling have zero offset. This amends the 2026-09-28
  preprocessing contract, which assumed left-aligned chroma for every input.
- **Descriptor copies share mutation tracking.** Copies already shared the
  Vulkan set but had separate write counters, so rebinding through a copy
  could silently redirect a recorded dispatch. The counter now belongs to
  shared state. A batch records both handle and counter, rejecting alias
  writes and wrapper reassignment before submitting any work. The wrapper
  passed to `dispatch` must still outlive `submit`, as the 2026-09-30 batching
  decision requires.
- **Every bilinear depth tap must be finite.** NaN and either infinity now
  trigger the same nearest-sample fallback as a non-positive hole, before
  interpolation. A valid nearest sample beside a NaN therefore fuses with
  weight 1, as beside a zero, instead of disappearing with weight 0.
- **Colour sight lines are clipped before projection.** Clip to the depth
  camera's near bound and image side planes in camera space. A lateral
  colour camera at depth zero then ends the walk at the image edge, and a
  tiny positive near bound cannot exhaust the 64 samples outside the image.
  The registered-camera case still has no segment to walk.

**Follow-up: zero holes are not occluders.** The repaired sight-line walk
reached a range-only check that admitted a zero depth when `min_depth` was
zero. Require a strictly positive measurement before the range and occlusion
comparisons. Replacing the fixture's 1.5 m occluder with zero holes must keep
the colour camera visible in the single-view pass and let it win the
multi-view selection at near bounds 0.1, 1e-8 and 0. The GPU regression fails
without the positive-depth guard; the existing positive occluder still blocks
the camera at all three near bounds.

Verification: Release on Apple M5 Max through MoltenVK, FFmpeg and Orbbec
enabled; all 57 CTests pass with `VR_TEST_HEVC_BACKEND=videotoolbox`.
Regressions exercise slot reuse with sharing on/off and with/without an empty
intermediate extract; copied and reassigned descriptor wrappers; all four
depth taps with zero, NaN and both infinities; and near bounds 0.1, 0 and
1e-8 for single-view and multi-view texturing. Chroma tests use continuous
ramps for all six locations and verify host resampling, storage-layout
equivalence, and the SDK frame handoff. JPEG software and VideoToolbox device
pictures prepare within 1 channel code of an independent centred-sampling
and colour-matrix reference, while its left-aligned control differs by at
least 30 codes on the same fixture. CUDA/nvJPEG and physical-camera capture
were not run on this host.

**CI exposed an older swscale interpolation bug.** Ubuntu 22.04 and 24.04
failed the host chroma ramp and the JPEG test's original swscale reference;
the current library passed both. Before libswscale 9, packed RGB's
one-luma-row path rounds vertical chroma weights to zero or one half,
including the one-quarter and three-quarter weights centred 4:2:0 needs
([FFmpeg fix 095f8038fa](https://github.com/FFmpeg/FFmpeg/commit/095f8038fa9180842cd38d4d61c7c47a02aad9ed)).
Host RGB conversion on those libraries now uses the planar RGB filter and
interleaves its output. The JPEG GPU test uses independently calculated
sampling weights and matrix constants, so a broken library cannot define
the expected image. The host ramps cover even and odd rows and columns.
The compatibility targets remain the distro FFmpeg packages in CI's Ubuntu
22.04, 24.04 and 26.04 jobs; no package upgrade is required. Local reproductions
of the converter and JPEG tests pass with FFmpeg 4.4.6, 6.1.1 and 9.0.2,
without relaxing numerical tolerances.

### 2026-10-02 — Device-local allocation is required and buffer residency is explicit.

The memory audit found that `MemoryUsage::DeviceLocal` selected VMA's
`AUTO_PREFER_DEVICE` policy without requiring `DEVICE_LOCAL`. Under pressure
or on a different memory topology, the allocator could therefore choose
non-local memory despite the residency contract. `StorageInput` checked a
borrowed buffer's storage usage and extent but had no memory information to
check; the TSDF device-input tests themselves used mapped upload buffers.
This was a correctness-of-contract gap, not a new measured slowdown.

**Allocation and provenance.** Require `VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT`
for `MemoryUsage::DeviceLocal` while retaining VMA's device preference.
Failure to find or allocate a suitable type is an allocation failure, never
a silent fallback. `HOST_VISIBLE` remains allowed alongside `DEVICE_LOCAL`:
unified and host-visible device memory are local. This corrects the earlier
shorthand equating all host-visible memory with system RAM. The 2026-09-28
measurements describe the selected memory on those runs, not every memory
type carrying that flag.

`BufferMemoryInfo` records the selected type's actual property flags, type
index and heap index. `Buffer::memory_info()` returns an optional value,
absent for an empty buffer or an adoption without provenance; moves and
destruction clear it with the other metadata. Both VMA allocation and the
raw Vulkan exported-buffer factory populate it from the selected memory
type. `Buffer::is_device_local()` tests the recorded `DEVICE_LOCAL` flag.

**Borrowed inputs and migration.** `StorageInput` refuses unknown and
non-local device inputs with `InvalidArgument`, including small images; an
image's contract must not change at a size threshold. This covers depth
allocation/fusion, fusion color, GPU-prep device color and the texturer's
direct device inputs. A caller adopting a raw buffer supplies the actual
bound allocation's metadata as the constructor's final optional argument.
Existing source still compiles without that argument, but such a buffer no
longer qualifies as a device input. A host caller can use the host-array
overload, or explicitly upload/copy into `device_storage_buffer` through a
`CommandBatch`. Transfer-only input copies and documented small parameter
or host-read-table exceptions remain legal; this is not a blanket ban on
mapped buffers or generic compute bindings.

**Regression coverage.** Device-input TSDF tests now upload genuine local
buffers, and unknown adoption is refused by allocation/fusion before any
work, including on an empty grid. Core tests compare allocation metadata
with the physical-device memory table, exercise every compatible raw memory
type usable with the enabled device features, accept types carrying both host
visibility and device locality, reject non-local types when present, and
cover metadata through move construction, live assignment, self-move and reset.
Exported buffer metadata is checked when that extension is available. The raw
allocator skips AMD device-coherent types because `Device::create` does not enable
`deviceCoherentMemory`; a synthetic memory table tests that refusal on other
GPUs too, using a compatible index so a missing guard would allocate a buffer.
No performance claim follows from these checks; discrete-GPU pressure/failure
behavior and timings remain separate hardware evidence.

Release with warnings as errors on Apple M5 Max: the full default suite
passed 45/45, and nine affected core, sensor, TSDF and texture tests passed
with the Khronos layer and synchronization validation enabled. The driver
offered local type 0 (`0x1`) and host-visible local type 1 (`0xf`) on heap 0;
both were accepted. It offered no non-local compatible type and no opaque-FD
export, so those capability-specific branches were not exercised on Apple.

The Linux RTX 4090 run of `27c331f`, Release with warnings as errors,
`VR_WITH_FFMPEG=ON` and `VR_WITH_CUDA=ON` (CUDA 13.4.92), passed 49/49 full
tests and 10/10 targeted tests with forced Khronos synchronization validation.
These included buffer memory and external export, command batching, TSDF
device and camera-set inputs, GPU frame prep, and HEVC/JPEG video paths.
The allocator selected type 1, heap 0, flags `0x1` (`DEVICE_LOCAL`). Raw
non-local types 0/2/3 on heap 1, with flags `0x0`/`0x6`/`0xe`, were refused
by `StorageInput`; type 4 on heap 0, flags `0x7`, was accepted despite also
being host-visible. This verifies allocation policy, recorded provenance
and input acceptance on a discrete GPU; it is not a performance result or
an allocation-pressure stress test. Logs are `tests.log` and `sync-tests.log`
under the remote checkout
`/home/taojin/ws/volumetric_kit/volumetric_kit_recon/.worktrees/gpu-residency-fix-27c331f/`.

### 2026-10-03 — Follow-ups to the 2026-10-02 merges: zero depth is missing in every reader, only an accepted remove moves the topology epoch, and an adopted buffer states its memory.

From a review of #137, #143, #145, #146, #148 and #149.

**Zero depth.** Amends the 2026-10-02 zero-near follow-up, which guarded only
the colour sight line. With `min_depth == 0`, depth allocation unprojected
each hole to the camera origin, integration fused `sdf = -zc` around the
camera, and `occluded_ok` accepted a hole. Every depth reader now requires
`d > 0` before its range test. With all-zero depth and a zero near bound, the
regression allocates no block and writes no weight; it fails with either the
allocation or the integration guard removed.

**Remove.** `VoxelHashMap::remove` moved the topology epoch before checking
its input, so a zero-count or refused call invalidated every cached block list
and forced a full re-mesh. Both overloads now move it in the shared input path,
after validation and before the dispatch.

**Residency.** Amends the 2026-10-02 residency entry. `Buffer`'s adopting
constructor has no default for the memory info: an adopter that omitted it
compiled, then failed on every device input. One that does not know passes
`std::nullopt`. `find_memory_type` always skips protected, lazily allocated
and AMD device-coherent types, which need features `Device::create` leaves
off.

**Kept, with measured costs.**
- Mesh binning scans the grid's whole capacity on the device, as the host scan
  it replaced did. For two triangles on the M5 Max, Release, medians of nine in
  three runs, device time is about 0.06 ms at 16k slots, 0.08–0.10 at 131k,
  0.13–0.30 at 393k and 0.18 at 786k. Scanning only touched slots needs an
  indirect multi-level dispatch, which mesh conversion does not justify.
  Retained scratch is about 16 bytes per slot (0.4% of an 8³ grid's `tsdf`
  and `weight`) plus the largest mesh's inputs and bins.
- Before swscale 9, RGB24 output goes through planar RGB for exact vertical
  chroma weights. A 4K frame on native arm64 Ubuntu 24.04 (swscale 7.5, GCC 13)
  takes 34.5 ms that way against 25.5 ms on the packed path, which rounds the
  weights; the interleave is 0.5 ms of it. GCC's x86-64 baseline leaves the
  interleave scalar; that is not measured natively.

**The bin-plan shader test is removed.** It dispatched the private scan
shaders with hand-copied push constants, against the public-API test rule. Only
it covered uint32 overflow of bin entries, which is unreachable:
`triangle_candidate_offsets` refuses more than 2^32 candidates, and candidates
bound the entries. The public mesh test covers split dispatches, compact-list
growth, the second scan level and the oversized-bin refusal.

### 2026-10-03 — Error handling comes from volumetric_kit_core.

**The rule.** recon's `Status`, `Result`, `VR_TRY`, `VR_ASSIGN`, `VR_CHECK`
and log sink are volumetric_kit_core's base tier, not recon's own.
`core/result.hpp`, `core/check.hpp` and `core/log.hpp` now only name the
core's types and functions in `vr::` (using-declarations, so `vr::Status` *is*
`volumetric_kit::core::Status`) and define the three macros as the core's
`VKC_TRY` / `VKC_ASSIGN` / `VKC_CHECK`. recon's `log_message(level, message)`
calls the core's with source `"vr"`. `third_party/` fetches the core pinned by
commit; `recon_core` links `volumetric_kit::core_base` PUBLIC, and the installed
package re-finds the core at the minor version it was built with.
`core/vk_result.hpp` (`vk_error`, `VR_VK_TRY`) stays here until the core's
vulkan tier exists.

**Why.** recon's and gfx's bottom layers were copies that had drifted, and the
family made `volumetric_kit_core` the one copy every sibling depends on (its
DECISIONS.md, "One core for the family"). The error types go first: they are
small, every tier names them, and sharing them is what lets a recon `Status`
pass to calib unchanged, and to gfx once it adopts the core. The Vulkan core
follows as the core's vulkan tier, seeded from recon's.

**What changed with the merge.** The core's `Status` is the union of the
three siblings': it adds `Code::Numerical` (calib's solver failures), and its
`backend_error(0, …)` aborts, as a success code is no failure. `named_failure`
in `compute_kernel.cpp`, which rebuilt a `Status` through a switch over every
code to prefix a kernel's name, is `with_context(name)` now, so a code the core
adds needs no case here. `Status` and `Result` are `[[nodiscard]]`; across
recon, with the Orbbec, FFmpeg, Assimp and viewer paths built, that flagged
only the five recording calls in `core_command_batch_test`'s threaded test,
which rely on a poisoned batch's `submit` returning the first refusal and now
discard their own `Status` explicitly. The rvalue accessors return `T` by
value. A failed `VR_CHECK` now logs with source `"core"`; the macro calls the
core's `check_failed` itself rather than forwarding to `VKC_CHECK`, which would
expand macros in the condition before stringizing it.

**Why the macros keep their names.** Renaming 837 uses across about 60 files
would conflict with every branch open on 2026-10-03 (six PRs and as many
worktrees). The aliases cost nothing at runtime; a `TODO` in
`core/result.hpp` marks the rename, after which they go.

**Consumers.** The iOS app's `RendererErrors.mm` switches over every
`vr::Status::Code` under `-Werror=switch`, so it learns `Numerical` in the PR
that bumps its recon pin (ios pins its siblings by commit from 2026-10-03).
gfx still has its own `vg::Status`, so the app keeps an `error_code` and a
`describe` overload for each; when gfx adopts the core the two types are one,
and each pair must merge into one overload or it is a redefinition. (gfx has
since adopted the core, at its #100-#106, so the app merges each pair when it
bumps both pins.)

**Validation.** Apple M-series, macOS, Release with warnings as errors, Orbbec,
FFmpeg, Assimp and the viewer: the build is warning-free and all 61 tests
pass, the GPU tests on MoltenVK. CUDA paths build only in CI's CUDA leg.

### 2026-10-04 — The Vulkan foundation comes from volumetric_kit_core.

**The rule.** recon's instance, device, VMA allocator, buffers, images,
descriptors, shader modules, compute pipelines and kernels, command batches,
GPU timers, stage metrics and exported buffers are volumetric_kit_core's
vulkan tier, not recon's own. The headers under `core/` keep their names and
now only name the core's types in `vr::` (using-declarations, so `vr::Device`
*is* `volumetric_kit::core::Device`); `core/vulkan.hpp` forwards to the core's
umbrella, and `VR_VK_TRY` is the core's `VKC_VK_TRY` under recon's name, with
the other macros. `recon_core` compiles only its version query and links
`volumetric_kit::core_vulkan` PUBLIC; recon no longer fetches or compiles VMA.
What stays recon's is what no other sibling needs: the camera and colour-space
vocabulary, the vector types, and `device_requirements()`.

**Why.** The core's vulkan tier was seeded from recon's foundation and then
hardened across five stages (core V1–V5): explicit memory placement, so a
discrete GPU never reads kernel memory across the bus; a command batch with
staged uploads; timers with tagged spans; exported buffers counted against
the heap budget; and the shared device. Keeping recon's copy would have meant
two foundations drifting again, and gfx and the iOS app could not hand a
`VkBuffer` across without two device types meeting at the seam.

**What recon states.** `device_requirements()` is the one statement of what
recon's kernels need: the core's defaults (a compute queue, timeline
semaphores) plus `scalarBlockLayout`, every recon shader's buffer ABI, and
`VK_KHR_external_memory_fd` and `VK_EXT_metal_objects` as optional
extensions. The core's default leaves `scalarBlockLayout` off, as calib needs
none; so `Device::create(instance, gpu, {})`, which was right when `{}` was
recon's own `DeviceConfig`, would now make a device recon's kernels cannot run
on. Every call site passes `device_requirements()`, and an embedder merges it
with its own. A device made without it is refused rather than run on: every
recon `create` that builds kernels first calls `check_device_requirements`,
which holds the device to `device_requirements()` through the core's
`Device::check_enabled` (core #11). A device records what it enabled, as
Vulkan cannot be asked, so the check sees an adopter's declaration too.
`core/fwd.hpp` forward-declares the core's classes in its
namespace and names them in `vr::`: a `class Device;` in `vr::` would declare
an unrelated class that collides with the using-declaration.

**What changed with the merge.**

- *Memory is placed explicitly.* `DeviceLocal`, `HostVisible` and `Auto` are
  gone; a buffer is `DeviceOnly`, `DeviceMapped` (device-local, host-mapped)
  or `Staging` (host memory, transfer only). recon's tiers already allocated
  through `device_storage_buffer` and the command batch, so three sites
  changed. `GpuFramePrep`'s kept upload buffer is `Staging`. The colour-parity
  test reads its results back through a batch instead of mapping what the
  kernel wrote.
- *The block-span table.* The marching-cubes span table was the one buffer
  the host and a kernel both read and wrote in place, in host-visible memory
  a discrete GPU's kernel reaches across PCIe. The core gives a shader no
  host memory, and its mapped device memory is the BAR window on a discrete
  GPU, which the host reads uncached (and a write-only mapping not at all),
  may not exist, and can fill. So the table is device-only, and the host
  keeps a copy: each dispatch reads back the spans up to the highest active
  slot in its own batch, an incremental extract uploads the entries it
  clears, and a grow copies the table on the device. `block_spans()` returns
  the copy, and `arena_bytes` counts its 16 bytes per block.
- *Exported buffers take an allocator.* The core makes them through VMA so
  they count against the heap budget, so `CudaPictures` needs an `Allocator`.
  `HevcDecoder::Options`, `JpegDecoder::Options` and `OrbbecStreamOptions`
  gain an `allocator` beside `device`. Without one, NVDEC's and nvJPEG's
  pictures come to the host, said once as a warning like any other device
  path that does not open.
- *Tests.* Nine tests of the foundation itself (`device`, `compute_raii`,
  `compute_smoke`, `core_buffer_memory`, `core_buffer_sharing`,
  `core_command_batch`, `core_external_memory`, `core_memory_stats`,
  `core_stage_metrics`) are gone. The core's suite covers the same ground,
  having been ported from them. `vulkan_smoke` stays, now creating a device
  from `device_requirements()`: that recon's statement is one a real driver
  meets is recon's to test. *Amended 2026-10-10:* the shared GPU test harness
  now performs that check for every GPU test; the standalone printing-only
  smoke executable is removed.

**The viewer.** `examples/viewer/shared_device.hpp` was the neutral bootstrap
of the 2026-08-02 decision; the core's `SharedDevice` is that bootstrap
generalized (it replaces this one and the iOS app's), so the viewer's header
now only makes the GLFW surface and converts gfx's requirements and payload,
as gfx still has a device type of its own. (Since 2026-10-05 it converts
nothing: gfx takes the core's types. Since 2026-10-12 it is gone: gfx's
`gfx_glfw` makes the surface and the device.) The queue-plan preference is the
core's, unchanged. Every queue now has a mutex (it was set only under the
shared-queue plan), as the core's `wait_idle` is a third thread touching both
queues.

**The VMA link hazard.** Until gfx adopts the core, a program linking both
carries two VMA implementations: gfx's `vma_impl.cpp`, and the core's,
compiled into `core_vulkan`'s `allocator.cpp`. A static link loads an archive
member only for a symbol still undefined, so it links when the core's
`allocator.o` is loaded first -- as in the viewers, whose link line names
`core_vulkan` before `gfx_core` -- and fails with duplicate symbols when gfx's
`vma_impl.o` is. The iOS app links both, so it bumps recon and gfx together,
once gfx compiles no VMA of its own. (gfx #104 removed its `vma_impl.cpp`, so
the hazard is gone at the viewers' gfx pin, #106, from 2026-10-05.)

**Validation.** Apple M5 Max, macOS, Debug with warnings as errors: all 39
tests pass, and all 52 with FFmpeg, Assimp and Orbbec (the camera tests skip
without a serial), each run with the Khronos validation layer forced on and no
message reported. GCC 16, Release: builds warning-free. The viewers build;
`fuse_viewer` fused 60 Replica room0 frames on the core's shared device (two
families on MoltenVK) under validation, drawing through gfx with no message.
CUDA paths build only in CI's CUDA leg.

### 2026-10-04 — recon writes the core's types and macros under the core's names (amends the two entries above).

**The rule.** recon names volumetric_kit_core's types, functions and macros as
the core does: `core::Status`, `core::Device` and `VKC_TRY`, `VKC_ASSIGN`,
`VKC_CHECK`, `VKC_VK_TRY` in recon's namespaces, and
`volumetric_kit::core::` (a `vkc` alias) in tests, examples and consumers.
The eighteen `core/` headers that re-exported the core's names into `vr::` are
gone, and so are recon's four macro aliases. What stays under `core/` is
recon's own: `device_requirements.hpp` (was `device.hpp`; only
`device_requirements()` and `check_device_requirements()`), `log.hpp`
(`log_message`, source `"vr"`), `fwd.hpp` (the core's classes forward-declared
in the core's namespace), and the camera, colour-space and vector vocabulary.
recon declares no `core` namespace of its own: `core::Status` in recon's
namespaces finds `volumetric_kit::core` only because nothing nearer is named
`core`.

**Why.** The aliases kept the branches open on 2026-10-03 merging cleanly
("Why the macros keep their names", above). The one still live, #162, now
waits for the migration to finish, so that reason is gone, and two spellings
of one type cost every reader: the viewer met the same device as `vr::Device`
and `vkc::Device`, and each re-export header was one more file to keep in step
with the core's. calib, and gfx from its #100, still name the core's types in
their own namespaces; recon no longer does. (The viewer's gfx pin, #98,
predates that and still has gfx's own `vg::Status`; since 2026-10-05 it is
gfx #106, which names the core's types as recon does.)

**What it costs.** #162 and the August branches (#70, #73–#75, #78) rebase
over about 180 files. The rewrite is mechanical: the four macros, `vr::X` →
`vkc::X`, and the include map in the CHANGELOG. The iOS app, which pins recon
from before the core, takes the new spellings when it bumps the pin.

**Validation.** Apple M5 Max, macOS, Debug with warnings as errors, Orbbec,
FFmpeg, Assimp and the viewer: the build is warning-free and all 52 tests
pass. The CUDA paths build only in CI's CUDA leg; their diff was read by hand.

### 2026-10-05 — The viewers build on gfx's core migration.

*Amended 2026-10-08 (the streamed-atlas entry, below):* the viewers pin gfx
#123, and recon the core at #18, which that gfx requires.

The viewers' gfx pin moves from #98 to #106. gfx #100-#106 moved gfx onto
volumetric_kit_core and dropped its own names for the core's types, so recon
and gfx now share one `Status`, `Device`, `AdoptedDevice`,
`DeviceRequirements`, `Image` and `StageRow`. Amends the device and timing
seams recorded above ("gfx still has its own `vg::Status`", "converts gfx's
requirements and payload", "The viewer's gfx pin, #98, predates that", and
`examples/viewer/stage_metrics.hpp` as the row mapping) and resolves the
2026-10-04 VMA link hazard.

- **The device seam converts nothing.** The viewers build the shared device
  from the same `WindowedAppConfig::device` they adopt the app with, so gfx's
  requirements have one source, and hand `WindowedApp::adopt` the core's
  `graphics_payload()` as it is; the two field-by-field converters, and the
  `TODO`s that waited for gfx, are gone.
- **The timing seam converts nothing.** gfx's `FrameMetrics::sections` is a
  vector of the core's `StageRow`, the type recon's `StageMetrics` holds, so
  the fuse thread publishes its `rows()` as they are, the render thread
  appends them to the overlay's sections, and `stage_metrics.hpp` is deleted.
- **One VMA.** gfx #104 removed its `vma_impl.cpp`, so a program linking both
  carries only the core's, and the iOS app's condition for bumping recon and
  gfx together is met.
- **What did change.** The viewers name gfx's former types as the core's
  (`vkc::Image`, `vkc::DescriptorPool`, `vkc::Status`), and the two
  `wait_idle` calls whose `Status` gfx now returns `[[nodiscard]]` report a
  failure in the exit code. Memory placement changed too: gfx's `DeviceLocal`
  preferred VRAM and fell back to host memory, but the core's `DeviceOnly`,
  which also places every image, refuses (the 2026-10-02 device-local rule).
  If VRAM runs out, `rig_viewer` says so once on stderr: its colour-by-camera
  tiles keep their camera colour, and a remesh without an atlas image keeps
  the previous view until one is acquired.

**Validation.** Apple M5 Max, macOS, Release with warnings as errors, Orbbec,
FFmpeg and the viewer: the build is warning-free and the 43 tests that need
no camera pass. `fuse_render` on 60 Replica room0 frames renders a PNG
byte-identical to the #98 pin's; `fuse_viewer` fused 120 frames on the shared
device under the Khronos validation layer with no message. The live rig
(`rig_viewer`) could not be run: the cameras' network was down at the time.

### 2026-10-06 — The family's camera vocabulary is recon's `camera` tier: a double-precision camera model and the sensor array's calibration file, version 2.

*Amended the same day (below):* the tier keeps only what a capture uses.

calib solves for camera models and writes the rig's calibration; recon's
drivers report their factory calibration and read the poses; ios fills the
same types from ARKit. Until now each had its own: recon a float
`sensor::LensCamera` and the rig file in `sensor`, which links `recon_core`
and so Vulkan; calib a double `PinholeBrownConrady` of five coefficients,
which cannot hold the Femto Mega's factory `k4`-`k6`. Core PR #14 proposed
a camera tier in the core and was closed (camera stays out of the core). The
2026-10-06 plan makes recon the library calib and ios build on for cameras and
sensors; this tier is its first step. The sensor interface (`IRgbdSensor`,
`RgbdFrame`), the vendor-neutral `SensorArray` that replaces `OrbbecRig`, and
pipelined processing follow (DESIGN.md, [Next work](DESIGN.md#next-work)).

- **A tier of its own, under `sensor`.** `recon_camera` links `core_base` and
  GLM only, so calib's solver links no Vulkan to use it. Configuring recon
  still finds Vulkan for its other tiers; calib finds it anyway, for its own
  GPU work on the core's vulkan tier, so a camera-only configure would serve
  no consumer. *Amended 2026-10-11:* calib's solver, which builds with no
  Vulkan SDK, will take its camera model from this tier (decided 2026-10-11;
  calib's CA1), so it is that consumer: `VR_CAMERA_ONLY` builds and installs
  the tier alone, with no Vulkan, glslc or core vulkan tier.
- **Double on the host, float once at upload.** Calibration solves in double
  and the file is written in the shortest form that reads back exactly. float
  would save nothing measurable (a set converts a few models) and cost both:
  Ceres evaluates in double, and float rounds a written calibration.
- **One projection for every scalar.** `distort_rational<T>` and
  `project_rational<T>` take plain arrays: double on the host, float in the
  sensor tier's lens, which `lens.glsl` mirrors, and a solver's dual numbers.
- **OpenCV's rational model only.** Its eight coefficients cover
  Brown-Conrady (`k4 = k5 = k6 = 0`) and the Femto's `BROWN_CONRADY_K6`.
  `CameraModel` carries no model tag yet; the file's `distortion.model` does,
  so a fisheye model is a tagged alternative, not a format change.
- **Projection and unprojection stay inside the invertible radius**: past
  it, a pixel has several rays, and past a pole a point images at a
  mirrored pixel. `project` refuses a point out there; `unproject` is calib's
  Newton with backtracking, generalised, and measures its residual with
  `project`'s own arithmetic, so its 1e-9 px bounds the round trip exactly.
  The radius is where the distorted radius stops growing or the denominator
  reaches zero, found by a 2% scan in r² and bisection, since the rational
  model has no closed form for it. The scan takes 1.5-1.7 µs against an
  unprojection's 0.044 µs (M5 Max, Release), so a `CameraProjection` checks
  a model and finds its radius once, and projects and unprojects with it; a
  free function taking the radius as a number could be handed another
  lens's.
- **Rescaling refuses another aspect ratio** (the half-pixel rule otherwise):
  the Femto's 4:3 colour modes crop its 16:9 sensor.
- **The calibration file, version 2** (`camera/array_calibration.hpp`):
  `format` and `version`; sensors by id, each with its colour camera's pose
  (OpenCV's extrinsic, as version 1), colour and depth models that record
  their `width` and `height` and whether they are `factory` or `calibrated`,
  and `depth_to_color`; and an optional world, a sensor or an AprilTag. Every
  field of a sensor is optional, so a calib session records factory lenses in
  the same format, unposed. Unknown keys are ignored and a later version is
  refused. Version 1 is read and never written: its intrinsics are kept only
  where they record their image, and `optimal_intrinsics`, the legacy
  pipeline's undistortion target, is dropped. A write goes to a temporary
  file of the process's own, synced and renamed into place, beside a
  symbolic link's target, with the replaced file's permissions.
- `OrbbecRig::Options::calibration` takes the `ArrayCalibration`, which must
  pose every camera of the rig. *Amended 2026-10-07 (below):* `OrbbecRig` is
  gone; `SensorArray::Options::calibration` takes it.

**Validation.** Apple M5 Max, macOS, Release with warnings as errors, Orbbec,
FFmpeg and the viewer: the build is warning-free and the 54 tests pass; the
camera tests that need the rig skip without it. `project` reproduces
`cv::projectPoints` to 1e-9 px and `rotation_from_rodrigues` `cv::Rodrigues`
to 1e-15 (OpenCV 5.0.0). Mutation check: dropping the half-pixel term,
swapping `p1` and `p2`, ignoring the denominator's pole, taking the axis
from `sin` past pi/2, keeping version-1 intrinsics without a size, loosening
the world sensor's origin check, letting Newton leave the invertible radius,
and inverting `depth_to_color` each fail a test; so do dropping `project`'s
radius check, measuring `unproject`'s residual in normalized coordinates (as
first written: 5 of 518,400 pixels of a rig lens came back past 1e-9 px),
and writing through a symbolic link or without the old permissions. Every
pixel of the four rig lenses round-trips within 1e-9 px.

### 2026-10-06 — The camera tier keeps only what a capture uses (amends the entry above).

*Amended 2026-10-12 (below):* calib is the first caller; version 2 (under
`device_calibration`), its writer and `rodrigues_from_rotation` return.

Once the tier merged, most of it had no caller outside its own tests:
`CameraProjection` (host projection and Newton unprojection),
`invertible_radius2`, `scale_camera_model`, `project_rational`, version 2 of
the calibration file with its writer, and `rodrigues_from_rotation`.
Reconstruction undistorts on the GPU with the forward model alone (the
2026-09-28 pre-processing decision), so it never inverts a lens, and calib,
the intended user, keeps its own camera model and links none of this. They
are removed, about 1,400 lines with their tests; each comes back with its
first caller.

- **What stays.** `CameraModel` and `check_camera_model` (`RgbdFrame` and the
  GPU frame prep carry and check it); `distort_rational<T>` (the sensor
  tier's float lens, which `lens.glsl` mirrors, and the tests' double
  reference); `check_rigid`, `rigid_inverse` and `rotation_from_rodrigues`
  (the driver's and the array's checks, and reading a pose).
- **The calibration file is the family's `device_calibration` layout
  again**, read for each sensor's pose by serial; lens fields and every other
  key are ignored. Version 2 had no writer outside the tests and no file in
  use: the rig's file is version 1.
- **`SensorCalibration::color_to_world` is a plain `Mat4d`**, since every
  sensor the file lists has a pose.

**Validation.** Apple M5 Max, macOS, Release with warnings as errors, Orbbec,
FFmpeg and the viewer: the 54 tests pass, and the rig's calibration file
reads all four poses. The rig hardware test, which now writes its file as
JSON, was not run.

### 2026-10-06 — Tests use no real hardware (amends the 2026-09-26 Orbbec driver entry).

*Amended 2026-10-07:* no test looks for a camera either. The SDK smoke test
checks only the runtime version, and no longer opens a context and
enumerates devices, which probed USB and the network; the conversion test's
refusal of an absent camera went with the same probe. Every test passes
with the process denied network access.

`recon_sensor_orbbec_capture`, `recon_sensor_orbbec_gpu_prep` and
`recon_sensor_orbbec_rig` are removed, about 1,260 lines. Each skipped unless
an environment variable named a camera or the rig's sync file, and no CI leg
sets one, so they ran only by hand at the rig. A test now runs wherever the
suite builds; a camera is checked by hand through `fuse_orbbec`, and reported
as run or not run.

- **What stays, with no camera:** the conversion test (units, layouts, the
  sync-mode mapping, the option checks `open` makes, and now the refusal of
  H.265 colour without the decoders, moved from the capture test), the
  sync-configuration, grouping, H.265 and MJPEG tests, and the SDK smoke
  test, which passes with no device.
- **What goes:** the checks only a camera gives — the frame contract on a
  live stream, restart, the GPU pass against the SDK's host alignment, and
  the rig's trigger sets. The measurements they produced stay in the entries
  that cite them.

### 2026-10-06 — A sensor's frame is `RgbdFrame`: each camera's `CameraModel`, the poses in double, a sequence number, and the pixels it holds; and the Femto Mega's factory extrinsic is made a rotation.

`RgbdFrame` (`sensor/rgbd_frame.hpp`) replaces `RawFrame`, the second step of
the plan above. It is the frame every driver will hand out once
`IRgbdSensor` lands; `sensor::LensCamera` and `LensDistortion` are gone.

- **Each camera's `camera::CameraModel`**, lens included, in double.
  `GpuFramePrep` narrows it to float once, as `lens.glsl` reads it, and
  refuses a model past float's range.
- **The colour camera's pose and the sensor's extrinsic**, not two poses:
  `color_to_world` is where the sensor sits, from a calibration or a
  tracker; `depth_to_color` is the sensor's own. The pass poses the depth
  camera at their product and refuses either unless rigid.
  `OrbbecCapture::Options::cam_to_world` is double too and refused at `open`
  unless rigid, so the driver never hands the pass a pose it refuses.
- **A sequence number**, the device's count of its frames, so a gap is a
  lost frame; the Orbbec driver stamps the depth frame's index.
- **The pixels it holds** (`pixels`), not ones it borrows until the next
  poll: a sensor array groups frames by trigger before it prepares any, so
  it keeps several of each sensor. The Orbbec driver hands over the SDK's
  pair with the SDK context, so its buffers go back once every copy of the
  frame is gone, before the SDK does, even after the capture.

**The Femto Mega's factory depth-to-colour rotation is not a rotation.**
Read through the SDK (2.9.3, firmware 1.3.1) from all four cameras of the
lab rig, it describes the depth camera's tilt, 6.1° to 6.7° about x, but its
first element is cos θ where Rodrigues' formula gives
cos θ + (1 − cos θ) k_x² ≈ 1: the other eight match that formula to 1e-5,
and the rows are 1.15% to 1.36% off orthonormal. The raw path used the
matrix as it came, so depth landed 0.6% short along the colour camera's x
(about 4 px at a 720p image's edge), and fusion, which inverts a pose by its
transpose, doubled the inconsistency. `camera::nearest_rotation`, the polar
factor, recovers the Rodrigues rotation to about 1e-5 rad, and the driver
applies it to every extrinsic it reads, refusing at `open` one that does not
come out a rotation (a zeroed or reflected matrix). The hardware test's comparison of the
prepared depth with the SDK's own host alignment moved from a 5.3 mm to a
6.7 mm median, consistent with the SDK aligning through the defective
matrix. Which is closer to the scene is for calib to measure, against the
colour board.

**Validation.** Apple M5 Max, macOS, Release with warnings as errors, Orbbec,
FFmpeg and the viewer: the 51 tests pass, `recon_sensor_gpu_frame_prep` on
the device with the new pose composition and refusals. Before the camera
tests went (the entry above), on the rig's primary, CL2A141000N, over Wi-Fi:
`recon_sensor_orbbec_gpu_prep` passed raw H.265, raw MJPEG and the forced
host-colour path, the first frame kept intact through the 45 polls after it
and prepared, and the sequence rising.

### 2026-10-06 — A sensor is an `IRgbdSensor`: what it is before a frame (`SensorInfo`), frames polled newest or drained oldest first; and the Femto Mega is `OrbbecSensor`.

The third step of the 2026-10-06 plan (`sensor/rgbd_sensor.hpp`): the
interface every driver implements, used on its own or by the sensor array
that replaces `OrbbecRig`.

- **`SensorInfo` is what a sensor is before a frame exists**: its id (the
  serial a calibration names), its cameras' factory models at the opened
  modes, its factory `depth_to_color`, its rig role, which clock its
  timestamps are on, whether a calibration or the sensor itself poses it,
  and its rate. An array needs each of these before it starts: the roles
  order the starts, grouping needs one clock, the id finds the pose.
- **Polled, not called back, and two ways to take frames.** A sensor holds
  up to `set_queue_depth` frames, set before `start`; `poll` hands out the
  newest and lets the older go, `drain` hands out every held frame oldest
  first. A standalone consumer polls; an array sets a deeper queue and drains,
  so frames of one trigger are still held when it groups them -- the
  Orbbec mailbox's `take`/`take_all`, made the contract. A `drain` that
  fails partway keeps the frames before the failure, handed out, rather
  than losing good frames to a sensor that just broke.
- **Counters every driver has** (`SensorStats`: received, delivered, dropped,
  failed). A driver keeps its own beside them: `OrbbecSensor::orbbec_stats`
  splits failed into failed and lost and counts host pictures. A raw MJPEG
  pair the JPEG decoder had no time for is dropped, as the mailbox's are,
  not lost, so backpressure never reads as decode failures.
- **`OrbbecSensor`** is the raw path behind the interface: frames as
  captured, each holding its SDK pair, models and extrinsic from the
  factory calibration, the pose given in double. `open` refuses what would
  only fail later: a `min_depth` of 0, which the GPU pass refuses, and a
  sync mode the driver does not know, whose start an array could not order.
- **`sync_clock_to_host` syncs this camera alone, once per `start`.** The
  context's `enableDeviceClockSync` runs one SDK thread over every camera
  the process created (`DeviceManager`, SDK 2.9.3), so one sensor's option
  would re-sync another that reports its own clock, and run on after it. The
  camera's own `timerSyncWithHost`, before it streams, keeps timestamps
  monotonic within a session, and a camera that cannot sync fails `start`
  rather than reporting `ClockDomain::Host` falsely. Its drift over a
  session is unmeasured (a `TODO(sensor)`). *Amended 2026-10-07 (below):*
  measured, 3–6 µs/s between cameras, so the option now stamps the SDK's
  global timestamps, the camera's clock mapped onto the host's; the one
  sync before each `start` stays.
- **Beside `OrbbecCapture` for now.** `OrbbecCapture`, `ICameraCapture`,
  `CapturedFrame` and the SDK's host path go when the Replica source moves
  onto the interface (a `TODO(sensor)`), and `OrbbecRig` when the array
  lands. *Amended the same day (below):* `OrbbecCapture` and the host path
  went first, apart from the Replica source. *Amended 2026-10-07 (below):*
  the Replica source is an `IRgbdSensor`, and `ICameraCapture` and
  `CapturedFrame` are gone.

**Validation.** Apple M5 Max, macOS, Release with warnings as errors, Orbbec,
FFmpeg and the viewer: the 52 tests pass. `recon_sensor_rgbd_sensor` drives
a fake through the interface, and `recon_sensor_orbbec_conversion` checks
what `OrbbecSensor::open` refuses before it touches the SDK. `OrbbecSensor`
was checked by hand, not by a test (tests use no hardware), through
`fuse_orbbec --gpu`, which reads one camera through it, on CL2A141000N: H.265
and MJPEG each fused 60 frames at about 30 fps, printing both cameras'
factory models, every received pair accounted for (H.265: 64 received, 60
fused, 3 dropped, 1 lost). With `--host-clock` the first frame sat 197-243
ms behind the host's system clock; without it, 276 ms, the camera keeping
the clock the run before set. The example does not `drain`: the fake checks
it, and the array's example will at a camera. *Amended 2026-10-10:* the fake
queue only tested its own implementation. The sensor vocabulary test now
checks defaults, return helpers, copied-frame ownership and enum names directly;
production array and decoder behavior remains covered separately.

### 2026-10-06 — Several sensors are a `SensorArray`, vendor-neutral: started in their rig's order, grouped by trigger or by sequence number, and posed by the calibration.

*Amended 2026-10-06 (trigger only, below):* `SyncMode::Sequence` went, and
with it `SyncMode`: an array groups by trigger only.
*Amended 2026-10-07 (the rig entry, below):* `OrbbecRig` and its start order
are gone, `TriggerGrouper` is the array's internals, and `tolerance_us`
defaults to 0.4 of the fastest member's frame period.

The fourth step of the 2026-10-06 plan (`sensor/array/sensor_array.hpp`,
target `recon_sensor_array`): the array of `IRgbdSensor`s that replaces
`OrbbecRig`, built from what `OrbbecRig` did that no vendor owns.

- **What it reads of each sensor is its `SensorInfo`.** The roles order the
  starts (secondaries, then free-running sensors, the primary last) and the
  stops (the reverse); the id finds the pose in the `ArrayCalibration`, which
  must pose every sensor and is stamped on each frame's `color_to_world`.
  What a vendor's rig needs beyond that -- checking and writing sync settings,
  the SDK's clock sync -- is set when its driver opens the sensor.
- **Two ways frames form sets.** `SyncMode::Trigger` is the rig's grouping,
  `TriggerGrouper` moved out of the Orbbec driver into the sensor tier
  unchanged: around the primary's frames, within `tolerance_us` on the host's
  clock, a missing secondary waited for a frame and a half of the slowest
  sensor, the newest ready set handed out. It needs one primary, the rest
  secondaries, all on the host clock. `SyncMode::Sequence` groups equal
  sequence numbers, every set in order once each sensor has sent its frame,
  moved past it or ended; it reads no clock, so a recording replays the same
  sets every time, which is what calib's session replay needs. A frame for a
  set already handed out joins none, and a sensor ahead of one that has
  fallen behind holds `queue_depth` frames, its oldest let go, so the array's
  memory is bounded in both modes.
- **Lifecycle.** A failed start, a stop, the destructor and a move over a
  running array all stop the primary first. Starting a running array starts
  each sensor again, as `OrbbecRig` did, so a camera that has gone away says
  so. A refused `open` leaves the sensors with the caller, and a drain that
  fails partway has its earlier frames, counted delivered, grouped before the
  failure is returned.
- **Refused for now, each a `TODO:`:** tracked sensors (an iPhone's
  pose needs registering to the array's world) and free-running members of a
  triggered array (joined by their nearest frame).
- `process(set)`, every stream of every sensor in one GPU batch, is the next
  PR; then `OrbbecRig` goes and `rig_viewer` and `fuse_orbbec` open an array of
  `OrbbecSensor`s.

**Validation.** Apple M5 Max, macOS, Release with warnings as errors, Orbbec,
FFmpeg and the viewer: the 54 tests pass. `recon_sensor_array` drives
scripted sensors through the refusals, the start and stop order, a failed
start, a start of a running array and a move over one, trigger sets
(complete, a silent secondary, the newest set, frames without a clock) and
sequence sets (out of turn, a missing frame, waiting on a live sensor, a
recording's end, a late frame, a sensor far ahead, a restart), and a drain
that fails partway; 50 runs in a row pass, its waits notwithstanding. Not
applying the poses, not waiting on a live sensor in sequence mode, and not
checking the clock each fail it, as does undoing any lifecycle or sequence
rule above. `recon_sensor_trigger_grouper` is the grouping test the rig's
was; the rig's start order is `recon_sensor_orbbec_start_order`.

### 2026-10-06 — An array's set is prepared in one batch: every stream of every sensor, one submit and one wait, host colour staged on a thread per camera first.

*Amended the same day (the device-only `GpuFramePrep` entry, below):* colour
comes on the device only, and with only depth to stage, the thread per
camera measured level with one thread and went.

`SensorArray::process(set)` prepares a set through
`GpuFramePrep::prepare_batch`, a pass per sensor: every frame is checked
first, so a refused set costs no work; device planes are taken over; each
pass stages its host data on a thread of its own; then every pass records
its uploads, and after them every pass its kernels, into one `CommandBatch`,
submitted and waited on once. Recording the uploads first saves a barrier
a camera: they write buffers of their own, so none is needed between them.
`prepare` is the same steps for one frame. `prepare_set`, a thread and a submit per camera,
is removed: `fuse_orbbec --rig` and `rig_viewer` call `prepare_batch` until
they open an array, rather than keep two ways to prepare a set. *They open
one since 2026-10-07 (below).*

**Why one batch, given PERF.md's P2 found it no faster.** It is not: it is
level. The 2026-10-01 measurement lost on the Mac's host colour because the
batch staged four 4K frames one after another; staging them on threads
first closes that. Median of 100 sets of four 640 x 576 depth and
3840 x 2160 colour frames, M5 Max, Release, each pair interleaved, three
runs: one batch against a thread and a submit per camera, 1.68 vs 1.66 ms
for host colour and 0.97 vs 0.95 ms for device colour; the uploads recorded
first against each pass's uploads and kernels together, 1.69 vs 1.74 ms and
0.99 vs 1.01 ms.

What it buys is one batch a set, the shape the pipelined stages (DESIGN.md's
Next work, step 4) need: a set's work submitted once, through the core's
`submit_async`, behind the previous stage on a timeline, where four threads
would make four submits to order. Until then it submits and waits, which a
`TODO:` at the submit marks. A failed submit abandons every pass's staging
and device planes, since the work may still run and `CommandBatch::submit`
cannot say whether it does; `PendingBatch::in_flight` will. The RTX 5090 was
level before the threaded staging and is not re-measured.

**Validation.** Apple M5 Max, macOS, Release with warnings as errors,
Orbbec, FFmpeg and the viewer: the 55 tests pass.
`recon_sensor_array_process` prepares a three-sensor recording's sets -- one
missing a frame -- and holds each prepared frame to `prepare` on its own,
byte for byte, posed by the calibration. `recon_sensor_gpu_frame_prep` does
the same for a batch whose colour comes as host planes, device planes taken
over from outside Vulkan and NV12 plane images, with depth kept within
colour; it checks that a refused set times no row, so no work began, and
that a dozen cameras' set is still timed (the timer's default window, 32
spans, held eight cameras' host colour). Both run clean under the Khronos
layer's synchronization validation. Dropping the timer's reserve, or opening
the stage before the checks, fails the test. The split `prepare` was checked
by hand on CL2A141000N over Wi-Fi through `fuse_orbbec --gpu`: the default
stream, `--mjpeg` and `--hevc` each fuse 30 frames into a mesh. `fuse_orbbec
--rig` and `rig_viewer` on `prepare_batch` build, and were not run on the
rig.

### 2026-10-06 — Every Orbbec frame is handed out as captured: the SDK's host path, `OrbbecCapture` and `fuse_orbbec --gpu` go (amends the 2026-09-26, 2026-09-27 and 2026-09-28 Orbbec entries).

**The rule.** Once a frame arrives it goes to the GPU and is processed there.
The Orbbec driver therefore has one path: raw depth and the decoded colour,
each camera with its factory model, for `GpuFramePrep`. Removed:

- `OrbbecCapture`, the SDK's host undistortion and registration
  (`ob::UnDistortionFilter`, `ob::Align`), the host conversion to metres and
  packed RGB, and `OrbbecStreamOptions::raw`.
- The rig's host reads (`poll`, the processed `poll_set`) and `poll_raw`:
  `OrbbecRig` is no longer an `ICameraCapture`. Its one read is `poll_set`,
  formerly `poll_raw_set`, handing out an `OrbbecRigSet` of `RgbdFrame`s, and
  `color_camera` is the factory `CameraModel`, with `color_to_world` beside it.
- `ICameraCapture::poll_raw` and `raw_frames`, which only the two Orbbec
  classes implemented.
- The H.265 decoder's RGB frames and the RGB profile it stamped on them.
- The RGB mode `open` read each colour camera from, and its byte comparison
  with the mode streamed. Only the host path streamed the RGB mode, so the
  comparison could refuse a camera over a calibration nothing used, with no
  path left to open it. A colour camera is read off the H.265 or MJPG mode
  streamed, the pictures the decoder hands out. The two matched byte for byte
  on the Femto Mega at 720p, 1080p and 4K, so the rig's models are unchanged.
- `fuse_orbbec`'s `--gpu`. It always reads raw, H.265 unless `--mjpeg`, and
  every poll goes through `prepare_batch` and `fuse_set`, one camera as a
  set of one. `--host-clock` stays for one camera: it is not host
  processing, and it is how `OrbbecSensor::Options::sync_clock_to_host`,
  which a `SensorArray` grouping by trigger needs, is checked by hand.
- `VR_ORBBEC_WITH_VIDEO`: `VR_WITH_ORBBEC` now needs `VR_WITH_FFMPEG`, since
  a driver without the decoders could open no camera. Both CI legs that
  build the driver build FFmpeg.
- The `TODO(calib)` comparing the made depth-to-colour rotation with the
  SDK's matrix through its host alignment: that alignment is gone, and
  calib's own solve answers the question.

The types the two remaining classes share move to `orbbec_stream.hpp`
(`OrbbecStreamOptions`, `OrbbecDeviceInfo`, `OrbbecColorCodec`, and
`OrbbecStreamStats`, formerly `OrbbecCaptureStats`), and `OrbbecSyncMode` to
`orbbec_sync_config.hpp`.

**Why now, not with the Replica source.** The plan tied the removal to the
Replica source moving onto `IRgbdSensor`, but nothing in it depends on
`OrbbecCapture`. Meanwhile the host path was `fuse_orbbec`'s default, at
15.5 ms of host time a 4K frame and eight times the run's CPU (the
2026-09-28 GPU pre-processing decision), and a rig through it fused its
cameras one at a time, uploading float depth and packed colour for each.

**Left for the next step.** The video decoders still hand out host pictures
where no device path opens (software, VAAPI, NVDEC without `VR_WITH_CUDA`)
or the stream was given no device or allocator. `VideoPixelLayout::Rgb24`
and swscale's RGB conversion, which lose their caller here, go with that
step rather than be rewritten twice. *Done the same day (below).*
`OrbbecRig` goes once the examples read a `SensorArray`, which first needs
`OrbbecSensor` to check and write the sync settings the rig does. *Done
2026-10-07 (below).*

**Validation.** Apple M5 Max, macOS, Release with warnings as errors,
Orbbec, FFmpeg and the viewer: the 55 tests pass. The conversion test now
checks the stream options, `OrbbecSensor::open`'s refusals and the factory
models' intrinsics checks; the H.265 test checks every decoded pair's I420
planes and their colour description. `fuse_orbbec` and `rig_viewer` build;
neither was run on a camera.

### 2026-10-06 — The video decoders hand out pictures on the device only: the hardware path is fixed per build, a stream the hardware refuses is an error, and off Apple `VR_WITH_FFMPEG` needs `VR_WITH_CUDA` (amends the 2026-09-27 decoder, 2026-09-28 decoded-frame and 2026-10-06 raw-frames entries).

**The rule.** Once a frame arrives it goes to the GPU and is processed there,
decoding included. `HevcDecoder` and `JpegDecoder` hand out pictures on the
device only, on one hardware path fixed per build: VideoToolbox on Apple
(H.265 through FFmpeg, JPEG directly), and elsewhere NVDEC through FFmpeg and
nvJPEG's hardware engine, both through CUDA. Removed:

- software decoding, VAAPI, NVDEC without `VR_WITH_CUDA`, and copying a
  hardware picture to the host;
- the fallbacks: `Auto`'s move to software on a refusal, a failed device path
  sending every later picture to the host, and nvJPEG's `GPU_HYBRID` back end
  with the software decode of what it refused. `GPU_HYBRID` decodes the
  entropy on the host, 4.2 ms of CPU a 4K frame against the engine's 0.38
  (the 2026-09-28 decoded-frame decision);
- the selection that had nothing left to choose: `VideoDecodeBackend`,
  `hardware_backends()`, the probe clip, `JpegDecodeBackend`, `backend()`,
  H.265's `layout` and `threads`, both decoders' `label`, and the JPEG
  decoder's FFmpeg (it uses none now);
- `PictureConverter` and libswscale with it, `VideoPixelLayout::Rgb24` and
  `DecodedPicture::plane`. The colour helpers stay, as `frame_color.hpp`;
- the host-picture warning, `OrbbecStreamStats::host_pictures` (PERF.md's D1)
  and the Orbbec driver's I420 host frames.

**The codes are the contract.**
- `IoError`: data that does not decode. It costs that frame, and for H.265
  the frames to the next key frame.
- `Unsupported`: a stream or JPEG the hardware does not take, or a `create`
  with no device path (no device or allocator, or a GPU the build's path
  cannot reach). An H.265 stream is held to 8-bit 4:2:0 at its SPS, through
  `sw_pix_fmt` (`YUVJ420P` too, a stream whose VUI says full range), and on
  VideoToolbox to no crop at the left or top; a JPEG to baseline 8-bit 4:2:0
  within 16384 a side on nvJPEG's engine and the device's image extent on
  Apple. An H.265 refusal stands from then on, after the pictures decoded
  before it.
- `Backend` or `OutOfMemory`: the device path failed. A CUDA error is
  `Backend` now, not `IoError`.

The Orbbec colour decoders stop on any code but `IoError`, and
`CameraStream::take` returns it, so `OrbbecSensor::poll` and
`OrbbecRig::poll_set` report a stream that can no longer be decoded rather
than starve quietly. The audit of 2026-10-06 found that, once the fallbacks
went, a decoder that kept failing would have looked like a slow camera.

**Off Apple, `VR_WITH_FFMPEG` needs `VR_WITH_CUDA`**, refused at configure,
and the FFmpeg floor rises to 6.1 (Ubuntu 24.04). In CI the 24.04 leg (CUDA:
NVDEC, and nvJPEG where the GPU has the engine) and the Mac (VideoToolbox)
build and test the decoders. A GPU with no hardware JPEG engine decodes no
MJPEG: one CI host's is now an RTX 4090, which has none (nvJPEG status 7),
so the JPEG tests skip there and run on the RTX 5090 host. The 22.04, 26.04 and sanitizer jobs no longer build them, so FFmpeg
4.4 and 7.x on NVDEC, and the decoders under the sanitizers, lose coverage.

**The tests** read each device picture back and hold it to the pattern the
fixture was made from, rather than to a software decode; bit-exactness against
software and swscale's RGB checks go with it. They skip where no device path
opens, unless `VR_TEST_HEVC_BACKEND` promises one, as CI's legs do; a GPU
with no JPEG engine skips the JPEG tests regardless.

**Not taken.** A failed device path is not latched inside the decoders: their
consumer stops at the first. The Orbbec driver still opens without a device
and refuses at `start`, which keeps its option checks testable with no GPU.

**Validation.** Apple M5 Max, macOS, Release with warnings as errors, Orbbec,
FFmpeg and the viewer: the 54 tests pass with
`VR_TEST_HEVC_BACKEND=videotoolbox`. Mutating VideoToolbox's crop refusal,
the MJPEG decoder's stop on `Unsupported`, or the H.265 decoder's on a
refused stream fails its test. CI's 24.04 leg on an RTX 4090 passes the
H.265 tests on NVDEC, the first run of that path. Not run on a camera.

### 2026-10-06 — `GpuFramePrep` takes colour on the device only, and a set's depth is staged on one thread (amends the 2026-09-28 decoded-frame and 2026-10-06 one-batch entries).

*Amended 2026-10-07 (below):* host colour is back, as packed R'G'B' words
(`RgbdFrame::color_packed`) staged beside the depth on the calling thread
and copied up in the pass's own batch; Y'CbCr planes stay device-only.

**The rule.** `GpuFramePrep` takes a frame's colour on the device only: I420
or NV12 planes in a buffer (`YuvImage::device`), or NV12's planes as images
(`YuvImage::image`). `YuvImage::plane`, the host planes, goes, with their
staging and the repacking of their rows. Only depth goes up from the host:
one copy into the pass's staging, and one in its batch. `prepare_batch`
stages each camera's depth on the calling thread; its thread per camera
(`run_each`) goes.

**Why.** Nothing made host colour any more. The decoders hand out device
pictures only (the entry above), so the host planes were a path only the
tests took, against the rule that a frame is on the GPU from its arrival.
With colour on the device, the threads staged only depth, 0.7 MB a camera
at 640 x 576, and measured level with one thread. Median of 400 sets of four
cameras, each pair interleaved, Apple M5 Max, Release, NV12 colour in a
device buffer, threads against one thread:

| depth, colour | threads | one thread |
|---|---|---|
| 640 x 576, 3840 x 2160 | 1.035 ms | 1.025 ms |
| 1024 x 1024, 3840 x 2160 | 1.054 ms | 1.063 ms |
| 640 x 576, 1280 x 720 | 1.033 ms | 1.024 ms |

The RTX 5090 is not re-measured: the staging is the same host copy there.

**Tests.** The prep tests upload each picture as a decoder leaves it, I420 in
a device buffer, its rows padded where a test asks, and keep their buffer
and image layouts; the host-plane cases go. `recon_sensor_array_process`'s
recording holds its colour in a device buffer.

**Validation.** Apple M5 Max, macOS, Release, Orbbec and FFmpeg: the 54 tests
pass with `VR_TEST_HEVC_BACKEND=videotoolbox`, and
`recon_sensor_gpu_frame_prep` and `recon_sensor_array_process` run clean
under the Khronos layer's synchronization validation.

### 2026-10-06 — Platforms: Linux, Android, macOS and iOS, with GCC or Clang; Windows and MSVC are not supported (amends the 2026-06-21 Vulkan-path entry).

recon builds for Linux, Android, macOS and iOS, with GCC or Clang (Apple's
included), as the family's core and calib do (their "Platforms" decisions;
calib's b98d8e4). Windows and MSVC are not supported: the core, whose headers
recon's include, builds nowhere else; the CUDA interop is Linux's and the
apps are Apple's; and no CI leg runs Windows, so a Windows branch could only
rot untested. The code and build assume those compilers outright: no
`_WIN32` or `_MSC_VER` paths and no MSVC flags. Removed: the MSVC branches
of `vr_target_warnings`, `VR_SANITIZE`'s MSVC refusal, `/EHsc` and `/w` in
`recon_io` and the viewer, the Orbbec tests' Windows `PATH` block, and the
codec's `_BitScanReverse64`. (The video decoders' `D3d11va` went with the
back-end selection, in the device-only decoder entry.) Supporting Windows
would be a new decision, landing with its CI leg -- and the core's.

### 2026-10-06 — Public API serves a caller: entry points only tests used go, or move into their tests (amends the 2026-07-04 GLM entry).

*Amended 2026-10-13 (the volume block-list entry, below):*
`VoxelHashMap::diagnostics()`, kept below though only tests called it, goes.
The other exceptions lie outside the volume tier; that entry's **Next** 5
audits them.

**The rule.** A public function, type or option has a caller outside the
tests -- an example, a sibling, the iOS app -- or it goes. A test that needs
a host-array convenience writes it from the public members.

**What went** (the 2026-10-06 audit's item 4b), by tier:
- `core`: `device_macros.hpp` (`VR_DEVICE_HOST` and friends; no CUDA
  translation unit exists, and the first brings them back), `vr::normalize`
  and `Vec3u8`.
- `volume`: the salvaged AoS `Voxel`, `VoxelData` and `HashTable`;
  `allocate_from_points` and its kernel, built into every map;
  `VoxelBlockGrid::stamp_blocks`, whose pass `free_stale_blocks` runs;
  `VoxelGridParams::defaults()`, a configuration nothing ran; and the
  sub-word attribute path: `VoxelBlockGrid::create` refuses an attribute
  whose block is not whole 4-byte words, so `remove` always zeroes on the
  device.
- `mesh`: `ExtractTimings::uncached_cells_per_block`, which only a block
  larger than 8 reports. The 2026-08-04 rule -- a limit the caller cannot
  see is the library's to check -- applies to limits a caller can reach.
- `texture`: `pack_atlas`, superseded by rig_viewer's device copies (which
  closes the 2026-09-28 projective-texturing entry's open item), and the
  device-depth `texture` overload.
- `codec`: `DeviceFrameWriter::write` and `DeviceFrameReader::read` move
  into their test; `check_intra_frame` becomes file-local; `RansWriter::size`
  and `Encoder::config` go, and `Encoder` defaults its moves, as `Decoder`
  does.
- `eval`: `compare_meshes`, which `ReferenceMesh` does, and
  `MeshDistance::reach`.

**What stays, though only tests call it:** `VoxelHashMap::diagnostics()`, the
one window on the device heap counter; the host multi-view and single-view
`TextureView` `texture` overloads, how the tests drive both kernels with
hand-built geometry; `FrequencyTable::empty`, which the table itself uses;
and the camera tier's `distort_rational` and Rodrigues helpers, which calib
will call.

**Validation.** Apple M5 Max, macOS, Release with warnings as errors, Orbbec
and FFmpeg: the 53 tests pass with `VR_TEST_HEVC_BACKEND=videotoolbox`.

### 2026-10-06 — Fusion runs over the blocks the scanning cameras reach: one frustum a frame, no near cut, far at `max_depth + trunc_dist`, their union compacted in one scan (amends the 2026-08-12 view-meshing and the 2026-09-30 one-batch and device-list entries).

*Amended 2026-10-06 (view culling, below):* the host
`compact_active_blocks_in_frustum` overloads are gone, and the frusta list
carries a serial of its own, so a later frustum compaction makes an earlier
list stale.

*Amended 2026-10-22 (PERF.md P6):* the fuse records the same compaction into
its own submit (`VoxelHashMap::record_compaction`) and sizes its dispatch
from the device's count, refusing none. The map's host copy of the heap
counter, not a second compaction, tells an empty grid from one the frames do
not reach.

**The rule.** `TsdfIntegrator::integrate` builds one frustum for each frame
it fuses -- the depth camera's intrinsics and pose, **no near cut** (near 0),
far at `max_depth + trunc_dist`, sides widened as `make_frustum_planes`
widens them -- and dispatches every frame over the active blocks inside
**any** of them, compacted in one scan of the table by the new
`VoxelHashMap::compact_active_blocks_in_frusta_on_device`. A frame whose
principal point lies outside its image, or whose planes are not finite,
falls back to the whole active set (`compact_active_blocks_on_device`), as
every call used before. The tick is unchanged: an empty grid does not tick,
and a grid the frames do not reach does, dispatching nothing (the map's full
list, asked for only then, tells the two apart). The dispatch-size refusal
now counts the reached blocks. Marching cubes is untouched here: culling it
by the viewing camera is a later change.

**Why it is the same result, bit for bit.** The fusion kernel changes a
voxel only after `project_to_image` accepts it -- `zc > 0` and the
projection inside the image -- and the sampled depth `d` passes
`d <= max_depth`, and `d - zc >= -trunc_dist`. So a changed voxel has
`0 < zc <= d + trunc_dist <= max_depth + trunc_dist`, in front of the camera
and inside the image: inside the frustum. Dynamic's clear needs only
`zc > 0`, which is why there is no near plane at `min_depth`: free space
nearer than `min_depth` is cleared too. The sides contain the image only
under three conditions, each checked or required:
- the focal lengths are positive and the principal point lies inside the
  image: the 0.9 focal scale moves a side plane outward only then (with
  `cx < 0` it moves the left plane *inward*, culling visible voxels -- the
  test's witness);
- every plane is finite; NaN, as a non-finite `max_depth` produces, is
  refused rather than reasoned about;
- the pose is rigid, which `DepthCameraParams` already requires: the planes go
  to world by the pose's inverse-transpose, the kernel by `transpose(R)`.

The block test is conservative per frustum: the AABB spans every voxel node
of the block with half a voxel to spare, and a p-vertex test keeps a box
that any point of it could pass. The half voxel and the widening also cover
the float rounding between the two computations. Each voxel is one
thread's and the frames run in order, so fusing a block no frame changes is
a no-op, and a list holding every block some frame reaches gives what the
whole active set gave: tsdf, weight, colour and `changed` stamps alike.

**What the dispatch now scales with.** The blocks the frames reach, not the
map: a rig fusing a corner of a large map no longer runs a thread per voxel
of the whole map per camera. The compaction still scans every hash slot, as
before. Measured on room0 at 1 cm, Apple M5 Max, Release, the map's 27 046
blocks replicated along x (copies, so the cameras see one room), integrate
per frame over 60 frames after 100, cull and the forced full list
interleaved on each frame, median, device time (fusion plus compaction):

| map | reached | culled | full list |
|---|---|---|---|
| 1 room, 27 046 blocks | 90% | 0.72–0.80 ms | 0.76–0.85 ms |
| 4 rooms, 108 184 blocks | 23% | 0.60 ms | 1.38 ms |
| 16 rooms, 432 736 blocks | 5.8% | 0.48–0.71 ms | 3.31–5.20 ms |

(two runs where a range is given; the Mac is shared and noisy). On a map one
room deep, the frustum reaches nearly all of it at an 8 m `max_depth`, and
the cull costs nothing measurable.

**What stays.** `MeshIntegrator` writes the blocks its triangles bin into,
so it is input-driven already and keeps its own path. The full list's reuse
(2026-09-30) is not evicted: the frustum list has a buffer of its own,
`check_device_block_list` refuses it as not the active set, and an extract
or the encoder after a fuse compacts the full list itself (a fuse no longer
leaves one). The host `compact_active_blocks_in_frustum` overloads run the
same kernel with one frustum and now write that buffer too. Nothing is
cached: the frusta change every frame.

**Not taken.** A tighter frustum (no widening, or the principal point's own
offset) would need a second `make_frustum_planes`; the fallback costs only
the frames whose principal point is outside the image, which no supported
camera has.

**Verified.** `recon_tsdf_integrate_cull` fuses the same frames into two grids,
each call of B's opening and closing with a frame of zero depth whose frustum
reaches every block, and compares tsdf, weight, colour, all three stamps and the
tick byte for byte: a static wall; a receding wall in dynamic and classic; a
block past `max_depth` but in the band; free space nearer than `min_depth`; two
cameras seeing different places; and a principal point outside the image. It
also checks that one camera reaches fewer blocks than the map holds, and the
tick of a missed and an empty grid. Mutants with the far plane at `max_depth`, a
near plane at `min_depth`, the first frame's frustum for every frame, no
principal-point fallback, and either tick rule broken each fail it.
`recon_volume_frustum` checks the union of two frusta, neither containing the
other, each block once, and that the full list still holds after both frustum
forms.

**Validation.** Apple M5 Max, macOS, Release with warnings as errors, Orbbec
and FFmpeg: the 56 tests pass with `VR_TEST_HEVC_BACKEND=videotoolbox`, and
`recon_tsdf_integrate_cull`, `recon_tsdf_integrate_set`,
`recon_tsdf_integrate` and `recon_volume_frustum` run clean under the Khronos
layer's synchronization validation.

### 2026-10-06 — A sensor array groups its frames by trigger only: `SyncMode` and sequence grouping go (amends the 2026-10-06 `SensorArray` entry).

**Why.** The owner: "we don't need it. the main use case for sensor array is
for live synced camera capture which frames arrive within fixed time
interval." Nothing outside recon's tests grouped by sequence -- no example,
calib or ios code opens a `SensorArray` yet -- and it had a known bug: a
sensor repeating a sequence number sent its set out twice.

**What goes.** `SyncMode`, its `to_string` and `SensorArray::Options::sync`,
which with one mode left chose nothing; the per-sensor queues, the last set's
number, and the start-order slot for a second primary. An array groups by
`TriggerGrouper` around the primary's frames on the host clock; more than one
sensor needs one primary, the rest secondaries, all on that clock, and a
single sensor needs neither. `SensorArray::exhausted` is `true` once any
sensor is. `queue_depth`, `tolerance_us`, `IRgbdSensor::exhausted`,
`RgbdFrame::sequence` and `FrameSet::sequence` (the primary's frame's) stay.
A replay that must give the same sets every time -- calib's session replay,
which sequence grouping was for -- is a new decision when one needs it.

**Verified.** Apple M5 Max, macOS, Release with warnings as errors:
`recon_sensor_array` drops its sequence cases and runs the failed drain and
an exhausted sensor through trigger grouping, and opens one free-running
sensor on its own clock; `recon_sensor_array_process` groups its three
sensors by trigger, one set a poll, the set missing a sensor waited for and
then handed out without it.

### 2026-10-07 — `GpuFramePrep` stages a host frame's packed colour beside its depth (amends the 2026-10-06 device-only `GpuFramePrep` entry).

**The rule.** A frame may carry its colour on the host as packed R'G'B'
words, `RgbdFrame::color_packed`: the colour camera's size, row-major,
`R | G << 8 | B << 16` with the high byte ignored (`io::load_color_packed`'s
layout), held by `pixels`, and set instead of `YuvImage`'s planes, never
beside them. `GpuFramePrep` stages the words as it stages host depth: one
copy into the pass's staging, after the depth, and one in its batch into the
colour input. The colour pass reads R, G and B as byte planes four bytes
apart through the planes' own bilinear sampler, so its clamping, weights and
coverage byte are the Y'CbCr path's; a push constant, `packed_rgb`, picks
the branch (`ColorParams` is 108 bytes). `prepare_batch` stages each
camera's packed colour with its depth, on the calling thread. Depth stays
16-bit with `metres_per_unit`: the frame gains no float depth.

**Why.** The Replica examples' source, a dataset decoded on the host,
becomes an `IRgbdSensor` whose frames go through the pass as the cameras'
do, so host colour has a real caller again; the 2026-10-06 entry dropped it
because nothing made it. The words go into the staging as decoded, with no
conversion on the host, so a frame is still one upload in the pass's own
batch, and Y'CbCr planes stay on the device.

**The threads stay gone.** Host colour staged one camera after another lost
to a thread per camera at four 4K cameras on the Mac (the 2026-10-01
measurement, in the one-batch entry). The one caller is a single camera; a
rig with host colour measures that again before the threads come back.

**Validation.** Apple M5 Max, macOS, Release with warnings as errors, Orbbec
and FFmpeg: the 56 tests pass. `recon_sensor_gpu_frame_prep` holds packed
words through a pinhole to their input exactly, the junk high byte replaced
by full coverage, and through a lens to a host bilinear reference within a
code, no coverage byte off; refuses words beside planes before any work, and
a PQ transfer; and prepares a fifth camera's packed colour in a batch byte
for byte as `prepare` does alone, depth kept within colour. It runs clean
under the Khronos layer's synchronization validation.

### 2026-10-07 — The Replica source is an `IRgbdSensor` and the examples fuse what `GpuFramePrep` prepares: `ICameraCapture`, `CapturedFrame` and `OwnedFrame` go, and `io` loads depth as stored (amends the 2026-08-02 sensor-tier, 2026-09-14 examples and 2026-10-06 `IRgbdSensor` entries).

*Amended 2026-10-08 (the streamed-atlas entry, below):* `fuse_viewer` copies
its keyframe's colour into the atlas on the device, and its
`atlas readback` row is gone.

The first step of the 2026-10-06 plan finishes: one interface for every
source, one frame type, one fusion path.

**The rule.**
- **The Replica source is an `IRgbdSensor`**: `ReplicaSensor`
  (`examples/common/replica_sensor.hpp`, formerly `ReplicaCapture`). Its
  `SensorInfo` names one pinhole `camera::CameraModel`, no distortion, for
  both cameras, since Replica renders depth and colour from one, with an
  identity `depth_to_color`; `FreeRun`, `Device` clock (every timestamp 0,
  "none"), `PoseSource::Tracked` (each frame carries its trajectory pose;
  the enum's doc now names a recording) and fps 0. A frame's depth is the
  PNG's 16-bit samples as stored, with `metres_per_unit = 1.0f / scale`
  computed in float as the old loader did; its colour is
  `RgbdFrame::color_packed`, declared canonical; its `color_to_world` is the
  trajectory pose, parsed in float as before and widened, so the pass
  narrows it back exactly; its `sequence` is how many frames the sensor
  handed out before it, so a stride is not a loss; and `pixels` owns both
  images. `poll` stays consumer-paced, `drain` appends what `poll` would
  return, `set_queue_depth` accepts at least 1 before `start` and changes
  nothing, and `stats` counts received and delivered. `open` refuses a
  `min_depth` of 0, which the pass reads as "no return", as
  `OrbbecSensor::open` does, and the three examples' `--min-depth` takes
  (0, max). The preload holds 6 bytes a pixel, not 8. The move constructor is
  defaulted and assignment deleted: the frames to play are one vector of
  entries, so a moved-from sensor plays nothing and is exhausted.
- **Every example fuses a `DeviceFrame`.** `fuse_replica`, `codec_replica`,
  `fuse_render` and `fuse_viewer` prepare each polled frame with one
  `GpuFramePrep` and fuse it through `fuse_set`, which `fuse_frame.hpp` now
  holds (`fuse_device_frame.hpp` folded into it; the host-depth
  `allocate_band` and `fuse_frame(CapturedFrame)` went, and
  `allocate_band_with` folded into the one `allocate_band` left). The
  viewers keep their keyframe as its `DeviceFrame`, texture through its
  colour camera with the single-view `TextureView`, and read its colour
  back with a `CommandBatch` for the atlas; `to_canonical` leaves them, as
  `prepare` refuses colour that is not canonical. `fuse_viewer` keeps its
  last successfully fused frame until the next preparation and fusion both
  succeed (`fuse_keyframe`), so a failed attempt leaves the final and later
  remeshes textured. This holds one previous output pair while the pass
  allocates the next; releasing it before preparation reused the buffers
  but lost the keyframe on a refused pose. It shows a `frame prep` row;
  its `atlas pack` row is `atlas readback`,
  with a `TODO(examples)` to copy the atlas on the device as `rig_viewer`
  does.
- **`ICameraCapture`, `CapturedFrame` and `OwnedFrame` go**, with the
  conventions test's `FakeCapture`; `recon_sensor_rgbd_sensor` covers the
  interface.
- **`io::load_depth_metres` is `io::load_depth_u16`**: the samples as
  stored, under the same 16-bit grayscale checks. Its one non-test caller
  was the Replica source, which now hands the samples on with their unit.

**Why.** A frame borrowed its pixels until the next poll, so a consumer
copied one to keep it (`OwnedFrame`); an `RgbdFrame` holds them, so the copy
has nothing to do. Every Replica run now goes through the pass a camera's
frame does, so the dataset examples exercise the path the live rig runs
rather than a host path no camera uses.

**What stays for the iOS scanner, and why.** The scanner, pinned at an older
recon, implements `ICameraCapture` and fuses host frames. What it calls
stays until it moves onto the interface: `VoxelHashMap::allocate_from_depth`
over host depth, `TsdfIntegrator::integrate`'s host overloads,
`ColorFrame::pixels`, `ProjectiveTexturer`'s host-depth overloads,
`sensor::to_canonical`, `cv_from_gl_camera` and
`depth_from_registered_color`. The 2026-10-06 test-only API rule counts a
sibling app's caller, so these are the scanner's, not test-only; they go or
move with its migration. `depth_from_registered_color` returns a
`DepthCameraParams`, which `RgbdFrame` has no slot for, so that migration
needs a `CameraModel` rescale for the 256x192 depth; it is not written
before it has that caller.

**No float depth.** DESIGN's Next work had the frame gaining float depth
for ARKit. It does not: depth stays 16-bit with `metres_per_unit`, one depth
format for the pass's one depth kernel, 2 bytes a pixel up. When the scanner
moves onto the interface it quantises ARKit's float `sceneDepth` on the
host, in 0.1 mm units, say, which reach 6.55 m.

**Verified.** Apple M5 Max, macOS, Release with warnings as errors, Orbbec
and FFmpeg: the 56 tests pass, `fuse_replica`, `codec_replica` and
`fuse_orbbec` build, and `recon_io_image` reads the 16-bit fixture as
stored. `fuse_replica`, streaming and with `--preload --stride 2`, and
`codec_replica` ran on a synthetic three-frame 64x48 scene in Replica's
layout, each vertex's colour the one its depth's column was painted, and
`--min-depth 0` is refused. `fuse_render` and `fuse_viewer` were not
compiled here (the viewer is off; CI's viewer leg builds them), nor was
`rig_viewer`, whose source lost only an include. On room0, the first 200
frames at 2 cm, `fuse_replica` before and after this change, streaming and
with `--preload`, give the same 309,353 triangles, positions, normals and
colours bit for bit; only their order differs, as compaction's does from
run to run.

**Review regression.** `recon_example_fuse_keyframe` drives the viewer's
fusion helper without a window or dataset. A refused pose and an integration
failure after successful preparation both leave the previous depth, atlas
bytes, coverage and cameras intact, and the retained frame still textures
the surface. A successful retry replaces it. The viewer publishes the pose
only after the helper succeeds, alongside the fused-frame count.

### 2026-10-07 — A rig is an `OrbbecSensor` per camera in a `SensorArray`, on the SDK's global timestamps (SDK 2.10.6 or later): each camera is checked against its sync settings at `open` and written only when asked, the tolerance is 0.4 of a frame period, and `OrbbecRig`, its start order and the public `TriggerGrouper` go (amends the 2026-09-24 SDK, 2026-09-27 rig and 2026-10-06 `IRgbdSensor` and `SensorArray` entries).

*Amended 2026-10-08:* opening a rig is the library's.
`open_orbbec_sensors` (`orbbec_sensor.hpp`) reads the sync file and opens
each camera, in the file's order, with its entry as `Options::sync` and on
the host's clock, returning the sensors `SensorArray::open` takes; the
caller's options give the streams and `apply_sync`, never a serial or sync
settings, which the file gives, or a pose, which the array's calibration
gives. `fuse_orbbec` and `rig_viewer` call it, as calib's live app is to, so
a new rule for a rig's cameras is made once. Test:
`recon_sensor_orbbec_open_sensors`, against the camera stub the sync-apply
test uses (`tests/orbbec_camera_stub.cpp`).

The array's plan finishes (DESIGN.md's Next work, step 2): `fuse_orbbec`
and `rig_viewer` open one `OrbbecSensor` per camera of the rig's sync
configuration and read them as one `SensorArray`. What `OrbbecRig` did for
the Orbbec SDK moves into `OrbbecSensor`, one camera at a time; the rest was
already the array's.

**The rule.**
- **A camera's sync settings are checked at its `open`.**
  `OrbbecSensor::Options::sync` takes the camera's entry of the rig's file
  (`read_orbbec_sync_config`). `open` compares it with what the camera
  stores (`sync_differences`, which reads `SECONDARY_SYNCED` as `SECONDARY`)
  and refuses a difference, `Unsupported`, naming the camera and each field,
  unless `apply_sync` writes them. `apply_sync` without `sync` is
  `InvalidArgument`, before the SDK is touched. The role and the
  software-triggering and unknown-mode refusals are decided after any write,
  so `apply_sync` can repair a camera left in software triggering.
- **Written only on a difference, then checked for SDK normalization.**
  The settings persist in the camera's flash, which Orbbec's sample warns
  frequent writes wear. After
  a write the SDK answers a read from its cache of the effective settings,
  not the camera (`DeviceSyncConfiguratorOldProtocol::getSyncConfig`).
  That cache still matters: its setter forces a primary's
  `triggerOutEnable = true` and `triggerOutDelayUs = 0` before writing and
  caching. `open` compares again and refuses any remaining difference,
  `Unsupported`, naming the camera and fields. Removing the old rig's
  comparison falsely accepted a request for `false` / `250`; a public
  `OrbbecSensor::open` test with a camera stub reproduces it without hardware.
  This checks normalization, not persistence: the next `open` reads the
  camera and checks the stored settings.
- **`sync_clock_to_host` means the SDK's global timestamps.** `open` refuses
  a camera without them (`isGlobalTimestampSupported`) and enables them
  (`enableGlobalTimestamp(true)`; the SDK's own config leaves the Femto
  Mega's off). Each `start` still sets the camera's clock to the host's once,
  before it streams (`timerSyncWithHost`), and the SDK then re-fits its
  mapping synchronously, so the first frames are mapped. `timestamp_ns` is
  the depth frame's `getGlobalTimeStampUs()`: the camera's clock mapped onto
  the host's, the mapping re-fitted every 10 s (2.10.6). Nothing re-syncs a
  streaming camera, and nothing calls `enableDeviceClockSync`. A zero, no
  mapping yet, stays zero, and the array counts it unmatched.
  `ClockDomain::Host` is `std::chrono::system_clock`, the SDK's default host
  clock; recon never calls `setTimestampClockType`, which is process-wide.
- **The tolerance follows the frame rate.** `SensorArray::Options::tolerance_us`
  defaults to 0, meaning 0.4 of the fastest member's frame period
  (13 333 µs at 30 fps), or 5 000 µs when no member reports a rate. An
  explicit value keeps the old checks: non-zero, under half a period.
- **`TriggerGrouper` is the array's internals**
  (`src/volumetric_kit/recon/sensor/array/`, unexported, not installed). Its
  test compiles the source, as the Orbbec internal tests do.
- **What goes.** `OrbbecRig`, `OrbbecRigSet`, `OrbbecRigStats`, the rig's
  start order (`rig_start_order` and `recon_sensor_orbbec_start_order`; the
  array's order is tested by `recon_sensor_array`) and
  `orbbec::validate(OrbbecRig::Options)`; `CameraStream`'s rig-only
  `timestamp_us`, `withdraw` and `color_to_world`; and `discover`'s list of
  serials. The redundant `Pipeline::enableFrameSync()` goes too: the
  `Pipeline` constructor turns frame sync on. `OrbbecRigSyncConfig` and
  `read_orbbec_sync_config` stay, for the rig's file.
- **SDK 2.10.6 is the floor** in `cmake/vr_orbbec.cmake`, and CI's pin, now
  one composite action (`.github/actions/install-orbbec-sdk`) that
  `_build.yml` and `viewer.yml` share. The viewer leg installs FFmpeg and the
  SDK, so it compiles `rig_viewer` with the other viewers on every PR.

**Why: measured on the rig.** Four Femto Megas on the wired 2.5 GbE link,
each triggered by the primary through a sync hub, depth 640 x 576 at
30 fps; a secondary's timestamp minus the primary's for
the same trigger, from probes on the SDK outside recon:

| Clock | Skew | Over time |
|---|---|---|
| device, one `timerSyncWithHost` per camera | 1–2.6 ms at start | 3.1, 3.5 and 6.0 µs/s apart: medians 6.3–8.4 ms after 21 min; a 5 ms window fails after ~10 min |
| device, `timerSyncWithHost` every 60 s, as `OrbbecRig`'s `enableDeviceClockSync` did | about ±2.5 ms | bounded, but each sync lands on a new offset |
| global, SDK 2.9.3, one sync before start | medians 0.8–4.2 ms | bounded but biased; worst ~5.8 ms over 15 min |
| global, SDK 2.10.6, one sync before start | medians under 0.5 ms, typically −1.2 to +2.5 ms | flat over 15 min, while the same run's device timestamps drifted from 1.0–1.6 to 4.0–6.1 ms |

Rare single frames land 10–15 ms off, in the device timestamps as much as
the global ones, so they are not the mapping's. One past the window is let
go, costing that camera one set.

This is Orbbec's current recipe (SDK 2.10.6's
`3.advanced.multi_devices_sync` sample and `docs/tutorial/timestamp.md`):
global timestamps on every camera, one `timerSyncWithHost` each before any
stream, the secondaries started before the primary, frames matched on the
global timestamp within half a frame period, and no periodic re-sync. Its
5 ms is only the spread its `analyze_sync.py` flags as abnormal.

**Why not the alternatives.**
- **Device timestamps synced once** drift apart without bound: even
  13.3 ms is gone within about half an hour.
- **Periodic re-sync** steps every camera to a new random offset at each
  sync and blocks ~60 ms a camera. Once, in the 60 s run, the SDK's network
  enumerator reported two streaming cameras removed, and every later sync
  failed with "Device is deactivated/disconnected". Global timestamps bound
  the error without touching the camera's clock.
- **`Context::enableDeviceClockSync`** is process-wide, since the SDK's
  `Context` and `DeviceManager` are singletons: it re-syncs every camera the
  process opened, calib's included, belongs to whichever context called it
  last, returns before its first pass (`OrbbecRig::start` raced it), and
  holds the device-manager mutex through each pass, stalling every device
  open and close. Orbbec's sample replaced it with the per-camera call on
  2026-09-29.
- **The hardware timer reset** over the sync cable gives a rig clock, not
  the host's, so frames could not report `ClockDomain::Host`. Orbbec's staff
  call it exclusive with host sync on the Femto Mega, the SDK registers only
  its enable flag and signal there, and the clocks drift on after it.
  Grouping needs no sub-millisecond agreement: the trigger already aligns
  the exposures.
- **The grouper tracking each member's offset** would let device timestamps
  group, but leaves Orbbec frames off the host clock that `ClockDomain::Host`
  promises and calib needs, and adds per-member state to a vendor-neutral
  class for an unmeasured gain. It is worth a look only if global timestamps
  show a large, constant per-camera bias.

**Why 0.4 of a period.** The trigger exposes every camera once a period, the
0–480 µs depth delays aside, so a timestamp only says which trigger a frame
belongs to. Any window under half a period does that while the clock error
stays inside it: at 30 fps a frame within 13.3 ms of its trigger matches it,
and the neighbouring trigger's, 20 ms or more away, never does. The fixed
5 ms was tighter than anything Orbbec uses and refused 2.9.3's global worst
case.

**Not handled.** If a re-fit fails, the SDK keeps its previous mapping and
only logs it, so a global timestamp can be non-zero and off by the step
until the next re-fit; the array's zero check cannot see that. An
`OrbbecSDKConfig.xml` in the working directory can switch the SDK's host
clock to a monotonic one; recon does not read `getTimestampClockType`.

**Verified.** Apple M5 Max, macOS, Release with warnings as errors, Orbbec
SDK 2.10.6 and FFmpeg: the 57 tests pass. `recon_sensor_array` groups by the
derived default with and without a rate (a secondary just inside it joins
the set, one just outside joins none), refuses an explicit 500 µs at 1 kHz
and accepts 499 µs; `recon_sensor_orbbec_conversion` refuses `apply_sync`
without `sync`. The public-open test
`recon_sensor_orbbec_sync_apply` uses a camera stub to check that SDK
normalization is refused (and fails without the post-write comparison),
matching settings are never written, a write repairs software triggering,
and a write error is preserved. Configure refuses an SDK below 2.10.6.
`rig_viewer` was not compiled here (the viewer is off locally); CI's viewer
leg compiles it.

On the rig, by hand (not a test), `fuse_orbbec --rig
config/femto_mega_sync.json`, H.265 720p at 30 fps, opened the four cameras
in 2.1–2.3 s and ran 19.8 min: 35 279 sets, 3.0% missing a camera. Each
secondary's mean skew to the primary was +0.13, +0.32 and +0.49 ms, its
160, 320 and 480 µs depth delay, flat over the run; the worst 2.8, 2.7 and
10.9 ms (one stray frame). The SDK reported no camera removed. The sets
missing a camera are H.265 colour lost to the decoder, 0.2–1% of each
camera's frames, through the same path as before. A sync file with one
secondary's depth delay changed was refused, naming the camera and the
field, and the camera kept its setting. Restarted three times with MJPEG,
1–3 of the first 31 sets missed a camera, so the start needs no settling
wait. One camera fused at 30 fps with and without `--host-clock`. Not run:
`apply_sync`'s write (it rewrites flash) and `rig_viewer`.

### 2026-10-08 — Fusion samples at integer pixel centres and refuses a non-finite camera (amends the 2026-07-06 depth-sampling entry).

**The rule.**
- **Integer pixel centres.** `tsdf_integrate.comp` takes its bilinear taps
  from `floor(u)`, `floor(v)`, with no half-pixel shift, and its
  nearest-pixel fallback and colour sample at the rounded pixel, clamped into
  the image (`nearest_pixel`, `tsdf_common.glsl`). The image bound stays
  `[0, W) × [0, H)`, on which the fusion cull's containment argument
  (2026-10-06) rests, so its last half pixel reads the edge pixel.
- **A non-finite camera is refused.** `TsdfIntegrator::integrate` refuses a
  depth or colour camera with a NaN or infinite intrinsic, depth bound or
  pose entry among its O(1) checks, before any work; a list with one such
  frame fuses none. The kernel's projection compares negated, so a NaN pixel
  fails its bounds too.

**Why.**
- The 2026-07-06 entry kept the half-pixel offset until the depth intrinsics
  were integer-centred. Every frame now is: `GpuFramePrep` writes pixel `i`
  from the ray through `i` (2026-09-28) and the Replica source goes through
  it (2026-10-07), the camera tier declares the convention, and allocation
  and texturing already sampled that way. Fusion alone moved each camera's
  surface half a pixel along that camera's image axes: on a slanted surface
  a depth error of the slope times the shift, in a different direction for
  each camera of a rig.
- A NaN in a pose or an intrinsic projected every voxel to a NaN pixel, which
  the ordered bounds test passed, and the cull fell back to the whole active
  set.

**Verified.** Apple M5 Max, macOS, Release with warnings as errors, Orbbec,
FFmpeg and the viewer: the 57 tests pass. A plane tilted across both image
axes, imaged at a 160 × 120 pinhole camera's pixel centres with an
off-centre principal point and fused from `GpuFramePrep` at 1 cm voxels,
crosses zero within 0.06 mm of the plane along 14 194 voxel columns; with
the half-pixel shift, within 5.5 mm (`recon_sensor_gpu_frame_prep`). Each of
these mutations fails a test: the old taps and fallback; the colour sample
floored, or rounded without the clamp; no finiteness check (NaN and
infinite poses, focal lengths, principal points and depth bounds, each
refused with the tick unchanged); and the projection's ordered compares,
which `recon_tsdf_integrate` runs alone and which pass a NaN point, focal
length or pose entry, and a finite point and translation a float range
apart. Not run: `fuse_orbbec` and `rig_viewer` on the rig.

### 2026-10-08 — The single-camera texture pass needs the vertex to face the depth camera, and both texture passes check a view one way (amends the 2026-08-11 per-vertex and 2026-09-28 colour-camera entries).

**The rule.**
- **The vertex faces both cameras.** The single-camera pass textures a
  vertex only where its normal points toward the depth camera and the colour
  camera, where the two signs only had to agree; a zero normal still passes.
  It is the per-vertex form of the several-view pass's facing test.
- **One view check:** one depth, a depth map, a depth range with
  `min_depth < max_depth`, and an image. Each pass then checks the depth and
  coverage buffers as it uses them, bound in place or copied. The
  single-camera overloads that take a depth pointer build a `TextureView`
  and go through the same path.

**Why.**
- With a registered image, or both cameras behind the surface, the two signs
  agree. The back of a sheet thinner than the occlusion threshold lies
  within it of the front the depth camera measured, so it took the front's
  image: the case the several-view pass's facing test exists for.
- Only the several-view pass refused an empty depth range, under which the
  single-camera pass carried every vertex and returned OK.

**Verified.** On the entry above's build, the 57 tests pass, the texture
tests with no message from the Khronos validation layer. Each of these
mutations fails `recon_texture_projective`: the agreeing-signs test (a 1 cm
sheet's back textured, registered and with a colour camera 5 cm aside); the
facing test without its zero-normal exemption; and the single-camera pass
without the depth-range check (both bounds zero, and inverted).

### 2026-10-10 — Fusion is library code, with strict defaults and opt-in partial coverage.

*Amended 2026-10-22 (PERF.md P6, its first step):* once its grid holds a
block, a partial set is one submit (`TsdfIntegrator::allocate_and_integrate`):
one allocation round, then the compaction and the fuse in its batch. That
round's lock races wait for the next set, and a grow for a capacity limit is
made after it, so what the set could not allocate lands with the next. An
empty grid's first set keeps the rounds: whether the call ticks depends on
whether the grid gains a block. Strict fusion is unchanged.

*Amended 2026-10-25 (the sender's batch):* strict fusion and an empty
grid's first set record one round too, the fuse gated on the whole band, and
`Fuser::finish` fuses a refused set again, blocking, at the tick it was
recorded at.

`tsdf::Fuser` owns the per-set sequence formerly in
`examples/common/fuse_frame.hpp`: grow ahead past `kGrowThreshold`, allocate
all frame bands, grow and retry capacity failures, then integrate the set.
This puts the shared sequence in the library ahead of the pipeline work,
while the `DeviceFrame` hand-off stays with callers because `tsdf` cannot
include `sensor/utils`.

**Complete coverage is the default.** `FuserConfig` has two options:
`max_grows_per_set` (4, including grow-ahead; 0 fixes capacity) and
`allow_partial` (false). Strict fusion returns an error before integration
when a band remains incomplete, and retries pure lock contention up to four
times beyond the allocation tier's own rounds. A failed call may already have
grown the grid or allocated zeroed blocks. Grow-ahead is opportunistic: if it
runs out of memory but the band fits the existing grid, fusion can succeed.

`fuse_viewer` and `rig_viewer` explicitly accept partial coverage and use a
two-grow bound. In this mode the remaining lock races wait for the next set,
and a failed grow backs off for 60 map ticks at the same grid size. The
backoff belongs to the fuser privately. `FuseReport` carries the before/after
bucket counts, grows, allocation failures, any grow error and final load.
The failure tally counts requests, not unique missing blocks.

**Shared mechanism stays small.** `volume::grow_grid` doubles or reaches a
larger requested minimum, within `VoxelBlockGrid::max_num_buckets`; an
unreachable minimum is refused before mutation. It keeps no state and serves
fusion, `codec_mesh` and the codec player without linking the latter two to
`tsdf`. Host allocation failures return `OutOfMemory`. A failure during
rehash restores the old map and heap before the caller can continue.

`VoxelBlockGrid::bytes_at` shares the map's buffer-size table and reports the
grid-sized storage footprint. It excludes temporary rehash and staging
buffers, host scratch, and allocator/driver overhead. It cannot be used by
itself to approve a resize against available memory. The scanner's memory
callback and allocation-stop threshold are deferred to its iOS port, where
the caller and peak-memory validation can land together. iOS keeps its own
policy until that port; this change does not modify the iOS repository.

**Regression coverage.** A three-camera sphere starting from eight buckets
needs four doublings and now matches a roomy reference grid bit for bit.
Strict failure leaves weights and the integration tick unchanged; partial
mode reports failures and integrates what fits. Fault injection covers a
failed input-vector allocation, a resize failure after the new table was
installed, preserved voxel data, subsequent retries, and private live
backoff. Separate growth tests cover content retention and invalid or
unreachable requested minima.

### 2026-10-10 — Only the owner triggers CI, and the runner tooling is the core's.

The core's decision
[Only the owner triggers CI](https://github.com/taojin-6/volumetric_kit_core/blob/main/DECISIONS.md#only-the-owner-triggers-ci)
holds here, with its rationale and how it was tested: the repository's
Actions policy lets only the owner's account trigger a workflow, and GitHub
refuses any other account's run before it creates a job. So no job in
`ci.yml` carries a fork guard, the build legs' and the viewer's included, and
the approval of every outside contributor's run stays on as a second gate. A
fork's change gets CI once a maintainer pushes its branch here. The runner
tooling is the core's `tools/runners`; recon keeps no copy.

### 2026-10-08 — A Dynamic set clears with every camera before it fuses with any (amends the 2026-09-30 one-batch entry).

**The rule.** `TsdfIntegrator::integrate` over a Dynamic set of two or more
frames records every frame's clear, then every frame's fuse: per frame, the
clear kernel (`tsdf_clear.comp`), then per frame the integrate kernel with
`fuse_only` set, which leaves free space past the band alone. A single frame,
and every Classic call, still run one dispatch a frame. The set's cameras act
as one instant: its grid does not depend on their order but for the running
average's arithmetic, as under Classic.

**Why.**
- **Last camera wins.** Fused one after another, a later camera that sees a
  voxel as free space past the band cleared what an earlier camera had just
  fused there. That happens wherever cameras disagree by more than the band:
  a ray that grazes a surface or passes its silhouette to the background, or
  a ToF return that multipath makes long. Marching cubes drops a cell with
  an unweighted corner, so the surface had a hole, in one camera order only.
  `rig_viewer` fuses Dynamic by default.
- **Two passes, not marks.** Clearing only the voxels no camera of the set
  places in its band would keep a contested voxel's history. It needs a
  mark per voxel, written by a pass over every frame before the fuse: the
  same two passes, plus a bit buffer, its atomics and its reset. And a
  camera whose faded band behind its surface reaches a stale surface would
  shield it from a camera that sees through it, which is what Dynamic
  exists to clear.
- **What it costs.** A voxel some camera sees past and another fuses holds
  only the set's observations, its history cleared each set, as it did
  before with the fusing camera last. Its block is stamped changed each set.
- **A kernel of its own.** The clear reads a voxel's weight first and skips
  an unweighted one before projecting it: under Dynamic most of what a set
  reaches holds no weight. The same test behind a push constant in the
  integrate kernel cost Classic 0.11 ms a set, though Classic never takes
  it. So the two kernels share one body,
  `tsdf_integrate.glsl`, and `VR_TSDF_CLEAR_ONLY` builds the clear one: the
  integrate kernel is unchanged, and both take one descriptor layout.

**Measured.** Four 640 x 576 depth cameras on a 1.5 m ring round a 0.3 m
sphere walking a 0.3 m loop in a 4 x 4 m room 3 m high, ray-cast on the host
and held in device-local buffers; 1 cm voxels, 4 cm band, max weight 20,
28 300 blocks reached. M5 Max, Release, the median of 120 sets after 30
warm-up sets, three interleaved runs each:

| integrate, per set | before | after |
|---|---|---|
| Dynamic, device | 1.22 ms | 1.92 ms |
| Dynamic, host | 1.68 ms | 2.43 ms |
| Classic, device | 1.27 ms | 1.27 ms |

On the way: the clear as a pass of the integrate kernel, Dynamic 2.10 ms and
Classic 1.38 ms; a clear pass without the weight test, Dynamic 2.39 ms.
PERF.md's camera-looping kernel would read each voxel once for both halves.

**Verified.** On this build (macOS, Release, warnings as errors, Orbbec,
FFmpeg and the viewer), the 58 tests pass, and the tsdf tests raise no
message under the Khronos validation layer with synchronization validation.
`recon_tsdf_integrate_dynamic_set` fuses a frontal camera and one 2 mm in
front of its wall, looking along it at a far wall, as a set in both orders,
twice. The grids agree bit for bit with each other and with the grazing
camera and then the frontal one fused a call each: the two never fuse one
voxel and the frontal one clears nothing the grazing one fuses, so that
order is every clear before every fuse, without the split. Their meshes
agree vertex for vertex, the far wall meshes, and the frontal camera's wall
keeps all 19 210 triangles it has fused alone, and its colour.
`recon_tsdf_integrate_set` now compares a Dynamic set with the same set
reversed. Each frame's clear and fuse in turn, as before, fails both; fuses
before clears leave 6 008 triangles on the wall; a split fuse without colour
loses it; a set that skips a depth-only frame's fuse, or its clear, fails
`recon_tsdf_integrate_dynamic_set`.
Not run: `rig_viewer` on the rig.

Still open: clearing on evidence (a weight decrement, an incidence term)
and behind a pixel with no return, each judged on the lab rig.

### 2026-10-10 — recon keeps exception handlers, shares frame preparation, and leaves warning policy to consumers.

**Build with C++ exceptions enabled.** CI uses the normal exception-enabled
configurations. The handlers that translate backend errors and checked host
allocation failures into `Status` remain in place. `Fuser`, `grow_grid` and
rehash keep their allocation-failure handling, and the shared
`test_allocation_failure` helper remains for their regression and the frame
preparation regression.

**One preparation path.** `GpuFramePrep::prepare` and `prepare_batch` use
one private path. All input checks run before the timing row opens, so a
refused frame adds no row. A host allocation failure while staging returns
`OutOfMemory` through both entry points; previously returned frames stay
intact and the call can be retried. The shared path retains the batch's
`std::bad_alloc` catch, which the single-frame path previously lacked.

**SDK exceptions stay private.** `vr_link_orbbec_sdk` links the Orbbec SDK
and enables exceptions privately for each target compiling its calls or
headers, including `recon_sensor_orbbec_open_sensors_test`. This follows the
I/O targets' existing treatment of their throwing backends; it does not
change a consumer's compiler flags.

**Consumer defaults.** `VR_WARNINGS_AS_ERRORS` defaults to
`PROJECT_IS_TOP_LEVEL`, as the core's and calib's do. A consumer's newer
compiler reports warnings without failing its build; recon's own CI keeps
`-Werror`. `recon_subproject_defaults` configures recon as a subproject and
checks that its tests, examples and `-Werror` default off.

**Validation.** Apple M5 Max, macOS, Release, with Orbbec, FFmpeg, Assimp,
examples and viewer targets enabled: the build and all 66 tests pass. The
allocation-failure regression checks both single and batch preparation,
retained frame contents and retry; linking it against the pre-fix
implementation reproduces an escaping `std::bad_alloc`. Frame preparation,
its allocation-failure regression, sensor-array processing, the retained
keyframe and fusion allocation-failure tests also pass with the Khronos
layer loaded and synchronization validation enabled. No live-camera or
interactive viewer check was run.

### 2026-10-08 — The examples parse their command lines with one parser, declare each shared flag family once, print stage rows in one table, and keep their reports out of their mains (amends the 2026-09-27 `eval` and 2026-10-03 codec-mesh entries).

**Decision.** Every example reads its command line through
`examples/common/cli.hpp`'s `Cli`, declares each flag family it shares once,
prints stage rows through `stage_table.hpp`, and keeps its reports out of
its `main`; [Examples](DESIGN.md#examples) states the parser's rules, the
families and where the reports live. With one home per rule, two examples
cannot read the same flag differently.

**What a command line sees.** A valid one runs as before. The spellings
unify:
- every example prints an error's cause and the whole usage line, and
  exits 2;
- a number is the whole argument, so `--max-frames 10x`, `--width abc`,
  `--fps +25` and `--color -1x1080` are refused;
- a value below a flag's minimum is refused rather than raised to it:
  `--stride`, `--remesh-every`, `--max-frames`, `--width` and `--height`
  at 1; `--mesh-every`, `--dirty-every`, `--hold-ms`, `--free-after`,
  `--sets` and `rig_viewer`'s `--frames` at 0;
- `--dynamic` and `--static` refuse each other, as `--hevc` and `--mjpeg`,
  `--lit` and `--normals`, `--serial` and `--rig`, and `--up-axis` and
  `--up-vector` do; `codec_mesh` keeps the last of a repeated `--up-axis`
  or `--up-vector`, as every flag does;
- every example that fuses takes the five fusion flags, so `fuse_render`
  and `fuse_viewer` gain `--max-weight`, and `codec_replica` gains them and
  `--cam-params`; its `--preload` reports its size as the others' does;
- a depth end given alone is checked against the other's default by the
  source, which names both values;
- `rig_viewer`'s Rig panel draws the map occupancy and recon heaps with
  `fuse_viewer`'s gauges, which change colour at the grow threshold and at
  90% of budget.

**Not taken.** Splitting each viewer's fuse thread into a class of its own.
Their `run()` keeps the shared-device bootstrap and the fuse thread, and the
split waits for the bootstrap to be shared.

**Verified.** Apple M5 Max, Release, every example built (Orbbec, FFmpeg,
Assimp and the viewers). `recon_example_cli` covers `parse_number`, every
`Cli` refusal, one-of groups and the generated usage, `check_voxel`, and
each flag family's defaults and validation; `recon_example_stage_table` the
table; and `recon_example_remesh_report` the survey's dilation on a device,
failing with the dilation flipped. Before and after on room0: `fuse_replica`'s mesh
is the same set of triangles, `codec_replica`'s bytes and distances and
`codec_mesh`'s on Rafa2 print the same, and `fuse_render`'s PNG is
byte-identical. Neither viewer nor `fuse_orbbec` was run.

### 2026-10-08 — The viewers draw with gfx's `StreamedAtlas` on its frame timeline: gfx owns the atlas ring, the in-frame copy and what a frame holds, and `fuse_viewer` copies its keyframe on the device (amends the 2026-08-08 seam-B, 2026-09-29 `rig_viewer`, 2026-10-05 viewers, 2026-10-07 Replica and 2026-10-08 exchange entries).

**The rule.** `fuse_viewer` and `rig_viewer` draw a live mesh with gfx's
`pipelines::StreamedAtlas`, through `viewer_atlas.hpp`'s `LiveAtlas`:

- A mesh's payload is an `AtlasJob`: the colour buffer of each camera that
  textured it, with its tile. The fuse thread checks each buffer with
  `copyable` before its camera textures the mesh, as `rig_viewer` did.
- The frame that commits the mesh records one update, every tile from its
  own buffer, into the ring image no unfinished frame uses, and binds
  `use(frame.number)`. The ring is the frames in flight plus one, so the
  copy never waits.
- The first textured job makes the ring, which for four 4K cameras is about
  400 MB, so a viewer that textures nothing holds none. A ring that cannot
  be made refuses the job like a refused copy: the mesh stays parked, the
  viewer says so once, and the next frame tries again.
- gfx's `RetireQueue` on the frame loop's timeline holds the job's buffers
  until the timeline reaches that frame, so a frame prep reuses one only
  after the copy has read it.
- At teardown, `LiveAtlas` destroys the gfx atlas before reclaiming those
  buffers. gfx waits for submitted frames or drains the queues if the
  newest frame was never submitted; draining the retire queue first would
  wait forever for that frame's unsignalled number after a failed submit
  and failed recovery. Sources stay alive until the submitted copies finish.
- A mesh no camera textured binds no atlas, and `HybridMeshPipeline` draws
  it in vertex colour against the fallback it owns.

`fuse_viewer` textures from one camera, so its job is one tile, the whole
colour image: its frame prep shares the colour with gfx's queue family, as
`rig_viewer`'s does, and the keyframe never visits the host. A keyframe
whose colour is not the colour camera's size, or not copyable, is not
textured, since the atlas is laid out once. `fuse_render` keeps its two
devices and its host readback (seam A), and uploads the keyframe into a
one-image `StreamedAtlas` in the frame that draws it.

**Why gfx's.** The ring, the barrier-ordered copy and "free it once the
frame that used it completes" are the renderer's to get right once, for
these viewers and the iOS renderer alike. gfx numbers its frames on a
timeline (gfx #118) and owns the atlas ring and the copy (#120), from
several buffers at once (#123), which a rig's atlas needs: one update a
camera would take a ring image each. What the viewers held instead goes:
`rig_viewer`'s image pool, its recorded barriers and copies, both viewers'
per-slot holds of an atlas and its colour buffers, and `fuse_viewer`'s
blocking upload, descriptor pool and set per remesh on the render thread.

**What stays.** `MeshExchange` retires per frame slot: `begin_frame`
still waits for a slot's last frame on its fence before handing the slot
out. The pins are open PRs: gfx #123 (on #120 and #118) and the core's #18,
which that gfx requires. gfx moves to its main once merged, and the core to
its v0.1.0 tag once its stack (#17, #19 and #20, plus #18) merges.

**Verified.** Apple M5 Max, macOS, Release with warnings as errors, Orbbec
SDK 2.10.6, FFmpeg and the viewers; 65 of 65 tests pass.
`recon_example_viewer_atlas` runs under the validation layer with
synchronization validation and no message. It checks each tile's copy
region, `copyable`'s refusals, an atlas that allocates nothing until its
first textured job and refuses the job when the ring cannot be made, a
frame's colour held while the frame waits and let go once it completes, a
refused copy that holds nothing and keeps the picture, and an untextured
mesh that binds no atlas. `fuse_render` on room0's first 120 frames, built
from the base and from this change, at the orbit view, `--follow 60`,
`--no-texture` and `--follow 60 --lit`: every PNG after is byte-identical
to one before. The fused mesh varies run to run in both builds: eight runs
of each build give the same two variants of a lit or untextured follow
view, a pixel apart. `fuse_viewer` on room0's first 80 frames ran in a
window for 45 s under synchronization validation (Khronos layer 1.4.363),
on MoltenVK's two queue families, so each keyframe's colour crosses
families: 5,280 frames and 2,653 textured meshes published, with no
message. That is macOS evidence only. `rig_viewer` was not run, since it
needs the rig.

**Teardown verification (2026-10-10).** A regression records a second
atlas update without submitting it while the first copy waits on a gate.
Both sources stay alive until the gate opens; destruction returns with the
timeline still at frame 1 and both sources released. The original destructor
hangs on frame 2. All 69 tests pass in Release; the atlas test and a separate
queue-submit OOM probe pass with the Khronos layer loaded and synchronization
validation enabled. Both live viewers build with warnings as errors; no
live-camera or interactive-viewer check was run for this fix.

### 2026-10-08 — recon's GPU tests run on the core's test policy: the validation layer on every CI leg fails a test on any error it reports, and a skip is a skip.

recon's tests made their instances without validation, and CI installed the
layer without loading it, so a missing barrier or a leaked object went
unreported; the core's tests ran under it.

- **One harness, on the core's.** `run_on_gpu` (`tests/gpu_test.hpp`) makes
  the instance, the physical device that meets recon's requirements, the
  device and an allocator on `volumetric_kit::core_test_policy` (core PR
  #17, included in the core #18 merge pin), and runs a test on them.
  `no_device.hpp`, `bare_device.hpp` and each recon device setup go. The
  viewer-atlas test keeps its gfx `HeadlessApp` and uses the same validation
  session, layer check and log capture around that context. `VKC_REQUIRE_VULKAN_DEVICE`, `VKC_TEST_VALIDATION` and
  `VKC_TEST_SYNC_VALIDATION` mean what they mean in the core, and
  `VR_REQUIRE_VULKAN_DEVICE` goes; `VR_TEST_HEVC_BACKEND` stays recon's.
- **Everything a test makes is destroyed inside the log capture**, so an
  object it leaks, which the layer reports at `vkDestroyDevice`, fails it.
- **A skip exits 77** (`test::kSkipExitCode`), every test's
  `SKIP_RETURN_CODE`, so CTest reports it as skipped rather than passed.
- **A device short of recon's requirements fails every GPU test**, not
  the Vulkan smoke test alone.
- **CI** sets `VKC_REQUIRE_VULKAN_DEVICE` and `VKC_TEST_VALIDATION` on every
  leg, and `VKC_TEST_SYNC_VALIDATION` on the NVIDIA legs and the sanitizer
  job.
- **LSan suppresses neither the loader's module nor the layer's**
  (`leak:libvulkan.so`, `leak:libVkLayer_khronos_validation`): they are on
  the stack of every Vulkan call and of the core's debug messenger callback,
  so either would hide a first-party leak made there. `leak:loader_` keeps
  the loader's own allocations. A handle leaked into lavapipe stays under
  `leak:lvp_`; the layer reports it instead.

**Initial validation.** Apple M5 Max, MoltenVK 1.4.2, the layer 1.4.363, Release with
Orbbec and FFmpeg: the 59 tests pass with `VKC_REQUIRE_VULKAN_DEVICE=1
VKC_TEST_VALIDATION=1`, and again with `VKC_TEST_SYNC_VALIDATION=1`; the layer
reports no error in recon's code in either run. `recon_gpu_test_harness`
checks that an error in a test's calls, a leaked semaphore and, under
synchronization validation, a missing barrier each fail a test that returned
0.


**Rebase verification (2026-10-10).** The core pin is the merged #18
(`511fed0`), including #17's test policy and legacy validation-environment
fix. The regressions added through recon #207 use the policy too, including
host OOM and viewer-atlas teardown. On macOS Release with warnings as errors,
Orbbec SDK 2.10.6, FFmpeg, Assimp and the viewers enabled, all 70 tests pass
with synchronization validation. Only the harness's deliberate violations
produce validation errors. The harness also passes with inherited legacy
layer settings; a missing Vulkan driver exits 77, or 1 when required.
No camera or interactive viewer was exercised in this rebase.

Ubuntu 24.04 and sanitizer CI exposed a collision in the fuser OOM test:
validation layer 1.3.275 allocated the same byte count as resize's buffer
owners, consuming the failure before growth. The test now captures that
allocation site in a direct grow and replays failures there. The original
test fails and the corrected test passes on Linux with that exact layer;
validation remains enabled throughout.

### 2026-10-08 — A decoded picture is its `YuvImage` with its pts and encoding: the one description of a device picture, which a frame takes as it is (amends the 2026-09-28 decoded-frame and 2026-10-02 GPU-regression entries).

**The rule.** `YuvImage` (`sensor/yuv_image.hpp`) is the one description of
a picture on the device: its planes and the queue family that wrote them, its
size, the matrix's weights, the range and the chroma siting. A decoder's
`DecodedPicture` is that `YuvImage` (`DecodedPicture::yuv`) with the pts and
the optional `ColorEncoding` beside it, and a frame takes it as its colour
unchanged. Whoever writes the picture describes it: `CudaPictures` and
nvJPEG give a CUDA buffer `kQueueFamilyExternal` (`place_cuda_buffer`),
`describe_color` an H.265 stream's weights, range and siting, and
`JpegDecoder::decode` JFIF's BT.601 full range and centred chroma, once for
both back ends. `VideoPixelLayout`, `DecodedPicture`'s own copies of the
fields and the Orbbec driver's `place_device_color` go. Two types had
described one picture, so each new source of decoded pictures had to map one
onto the other, and a source that missed the queue family or the matrix
would read CUDA's writes without taking them over, or fuse the wrong colour,
which `GpuFramePrep`'s checks cannot tell.

**Where it lives.** `yuv_image.hpp` is a header of `recon_sensor` that names
no camera, so `recon_sensor_video` describes its pictures with it without
linking the camera tier, as it already used `chroma_location.hpp`. A picture
carries the weights, which name any matrix. `VideoColorMatrix` and
`ycbcr_weights`, a named matrix's weights, live beside it, so a source that
is no decoder, such as an iOS camera whose buffer names its matrix, maps the
matrix with no header of the FFmpeg-gated decoders.

**Validation.** Apple M5 Max, macOS, Release with warnings as errors, Orbbec
and FFmpeg: the 58 tests pass with `VR_TEST_HEVC_BACKEND=videotoolbox`.
Without JFIF's description or `describe_color`'s weights, a test fails. The
decoder tests check the queue family each producer sets, so NVDEC's and
nvJPEG's `kQueueFamilyExternal` is CI's CUDA leg to check, as is the CUDA
code itself. Not run on a camera.

### 2026-10-08 — Apple's pixel-buffer importer is public: `PixelBufferImporter` in `recon_sensor_apple`, on every Apple build, iOS included, with no FFmpeg.

`PixelBufferImporter` (`sensor/apple/pixel_buffer_importer.hpp`, target
`recon_sensor_apple`), formerly the decoders' private `VtPictures`, is built
on every Apple platform, iOS included, and needs no FFmpeg, which it never
used, so an iOS camera can hand its colour (ARKit's `capturedImage`) to
`GpuFramePrep` on the device rather than convert it on the host. Its import,
surface cache and hold are as they were. It fills a `YuvImage`'s planes,
drops any buffer the image named, and leaves the colour description to its
caller, since a pixel buffer's format need not say how its samples are
coded.

- **In recon, not the family core.** What it fills is recon's `YuvImage`,
  and the iOS app reaches it through recon.
- **`VtJpeg` stays inside `JpegDecoder`**, its public face, and so behind
  `VR_WITH_FFMPEG`: on iOS it would reach a VideoToolbox key iOS has only
  from 17, above the app's floor of 16.
- **CI holds both promises.** The macOS leg, whose own build has FFmpeg,
  builds and runs the importer's test again without it, and compiles the
  importer for iOS 16 with exceptions enabled and `-Werror`.

**Validation.** Apple M5 Max, macOS, Release with warnings as errors: the 59
tests pass with Orbbec and FFmpeg (`VR_TEST_HEVC_BACKEND=videotoolbox`), and
the 49 of a build with neither, `recon_sensor_apple_pixel_buffer_importer`
among them. It and the H.265 and JPEG decoder tests run clean under the
Khronos validation layer with synchronization validation; without the
import's drop of a stale buffer it fails. The importer compiles against the
iOS 16 SDK with `-Werror`, once it includes `IOSurfaceRef.h` (iOS has no
`IOSurface.h`). Not run on an iPhone.


**Rebase verification (2026-10-10).** All 71 tests pass on macOS Release
with synchronization validation, warnings as errors, FFmpeg, Orbbec SDK
2.10.6, Assimp and the viewers enabled. The importer uses the core GPU test
policy introduced in #208; the existing OOM and viewer teardown regressions
remain intact. The no-FFmpeg importer build and test pass under the same
validation policy, and its implementation and public header compile for
arm64 iOS 16 with the iOS 27 SDK. The CI compile check keeps exceptions
enabled. No camera, iPhone or interactive viewer was exercised.

### 2026-10-11 — Decoded pictures name the queue families that read them: VideoToolbox's images are CONCURRENT across two or more.

calib's live view draws each decoder's NV12 picture as a tile on gfx's queue
while recon reads the same images on its own. MoltenVK gives the two
libraries different queue families, and the importer made its images
EXCLUSIVE, so gfx's copy was undefined in the way that appears to work on
Apple. `PixelBufferImporter::create` now takes the families, and the decoders
and `OrbbecStreamOptions` pass them on.

- **Families, not a mode**, as for buffers (2026-08-03): a caller passes
  recon's and gfx's unconditionally, and they collapse to EXCLUSIVE where
  they are one family.
- **The device's own family must be among them.** The importer moves the
  images to `GENERAL` on it and the GPU pass copies them there; a list
  without it would make both undefined.
- **`OrbbecStreamOptions` holds them by value**, like
  `GpuFramePrepConfig::color_queue_families`, since each start opens the
  decoders afresh. The decoders read a pointer once, at `create`, as
  `BufferDesc` does.
- **NVDEC and nvJPEG wait** (`TODO(sensor)`): a Linux tile also needs gfx
  to map I420.
- **The importer repeats the core allocator's family checks for now.** They
  live in the core's internal `sharing_for`, and the importer creates its
  images itself, so it cannot reach them; a `TODO(sensor)` replaces the copy
  once the core makes them public and recon's core pin reaches that.

**Validation.** Apple M5 Max, MoltenVK, Release with warnings as errors,
FFmpeg 9.0.2 and Orbbec SDK 2.10.6, under the Khronos validation layer with
synchronization validation: the importer's test imports a picture on recon's
family of a windowless `SharedDevice` (`TwoFamilies`) and copies both planes
on the renderer's, bytes exact. The layer checks a CONCURRENT image's create
info (it reports a duplicate index or a single family) but does not track
which family owns an EXCLUSIVE image: made EXCLUSIVE, the same copy passes
with no message. So the guard is the sharing mode recorded on the image,
which the tests check. No camera was opened.

### 2026-10-11 — Examples load the core's scenario format and keep only runtime adapters

*Amended 2026-10-12 (core 0.6.0, core #30):* the codec's step rules are
recon's: core no longer checks them, and `encoder_config` applies them
(DESIGN.md → "Examples"). *Amended 2026-10-14 (core 0.7.0, core #33; the
mesh-sequence entry, below):* so are the mesh conversion's shell bound
(`MeshSdfParams::validate`) and the frame number in a mesh source's path.

This replaces the 2026-10-08 command-line parser decision. Every executable
accepts `--config PATH` and `--help`, and directly consumes
`core::config::Config` for its application profile. The optional CPU-only
`core_config` target owns the versioned JSON vocabulary, source/operation
selection, typed defaults, numeric policy, field presence, paths and all static
validation. Recon's examples register no flags and parse no scenario JSON.

The adapters in `examples/common` only map those typed values to runtime sensor,
codec and mesh APIs and inject borrowed device resources. Camera discovery,
asset I/O, GPU and stream capabilities, calibration data and library API
preconditions remain with their existing owners. The SDK sync format and
recon calibration format are resources referenced by the scenario; neither is
replaced by this application-input change.

Defaults retain each executable's behavior, including source-specific depth
limits and dataset frame caps. The stream operation preserves #220's refusal
of explicit fusion/output settings. Core resolves relative paths against the
scenario file's directory before any resource is opened. Core's tests own
input-validation cases; recon tests pin the adapter boundary and run every
executable's help, legacy-input refusal and invalid-config startup without
opening hardware. `config/README.md` is the CLI migration guide.

### 2026-10-11 — Picture readback publishes buffer ownership for subsequent stages

This amends the 2026-09-28 decoded-frame ownership contract:
`YuvImage::queue_family` names the current owner, initially the writer.
`PictureReadback::read` acquires a buffer from that family. Both overloads
now take mutable pictures/frames and update the field only after a successful
submit and wait: the readback's family for EXCLUSIVE buffers, and
`kQueueFamilyIgnored` for CONCURRENT buffers. Later stages use that updated
descriptor; copies made before the read still carry the old metadata.
`fuse_orbbec` passes the updated set to preparation.

The regression releases an EXCLUSIVE picture from one queue family to
another, then reads, prepares and reads it again through both overloads.
Before the fix, synchronization validation reports an acquire without a
matching release (`VUID-vkQueueSubmit-pSubmits-02207`). A rejected picture
keeps its original ownership metadata. The two-family case is reported as
skipped on devices that provide only one family.

Timer creation also belongs inside `PictureReadback::create`'s allocation
handler. A failure injected at a captured timer allocation size and call
site must return `OutOfMemory`, and creation must succeed on retry. Before
the fix, that regression catches an escaping `std::bad_alloc`.

### 2026-10-12 — The viewers' window glue comes from gfx's `gfx_glfw`; recon keeps no copy (amends the 2026-08-02 bootstrap and 2026-10-04 Vulkan-foundation entries).

gfx #133 made the GLFW glue every windowed app repeated a gfx library,
`gfx_glfw`: the instance extensions, the surface, the framebuffer size, the
device shared with a compute library (`make_shared_device`) and ImGui's GLFW
backend (`ImGuiGlfw`). The viewers take all of it from there. What stays in
recon is what is recon's, once, in `viewer_common.hpp`: the shared device's
settings (recon's `device_requirements()`, the app's graphics requirements and
`window.validation`) and gfx's adoption of it.

### 2026-10-12 — The calibration file records lenses and is written: version 2 and its writer return with calib, its first caller (amends the 2026-10-06 camera-tier entries).

*Amended 2026-10-18 (the AprilTag-world entry, below):* the world may also
be a printed AprilTag, in OpenCV's marker axes.

*Amended 2026-10-27 (the depth-offset entry, below):* version 3 adds a
sensor's depth offset; the writer writes it only for a calibration that
carries one, and version 2 otherwise. *Amended again the same day (the
colour-controls entry, below):* version 3 also carries the colour cameras'
exposure, `color_controls`.

calib's rig app is the caller the 2026-10-06 cut-back waited for. Each
session records every camera's factory colour and depth models, unposed, and
each solve writes the posed rig with the lenses it solved with; both are
recon's file, written by recon, so the family has one schema and one writer.

- **What returns:** version 2 with `CameraCalibration` (the model and its
  `factory` or `calibrated` source), an optional pose, `depth_to_color` and a
  world sensor; `format_` and `write_array_calibration`;
  `rodrigues_from_rotation` and `rodrigues_from_matrix`.
- **What stays removed:** `CameraProjection`, `invertible_radius2`,
  `scale_camera_model` and `project_rational`. calib projects and unprojects
  with OpenCV, and the driver reports factory models at the streamed mode.
- **Version 2 keeps version 1's `device_calibration` key**, not 2026-10-06's
  `sensors`, so a version-1 reader (`verify_rig.py`, and recon before this)
  still finds the poses. `format` and `version` tell the two apart; a later
  version is `Unsupported`. Version 1 is still read for its poses alone.
- **Every field of a sensor is optional**, so a factory record is unposed;
  `SensorArray::open` refuses a sensor the file lists without a pose, as one
  it does not list.
- **The world is a sensor's colour camera only.** A floor AprilTag comes with
  calib's re-anchor, whose axes (z up out of the floor) are not
  libapriltag's; until then another world is `Unsupported`.
- **A written distortion names its model**, which the reader requires: the
  eight numbers mean nothing without it.
- **File I/O is the core's.** The writer takes the core's `WriteMode` (calib
  never overwrites a session file); the whole-file reads of this file, the
  Orbbec sync file and the Replica example's `cam_params.json` are
  `core::read_file`, so a missing file is `NotFound`; and `io`'s exception
  mapping is `core::exception_status`. recon needs core 0.2.0.
- **`rodrigues_from_rotation` is accurate to round-off at every angle**:
  `atan2` for the angle, first order only below 1e-8 rad, and past pi/2 the
  axis from the symmetric part. `cv::Rodrigues` returns exactly zero below
  about 1e-5 rad, so calib converts with this.

**Validation.** Apple M5 Max, macOS, Release with warnings as errors, FFmpeg,
Orbbec SDK 2.10.6, Assimp and the viewer: the 92 tests pass under the
validation layer; the camera tier alone, with no Vulkan, builds and passes
with Apple Clang and GCC 16; and the camera, sensor-array and io tests pass
under ASan and UBSan. `rodrigues_from_rotation` comes back within 3e-16 of
the angle, relative, at 0, 1e-9, 1e-6, 1e-5, pi - 1e-6 and pi. Mutation
check: a `cv::Rodrigues`-style zero below 1e-5 rad, `acos` for the angle,
first order up to 1e-6 rad, the axis from `sin` past pi/2, the pose written
uninverted, six significant digits, the `sensors` key, a distortion without
its model, the write mode ignored, a later version accepted, and an array
taking an unposed sensor each fail a test. No camera was opened.

### 2026-10-16 — A shared GLSL pinhole for recon and its consumers: `core/shaders/pinhole.glsl`, exported under the include root `recon_core` names (amends the 2026-08-11 texturing entry's per-tier copies).

*Amended 2026-10-22 (the install entry):* recon installs nothing, so the
root is the build tree's copy alone.

*Amended 2026-10-28 (the camera-free volume entry):* volume's depth
allocation no longer goes through this file; it unprojects through
`volume/shaders/depth_pinhole.glsl`, a copy of `vr_unproject_pinhole`,
operation for operation, that `recon_volume_depth_unproject` holds to it
bit for bit.

calib's depth checks run kernels of their own over recon's prepared depth,
and they must project and unproject it by recon's rules. A copy in calib
would be a second pinhole. The core cannot hold it, since its decisions lock
"No camera or sensor tier". Nor can recon's `camera` tier, the family's
camera vocabulary: it builds without Vulkan (`VR_CAMERA_ONLY`), and `volume`
and `tsdf` do not depend on it, so they could not include it. So recon's
`core` tier holds it, beside `color_common.glsl`, and recon exports it. gfx
depends on no recon, so its planned point-cloud shader restates the in-image
test and the nearest-pixel rule on its own, and its tests must hold that copy
to recon's rule.

- **Five functions, taking values:** `vr_world_to_camera(cam_to_world,
  world)`, `vr_unproject_pinhole(px, z, k)`, `vr_pinhole_pixel(p_cam, k, out
  px)` (the divide, wherever the pixel lands), `vr_project_pinhole(p_cam, k,
  size, out px)` (the same within the image) and `vr_nearest_pixel(px,
  size)`, with `k` = (fx, fy, cx, cy) and `size` a `uvec2`. A struct would
  fix one buffer layout; values let recon's scalar-layout cameras and calib's
  std430 ones pass theirs.
- **Today's rules:** every test negated, so a NaN fails it; the domain
  `[0, W) × [0, H)`; the last half pixel clamped to the edge pixel
  (2026-10-08).
- **recon's own kernels go through it.** tsdf's `project_pinhole` and
  `nearest_pixel` are thin wrappers, volume's depth allocation unprojects
  with it, and texture's `world_to_camera` and `camera_to_image` call its
  rigid inverse and divide. Each keeps the order of operations its caller
  had, so none changes a bit. The sensor prep's colour test unprojects and
  divides through it too, a rounding-level change its tests pass unchanged.
  What stays each caller's is the acceptance: texture's projection
  extrapolates past the image (2026-08-11), and the colour test bounds the
  rounded pixel, `[-0.5, W - 0.5)`, as the colour pass marks coverage.
  `lens.glsl`'s normalization at z = 1 stays with the lens model it mirrors
  (`camera::distort_rational`).
- **Exported through a target property.** `recon_core`'s
  `VR_SHADER_INCLUDE_DIR` names the root a consumer passes its shader
  compiler, as the core's `VKC_CORE_VERSION` rides `core_base`, so it
  reaches a project that fetches recon and one that finds it. In the build
  tree the root is a copy of the exported files alone, so a kernel that
  compiles against a fetched recon cannot include recon's internal GLSL and
  then fail against an installed one. Installed, it is the include
  directory, which the package config sets on the imported target, found
  from the config's own directory: under CMake 3.29 and older a
  dependency's package config resets `PACKAGE_PREFIX_DIR` (Ubuntu's Assimp
  gave `/usr`).

**Validation.** Apple M5 Max, MoltenVK, Release with warnings as errors,
under the Khronos validation layer. `recon_core_pinhole_gpu` runs the five
functions in a std430 kernel against the host's double: 28 written-out
edges, 3 unprojections, 8 unbounded projections and 2 rigid poses in exact
arithmetic, a general rotation, and 2420 grid cases around five cameras,
from 1 × 1 to 3840 × 2160. `recon_subproject_glsl` and `recon_package_glsl`
compile that kernel through the fetched and the installed root, and
`recon_package_quiet_not_found` checks a refused package leaves no `_vr_`
variable behind. The tsdf, volume, mesh, texture and sensor suites pass,
with four cases added: `recon_tsdf_integrate` an 8 × 2 colour image, so its
nearest pixel's clamp is checked against the width; `recon_volume_allocate`
a camera that tells a swapped focal length or principal point apart;
`recon_sensor_gpu_frame_prep` colour principal points a fraction of a pixel
off, which pin the rounded bounds; and `recon_texture_projective` a vertex
past the left border, so the extrapolation holds on both sides.

### 2026-10-16 — A brew upgrade needs no cache clearing: the Mac jobs install nothing, and the dependency cache keys on the core's Homebrew key.

recon follows the core's decision
[A brew upgrade needs no cache clearing](https://github.com/taojin-6/volumetric_kit_core/blob/main/DECISIONS.md#a-brew-upgrade-needs-no-cache-clearing).
Here a cached tree goes stale rather than failing: the cache holds only
`build/_deps`, which reaches Homebrew through version-free links such as
`/opt/homebrew/include/vulkan`, and Homebrew keeps each file's source
mtime, so after an upgrade make finds the new headers older than the
objects and keeps those built on the old ones. The Cellar paths (FFmpeg's
pkg-config results) are in the rest of `build/`, which checkout's
`git clean -ffdx` deletes before every job.

- **Keyed on the versions.** A `_build.yml` leg names its formulae once, and
  the dependency cache's key and restore prefix carry the key the core's
  `homebrew-key` action computes from their installed versions and their
  dependencies' (`none` off macOS). The first run after an upgrade rebuilds
  `build/_deps`.
- **Installed by the owner.** A job on the self-hosted Mac, the build legs'
  and the viewer's, installs nothing: it fails, naming any formula missing.
  A hosted runner installs what its image lacks.

### 2026-10-16 — A prepared frame's depth is shared like its colour, `GpuFramePrepConfig::depth_queue_families`; recon builds no point clouds and makes no calibration measurements; the same-K depth crop and nearest resampling are accepted on a stand-in lens, pending a fold check of the rig's own (amends the 2026-09-29 `rig_viewer` entry).

**Depth may be shared.** gfx is to draw points from each camera's prepared
depth in place, on its own queue family (GF5), for calib's 3D view of a rig
of any size. `GpuFramePrepConfig::depth_queue_families` names the families a
pass's depth is shared with, as `color_queue_families` does for colour and
independently of it. For either output, `create` refuses a count past
`kMaxQueueFamilies` or a family the device does not have, naming the output.
A missing family used to surface only at the first output, as core's
allocation error, and for colour not until a frame carried colour. The
buffer has no usage beyond the pass's storage and transfer usage, so a
consumer binds it as a storage buffer, not as vertices. Left empty, depth
stays EXCLUSIVE to the pass's family, as before; `rig_viewer`, which draws no
points, leaves it so. `SensorArray::Options::prep` gives every pass of an
array the same config.

**recon prepares RGB-D frames and stops there.** It builds no point clouds
and makes no calibration measurements: gfx is to draw points from depth and
a camera, and calib assembles the rig's views and runs its depth checks as its
own kernels. The point-cloud pass planned for recon is withdrawn.

**The depth crop and nearest resampling are accepted on a stand-in lens.**
The pass keeps the depth camera's K, so its pinhole image is the lens's view
cropped to that K and resampled at the nearest pixel: about 65 x 59 degrees
at the test's K (fx about 504 px), of the stand-in's raw 73 x 66. A K
reaching further would pass the lens's monotonic radius, where
`distort_rational` folds and two output pixels read one raw pixel.
`recon_sensor_depth_lens_fold` scans `distort_rational` forward along each
border pixel's ray. Its lens is a stand-in, not an SDK report: of
CL2A141000N's depth lens only k1 29.29 and k6 3.77 are on record (the
2026-09-28 GPU pre-processing entry). The other terms are calib's Femto-like
test lens, and the test holds the stand-in to the rest of the record. Its
rays reach its whole raw image, and undistorting moves a raw pixel 30.7 px
on average and 224.7 px at most, against the 31.5 px and 220.9 px measured
on CL2A141000N (the 2026-09-26 Orbbec driver entry). On it the widest output
pixel is at r = 0.914, and nothing folds within r = 3. Where a lens folds
depends on the terms not on record, so the crop holds for a rig only once
each camera's depth lens, as the SDK reports it, passes the same scan: a
hand check, not yet run.

**Verified.** Apple M5 Max, MoltenVK, Release with warnings as errors, core
`1315d80` (core #36): the 102 tests pass under the validation layer with
`VKC_REQUIRE_VULKAN_DEVICE=1`, and the sensor, texture and tsdf tests under
synchronization validation too. The changed tests also pass under ASan and
UBSan (Debug). `recon_sensor_gpu_frame_prep` crosses families on core's
`test::TwoFamilyDevice`: sets of 1, 3 and 5 frames prepared on recon's
family read the same on the renderer's. `recon_sensor_array_process` opens
arrays of 1, 3 and 5 sensors. `recon_sensor_apple_pixel_buffer_importer`'s
and `recon_sensor_picture_readback`'s cross-family cases move onto the same
fixture, and `test_image::make` fills through `test::write_image`. Planted
bugs each failed a test: depth's families ignored, depth given colour's,
colour given depth's, no check of the counts, no check that the device has
a family, colour's families checked as depth's, colour refused as depth,
vertex usage on the outputs, the array dropping its prep options, the fold
scan finding no fold, the guard missing the image's corners, a raw pixel no
ray reaches let through, and the stand-in replaced by a lens that folds
before its raw corners or one that moves raw pixels less than measured.

### 2026-10-18 — The calibration file's world may be a printed AprilTag, in OpenCV's marker axes, and the file stays version 2 (amends the 2026-10-12 calibration-file entry).

calib re-anchors its solved rig at a tag on the floor (calib's CA12b
plan). Its file could only leave `world` out, which records nothing, so
only calib's report would name the world. A world named in recon's file
keeps one home for what the world is.

- **OpenCV's marker axes, not libapriltag's.** calib finds the tag with
  OpenCV's detector and fits the frame to its corners in OpenCV's order. The
  re-anchor wants z up out of the floor. libapriltag's frame puts a floor
  tag's z down; the legacy detector used it, and so did the 2026-10-06
  draft (`d5dd23c6`). The draft's `{family, id, size_m}` is kept, but not
  its axes.
- **A sensor or a tag, never both, and no origin check for a tag.** A tag
  is not in the array, so it poses no sensor, and only its own fields are
  checked. A file that names both kinds has both read, so validate refuses
  it, for a file as for a struct.
- **The family is a free string, non-empty UTF-8.** recon detects nothing,
  so it keeps no list of families; the writer names one (`tag36h11`).
  `size_m` records what was printed, and nothing in recon reads it.
- **Still version 2.** Readers before this already refuse every world but
  a sensor as `Unsupported`, so they refuse a tag-world file rather than
  misread it. A file with a sensor world or none is unchanged byte for
  byte.

**Validation.** Apple M5 Max, macOS, Release with warnings as errors:
the default build's 89 tests pass under the validation layer, and so do
108 of 112 with the Orbbec SDK 2.10.6, FFmpeg, Assimp and the viewer. The
other four start `fuse_orbbec`, `fuse_render` and the viewers, and were
not run on that laptop; CI runs them. The camera tier alone, with no
Vulkan, builds and passes. Six planted bugs each failed a test: the
writer dropping the tag, a world with both kinds accepted, an id of -1
accepted, an infinite size accepted, the tag read but not kept, and a tag
world that needs a sensor at the origin. No camera was opened.

### 2026-10-20 — Mesh sequences clear the grid per frame, live rigs fuse Dynamic and free stale blocks, and `fuse_orbbec`'s stream stays in the calibration's world, uncropped.

A rig stream is to show what the Rafa2 stream shows: the reconstructed
scene, frame by frame, without colour for now. Core 0.13.0 opens the
`codec` section, `output.stream` and `output.csv` to FuseOrbbec, and core
0.14.0 `fusion.free_after`.

- **A mesh sequence clears per frame.** `codec_mesh` converts each mesh
  into a cleared grid, so each frame is that mesh alone, as before.
- **A live rig fuses Dynamic and frees stale blocks.** Each set fuses into
  the grid so far. Dynamic integration clears a surface the cameras see
  past, which takes it out of the stream at the first set that sees past
  it; every `fusion.free_after` sets (default 30) `free_stale_blocks` then
  gives back the blocks it left. The pass frees only blocks without an
  observed voxel, which the codec leaves out anyway, so it changes no
  frame. Under Classic no voxel loses weight, so a surface the cameras see
  past keeps its blocks, and every frame codes them, whatever the running
  average leaves of its mesh. `rig_viewer` fuses Dynamic by default and
  `codec_orbbec.json` sets `fusion.dynamic`; FuseOrbbec's default stays
  Classic, so a plain scan's PLY is unchanged. *Amended 2026-10-20:* by the
  owner's decision, FuseOrbbec defaults to Dynamic too (core 0.15.0, core
  #45), so a plain scan fuses Dynamic and `codec_orbbec.json` leaves the
  key out; `fusion.dynamic: false` selects Classic.
- **The calibration's world, uncropped.** The stream holds the geometry
  where `source.calibration` puts it: no upright transform, and the
  Geometry track's params stay empty. No such world is upright in the
  player, which keeps +Y up: a world anchored to a camera is that camera's,
  +Y down and on its side for a camera mounted on its side, and a tag's
  world is z-up (the 2026-10-18 entry). Turning a world upright is later
  work, on the writer's side or the player's. No crop:
  `fusion.max_depth_m` is the limit there is.
- **Numbered by the run, timed by the set**, so a set the loop missed is a
  gap in pts, not in numbers, and a clock stepping back does not end the
  run.
- **Deferred**, with the rest in DESIGN's Next work: a crop, raw
  recording, colour and an upright world.

**Validation.** `recon_example_set_codec` codes nine sets of two fake
cameras, read through a `SensorArray` and prepared on the GPU, into a
`.vrst` under the validation layer. A near wall the cameras later see past
leaves the Dynamic frames at the first set that sees past it and the grid
at the free passes the clock's ticks give; under Classic its observed
blocks stay in the grid and in every frame. No camera was opened; `fuse_orbbec`
has not yet run on the rig.

### 2026-10-22 — recon is fetched by commit, never installed: no install rules, package config or package tests (amends the 2026-10-16 GLSL pinhole entry and the 2026-10-17 amendment of the 2026-10-13 block-list entry).

**The rule** is the owner's, for the whole family: a sibling is consumed by
commit (`FetchContent`), as the player and ios fetch recon; only third-party
SDKs are installed prerequisites. calib's package config and install tests
were the one user of an installed recon; calib drops them before it pins
recon past this change. With siblings pinning different core minors, one
prefix could not hold them together anyway.

**What goes.** `VR_INSTALL` and every `install()` rule,
`cmake/volumetric_kit_reconConfig.cmake.in` and what fed it only
(`VR_FFMPEG_CONSUMER_MODULES`), and `tests/package` with its nine
`recon_package_*` tests, which built and installed a base-only core per run.
`VR_SHADER_INCLUDE_DIR` names the build tree's copy of the exported GLSL
alone.

**What stays.** The add_subdirectory checks (`tests/subproject`): the
defaults, the GLSL root, the core refusals, and the public headers' check of
the hash table's internals, now `recon_subproject_headers` on the headers
under `include/`. `recon_subproject_camera_only` drops its install step and
is registered only in a `VR_CAMERA_ONLY` build: its configures are Release
with Vulkan hidden whatever the leg, so the camera-only job, the one place
with no Vulkan at all, is its home.

### 2026-10-23 — A stage's device time holds every command recorded inside it, so both totals skip breakdowns; the extract and the decode's grid edits are timed, and a grid builds its block kernels at create (amends the 2026-08-10 breakdown entry).

**What the core changed.** Core 0.20.0 (#50; D3 of the data-flow design,
which the owner approved) tiles a timed batch: a timestamp before its first
command, at each change of tag and after its last. A stage's device time
therefore holds its fills, copies and barriers, and a tile counts to every
scope open around its tag. A row's host time splits into busy and blocked
(`wait_ms`, which every core wait reports), and `DeviceTime` says what a
device half holds: measured, run untimed (n/t), or not timeable (n/a).

**The 2026-08-10 rule reverses.** That entry made `total_gpu_ms` count
breakdowns, because a device span covered one dispatch and no parent held
it. A tile's parent holds it now, so both totals skip breakdowns, as the
core's do.

**Two breakdowns of one blocking submit nest.** A blocking submit credits
the scopes open at it, and two scopes open at once nest. The fused set's
allocation round (`allocate_and_integrate`) opens inside the compaction's
scope, so the compaction's tiles hold the round's. The integrator takes the
round's off, which is exact, as the two tile one submit. The round becomes
`"integrate"`'s `"  ..allocate"` breakdown: as the top-level `"allocate"`
row it was, the totals would count it twice. An async submit would not
separate them either, since a tile's scopes are fixed when its command is
recorded.

**What it adds**, with no submit, wait or result changed:
- `MarchingCubes::extract_device` takes `StageMetrics`: a `"marching cubes"`
  row over the compaction's `"  ..compact"` and the attempts'
  `"  ..dispatch"`. A timed extract writes timestamps into the compaction's
  submit and each attempt's, so one submit per extract stays the
  `TODO(mesh)`.
- The decoder's breakdowns tag their own commands, so each has a device half
  and `"codec decode"`'s is their sum. `"  ..apply"` holds the grid's zeroing,
  hash deletes and allocations, every round, through the `GpuStageScope` that
  `VoxelBlockGrid::remove` and `VoxelHashMap::allocate` and `remove` now
  take. Those rounds were untimed, and `"codec decode"` read as measured
  without them. `"  ..inverse"` holds the deblock's passes, in its submit
  (the 2026-10-21 deblock entry).
- `VoxelBlockGrid::create` builds the block kernels for a grid with
  attributes: the first remove, a player's first P-frame or a rig's first
  garbage collection, compiled two pipelines inside the frame.
- The examples' table prints host busy, wait and device, with n/t and n/a,
  and a codec row's device column is unset unless every stage was measured.
- The libraries need core 0.20.0.

**The extract's rows are named for what they time**, not as the design
sketched them (`mesh` over `..compact` and `..mc`). `"mesh"` is the player's
own row around the call, as `"decode"` is around `"codec decode"`, and the
two would merge. `"  ..dispatch"` is the attempt's whole submit -- reset,
dispatch, readback and a refit -- which `ExtractTimings::dispatch_ms` names
so, not the kernel alone. The compaction is `"  ..compact"`, as
`ExtractTimings::compact_ms` names it, and the map takes the name
(`compact_active_blocks_on_device`'s `row`): as the map's own
`"  ..active set"` it would merge with a fuse's or an encode's timed into the
same metrics. The decoder's breakdowns keep their names until its one-batch
form splits them as the design does.

**A timer is made by the first call that is timed.** An extractor, and a
grid for its block pass, make their `GpuTimer` when a caller first passes
metrics, not at create. A timer allocates its query pool when made, and
MoltenVK backs each pool with a Metal counter sample buffer, of which a
device has few: with a pool made at every create, the macOS runner refused
them in `recon_mesh_marching_cubes_sparse` (33 grids and 14 extractors,
`Could not create MTLCounterSampleBuffer`), and past that refusal MoltenVK
emulates the timestamps while the timer still reads available. Every other
tier's timer is made at create, as before this change.

### 2026-10-25 — `HevcDecoder` decodes through VideoToolbox directly on Apple: no FFmpeg, the hardware decoder or none, pictures in display order by their order count, and `VR_WITH_FFMPEG` on Apple a configure error (amends the 2026-10-06 device-only decoder and 2026-10-08 pixel-buffer importer entries).

**The rule.** On Apple, `HevcDecoder` decodes through a
`VTDecompressionSession` of its own (`vt_hevc.cpp`) and hands its pictures
out through `PixelBufferImporter`, as before, with no copy. FFmpeg's
VideoToolbox path is deleted. The class, `DecodedPicture` and the codes are
unchanged, and so is Linux, NVDEC through FFmpeg. `recon_sensor_video`
builds on every Apple platform, as `recon_sensor_apple` does, and the
Orbbec driver builds on macOS without FFmpeg. Nothing keeps the decoders
off iOS now: they need no FFmpeg, and the floor is 26.0, above the iOS 17
that VideoToolbox's hardware key needs (the 2026-10-08 entry), so they
cross-compile to iOS unchanged, as every tier does (the 2026-08-01 entry).
`VR_WITH_FFMPEG` on Apple fails the configure and names the fix, so no
option is left that does nothing.

**Why.** A Mac decodes the Femto's H.265 with nothing but the platform:
nothing to install beside the Orbbec SDK, and one decoder from the
bitstream to the picture, `rig_viewer` and `fuse_orbbec` through the Orbbec
sensor as before.

**What the decoder reads itself** (`hevc_syntax.cpp`); VideoToolbox decodes
the slices:
- **The parameter sets.** Each SPS is read whole, to its trailing bits,
  before it can refuse the stream, so bytes that do not parse are
  `IoError` and latch nothing. It refuses what the device path cannot hand
  out: not 8-bit 4:2:0 (Main 10, 4:0:0), and a window off the coded corner,
  which VideoToolbox fills from the corner (measured on an M4: the
  left/top-cropped clip decodes to other pixels than libavcodec's). It
  crops the right and the bottom itself. A format is made from the
  sequence's VPS, SPS and PPSs; a changed set makes a new one, on the same
  session where `VTDecompressionSessionCanAcceptFormatDescription` allows
  (an M4 allows none of the test clips' SPS changes, the range alone
  included, so each takes a new session).
- **Display order.** VideoToolbox hands pictures back in decode order. The
  decoder computes each picture's order count (8.3.1) and holds pictures
  as the output process does (C.5.2): shown once more wait than the SPS
  reorders (`sps_max_num_reorder_pics`), and all at a sequence started
  afresh, unless it drops them, at an end of sequence and at the end. A
  stream with no B-frames, the Femto's, shows each picture as it decodes.
- **Where decoding starts.** The first key frame after `create` or
  `reset()` starts a sequence afresh; its leading pictures, predicted from
  before it, are skipped, as FFmpeg skips them, and a picture before any
  key frame is `IoError`. `reset()` drops the session, so that key frame
  decodes on a fresh one.
- **The range.** The session writes NV12 at the range the VUI declares,
  video range without one. Asked for the other range, VideoToolbox converts
  the samples (an M4 decoding the full-range clip as video range no longer
  matches libavcodec). `frame_color.cpp` describes them from H.273 codes,
  which the VUI and FFmpeg's frame fields share, one home for both paths.

**Hardware or nothing.** `create` returns `Unsupported` where VideoToolbox
has no hardware HEVC decoder (`VTIsHardwareDecodeSupported`), and the
session requires it
(`kVTVideoDecoderSpecification_RequireHardwareAcceleratedVideoDecoder`),
with no software fallback: a stream it cannot open is refused. iOS 26, the
floor, has the key, so there is no availability check. One internal home,
`videotoolbox.cpp`, opens the sessions of both decoders (NV12 on
Metal-compatible IOSurfaces, the range its caller asks for: the stream's,
or JPEG's full range), decodes a sample and maps a failure to its code.

**A skipped access unit is reported.** VideoToolbox reports a picture whose
reference is missing as bad data, so each picture predicted from a skipped
access unit is `IoError` up to the next key frame (on an M4: the patch clip
without frame 1 fails frames 2 and 3 and decodes from 4).

**Tests.** Three `testsrc2` clips at 128x64 with B-frames
(`tools/make_hevc_fixtures.sh`) decode on VideoToolbox to the MD5 of
libavcodec's own decode, in display order: one with no colour description
(video range), one full range, and the first cropped 8 at the right and the
bottom. NVDEC takes nothing under 144x144, so they are Apple's. Main 10 and
the left/top crop are refused; a skipped access unit is reported; one
decoder sent the patch clip, its full-range copy and two `testsrc2` clips
in turn hands out each sequence at its own size and range, the samples the
stream's own; the decoder, Orbbec and JPEG tests pass on the direct path.
`recon_sensor_video_hevc_syntax` reads every committed clip's syntax and
refuses every truncated SPS, on every leg, the sanitizers' included. On
Apple, a configure with `VR_WITH_FFMPEG` fails with the message
(`recon_subproject_apple_ffmpeg`). The macOS leg's iOS check compiles
`recon_sensor_video` and the importer for iOS 26.0 and links them against
the iOS SDK (`tools/ios_build_check.sh`), leaving unresolved only their
dependencies' symbols (the core's, recon's other tiers' and Vulkan's).
Mutating the range, the reordering, the leading-picture skip or the crop
refusal fails a test.

**CI.** The macOS legs, the viewer's included, build without FFmpeg and
need no `ffmpeg` formula; the NVIDIA leg is unchanged.

**Validation.** Apple M4 (Mac mini), macOS 27.0.1, Release with warnings
as errors, the Orbbec SDK 2.10.6 and no FFmpeg: the 115 tests pass under
the validation layer with `VR_TEST_HEVC_BACKEND=videotoolbox`, the threaded
ones again under ThreadSanitizer (Debug), the iOS check links (Xcode 27.0),
and the viewer leg's targets build and their 6 tests pass.
RTX 5090, Ubuntu, Release with FFmpeg 8.0 and CUDA 13.4: the 115 tests
pass on NVDEC (`VR_TEST_HEVC_BACKEND=cuda`), and the host-only video tests
under ASan and UBSan. Not run on a camera; for iOS, compiled and linked
only.

### 2026-10-25 — A sender records each set into two batches, every count on the device, and its figures are per set (amends the 2026-09-27 encoder, 2026-09-28 device-memory, 2026-10-10 fusion and 2026-10-21 chunk entries).

*Amended 2026-10-26 (below):* `SetSender` and `CodecStream` are
`recon_sender`'s, and the frame's halves (`record`, `front_ran`, `redo`,
the pending `finish`) an internal class's.

**Per set, not per camera frame.** fuse_orbbec divided its stage totals by
the camera frames, and codec_replica printed none, so neither showed what a
set costs or how it varies. Both now log each set (`SetLog`,
`examples/common/set_log.hpp`): its rows, the device's counters across it,
and its latency from arrival and from capture to the frame's publish, on
the system clock. At the end they print, for P-frame, I-frame and uncoded
sets apart, each row's mean, nearest-rank p50 and p95, and max of host busy,
wait and device time over the sets that ran it; the counters per set; and
the latencies. Every set is timed: timing every frame cost 0.02 ms of host
time on the 4090 (M0). A set ends at its frame's publish, so the decode a
codec example runs after it counts in neither its rows nor its counters.
Writing and publishing a frame are rows of their own (`"write"`,
`"publish"`), and so is a sender's one CRC of it (`"crc"`). The `"set"`
row's wait is what its rows waited since the set arrived, so its busy and
wait add up to its wall time. A row first seen in a later set takes its
place after the row before it there, so a breakdown stays inside its
stage. A codec row's host columns are busy time on every path: `encode`
waits inside its row, a sender outside it (`make_codec_row`).

**The compactions are named apart.** The 2026-09-27 entry kept the
encoder's compaction under the map's `"  ..active set"`, and the free
pass's was too. One set's rows hold all three, which then summed into one
row. The map takes a row's name from its caller since the extract's
`"  ..compact"`, so the encoder's is `"  ..encode set"`, the free pass's
`"  ..stale set"`, and the fuse keeps `"  ..active set"`. The encoder's
`"  ..observed"`, `"  ..forward"` and `"  ..rans encode"` tile their own
device time too, where they held host time alone, so the sort's cost shows
in `"  ..forward"`.

**A sensor may notify, and the array waits on it (amends the 2026-10-06
`IRgbdSensor` and 2026-10-07 rig entries).** fuse_orbbec slept 1 ms between
empty polls. `IRgbdSensor::set_frame_notify` lets a sensor call a consumer as
each frame is held, or as it fails; the default refuses
(`Status::Code::Unsupported`), so a sensor that does not notify is polled as
before. `SensorArray::open` asks each sensor, and `wait_set` blocks on one
condition variable until a notification or the grouping's next deadline,
when a trigger stops waiting for a late sensor. The notification owns its
wake state through a `shared_ptr`, so a call in flight as the array goes
touches nothing freed. The Orbbec driver notifies from its mailbox after a
pair lands (no lock held across the two), on a disconnect and when the
colour decoder fails, so a fault surfaces at once rather than at the 10 s
silence limit. Each frame's `arrival_ns` is the system clock at the SDK's
frame callback, written into the depth frame's system timestamp, which both
colour decoders pass on unchanged (`rebuilt_pair`), so nothing new is
plumbed; a set's is its latest frame's.

**The prep records into the caller's batch (amends the 2026-09-28 residency
entry's kept staging).** `GpuFramePrep::record` and `record_set` record a
frame's copy and both passes into a caller's batch, and `SensorArray::
record_process` a set's. The depth, and packed colour at the binding
alignment after it, goes into the pass's one `core::HostRing` slot: in place
where mapped device memory is cached, one copy of a write-combined slot
elsewhere (core 0.22.0's `HostAccess::SequentialWrite`; the host only writes
it). A decoder's input slot stays cached, as the host checks, parses and
hashes the record in it. One slot suffices: the frame is copied in at
record, when the pass's last batch, which binds the slot and the kernels'
sets, must have resolved. Every frame's slot is
claimed before any buffer grows, so a record refused while that batch is
unresolved frees nothing it reads. `prepare` and
`prepare_batch` are the record, one submit and its wait, and abandon the
device planes only while `PendingBatch::in_flight` says the device may still
read them. The outputs are leased (a held flag the frame's deleter clears
with release order), as the core's ring slots are, where `use_count() == 1`
and an acquire fence raced under TSan with libc++.

**A set fuses in the caller's batch, strict too (amends the 2026-10-10
fusion entry and its 2026-10-22 amendment, and the 2026-10-24 one-batch
entry).** `Fuser::record_fuse` records one allocation round of every frame's
band (`VoxelHashMap::record_allocate_from_depth`) and the fuse
(`TsdfIntegrator::record_integrate`) after the prep. Where the set needs its
whole band -- a strict fuser, or any grid's first set -- the fuse is gated on
a word the round's gather closes on any failure (`hash_rounds.comp`, a third
instance with the depth round between its two passes). The set is checked
as the fuse checks it first (`TsdfIntegrator::check_frames`), so a refused
set records no round. `finish` settles the round and fuses a refused set
again, blocking (`FuseReport::redone`), as a set was fused before. That redo keeps the clock: a gated fuse takes its tick
at record and writes nothing, so `finish` takes the tick back
(`VoxelHashMap::rewind_tick`) first. Without that, each redo added a tick,
a free pass freed blocks a set early, and `recon_example_set_codec` failed
on the 4090, whose first and fourth sets redid. A redone set's batch recorded what came
after the fuse (the free pass, an encode's front half) over the grid without
the set. One round, rather than K device rounds of depth, is the owner's Q1:
a strict room0 set already took a single round (2026-10-22), so a redo
should be rare; the senders count them. Partial fusion is unchanged.

The blocking calls are these records, a submit and the settle:
`allocate_from_depth` repeats its round while lost races alone remain, up
to 10, with no readback buffer, and `free_stale_blocks` is
`VoxelBlockGrid::record_free_stale` (the compaction, the stamp pass sized
from its count, `record_remove` of the list it writes), one recording of
K = 3 rounds where it was up to 10 host rounds, so a contended block waits
for the next pass. Each record takes a serial, and `settle` moves the heap's
host copy only past every recording settled and every blocking call since,
so a redo or a free pass is never undone by an older settle.
`allocate_and_integrate` and `allocate_from_depth`'s `then` go;
`dispatch_with_retry` stays for triangles and the rehash. A warm strict set
of device frames is one submit, one wait and no staging buffer
(`recon_tsdf_fuser`), where it took two waits.

**An encode is two halves in the caller's batches (amends the 2026-09-27
encoder, 2026-10-17 count and 2026-10-24 one-batch entries).** The front
(`Encoder::record_front`) records, after a sender's fusion, everything whose
count only the device knows: the active set -- the map's held list, or a
compaction in the batch -- the observed pass, the sort, the forward, a
P-frame's diff, the count and the layout, each sized for the grid's block
slots and run for the device's count: the observed pass and the forward
from indirect slices, the sort's levels and the diff's and count's passes
over the bound, a group past the count returning. The sort's last level
writes the store's spare list, so no copy follows it. One gather
(`encode_results.comp`) leaves the host's figures in a results slot. The
back (`record_back`) builds the tables on the host, uploads them inline
first, and codes into a payload slot; `finish` lays the head out, PAYLOAD
last (`assemble_head`), the payload left in the slot (`EncodedFrame`), and
hashes the two unjoined. On a discrete GPU the slot is copied out to the
last frame's size per entry plus 25%; a frame past it gathers the rest again
into its own output range (a tail), since the core's test poison fills a
copied output range at its batch's front (the owner's Q3). `encode` is the
halves, 2 submits where it took 4, and no staging buffer warm where it took
4, one a batch that read back. The quantization steps never change, so
once a frame has finished, the front records no update for them. Two
choices follow from the device deciding the count. Whether the grid holds a block to code is the
front's to answer, so a P-frame planned at record over a grid of none
finishes as an I-frame of none, with no back batch, the bytes `encode` gave
a grid of none. And a compaction recorded into the front's batch gives the
map no count to hold, so an extract after the encoder compacts its own list,
where it took the encoder's back.

**A record's payload is one CRC in two runs (amends the 2026-10-21 chunk
entry).** A frame's file record and its live chunk each computed the
payload's CRC, over a vector joined from the frame. `stream::RecordPayload`
holds the head and the payload as two runs and their CRC, computed once by
`of`; `ChunkEncoder::frame`, `StreamWriter::append` (one `writev` of four
parts) and `StreamServer::publish` (its one copy, of both runs) take it and
compute none. The data-and-size overloads are `of` and that call. Bytes
changed after `of` go out under the CRC of what they held, which a reader
refuses: the tests change one to show no second CRC.

**The senders record a set into two batches (the 2026-10-10 fusion entry's
2026-10-22 amendment, its next step).** `SetSender`
(`examples/common/set_codec.hpp`), which fuse_orbbec and codec_replica share
in place of `fuse_and_code`, records a set's prep, its fuse, the free pass
when due and a coded set's front half (`CodecStream::record`) into one batch
and waits on it once; `CodecStream::finish` records the back half into a
second, waits, and writes and publishes the frame from its slot. A set fused
again records its front half again, in a batch of its own
(`CodecStream::redo`, `"wait: redo"`), and its frame's rows keep both
front halves. A set begins when its frames are handed out and ends
at the frame's publish, or its fuse's finish when it is not coded. Per set,
on the four-frame wall `recon_codec_replica_run` codes (RTX 4090, Release):

| set | submits | host waits | staging buffers | dispatches | barriers |
|---|---|---|---|---|---|
| P-frame, before | 6 | 6 | 4 | 18 | 31 |
| P-frame, after | 2 | 2 | 0 | 33 | 43 |
| I-frame (the first set), before | 7 | 7 | 5 | 16 | 30 |
| I-frame (the first set), after | 2 | 2 | 11 | 30 | 40 |

The first set places the rings' slots, which later sets reuse; a frame
larger than any before grows its payload slot, one staging buffer, and a
payload past the slot's copied guess adds a tail's submit. Dispatches and
barriers rise because every pass is sized on the device: an indirect pass
adds the dispatch that writes its arguments, and the sort, the diff and the
count run over the grid's block slots. `recon_example_set_codec`'s warm
coded sets have a front batch of 31 dispatches at two cameras, 41 at a
rig's four, 45 and 55 with the free pass, and a back batch of 4; it pins
every warm front below the Apple step near 80 at both counts. A set fused
again costs 3 more submits (the blocking round and fuse, and the front half
again), 2 when it is not coded.

**What follows the front batch's first dispatch.** The R3 rule records
small parameters ahead of a batch's first dispatch, since MoltenVK encodes
an update as a blit. Each stage keeps it within itself, but a sender's
front batch starts with the set's prep, so what the fuse and the encode
write comes after dispatches. A probe that counted each batch's commands
(RTX 4090 and lavapipe alike, two or four cameras) found, after the first
dispatch of a warm coded set's front batch, 5 inline updates (the fusion's
cameras and frustum planes), 7 fills and 2 copies -- 10 fills with the free
pass -- or 6 compute-to-transfer switches (9). The quantization steps were
a sixth update, which a warm encoder no longer records. Whether a fill is a
blit on MoltenVK is V1's question, and what the switches cost on Apple is
unmeasured: M5 and M6 run on the 4090.

**Redos, Q1's input.** The test's strict sets redo on the 4090 -- its first
set, a grid's first and so gated, and its fourth, whose new band loses
races, at two cameras and at four -- each growing nothing; lavapipe redid
the first in one run of two. At a rig's grid (four cameras of 640x576, 2 cm
voxels, 16,384 buckets, Dynamic, 24 to 156 new blocks a set) a probe redid
none after the first in 49 and 59 sets. The senders count redos
(`fused again`); codec_replica sends one camera a set, so M6 cannot count
four-camera redos, and the owner's rig run is what decides Q1.

### 2026-10-25 — The H.265 syntax reader is public, in a target of its own on every platform (amends the 2026-10-25 VideoToolbox decoder entry).

**The rule.** The H.265 syntax `HevcDecoder` reads itself on Apple is a
public header, `sensor/video/hevc_syntax.hpp` (`vr::sensor::hevc`), in a
target of its own, `recon_sensor_video_syntax`: host code on the family
core's base tier alone, with no codec, GPU or FFmpeg, so it builds on every
platform. `recon_sensor_video` links it privately, and VideoToolbox's path
reads streams through it as before. `ColorCodes`, the H.273 codes an SPS's
VUI declares, moves from the internal `frame_color.hpp` to the public
`color_codes.hpp`, since `Sps::color` is one; FFmpeg's path describes its
frames from the same type, and no copy is left.

**Why public.** `HevcDecoder` asks its caller for every access unit and
speaks of key frames: a lost unit costs the pictures up to the next one,
and `reset` starts the stream afresh at one. Public, the reader gives that
caller the key frames (`split_annex_b`, `is_irap`) and what a stream's SPS
declares, from the code the decoder reads with, at the cost of no
dependency past the core's base tier. Its callers are `HevcDecoder` on
Apple and `recon_sensor_video_hevc_syntax`.

**Its contract is recon's.** The parsers return `Result` and
`split_annex_b` a `Status`: bytes that do not parse are `IoError`, naming
why (the decoder passes them on under its name, so its codes are
unchanged); a NAL unit of another type than the call reads, or null data
with a size, is `InvalidArgument`; a host allocation failure is
`OutOfMemory` (`core::exception_status`, built with `-fexceptions`), as in
recon's other host calls, so no exception leaves the reader. A `Nal` of
fewer than three bytes has no payload, where its size less the header would
wrap. `BitReader` is public with the parsers, since every one of them reads
through it: the RBSP's bits past the emulation prevention bytes, u(n)
(`VKC_CHECK` past 32 bits), ue(v), se(v) and the trailing bits, with zeros
and `overrun()` past the end.

**Fields.** The structs keep the fields the decoder and the tests read;
`Pps::dependent_slice_segments_enabled` and `kBlaWLp`, which nothing read,
go.

**Tests.** `recon_sensor_video_hevc_syntax` links the public target alone
and runs on every leg: the bit reader's own cases (an 03 skipped after two
zeros and kept after one, Exp-Golomb codes up to 31 leading zeros and the
refusal of 32, reads past the end, the trailing bits, a unit with no
payload); the clips' syntax as before; every truncated SPS in storage of its
own length; a slice without its PPS or its SPS; each wrong NAL unit type;
and noise behind each header, refused only as `IoError`; and each call's
largest allocation failed, returned as `OutOfMemory`. The decoder tests
are unchanged. The macOS leg's iOS check compiles and links the new target
with the decoders.

**Validation.** RTX 5090, Ubuntu 26.04, Release with warnings as errors,
under the validation layer: the 107 tests pass without FFmpeg or CUDA, and
the 112 with FFmpeg 8.0 and CUDA 13.4 on NVDEC
(`VR_TEST_HEVC_BACKEND=cuda`); the host-only video tests pass under ASan
and UBSan (Debug, GCC 15). Apple M4 (Mac mini), macOS 27.0.1, Release with
no FFmpeg: the 112 tests pass on VideoToolbox
(`VR_TEST_HEVC_BACKEND=videotoolbox`) under the validation layer, and the
iOS check links (Xcode 27.0). Mutating the reader -- an 03 skipped after
one zero or never, a code of 32 zeros taken, se(v)'s sign, the trailing
bits unchecked, a short unit's payload, the null-data and type checks --
fails the test.

### 2026-10-25 — Frame prep samples NV12 plane images where they are, in a second entry of the colour pass, and the plane copies go (amends the 2026-09-28 decoded-frame and 2026-10-08 pixel-buffer importer entries).

**The rule.** `GpuFramePrep` reads a picture's `YuvImage::image` planes in
place. The colour pass has two entries over one body
(`undistort_color.glsl`: the lens, the coverage test, the chroma siting, the
bilinear weights and the conversion): `undistort_color.comp` reads planes
in a buffer or packed host words, and `undistort_color_images.comp` reads
the two plane images through combined image samplers at integer coordinates
(`texelFetch`), making each sample its 8-bit code again with
`round(v * 255)`. Both entries therefore give the same words for the same
samples. The two plane copies `record_inputs` recorded go, with the pass's
`sensor.raw_color` buffer (12.4 MB a 4K camera) and two of a frame's timed
spans. `KernelSetBuilder` binds storage buffers only, so the image entry's
layout is built by hand: two combined image samplers and the output, its
set from a pool of its own, its sampler nearest and clamped, which
`texelFetch` never uses to filter. No core change.

**What a plane image needs.** Frame prep takes `SAMPLED` usage and a view,
in `GENERAL` or `SHADER_READ_ONLY_OPTIMAL`, of a 2D image of one layer and
one sample: the entry reads it as a `sampler2D`, which a 3D, arrayed or
multisampled image's view does not match, where the copy took the first
layer and refused multisampling. `PictureReadback` still copies
the images and takes `TRANSFER_SRC`. One check serves both, told which
reader asks (`detail::ImageRead`), and each refusal names what its reader
needs. `PixelBufferImporter`'s planes carry both usages and a view, and
their one barrier out of `UNDEFINED` makes them visible to compute reads as
well as copies. Planes in a buffer (NVDEC, nvJPEG) and packed host words are
read as before.

**Why.** The copy cost 0.28-0.31 ms of GPU a 4K picture on the M5 Max (the
2026-09-28 decoded-frame entry), every set, and a scratch buffer per camera.
Sampling in place nets 0.80 ms of device time a 4-camera 4K set on the M4,
back to back, though the pass reads images more slowly than the copied
buffer (PERF.md's L3 row has both arms, and paced sets).

**Tests.** Plane images larger than the picture, sampled in place in either
layout, prepare to the words the same samples give from buffer rows, bit for
bit (`recon_sensor_gpu_frame_prep`, every leg), and on a Mac so does a
patterned picture on a real IOSurface, its rows longer than the picture,
against its own bytes in a buffer
(`recon_sensor_apple_pixel_buffer_importer`). Images without `SAMPLED`
usage (a view present), without a view, in a copy's layout, or 3D, arrayed
or multisampled (either plane) are refused by frame prep, and a
sampled-only image by readback. The claim-before-grow case no longer has
the copy's buffer to catch a growth recorded before its claim; it catches
it through the outputs.

**Validation.** Apple M4 (Mac mini), macOS 27.0.1, Release with warnings
as errors, under the validation layer: the 112 tests pass on VideoToolbox,
the sensor tests under sync validation too, and the iOS check links; four
4K surfaces' imported planes, views included, allocate in Metal exactly what
bare textures do. RTX 4090, Release with warnings as errors: the 107 tests
pass on the NVIDIA driver and on lavapipe under the validation layer, and
the frame-prep, readback, array and set-codec tests under sync validation.
Mutants, on every leg: growing the buffers before the claim (#282's
mutation) fails the claim-before-grow case, the validation layer reporting
the unresolved batch's outputs destroyed; Cb and Cr swapped, the luma read
a column over and an image without a view taken each fail, and so does the
check without its `SAMPLED` condition, without any one of its 2D, layer and
sample conditions, or with either plane's shape unchecked. The luma left
unrounded (`v * 255`) passes on the M4 and the RTX 4090 and fails on
lavapipe, which is why the entry rounds.

### 2026-10-25 — A run's sets, the fusion grid and the block layout are library calls: `RigSets` in the array's tier, `create_fusion_grid` in `tsdf` and `block_grid_params` in `volume`; opening a rig stays the caller's.

**The rule.** Three pieces the examples kept in `examples/common` move to
the tier each depends on, and no copy is left:
- `sensor::RigSets(array, frames)` (`sensor/array/rig_sets.hpp`, in
  `recon_sensor_array`): `next(metrics)` waits for a started array's next
  set on its sensors' notifications (`SensorArray::wait_set`), keeps each
  secondary's skew to the primary (`skew(i)`), and cuts the set that reaches
  the run's N frames to its first frames in the array's order; `IoError`
  after `kSilenceLimit` (10 s) with no set, naming a lone sync secondary's
  cause.
- `tsdf::create_fusion_grid` and `fusion_buckets` (`tsdf/fusion_grid.hpp`):
  the grid a fusing caller starts from, carrying the fuser's float `tsdf`
  and `weight` and packed `color`, its start sized to the voxel and capped
  to the device, as the examples' grid was.
- `volume::block_grid_params` (`volume/voxel_grid.hpp`): 8³-voxel blocks in
  the default hash layout, as many blocks as bucket entries. Both grid calls
  build with it, `codec::make_decode_grid` too, in place of its hand-built
  copy, and a static assertion holds the codec's 8³ to it.

**Why the library.** Six examples built the same grid from an example
header, codec_mesh took its layout from another, and `make_decode_grid`
wrote that layout out again by hand; the run's count and cut were tested
only where the Orbbec driver builds. In the library each has one home and
its tests run on every leg. Each goes to the tier it needs and no further: `RigSets` takes a `SensorArray`, and drivers
never link the array, so it is vendor-neutral and not the Orbbec driver's;
the fusion grid needs `volume` alone, which `tsdf` links; the layout is
`VoxelGridParams`', in `volume`, where `codec` reaches it.

**What stays the caller's.** Opening a rig is already three public calls
(`open_orbbec_sensors` or `OrbbecSensor::open`, `read_array_calibration`,
`SensorArray::open`), so no open helper is added: one would live in the
driver, which may not link the array. fuse_orbbec's one-camera choice
(`posed_camera`), its camera lines, the first frame's age and the counters'
report (`print_rig_stats`) stay in `fuse_orbbec_sets.hpp`: the library
prints nothing. The prep stays the caller's (`SensorArray::record_process`),
recorded into the set's front batch with its fusion, so a coded set stays 2
submits. Nothing changes in a run: fuse_orbbec prints the same lines, and
every grid, and so every coded frame, is the one it was.

**Tests.** `recon_sensor_array_rig_sets`, on every leg: a run made without
throwing, its skews empty before a set and read as copies, so none is left
behind by a set; the run's count, cut and skew over pushed fake sensors
(moved from `recon_example_fuse_orbbec_sets`), the cut past a missing
sensor letting the cut frame's picture go, a set past the run's frames
keeping none, the silence limit counted from the last set and then the
`IoError` naming a lone sync secondary (a 10-s wait), and the array's
refusal of a sensor that notifies nothing passed on; `recon_example_fuse_orbbec_sets` keeps the one-camera
choice and the report, where the driver is built. `recon_tsdf_fusion_grid`
is `recon_example_fusion_buckets`, plus the grid's layout and attributes
and a NaN voxel refused with no undefined conversion on the way.
`recon_volume_coords` checks `block_grid_params`, up to the most buckets
whose block pointers fit `int32` and saturating past it with no overflow;
the decode-grid tests and `recon_example_set_codec` pass unchanged on the
library's grids.

**Validation.** Release with warnings as errors, under the validation
layer. RTX 4090, Ubuntu 26.04, GCC 15, with Orbbec, FFmpeg, CUDA and
Assimp and synchronization validation: the 123 tests pass, the 4 that load
the Orbbec SDK not run and the JPEG test skipping on a GPU with no JPEG
engine. RTX 5090, with FFmpeg, CUDA and Assimp and synchronization
validation: the 118 pass. Apple M4 (Mac mini), macOS 27.0.1, with Orbbec,
Assimp and the viewers: the 133 pass on VideoToolbox, the three viewers
build, and the iOS check links. The changed tiers' tests pass under ASan
and UBSan (Debug, clang 21) on both GPUs. Mutating the moved code -- no
cut, a cut to none skipped, a cut set's frames uncounted, the primary's
skew kept, a skew read by reference, the silence limit not reset by a set,
a lone sync secondary's cause left out, fewer blocks than bucket entries,
another chain length, a fusion grid without `color` or with another band, a
NaN voxel's start passed to the integer conversion, the block count's int32
product -- fails a test or the build.

### 2026-10-26 — A sender's set step and its codec stream are library calls, in `recon_sender` after `networking`; the frame's halves stay internal, with no friend (amends the 2026-10-25 two-batch sender and keys-on-demand entries).

*Amended 2026-10-28 (below):* `CodecStream`, `CodecOutputs` and
`SentFrame` are `codec_stream::`, in a tier of their own, and the frame's
halves are the stream's public calls; `coder()` is gone.

**The rule.** `SetSender`, `CodecStream`, `CodecOutputs`, `SentFrame` and
`fuse_inputs` move from `examples/common` (`set_codec.hpp`,
`codec_stream.hpp`, `fuse_frame.hpp`) into `recon_sender` (`sender/`), and
no copy is left. The tier links `sensor_utils`, `tsdf`, `codec`, `stream`
and `networking`; no earlier tier links both `tsdf` and `codec`, so it
comes after `networking`. It prints nothing: `SetSender::send` takes the
set's rows and a `sent` callback, run once the set is published, and
returns what the set did (`Sent`: the fusion's report, a free pass's
refusal, the frame); `CodecStream::code` returns the frame (`SentFrame`).
`create` takes the session, and refuses one that is not one Geometry
track. The examples keep their reports, built from what the calls
return: the per-set table (`SetLog`), and the stream's rows, CSV and
summary (`CodecReport`, `codec_report.hpp`).

**The halves without a friend.** AGENTS.md has no `friend`. The front half,
its device rows, the front half again after a fusion done again and the
back half are an internal class, `sender::detail::StreamCoder`
(`stream_coder.hpp`, beside the tier's sources): `CodecStream` owns it and
hands it out only as an incomplete type (`coder()`), and `SetSender`'s
source includes the internal header. The public header gains one accessor
that nothing outside the library can call through, rather than four record
calls a caller could misuse.

**What changes in a run.** No frame: codec_replica's `.vrst` is the one
the examples wrote before, byte for byte. A CSV row the example cannot
write no longer makes the next frame an I-frame: the CSV is the report's,
written after the frame went out, and a frame the file and the player both
hold is a reference they can use. A moved-from stream refuses every call.

**Tests.** `recon_sender_codec_stream` holds the stream's cases of
`recon_example_codec_stream`, a sender's frames through `SetSender::send`
the file `code` writes, and the refused session; `recon_sender_set_sender`
is `recon_example_set_codec`, every count pinned as before, each set
published once with its frame's rows. `recon_example_codec_report` keeps
the rows, CSV, sizes, rates and comparison cases, and
`recon_example_config` the outputs, the session and the sender's config.

**Validation.** Release with warnings as errors, under the validation
layer. RTX 4090, Ubuntu 26.04, GCC 15.2, with Orbbec, FFmpeg, CUDA and
Assimp and synchronization validation: 127 tests pass and the 2 JPEG tests
skip, with no JPEG engine. codec_replica's `.vrst` from 00530fe6 and from
this change, on that GPU, compare equal byte for byte: the run test's wall,
and Replica room0's 400 sets at 1 cm, coded every set at period 30
(8,361,512 bytes) and every 7th set at period 4, its last frame through
`code` (4,838,104 bytes). Apple M4 (Mac mini), macOS 27.0.1, with Orbbec,
Assimp and the viewers: the 135 tests pass, and the iOS check links. Debug
with clang 21 under ASan and UBSan on the RTX 4090, the sender, report,
config and replica tests pass; the one leak reported, 248 bytes from the
driver's device enumeration, is every GPU test's.

### 2026-10-26 — NVDEC's pictures land in two plane images, VideoToolbox's form: CUDA's one copy writes them, a reader takes each over from outside Vulkan, and nothing hands it back (amends the 2026-10-08 decoded-picture and 2026-10-25 plane-image entries).

**The rule.** `HevcDecoder`'s NVDEC path copies each picture, with the one
CUDA copy it always made, into an R8 luma image and an R8G8 chroma image of
the picture's size, made with `core::create_exported_image` (core 0.24.0)
and mapped into CUDA once (`CudaImageMapping`, public beside
`CudaBufferMapping`), and hands them out as `YuvImage::image` with
`kQueueFamilyExternal`: the form VideoToolbox's pictures have, so every
platform's HEVC picture is NV12's two plane images. Its NV12 buffer goes,
with no switch back. nvJPEG keeps its I420 buffer: it writes only linear
memory. `GpuFramePrep` samples the images through the entry Apple's
pictures already take, and `PictureReadback` copies them.

**The crossing.** A new set of images is handed over to CUDA once, in
`GENERAL`, by a batch of the decoder's own before CUDA's first write: the
core refuses to take back contents still `UNDEFINED`. Each picture's reader
then takes the images over from `VK_QUEUE_FAMILY_EXTERNAL` in the layout
they are in, as it took the buffer: `YuvImage::queue_family` covers images
now. A readback names them its family's after, or `kQueueFamilyIgnored`
where they are CONCURRENT, so a later stage does not take them again;
images that needed no transfer keep `kQueueFamilyIgnored`, as Apple's do.
Nothing hands them back to CUDA, as nothing handed the buffer back: CUDA
overwrites the whole picture, the layout stays `GENERAL`, and Vulkan only
reads them, so nothing need survive the crossing. A set is reused only once
no picture holds either image: the two are aliases of one owner, so one
count says. The data flow is unchanged: no submit or wait is added for a
picture, and a new set's hand-over is one submit, once. An EXCLUSIVE image
crosses only from outside Vulkan (core), so `GpuFramePrep` refuses one named
as another family's with the frame's checks; for readers on other families
the images take `HevcDecoder::Options::queue_families`, which NVDEC ignored,
by `PixelBufferImporter`'s rule, now one check for both
(`sensor/sharing_families.hpp`).

**Why.** One picture form on every platform: frame prep and readback read
NVDEC's planes through the image path they use for VideoToolbox's, and a
viewer of the cameras gets image planes on Linux too, where a buffer would
cost it a Vulkan copy into images every picture (calib's live mosaic copies
a picture's planes only from images and refuses planes in a buffer).
Measured on the lab RTX 4090 (driver 615.71.10), Release, a 3840x2160
picture from a device surface with padded rows, CUDA events, the median of
each of three interleaved rounds of 400 copies: the copy into the two images
takes 0.030 ms, into the buffer 0.016 ms. Frame prep of one 4K camera with a
640x576 depth frame, the same picture, device time, medians of 600 prepares
over six interleaved rounds, other processes keeping the GPU busy
throughout: 0.100 ms from the images, 0.107 ms from the buffer, with words
bit for bit the same. One multi-planar image with plane views was the
alternative; it needs multi-planar formats in the core, and two images
need none.

**Tests.** `recon_sensor_video_hevc`, on NVDEC and VideoToolbox both: the
patch clip and its full-range copy decode to libavcodec's MD5 through
readback, and on NVDEC the cropped clip too (the testsrc2 clips stay
VideoToolbox's, under NVDEC's smallest picture); each picture is two images,
on NVDEC of its size in `GENERAL` with `kQueueFamilyExternal`; a held
picture's images are its own and keep its samples, and on NVDEC a dropped
one's are reused; frame prep from each decoded picture gives, bit for bit,
the words the same samples give from NV12 planes in a buffer; a readback
names NVDEC's images its family's; and the decoder's queue families make
them CONCURRENT, a readback leaving those `kQueueFamilyIgnored`, and a list
the importer would refuse fails `create`. `recon_sensor_video_cuda_interop`:
R8 and R8G8 images of odd size, and an R8G8B8A8 one mapped as the colour
attachment its usage makes it, which CUDA writes, read back with its bytes
three rounds over after one hand-over, never handed back; a taken
descriptor, an empty image and a 16-bit format are refused.
`recon_sensor_gpu_frame_prep`: plane images taken over from outside Vulkan
or named as the pass's family give the same words, in either layout; another
family's EXCLUSIVE images and a family the device lacks are refused.
`recon_sensor_picture_readback`: the same for its copies, each family's
naming after. `recon_sensor_orbbec_hevc`: the driver's pictures are images
on both platforms, CONCURRENT with the families it is given.

**Validation.** Release with warnings as errors, under the validation layer
and synchronization validation. RTX 4090, Ubuntu 26.04, with Orbbec,
FFmpeg, CUDA and Assimp: the 129 tests pass, the 2 JPEG tests skipping with
no JPEG engine. RTX 5090, Ubuntu 26.04, GCC 15.2, the same options: the 129
tests pass, the JPEG tests on its JPEG engine; Debug with clang 21 under
ASan and UBSan, NVDEC and nvJPEG included, the 14 sensor tests the change
touches pass, the only leaks reported the driver's. Apple M4 (Mac mini), macOS
27.0.1, with Orbbec and Assimp, on VideoToolbox: the 129 tests pass, the
sensor tests under synchronization validation too, and the iOS check links.
Mutants on the RTX 4090: a set reused while a picture holds it, a readback
that leaves NVDEC's images named as outside Vulkan's, frame prep without its
EXCLUSIVE-image check, and a set never handed over to CUDA each fail a test.
The layer cannot see CUDA's writes, so it passes with a reader's acquire
left out, and with the hand-over left out where the images are still
recorded as in `GENERAL`: both rest on Vulkan's rule for external memory, as
the buffer's acquire did. A colour attachment mapped without CUDA's flag
also passes on the RTX 5090, which lays it out alike; the flag rests on
CUDA's documented rule.

### 2026-10-27 — A sensor's depth offset lives in the calibration file, version 3, and frame prep subtracts it, the calibration its one source (amends the 2026-10-12 calibration-file entry).

*Amended the same day (the colour-controls entry, below):* the colour
cameras' exposure, `ArrayCalibration::color_controls`, joins version 3 the
same way, rig-wide.

A depth sensor can read a few millimetres too far or too near along its
axis, a constant of the unit, so its fused surface sits off the other
sensors' and off its own colour. The offset is measured with the poses, so
it belongs in the rig's calibration beside them, and it is applied where
depth becomes metres, so every reader of prepared depth sees the same
corrected value.

- **`SensorCalibration::depth_offset_m`**, in metres, + where the depth
  reads too far. Optional, 0 when absent; finite and at most 0.05 m either
  way, since a sensor's error is millimetres and more is a unit or sign
  mistake. A double, as every other number of the file, so the file shows
  what its writer computed and reads it back exactly; the array narrows it
  to float once for the pass, as the pass narrows the models.
- **Version 3, only for a corrected calibration.** A version-2 reader
  ignores keys it does not know, so a corrected file written as version 2
  would run uncorrected there; written as version 3 it is refused
  (`Unsupported`), refusing rather than misreading as the 2026-10-18 entry
  does. A calibration without a correction is written as version 2, byte
  for byte as before, so nothing that needs no correction changes. The
  reader takes versions 2 and 3, and refuses a `depth_offset_m` in a file
  that says version 2, even a 0: a file edited by hand would otherwise run
  uncorrected on an older reader. A later correction joins version 3 the
  same way.
- **Frame prep subtracts it in the depth pass it already runs:**
  `z = raw * scale`, then for `z > 0`, `z = max(z - offset, 0)`. 0 stays no
  return whatever the sign, so a negative offset never makes a hole a
  surface, and a return the offset takes to 0 or below is no return. It
  runs before the within-colour mask, which tests the corrected point.
  `GpuFramePrepConfig::depth_offset_m` carries it; the depth push constants
  grow from 64 to 68 bytes, and `GpuFramePrep::create` refuses a non-finite
  offset, whose `max` the shader leaves undefined. No pass, buffer, copy or
  host step is added.
- **The calibration is the one source.** `SensorArray::open` copies
  `Options::prep` for each sensor and sets the copy's offset from the
  calibration it already reads for the poses. An offset in `Options::prep`
  is refused: it would be a second source, and one for every sensor alike.
  Callers without a calibration (datasets, an array opened to measure raw
  depth) keep raw depth, and a frame's `RgbdFrame::depth` stays raw.
- **Who sees it:** every reader of the prepared depth: fusion, the
  texturer's occlusion test and a renderer drawing the depth in place.

**Tests.** `recon_camera_array_calibration` reads a version-3 file (+4.17
and -0.96 mm, and a sensor with none), takes the bound at ±0.05 m and
refuses past it, a non-number, an offset in a version-2 file (even 0) and
version 4; validate and the writer refuse NaN, ±infinity and an offset past
the bound; a bare sensor is written byte for byte as before, -0.0 and 0
alike, and with an offset as version 3; and every field of every sensor
round-trips, the text being the uncorrected one's but for the version and
the offsets' lines. `recon_sensor_gpu_frame_prep` checks depth to be
exactly `raw * scale - offset` for +4.17 mm, -0.96 mm and an offset equal to
a return (a power-of-two scale, so whether the compiler fuses the multiply
and the subtract does not matter), 0 where the offset leaves no return and
a hole a hole for either sign; the within-colour mask keeping exactly the
pixels whose corrected point the colour camera records, where a mask on
the raw depth would get a column or two wrong at each side of every row;
and a non-finite offset refused at create. `recon_sensor_array_process`
gives three sensors offsets of 0, +4 and -2 mm: each prepared frame equals
a pass made alone with its sensor's offset, and differs from one with none
exactly where the offset is not 0; an offset in `Options::prep` is refused,
with a device and without, NaN included.

**Validation.** Release with warnings as errors, under the validation
layer. RTX 4090, Ubuntu 26.04.1, GCC 15.2, CUDA 13.4, with the Orbbec SDK
2.10.6, FFmpeg and Assimp, and synchronization validation: 127 of the 129
tests pass, and the two JPEG tests skip, with no JPEG engine; the camera
tier alone, with no Vulkan, builds and passes. Apple M4 (Mac mini), macOS
27.2, with Assimp and the viewers (no Orbbec SDK there): the 125 tests
pass, and the three changed ones under synchronization validation too.
Thirteen planted bugs each failed a test: the offset added, not
subtracted; a hole made a return by a negative offset; the offset
subtracted after the within-colour mask; the pass ignoring its config's
offset; a NaN offset taken at create; the writer always writing version 2;
a 0 offset written; an offset read from a version-2 file; version 3
refused; the 0.05 m bound unchecked; and the array dropping each sensor's
offset, giving it a neighbour's, or taking one from its options. No camera
was opened.

### 2026-10-27 — The Orbbec driver defaults to H.265 at 3840x2160 and 25 fps, and opens each colour camera at the calibration's exposure: written where the camera differs, auto white balance on, and read back; an array refuses a sensor opened at another (amends the 2026-09-28 H.265 entry and the depth-offset entry above).

Reconstruction textures from each camera's colour at the camera's own size,
so the rig streams the colour cameras' 4K mode, in the codec the GPU
decoders take at the least bandwidth. A rig's cameras under auto exposure
each meter their own view, so one surface comes out brighter in one camera
than in another, and anything fitted to the colour at one exposure does not
hold at the next; so every camera of a calibrated rig runs at one manual
exposure, recorded with the calibration.

- **H.265 at 3840x2160 and 25 fps is the driver's default**
  (`OrbbecStreamOptions`), depth 640x576 as before: the streams core
  0.25.0's Orbbec scenarios default to, so the driver and the scenarios
  agree. The Femto Mega offers 3840x2160 at 25 fps at most, and both
  streams run at one rate, so a set has 40 ms. H.265 takes about 21 Mbit/s
  per camera at 4K against MJPEG's 185, and decodes to the camera's MJPEG
  colour (the 2026-09-28 entry). `Mjpeg` stays selectable: for calibration
  captures, whose frames are each coded on their own at nine times the
  rate, and for diagnostics such as fuse_orbbec's set dump. The 1280x720
  mode stays selectable and uncorrected: its content sits 1.17 px lower
  than its profile's `cy` (the 4K value / 3) says, about 1.6 mrad, measured
  against 4K frames on MJPEG. Recorded here; nothing defaults to it.
- **`camera::ColorControls`**, an exposure time and a gain in the camera's
  units, both above 0, is a manual exposure: auto exposure off, auto white
  balance on. **`ArrayCalibration::color_controls`** holds the one for every
  camera of the array, a version-3 field beside the world, written only
  when present, as the depth offset is; empty is auto exposure. No value is
  a public default: the right exposure depends on the room's light and its
  mains frequency, so the rig's calibration names it.
- **Auto white balance stays on.** Switched off, the camera returns to its
  stored temperature (6500 K as shipped) within 0.8 s (measured), a warm
  cast under a room's warmer light; a fixed white balance would need a
  temperature chosen per room. Auto white balance off and the temperature
  are never written.
- **Written at every open** (`OrbbecStreamOptions::color_controls`, by
  `OrbbecSensor::open` after the sync and role checks), unlike the sync
  settings, which are written only when asked: a camera keeps the last
  exposure written to it, in another process too (measured), so a camera
  left as it was would keep an earlier run's. Auto exposure is written the
  same way. Only what differs is written: auto exposure off, then the
  exposure and the gain, then auto white balance on; or auto exposure on,
  the stored values untouched. A camera under auto exposure reads its stored
  values, not the ones it runs at (measured), so one switched from auto
  exposure has both values written.
- **Checked before the first write, read back after the last.** A manual
  exposure needs auto exposure, exposure, gain and auto white balance to
  read and write, and both values within the camera's ranges and on their
  steps, or the open is refused (`Unsupported`, the control named) with
  nothing written; auto
  exposure leaves a control the camera lacks alone, so another model still
  opens. The flags, and under a manual exposure both values, are read back,
  and a camera that differs is refused naming each field, as the sync check
  is. `SensorInfo::color_controls` holds the manual exposure as read back,
  empty under auto exposure. Reading five colour properties takes a median
  6.1 ms per camera, at most 13.8 ms (measured), so this adds tens of
  milliseconds once per camera to an open that takes seconds, and nothing
  per set.
- **An array checks it** (`SensorArray::open`): with a calibration that
  names colour controls, a sensor whose `SensorInfo::color_controls` differ,
  as one opened without them, is refused rather than coloured at another
  exposure, whether the calibration lists sensors or not. A calibration
  without them checks nothing.
- **One place sets them.** The examples' `open_orbbec_cameras` gives every
  camera its calibration's controls; `rig_viewer` opens through it too, so
  it has no open of its own. A capture with no calibration runs at auto
  exposure, written.

**Tests.** `recon_sensor_orbbec_color_controls` opens a stub camera whose
colour properties go through the driver's own sequence: a camera that
matches gets no write, in either mode; a manual exposure over auto exposure
writes auto exposure off, the exposure, the gain (though it reads the same
already) and auto white balance on, in that order, and over a manual one
only the value that differs; auto exposure over a stored manual exposure
writes auto exposure on alone, neither value written nor compared; auto
white balance off is switched on in either mode, and the temperature is
never written; an exposure or a gain past the camera's range or off its
steps, or a control the camera lacks, is refused with nothing written, while
auto exposure opens a camera that lacks any of them and touches none of
those; a write the SDK fails is `IoError` with its message; a gain or a flag
read back otherwise is refused naming it; and a camera its sync check
refuses gets no colour write. `recon_sensor_orbbec_open_sensors` opens every
camera of a rig at the options' controls. `recon_camera_array_calibration`
reads the controls, refuses them in a version-2 file, at 0, as a non-number
and as a non-object, writes them byte for byte, version 3 only with them,
and round-trips them beside depth offsets, the text otherwise unchanged.
`recon_sensor_orbbec_conversion` refuses a 0 before the camera is looked
for; `recon_sensor_array` refuses a sensor at auto exposure or at another
gain under a calibration's controls, naming both, listed sensors or not, and
checks nothing without them; `recon_example_config` maps a live scenario
that names no colour mode to exactly the driver's defaults;
`recon_example_fuse_orbbec_sets` checks the report's exposure.

**Validation.** Release with warnings as errors, under the validation layer.
RTX 4090, Ubuntu 26.04.1, GCC 15.2, CUDA 13.4, with the Orbbec SDK 2.10.6,
FFmpeg and Assimp, and synchronization validation: 128 of the 130 tests
pass, and the two JPEG tests skip, with no JPEG engine; the camera tier
alone builds and passes. Apple M4 (Mac mini), macOS 27.2, with the Orbbec
SDK 2.10.6, the viewers and Assimp: the 137 tests pass. Of twenty planted
bugs, nineteen each failed a test: the range check, the control check, the
values written only where they differ or always, auto white balance
following the mode, auto exposure touching a control the camera lacks, the
gain or a flag not read back, a 0 taken, the controls written before the
sync check or not reported, the writer's version and its controls line, the
reader taking the controls from a version-2 file, validate taking a 0, the
array not checking them, the driver defaulting to MJPEG or 30 fps, and the
report leaving out the exposure. The twentieth, `open_orbbec_cameras`
dropping the controls, no host test catches, since opening cameras needs a
device; on a rig, `SensorArray::open` then refuses every camera. Three more
each failed a test: the array checking the controls only where the
calibration lists sensors, the step check dropped, and a rig's cameras
opened without the options' controls. No camera was opened.

### 2026-10-28 — `io` is two targets: images on the family core's base tier alone, mesh files on `mesh` (amends the 2026-10-03 asset I/O entry).

**The rule.** `recon_image_io` holds image decoding and PNG encoding
(`io/image_io.hpp`, with stb's implementation private) and links
`volumetric_kit::core_base` alone: no Vulkan and no recon tier.
`recon_mesh_io` holds the mesh files: PLY export (`io/ply_writer.hpp`), the
host triangle containers (`io/triangle_mesh.hpp`) and the opt-in
normalization helpers (`io/mesh_normalization.hpp`), with tinyply's
implementation private. It links `recon_mesh` PUBLIC, because `write_ply`
takes a `mesh::Mesh`. `recon_mesh_io_assimp` replaces `recon_io_assimp` on
top of it (`io/mesh_io.hpp`, `io/textured_mesh.hpp`). The headers stay in
`io/` and the namespace stays `vr::io`. Each target's export header and
macro are named after it (`image_io_export.hpp` and `VR_IMAGE_IO_API`, and
likewise for the other two). No `recon_io` target remains, and no alias.

**Why.** `recon_io` linked `recon_mesh` PUBLIC for `write_ply` alone. So
every image user pulled in `volume`, `recon_core` and Vulkan: the Replica
sensor, and a calibration capture that only writes PNGs. The two halves also
have different users, images on the capture side and mesh files on the mesh
and codec side. With separate targets, each consumer names the half it uses
once; the files can move later without a second rename. The 2026-10-03
entry's "depends on `mesh`" now holds for `recon_mesh_io` alone.

**Tests.** `recon_io_image` links `recon_image_io` alone, and
`recon_subproject_defaults` fails when `recon_image_io` links anything but
the core's base tier. `recon_io_ply` and `recon_io_mesh_normalization` link
`recon_mesh_io`; `recon_io_mesh_file` and `recon_io_textured_mesh` link
`recon_mesh_io_assimp`, the latter with `recon_image_io` for the PNG it
writes. The examples link the half they call: `vr_example_common` links
images; `fuse_replica`, `codec_replica` and `fuse_orbbec` link mesh files
for their PLY.

**Validation.** Apple M5 Max, macOS 27.2, Release with warnings as errors
and Assimp 6.0.5: the build is warning-clean, and the 23 affected tests
(`io`, the examples that read images or write PLY, the subproject consumer)
pass under the validation layer with a device required.
`recon_io_image_test` links `librecon_image_io.a` and the core's base tier,
nothing else. Adding `recon_mesh` to `recon_image_io`'s links fails
`recon_subproject_defaults`.

### 2026-10-28 — recon takes the core's GLSL export, sibling floor, test support and forward declarations, and keeps no copy (amends the 2026-10-16 GLSL pinhole entry and the 2026-10-08 test-policy entry).

**The rule.** recon keeps no copy of what core 0.27.0 to 0.29.0 hold for
every sibling (the core's 2026-10-28 entries):
- `recon_core` exports `pinhole.glsl` with
  `vkc_export_glsl(recon_core PREFIX volumetric_kit/recon/core/shaders ...)`,
  at the include path it had, and a consumer's kernels reach it with
  `INCLUDE_TARGETS volumetric_kit::recon_core`. `VR_SHADER_INCLUDE_DIR` is
  gone; the rest of the 2026-10-16 rule holds.
- The umbrella `volumetric_kit::recon` carries its `project(VERSION)` as
  `VKC_PACKAGE_VERSION`, so a consumer checks its floor on recon with
  `vkc_require_sibling`, as recon checks its own on the core.
- A header that names the core's Vulkan classes only by pointer or reference
  includes the core's `vulkan/fwd.hpp`; recon's `core/fwd.hpp` is gone.
- The tests are `main()`s on the core's test support: every test links
  `core_test_base` (`CHECK`, `VKC_TEST_SANITIZED`, `test::Scratch`), a GPU
  test `core_test_policy` (`run_on_gpu(vr::device_requirements(), ...)`,
  `test::Gpu`, the buffer readbacks), and one that fails or measures
  allocations `core_test_allocation_failure`. recon keeps only what is its
  own, in `vr_test`: the decoder skips and `bare_device`
  (`tests/decoder_test.hpp`) and its fixtures. A camera-only build's tests
  take the core's Vulkan-free part.
- `VR_CORE_MIN_VERSION` is 0.29.0, the oldest core with all of them.

**Why.** One home for each: the core took them so that the packages recon
is splitting into share them, and a copy left here would drift from it. No
header forwards and no alias keeps an old name.

**What stays.** The library code, the kernels and every test's checks; the
tests change only their includes, names and links. The harness's own test,
its LeakSanitizer case included, is the core's
`vkc_testing_gpu_test_harness_test`; recon keeps the decoder skips' test,
`recon_decoder_test_harness`.

**Tests.** `recon_subproject_defaults` configures a consumer whose
`vkc_require_sibling(volumetric_kit::recon <recon's version>)` accepts
recon, and `recon_subproject_old_recon` one whose floor, a minor above,
is refused naming recon's version. `recon_subproject_glsl` builds the
consumer's kernel through `INCLUDE_TARGETS`.

## Measured lessons

Not decisions, but the measurements that overturned an assumption about
where this pipeline spends its time. Kept because the mistake generalises.

The overlay's first finding **corrected a wrong assumption and redirected the
roadmap**. At `--remesh-every 1` on a room0 mesh of ~790 k vertices / ~264 k
triangles, `extract` cost ~55 ms/frame, which looked like whole-volume marching
cubes and pointed at the incremental block-mesh pool. The `ExtractTimings`
breakdown said otherwise:

| phase | ms |
|---|---|
| `arena alloc` | **49.8** |
| `readback` | 2.8 |
| `dispatch` | **2.0** |
| `neighbour lut` | 0.6 |
| `compact` / `descriptors` / `inputs` | < 0.2 |

(`neighbour lut` is the host table of the day; it is gone as of 2026-08-08 and
so is the field — see that decision, where the same instrument caught it costing
102 ms of a 133 ms extract at 107 k blocks.)

The GPU marching cubes was 2 ms — the pool would have optimised the one thing
that was already fast. (*On device that ordering inverts: see the 2026-08-09
incremental-extraction decision, where meshing dominates the frame and the pool
is back on. The lesson below is about the method, not about the pool.*) The cost
was `make_output_buffers` allocating a fresh
worst-case vertex arena (5 triangles per cell → hundreds of MB) **every call**,
so the driver faulted in and zeroed that many pages per frame. Making the arena
persistent and grow-only (below) took a 100-frame `--preload --mesh-every 1`
room0 run from **6.3 s to 0.8 s** — **15.9 → 132.5 fps**, an **8.3×** end-to-end
win — with a mesh identical triangle-for-triangle (same 793,473 vertices /
264,491 triangles, same canonical hash over the ordered triangle set). The
lesson is recorded because it generalises: *measure the phases before choosing
the optimisation* — three of us (the TODO, the roadmap, and the first analysis)
had independently guessed the wrong bottleneck.

The follow-up **fits the arena to the surface instead of to the worst case**, so
what the extractor retains is the mesh's real size rather than the 5-triangles-
per-cell ceiling it can never reach (a room scan fills ~2% of it). Correctness
does not rest on the guess: the kernel counts *every* triangle the field
produces and drops only those past `capacity`, so an undersized arena is
**detected**, never silently truncated — the host refits to the reported count
and re-runs, and since the count is a property of the field rather than of the
atomic ordering, that retry is guaranteed to fit. Steady state is therefore one
dispatch, against the two an unconditional count-then-fill pass would always
cost, and the plan is **predictive rather than reactive**: each extract records
the triangles-per-active-block density it measured, and the next call scales
that by *its own* active set (a seed of 64/block covers the first call, against
the 2560 worst case), so a growing scan plans ahead of its surface instead of
discovering every size increase by throwing a dispatch away. It scales that
density by nothing else — in particular **not** by what an arena already holds,
which is what made it compound across the output ring until an iPad Pro lost the
device (the 2026-08-03 slot-independence decision); and with `slot_count > 1`
"what the extractor retains" is one such fitted arena *per slot*, which is what
`ExtractTimings::arena_bytes` sums — arenas *and* index runs since the
2026-08-08 sharing decision, because sharing removes the proportionality that
let the run be dismissed as a sixteenth of the arena it covers. Measured on the
full 400-frame room0, peak RSS: **1250 → 985 MB (−21%)** at `--mesh-every 0`,
**1568 → 1302 MB (−17%)** at the default `--mesh-every 50`, with the mesh
identical triangle-for-triangle in both (991,167 vertices / 330,389 triangles,
one canonical hash over the sorted triangle set across main, this branch, and
both flag settings). It is also much *faster* end to end at `--mesh-every 0` —
17.4 s → 5.6 s (23.0 → 71.6 fps) — because on main the single final extract
spends ~12 s faulting in and zeroing a ~2 GB worst-case arena.

Landing it exposed a latent bug in the kernel: `mcEmitCell` **returned** on
overflow instead of continuing, abandoning the rest of that cell's triangles
*uncounted*, so the reported total was a lower bound. Harmless while capacity
was the unreachable worst case, fatal the moment the host started trusting the
count — the first refit undershot and the retry overflowed again. The emit loop
now `continue`s, which is what makes "tri_count is the field's true total" an
honest contract, and `marching_cubes_sparse_test` pins it with a fixture whose
density (one block of a sign-alternating field, ~1400 triangles where the seed
plans ~64) *forces* the refit path: reverting the `continue` fails it
deterministically with an `out_of_memory`, where the sphere fixtures fit inside
the growth headroom and could not tell the difference. Two further consequences
of the host trusting the count: the arena's `maxStorageBufferRange` guard now
tests the *request* rather than the request plus growth headroom (or it would
reject the top third of legal mesh sizes), and the extract stamps its
generation when it first touches the arena rather than on success — a call that
overwrites the arena and then fails must still invalidate every outstanding
`DeviceMesh`, and so must the dense overload, which shares that arena and can
now reallocate it.

**A hang read twice as something else (2026-09-28): the bucket locks were
across PCIe.** `recon_tsdf_mesh_integrate`'s divided sheet — a metre square
of 320 000 triangles, a quarter voxel each — failed ubuntu-26.04 CI with a
bare `vkWaitForFences`. The host's kernel log showed NVIDIA Xid 8 (the RC
watchdog's 7 s) and Xid 109 (context-switch timeout) against the test. The
first reading was load: two jobs shared the RTX 5090, and a re-run passed. The
second was bandwidth. The test ran 0.23 s on an M5 Max and 4–22 s on the
NVIDIA hosts, every buffer the map and the integrator bind is host-visible,
and the integrate kernel re-reads vertices per voxel. Both were wrong.
Per-dispatch timers on the RTX 5090 put the integrate kernel at 4.3 ms,
faster than the Mac. The failure reproduced there alone on an idle GPU, and
it was the sheet's **allocation**: some 2.6 M candidate items racing into
~400 blocks, each miss spinning on a bucket lock that lived in host-visible
memory, so every attempt was an atomic across the bus. Moving only
`bucket_mutex` to device memory:

| sheet (triangles) | RTX 5090, locks host-visible | locks device-local | M5 Max |
|---|---|---|---|
| 5 000 | 1 969 ms | 3.4 ms | 8.6 ms |
| 20 000 | 7 006 ms | 8.0 ms | 7.5 ms |
| 80 000 | 10 861 ms | 14.7 ms | 17.7 ms |
| 180 000 | 13 268 ms | 19.2 ms | 14.7 ms |
| 320 000 | past the watchdog | 24.4 ms | 19.4 ms |

The same instinct produced both wrong guesses: a cost invisible where it is
tested (unified memory hides host visibility entirely), read off totals rather
than phases. The per-slot bin counters are the next instance: host-visible
because the host reads them back, they cost the count and fill passes 115 ms
and 143 ms for the sheet on the RTX 5090 against ~5 ms on the Mac. Step 2 of
the residency decision moved them to the device.

**A flake read as a livelock (2026-09-28).** `recon_volume_delete` removed
8 192 blocks in one call and failed 2 to 7 Debug runs in 100 on the RTX
5090, with 1–6 blocks left to lock contention and none lost. A review read the
residue as two threads each holding the bucket the other spins on. Measured
per variant, 600 removes each: the code as it stood left blocks behind in 5,
taking the second bucket in index order in 4, and each of the two changes
that shipped in none. The retry loop had read two rounds without progress as a
capacity limit, which a remove never hits, and every round re-ran every coord,
so blocks already removed took their locks again. The residue was plain
contention, so the lock order was dropped.
