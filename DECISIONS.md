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
An entry that governs sensor's code is sensor's
([its DECISIONS.md](https://github.com/taojin-6/volumetric_kit_sensor/blob/main/DECISIONS.md#decision-index)),
with a line here for each too.

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
- [**2026-08-02**](https://github.com/taojin-6/volumetric_kit_sensor/blob/main/DECISIONS.md#2026-08-02--the-sensor-tier-is-a-contract-not-a-driver-collection-a-capture-driver-lives-here-only-if-this-repo-can-build) —
  The `sensor` tier is a *contract*, not a driver collection: a capture driver
  lives here only if this repo can build (`ICameraCapture` went 2026-10-07)
- [**2026-08-02**](https://github.com/taojin-6/volumetric_kit_grid/blob/main/DECISIONS.md#2026-08-02--the-mesh-arenas-extra-buffer-usage-is-declared-by-the-consumer-not-named-by-this-tier--and-usage-alone-does-not-reach-seam-b) —
  The mesh arena's extra buffer usage is declared by the *consumer*, not named
  by this tier — and usage alone does not reach seam B.
- [**2026-08-02**](https://github.com/taojin-6/volumetric_kit_sensor/blob/main/DECISIONS.md#2026-08-02--one-color-space-rule-and-a-named-working-space-8-bit-color-is-encoded-float-color-is-linear-converted-once-at-the-sensor-boundary-and-encoded-once-at-presentation) —
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
- [**2026-09-14**](https://github.com/taojin-6/volumetric_kit_sensor/blob/main/DECISIONS.md#2026-09-14--the-examples-poll-their-frames-through-the-sensor-contract-the-replica-reader-is-an-icameracapture-a-source-says-when-it-is-exhausted-and-a-frame-kept-past-the-next-poll-is-copied--the-last-one-included) —
  The examples poll their frames through the sensor contract: the Replica
  reader is an `ICameraCapture`, a source says when it is exhausted, and a
  frame kept past the next poll is copied — the last one included. (The
  reader became an `IRgbdSensor`, and the copy went, on 2026-10-07.)
- [**2026-09-24**](https://github.com/taojin-6/volumetric_kit_sensor/blob/main/DECISIONS.md#2026-09-24--the-orbbec-sdk-is-a-prerequisite-behind-vr_with_orbbec-installed-once-for-the-family-found-and-never-fetched) —
  The Orbbec SDK is a prerequisite behind `VR_WITH_ORBBEC`: installed once for
  the family, found, and never fetched. The floor rose to 2.10.6 on
  2026-10-07.
- [**2026-09-26**](https://github.com/taojin-6/volumetric_kit_sensor/blob/main/DECISIONS.md#2026-09-26--the-orbbec-driver-lands-as-sensororbbec-a-target-of-its-own-it-undistorts-colour-and-then-registers-depth-to-it-on-the-host-and-it-reads-the-rigs-sync-roles-without-writing-them) —
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
- [**2026-09-27**](https://github.com/taojin-6/volumetric_kit_sensor/blob/main/DECISIONS.md#2026-09-27--the-rig-is-orbbecrig-it-checks-the-cameras-against-the-rigs-sync-configuration-and-writes-it-only-when-asked-starts-the-secondaries-before-the-primary-keeps-the-cameras-on-the-hosts-clock-and-builds-each-set-around-a-primary-frame-poses-come-from-the-calibration-file) —
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
- [**2026-09-27**](https://github.com/taojin-6/volumetric_kit_sensor/blob/main/DECISIONS.md#2026-09-27--the-video-decoder-is-recon_sensor_video-a-target-of-its-own-over-an-installed-ffmpeg-hevc-in-host-pictures-out-on-the-first-hardware-back-end-that-decodes-a-built-in-clip-nvidia-ahead-of-an-integrated-gpu-each-ci-leg-requires-the-hardware-it-has) —
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
- [**2026-09-28**](https://github.com/taojin-6/volumetric_kit_sensor/blob/main/DECISIONS.md#2026-09-28--the-orbbec-driver-streams-colour-as-h265-on-request-every-colour-frame-is-decoded-in-order-on-a-thread-per-camera-ahead-of-the-mailbox-a-lost-frame-is-read-off-the-frame-index-not-the-clock-and-the-femto-megas-stream-is-decoded-as-bt601-full-range-which-it-codes-and-does-not-say) —
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
- [**2026-09-28**](https://github.com/taojin-6/volumetric_kit_sensor/blob/main/DECISIONS.md#2026-09-28--gpu-pre-processing-is-recon_sensor_utils-a-driver-hands-out-the-frame-as-captured-the-device-undistorts-depth-and-undistorts-and-converts-colour-and-fusion-reads-the-buffers-in-place-depth-and-colour-each-with-its-own-camera-rather-than-registered) —
  GPU pre-processing is `recon_sensor_utils`: a driver hands out the frame as
  captured, the device undistorts depth and undistorts and converts colour,
  and fusion reads the buffers in place, depth and colour each with its own
  camera rather than registered. *Amended 2026-10-29:* depth mixes the four
  raw pixels around the source where they are one surface, nearest only at
  an edge or a hole; and a frame with colour keeps only what both its
  cameras see, depth and colour each cropped to the other's field of view,
  no longer an option (depth's an option from 2026-09-29; the same-view
  entry).
- [**2026-09-28**](#2026-09-28--memory-the-kernels-use-lives-on-the-device-on-every-platform-and-the-host-only-records-commands-against-it-one-commandbatch-per-call-parameters-inline-bulk-bytes-staged-at-the-edges-small-results-read-back) —
  Memory the kernels use lives on the device on every platform, and the host
  only records commands against it: one `CommandBatch` per call, parameters
  inline, bulk bytes staged at the edges, small results read back.
  *Amended 2026-10-13:* `diagnostics()` is gone; the tests check
  `load_factor()`'s copy of the heap counter by behaviour. *Amended
  2026-10-24:* a frame's results and input go through core's `HostRing`
  slots, in place where device-mapped memory is cached and copied
  elsewhere, chosen by the device, with no switch.
- [**2026-09-28**](https://github.com/taojin-6/volumetric_kit_sensor/blob/main/DECISIONS.md#2026-09-28--a-decoded-colour-frame-never-leaves-the-gpu-the-platforms-hardware-decoder-for-h265-and-mjpeg-alike-hands-its-picture-to-vulkan-in-place-and-gpuframeprep-takes-i420-or-nv12-from-the-host-or-the-device) —
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
  too; the viewer keeps it EXCLUSIVE. *Amended 2026-10-29:* the viewer no
  longer chooses whether depth is kept to the colour view: sensor 0.9.0
  crops every frame with colour to what both its cameras see.
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
- [**2026-10-06**](https://github.com/taojin-6/volumetric_kit_sensor/blob/main/DECISIONS.md#2026-10-06--the-familys-camera-vocabulary-is-recons-camera-tier-a-double-precision-camera-model-and-the-sensor-arrays-calibration-file-version-2) —
  The family's camera vocabulary is recon's `camera` tier: a double-precision
  camera model with one templated projection, rigid transforms, and the
  sensor array's calibration file, version 2. It links the core's base tier
  and GLM, not Vulkan. *Amended 2026-10-11:* `VR_CAMERA_ONLY` configures it
  alone. Cut back the same day, below.
- [**2026-10-06**](https://github.com/taojin-6/volumetric_kit_sensor/blob/main/DECISIONS.md#2026-10-06--the-camera-tier-keeps-only-what-a-capture-uses-amends-the-entry-above) —
  The camera tier keeps only what a capture uses: the camera model and its
  check, the lens's forward template, rigid checks, and each sensor's pose
  from the calibration file's version-1 layout. Unprojection, rescaling,
  version 2 and the writer go until a caller needs them. *Amended
  2026-10-12:* version 2 and the writer are back, for calib.
- [**2026-10-06**](https://github.com/taojin-6/volumetric_kit_sensor/blob/main/DECISIONS.md#2026-10-06--tests-use-no-real-hardware-amends-the-2026-09-26-orbbec-driver-entry) —
  Tests use no real hardware: the three camera tests are removed, and a
  camera is checked by hand through `fuse_orbbec`. *Amended 2026-10-07:* no
  test looks for a camera either.
- [**2026-10-06**](https://github.com/taojin-6/volumetric_kit_sensor/blob/main/DECISIONS.md#2026-10-06--a-sensors-frame-is-rgbdframe-each-cameras-cameramodel-the-poses-in-double-a-sequence-number-and-the-pixels-it-holds-and-the-femto-megas-factory-extrinsic-is-made-a-rotation) —
  A sensor's frame is `RgbdFrame`: each camera's `CameraModel`, the poses in
  double, a sequence number, and the pixels it holds. The Femto Mega's factory
  depth-to-colour rotation is not one, and is made one. *Amended
  2026-10-29:* its depth may be on the device instead
  (`RgbdFrame::depth_device`), held by its buffer.
- [**2026-10-06**](https://github.com/taojin-6/volumetric_kit_sensor/blob/main/DECISIONS.md#2026-10-06--a-sensor-is-an-irgbdsensor-what-it-is-before-a-frame-sensorinfo-frames-polled-newest-or-drained-oldest-first-and-the-femto-mega-is-orbbecsensor) —
  A sensor is an `IRgbdSensor`: what it is before a frame (`SensorInfo`),
  frames polled newest or drained oldest first. The Femto Mega is
  `OrbbecSensor`. (The Replica source followed on 2026-10-07, and
  `sync_clock_to_host` became the SDK's global timestamps.)
- [**2026-10-06**](https://github.com/taojin-6/volumetric_kit_sensor/blob/main/DECISIONS.md#2026-10-06--several-sensors-are-a-sensorarray-vendor-neutral-started-in-their-rigs-order-grouped-by-trigger-or-by-sequence-number-and-posed-by-the-calibration) —
  Several sensors are a `SensorArray`, vendor-neutral: started in their rig's
  order, grouped by trigger or by sequence number, and posed by the
  calibration. (Sequence grouping went later that day; on 2026-10-07
  `TriggerGrouper` became its internals and the tolerance 0.4 of a frame
  period.)
- [**2026-10-06**](https://github.com/taojin-6/volumetric_kit_sensor/blob/main/DECISIONS.md#2026-10-06--an-arrays-set-is-prepared-in-one-batch-every-stream-of-every-sensor-one-submit-and-one-wait-host-colour-staged-on-a-thread-per-camera-first) —
  An array's set is prepared in one batch, every stream of every sensor, one
  submit and one wait; host colour is staged on a thread per camera first,
  and `prepare_set` goes. (Host colour and the threads went later that day.)
- [**2026-10-06**](https://github.com/taojin-6/volumetric_kit_sensor/blob/main/DECISIONS.md#2026-10-06--every-orbbec-frame-is-handed-out-as-captured-the-sdks-host-path-orbbeccapture-and-fuse_orbbec---gpu-go-amends-the-2026-09-26-2026-09-27-and-2026-09-28-orbbec-entries) —
  Every Orbbec frame is handed out as captured: the SDK's host path,
  `OrbbecCapture` and `fuse_orbbec --gpu` go, and the Orbbec driver needs
  the video decoders.
- [**2026-10-06**](https://github.com/taojin-6/volumetric_kit_sensor/blob/main/DECISIONS.md#2026-10-06--the-video-decoders-hand-out-pictures-on-the-device-only-the-hardware-path-is-fixed-per-build-a-stream-the-hardware-refuses-is-an-error-and-off-apple-vr_with_ffmpeg-needs-vr_with_cuda-amends-the-2026-09-27-decoder-2026-09-28-decoded-frame-and-2026-10-06-raw-frames-entries) —
  The video decoders hand out pictures on the device only: the hardware path
  is fixed per build, a stream the hardware refuses is an error, and off
  Apple `VR_WITH_FFMPEG` needs `VR_WITH_CUDA`. *Amended 2026-10-25:* on
  Apple the H.265 decoder uses VideoToolbox directly, with no FFmpeg, and
  `VR_WITH_FFMPEG` there is a configure error.
- [**2026-10-06**](https://github.com/taojin-6/volumetric_kit_sensor/blob/main/DECISIONS.md#2026-10-06--gpuframeprep-takes-colour-on-the-device-only-and-a-sets-depth-is-staged-on-one-thread-amends-the-2026-09-28-decoded-frame-and-2026-10-06-one-batch-entries) —
  `GpuFramePrep` takes colour on the device only, and a set's depth is
  staged on one thread. (Packed host colour came back the next day, and
  host Y'CbCr planes on 2026-10-29.) *Amended 2026-10-29:* depth may come
  from the device, read in place with no copy; only host depth is staged.
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
- [**2026-10-06**](https://github.com/taojin-6/volumetric_kit_sensor/blob/main/DECISIONS.md#2026-10-06--a-sensor-array-groups-its-frames-by-trigger-only-syncmode-and-sequence-grouping-go-amends-the-2026-10-06-sensorarray-entry) —
  A sensor array groups its frames by trigger only: `SyncMode` and sequence
  grouping go.
- [**2026-10-07**](https://github.com/taojin-6/volumetric_kit_sensor/blob/main/DECISIONS.md#2026-10-07--gpuframeprep-stages-a-host-frames-packed-colour-beside-its-depth-amends-the-2026-10-06-device-only-gpuframeprep-entry) —
  `GpuFramePrep` stages a host frame's packed colour beside its depth
  (`RgbdFrame::color_packed`): one copy in the pass's own batch, no host
  conversion, no threads. *Amended 2026-10-29:* Y'CbCr planes on the host
  are staged the same way.
- [**2026-10-07**](https://github.com/taojin-6/volumetric_kit_sensor/blob/main/DECISIONS.md#2026-10-07--the-replica-source-is-an-irgbdsensor-and-the-examples-fuse-what-gpuframeprep-prepares-icameracapture-capturedframe-and-ownedframe-go-and-io-loads-depth-as-stored-amends-the-2026-08-02-sensor-tier-2026-09-14-examples-and-2026-10-06-irgbdsensor-entries) —
  The Replica source is an `IRgbdSensor` and the examples fuse what
  `GpuFramePrep` prepares: `ICameraCapture`, `CapturedFrame` and
  `OwnedFrame` go, and `io` loads depth as stored. What the iOS scanner
  calls stays until it migrates; the frame gains no float depth. The viewer
  retains its last successful keyframe through a failed preparation or fuse.
- [**2026-10-07**](https://github.com/taojin-6/volumetric_kit_sensor/blob/main/DECISIONS.md#2026-10-07--a-rig-is-an-orbbecsensor-per-camera-in-a-sensorarray-on-the-sdks-global-timestamps-sdk-2106-or-later-each-camera-is-checked-against-its-sync-settings-at-open-and-written-only-when-asked-the-tolerance-is-04-of-a-frame-period-and-orbbecrig-its-start-order-and-the-public-triggergrouper-go-amends-the-2026-09-24-sdk-2026-09-27-rig-and-2026-10-06-irgbdsensor-and-sensorarray-entries) —
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
  *Superseded 2026-10-29:* Dynamic no longer clears on one sample; a set
  tests every camera for change, resets along changed rays, then fuses.
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
- [**2026-10-08**](https://github.com/taojin-6/volumetric_kit_sensor/blob/main/DECISIONS.md#2026-10-08--a-decoded-picture-is-its-yuvimage-with-its-pts-and-encoding-the-one-description-of-a-device-picture-which-a-frame-takes-as-it-is-amends-the-2026-09-28-decoded-frame-and-2026-10-02-gpu-regression-entries) —
  A decoded picture is its `YuvImage`, with its pts and encoding: one
  description of a device picture, which a frame takes as it is.
  *Amended 2026-10-26:* NVDEC's picture is two plane images, which CUDA
  wrote; nvJPEG's alone is a buffer.
- [**2026-10-08**](https://github.com/taojin-6/volumetric_kit_sensor/blob/main/DECISIONS.md#2026-10-08--apples-pixel-buffer-importer-is-public-pixelbufferimporter-in-recon_sensor_apple-on-every-apple-build-ios-included-with-no-ffmpeg) —
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
- [**2026-10-11**](https://github.com/taojin-6/volumetric_kit_sensor/blob/main/DECISIONS.md#2026-10-11--decoded-pictures-name-the-queue-families-that-read-them-videotoolboxs-images-are-concurrent-across-two-or-more) —
  Decoded pictures name the queue families that read them: VideoToolbox's
  images are CONCURRENT across two or more.
- [**2026-10-11**](#2026-10-11--examples-load-the-cores-scenario-format-and-keep-only-runtime-adapters) —
  Examples load the core scenario format; JSON, defaults and static validation
  have one owner. *Amended 2026-10-12:* the codec's step rules are recon's.
  *Amended 2026-10-14:* so are the mesh conversion's shell bound and a mesh
  path's frame number.
- [**2026-10-11**](https://github.com/taojin-6/volumetric_kit_sensor/blob/main/DECISIONS.md#2026-10-11--picture-readback-publishes-buffer-ownership-for-subsequent-stages) —
  Picture readback updates ownership before subsequent reads or preparation.
- [**2026-10-12**](#2026-10-12--the-viewers-window-glue-comes-from-gfxs-gfx_glfw-recon-keeps-no-copy-amends-the-2026-08-02-bootstrap-and-2026-10-04-vulkan-foundation-entries) —
  The viewers' window glue comes from gfx's `gfx_glfw`; recon keeps no copy.
- [**2026-10-12**](https://github.com/taojin-6/volumetric_kit_sensor/blob/main/DECISIONS.md#2026-10-12--the-calibration-file-records-lenses-and-is-written-version-2-and-its-writer-return-with-calib-its-first-caller-amends-the-2026-10-06-camera-tier-entries) —
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
- [**2026-10-16**](https://github.com/taojin-6/volumetric_kit_sensor/blob/main/DECISIONS.md#2026-10-16--a-shared-glsl-pinhole-for-recon-and-its-consumers-coreshaderspinholeglsl-exported-under-the-include-root-recon_core-names-amends-the-2026-08-11-texturing-entrys-per-tier-copies) —
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
- [**2026-10-16**](https://github.com/taojin-6/volumetric_kit_sensor/blob/main/DECISIONS.md#2026-10-16--a-prepared-frames-depth-is-shared-like-its-colour-gpuframeprepconfigdepth_queue_families-recon-builds-no-point-clouds-and-makes-no-calibration-measurements-the-same-k-depth-crop-and-nearest-resampling-are-accepted-on-a-stand-in-lens-pending-a-fold-check-of-the-rigs-own-amends-the-2026-09-29-rig_viewer-entry) —
  A prepared frame's depth may be shared with another queue family, as its
  colour is: `GpuFramePrepConfig::depth_queue_families`, with no vertex
  usage. recon builds no point clouds and makes no calibration
  measurements. The same-K depth crop and nearest resampling are accepted on
  a stand-in lens, pending a fold check of each rig camera's own (by hand,
  not yet run). *Amended 2026-10-29:* the resampling mixes where the raw
  pixels are one surface; the crop stands.
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
- [**2026-10-18**](https://github.com/taojin-6/volumetric_kit_sensor/blob/main/DECISIONS.md#2026-10-18--the-calibration-files-world-may-be-a-printed-apriltag-in-opencvs-marker-axes-and-the-file-stays-version-2-amends-the-2026-10-12-calibration-file-entry) —
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
- [**2026-10-25**](https://github.com/taojin-6/volumetric_kit_sensor/blob/main/DECISIONS.md#2026-10-25--hevcdecoder-decodes-through-videotoolbox-directly-on-apple-no-ffmpeg-the-hardware-decoder-or-none-pictures-in-display-order-by-their-order-count-and-vr_with_ffmpeg-on-apple-a-configure-error-amends-the-2026-10-06-device-only-decoder-and-2026-10-08-pixel-buffer-importer-entries) —
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
- [**2026-10-25**](https://github.com/taojin-6/volumetric_kit_sensor/blob/main/DECISIONS.md#2026-10-25--the-h265-syntax-reader-is-public-in-a-target-of-its-own-on-every-platform-amends-the-2026-10-25-videotoolbox-decoder-entry) —
  The H.265 syntax reader is public (`sensor/video/hevc_syntax.hpp`,
  `recon_sensor_video_syntax`): host code on the core's base tier alone,
  built on every platform, its parsers returning `Result` and its
  `BitReader` public with them.
- [**2026-10-25**](https://github.com/taojin-6/volumetric_kit_sensor/blob/main/DECISIONS.md#2026-10-25--frame-prep-samples-nv12-plane-images-where-they-are-in-a-second-entry-of-the-colour-pass-and-the-plane-copies-go-amends-the-2026-09-28-decoded-frame-and-2026-10-08-pixel-buffer-importer-entries) —
  Frame prep samples NV12 plane images where they are, through combined
  image samplers in a second entry over the colour pass's body, to the
  buffer form's words bit for bit; the two plane copies and their scratch
  buffer go. Plane images need `SAMPLED` usage and a view for frame prep,
  `TRANSFER_SRC` for readback, and the importer's carry both.
  *Amended 2026-10-26:* NVDEC's pictures are plane images too, which both
  readers take over from outside Vulkan (the NVDEC plane-image entry).
- [**2026-10-25**](https://github.com/taojin-6/volumetric_kit_sensor/blob/main/DECISIONS.md#2026-10-25--a-runs-sets-the-fusion-grid-and-the-block-layout-are-library-calls-rigsets-in-the-arrays-tier-create_fusion_grid-in-tsdf-and-block_grid_params-in-volume-opening-a-rig-stays-the-callers) —
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
- [**2026-10-26**](https://github.com/taojin-6/volumetric_kit_sensor/blob/main/DECISIONS.md#2026-10-26--nvdecs-pictures-land-in-two-plane-images-videotoolboxs-form-cudas-one-copy-writes-them-a-reader-takes-each-over-from-outside-vulkan-and-nothing-hands-it-back-amends-the-2026-10-08-decoded-picture-and-2026-10-25-plane-image-entries) —
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
- [**2026-10-27**](https://github.com/taojin-6/volumetric_kit_sensor/blob/main/DECISIONS.md#2026-10-27--a-sensors-depth-offset-lives-in-the-calibration-file-version-3-and-frame-prep-subtracts-it-the-calibration-its-one-source-amends-the-2026-10-12-calibration-file-entry) —
  A sensor's depth offset (`SensorCalibration::depth_offset_m`, + where it
  reads too far) lives in the calibration file, version 3, which the writer
  writes only for a corrected calibration; frame prep's depth pass
  subtracts it, 0 staying no return for either sign, before the
  within-colour mask. `SensorArray` sets each pass's offset from the
  calibration, its one source, and refuses one in `Options::prep`.
  *Amended the same day:* the colour cameras' exposure joins version 3.
- [**2026-10-27**](https://github.com/taojin-6/volumetric_kit_sensor/blob/main/DECISIONS.md#2026-10-27--the-orbbec-driver-defaults-to-h265-at-3840x2160-and-25-fps-and-opens-each-colour-camera-at-the-calibrations-exposure-written-where-the-camera-differs-auto-white-balance-on-and-read-back-an-array-refuses-a-sensor-opened-at-another-amends-the-2026-09-28-h265-entry-and-the-depth-offset-entry-above) —
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
- [**2026-10-28**](https://github.com/taojin-6/volumetric_kit_sensor/blob/main/DECISIONS.md#2026-10-28--io-is-two-targets-images-on-the-family-cores-base-tier-alone-mesh-files-on-mesh-amends-the-2026-10-03-asset-io-entry) —
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
- [**2026-10-29**](https://github.com/taojin-6/volumetric_kit_sensor/blob/main/DECISIONS.md#2026-10-29--the-camera-and-sensor-tiers-are-the-package-volumetric_kit_sensor-in-recons-sensor-until-it-has-a-repository-of-its-own-and-recons-libraries-link-its-frame-tier-alone-amends-the-2026-08-02-sensor-tier-2026-10-06-camera-tier-2026-10-16-glsl-pinhole-and-2026-10-28-io-entries) —
  The camera and sensor tiers are the package `volumetric_kit_sensor` (now
  its own repository, which recon fetches by commit): tiers `sensor_*`,
  namespace `volumetric_kit::sensor`, macros and options `VS_`, on the core
  alone. recon's libraries link `sensor_frame` alone, and nothing in the
  package includes recon or grid. The owner's decisions of 2026-10-28:
  repositories now, these names, Blackmagic stereo NVIDIA-only at first.
- [**2026-10-29**](https://github.com/taojin-6/volumetric_kit_sensor/blob/main/DECISIONS.md#2026-10-29--prepared-depth-mixes-the-four-raw-pixels-around-each-source-position-where-they-are-one-surface-and-keeps-the-nearest-pixel-at-an-edge-or-a-hole-amends-the-2026-09-28-gpu-pre-processing-and-2026-10-16-depth-crop-entries) —
  Prepared depth is the bilinear mix of the four raw pixels around each
  pixel's source position where all four return within 4 cm
  (`kBlendSpanM`, the truncation the rig fuses with), and the nearest raw
  pixel otherwise: a slope no longer comes out in half-pixel terraces, which
  fusion shaded as bands on floors and walls, and an edge or a hole is never
  blended. No option; about 1-2 µs of device time a frame.
- [**2026-10-29**](https://github.com/taojin-6/volumetric_kit_sensor/blob/main/DECISIONS.md#2026-10-29--gpuframeprep-takes-ycbcr-planes-from-the-host-too-a-cameras-yuyv-among-them-staged-beside-the-depth-and-converted-on-the-device-amends-the-2026-10-06-device-only-gpuframeprep-and-2026-10-07-packed-colour-entries) —
  `GpuFramePrep` takes Y'CbCr planes from the host too (`YuvImage::host`),
  and YUYV (`YuvLayout::Yuyv`, 4:2:2) among the layouts: a UVC camera's
  colour as its SDK hands it over, staged beside the depth as packed words
  are and converted on the device. `PictureReadback` refuses YUYV.
  *Amended the same day (the stereo entry):* luma alone (`YuvLayout::Luma`)
  joins the layouts; `ColorParams` is 120 bytes.
- [**2026-10-29**](https://github.com/taojin-6/volumetric_kit_sensor/blob/main/DECISIONS.md#2026-10-29--gpuframeprep-reads-depth-on-the-device-in-place-a-gpu-stages-depth-rgbdframedepth_device-taken-over-from-its-writers-queue-family-and-bound-where-it-is-amends-the-2026-10-06-sensor-frame-and-device-only-gpuframeprep-entries) —
  `GpuFramePrep` reads depth on the device in place:
  `RgbdFrame::depth_device` (a `DeviceDepth`), the 16-bit samples a GPU
  stage such as stereo matching wrote, at any even byte of a storage buffer,
  taken over from its writer's queue family and bound where it is, the
  image's first sample a push constant. No copy and no host round trip; a
  frame with nothing on the host still binds a word of the pass's slot. A
  buffer several depths and pictures share is taken over once a batch.
- [**2026-10-29**](https://github.com/taojin-6/volumetric_kit_sensor/blob/main/DECISIONS.md#2026-10-29--calibrated-stereo-pairs-are-the-stereo-tiers-and-stereorectifier-rectifies-them-on-the-gpu-as-opencvs-stereorectify-computes-it-sampling-through-the-colour-passs-own-code-amends-the-host-planes-entry-above) —
  Calibrated stereo pairs are the `stereo` tier's (`IStereoSensor`,
  `StereoFrame`), and `StereoRectifier` rectifies them: Bouguet's
  rectification as OpenCV's `stereoRectify` computes it, once, then a
  Vulkan pass a view that samples through the colour pass's own code, into
  views CUDA can import. `YuvLayout::Luma` for mono pairs.
  *Amended the same day (the stereo review entry):* rays past a lens's fold
  are 0; a shared buffer is taken over once; the views are used in turn and
  taken back with no transfer; a pair rectified already keeps square pixels.
  *Again (the RealSense stereo entry):* `StereoInfo` carries the source's
  id, model, clock, rate and colour exposure, which `depth_info()` copies;
  `IStereoSensor` gains `set_frame_notify`.
- [**2026-10-29**](https://github.com/taojin-6/volumetric_kit_sensor/blob/main/DECISIONS.md#2026-10-29--the-realsense-d400-driver-is-sensor_realsense-the-cameras-own-depth-and-its-colour-cameras-yuyv-on-the-host-librealsense-a-prerequisite-ci-installs-tested-against-rsutilh-with-no-camera) —
  The RealSense D400 driver is `sensor_realsense` (`RealSenseSensor`): the
  camera's own depth and its colour camera's YUYV on the host, converted on
  the device by `GpuFramePrep`; librealsense 2.59.1 a prerequisite that CI
  installs; tested against librealsense's own `rsutil.h`, with no camera and
  no virtual one.
- [**2026-10-29**](https://github.com/taojin-6/volumetric_kit_sensor/blob/main/DECISIONS.md#2026-10-29--stereorectifier-after-its-review-rays-past-a-lenss-fold-are-0-a-buffer-the-pictures-share-is-taken-over-once-and-the-views-are-used-in-turn-and-taken-back-with-no-transfer-amends-the-stereo-entry-above) —
  `StereoRectifier` after its review: rays past a lens's fold are 0; a
  buffer the pictures share is taken over once; the views are used in turn
  (rectify, read, drop) with no pool, a pair dropped only once its reads
  have completed, and taken back with no transfer; a pair rectified already
  is scaled by one factor; luma alone needs no matrix.
- [**2026-10-29**](https://github.com/taojin-6/volumetric_kit_sensor/blob/main/DECISIONS.md#2026-10-29--a-realsenses-infrared-cameras-are-realsensestereo-an-istereosensor-luma-on-the-host-the-projector-off-the-factory-calibration-as-stereoinfo-on-the-camera-realsensesensor-runs-on) —
  A RealSense's two infrared cameras are `RealSenseStereo`, an
  `IStereoSensor`: their Y8 as the camera rectifies them, luma on the host,
  and the colour camera's YUYV, the projector off by default, the factory
  calibration as `StereoInfo`; both RealSense drivers run on one internal
  camera; `IStereoSensor` gains `set_frame_notify`, and `StereoInfo` what
  the source is (id, model, clock, rate, colour exposure), which
  `StereoRectifier::depth_info()` copies.
- [**2026-10-29**](https://github.com/taojin-6/volumetric_kit_sensor/blob/main/DECISIONS.md#2026-10-29--stereo-depth-on-nvidia-ffsstereo-runs-fast-foundationstereo-on-tensorrt-between-two-cuda-kernels-from-the-rectifiers-views-straight-into-a-vulkan-depth-buffer-and-stereodepthsensor-makes-any-stereo-camera-an-irgbdsensor) —
  Stereo depth on NVIDIA (`sensor_ffs`, `VS_WITH_FFS`): `FfsStereo` runs an
  FFS export on TensorRT, its plan cached by GPU, TensorRT, model and size,
  between a CUDA kernel reading the rectifier's views in place and one
  writing 16-bit depth into a Vulkan buffer (`RgbdFrame::depth_device`);
  `StereoDepthSensor` is any `IStereoSensor` as an `IRgbdSensor`, on a
  thread of its own. Research use: NVlabs' code and the 23-36-37 weights,
  exported by `tools/ffs/export_onnx.py`, never vendored.
- [**2026-10-29**](https://github.com/taojin-6/volumetric_kit_sensor/blob/main/DECISIONS.md#2026-10-29--a-frame-with-colour-keeps-only-what-both-its-cameras-see-depth-outside-the-colour-cameras-view-and-colour-outside-the-depth-cameras-are-dropped-wherever-the-depth-comes-from-and-gpuframeprepconfigdepth_within_color-goes-amends-the-2026-09-28-gpu-pre-processing-entry) —
  A frame with colour keeps only what both its cameras see by their fields
  of view, each sensor's overlap from its own intrinsics and extrinsic, the
  baseline left out of the view test: colour whose direction lies outside
  the depth camera's view is a 0 word, and depth, from the host or the
  device, is zeroed outside the colour camera's view or where it lands on
  colour that is dropped. The depth range plays no part, and a frame
  without colour keeps all its depth.
  `GpuFramePrepConfig::depth_within_color` goes, with no option in its
  place. In a rig a camera no longer fuses depth only another camera can
  colour, nor colours surfaces only another camera's depth saw; the owner
  chose that.
- [**2026-10-29**](#2026-10-29--a-fixed-camera-flags-its-own-real-change-a-cusum-on-each-pixels-own-noise-on-the-device) —
  A fixed camera flags its own real change: `tsdf::ChangeDetector` keeps
  each pixel's history and own noise on the device, floored by the sensor's
  `DepthNoise`, and alarms on a two-sided CUSUM (k 0.5, h 8) or one frame
  past 6 sigma, the operating point measured on the rig's static takes; a
  surface first seen over no return that gives way to no return again
  flags `Vanished`, on the second invalid sample in a row: a lone dropout
  changes nothing.
- [**2026-10-29**](#2026-10-29--dynamic-fusion-evicts-on-change-and-averages-conflict-supersedes-the-2026-10-08-dynamic-set-entry) —
  Dynamic fusion evicts on change and averages conflict: a set tests each
  fixed camera for change (`FrameInput::slot`), resets along the changed
  rays (the new surface's band and the free space in front, keeping the
  history behind), then fuses in slot order; free space averages in at a
  tenth of the weight, only into voxels that hold weight. Nothing is
  cleared on one sample any more. Classic is unchanged.
- [**2026-10-29**](#2026-10-29--near-an-outline-a-voxel-takes-the-true-distance-in-both-modes-under-dynamic-not-where-its-camera-saw-a-change-and-a-departure-takes-what-it-left) —
  Near an outline a voxel takes the true distance, in both modes: the
  Euclidean distance to the nearest return around it, where the projective
  sample sees past the outline or into a gap. Under Dynamic not within
  6 pixels of a change its camera saw this set; and within 6 pixels of a
  departure the reset also clears the positive voxels the camera's rays see
  past or into a gap whose nearest return left, whoever fused them (a
  camera that supports them fuses them again that set, at one sample's
  weight).
- [**2026-10-29**](#2026-10-29--fuse_realsense-is-an-example-of-its-own-on-fuse_orbbecs-run-and-a-core-profile-of-its-own) —
  `fuse_realsense` is an example of its own on `fuse_orbbec`'s run
  (`run_live_fusion`, the one live loop) and a core profile of its own
  (`FuseRealsense`): the camera's own depth, colour at auto exposure, no
  codec yet.
- [**2026-10-29**](#2026-10-29--fuse_realsense-takes-ffs-depth-from-the-cameras-infrared-pair-through-the-same-opener-and-loop-only-where-sensor-builds-sensor_ffs) —
  `fuse_realsense` takes FFS depth from the camera's infrared pair, projector
  off, through the same opener and loop (`source.depth`), where sensor builds
  `sensor_ffs`; any other build refuses the scenario before anything opens.

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

*Amended 2026-10-29:* the choice is gone. Sensor 0.9.0 crops every frame
with colour to what both its cameras see, colour to the depth camera's view
as well (sensor's same-view entry), with no option, so the viewer no longer
reads `fusion.depth_within_color` (what `--all-depth` became). The table
measured the earlier depth-only mask; the crop has not been measured on the
rig.

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

### 2026-10-12 — The viewers' window glue comes from gfx's `gfx_glfw`; recon keeps no copy (amends the 2026-08-02 bootstrap and 2026-10-04 Vulkan-foundation entries).

gfx #133 made the GLFW glue every windowed app repeated a gfx library,
`gfx_glfw`: the instance extensions, the surface, the framebuffer size, the
device shared with a compute library (`make_shared_device`) and ImGui's GLFW
backend (`ImGuiGlfw`). The viewers take all of it from there. What stays in
recon is what is recon's, once, in `viewer_common.hpp`: the shared device's
settings (recon's `device_requirements()`, the app's graphics requirements and
`window.validation`) and gfx's adoption of it.

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

### 2026-10-29 — A fixed camera flags its own real change: a CUSUM on each pixel's own noise, on the device.

**The rule.** `tsdf::ChangeDetector` tests each fixed camera's depth a pixel
at a time against that pixel's own past, one pass a camera a frame, and
writes a mask (`PixelChange`: none, nearer, farther, vanished). Per pixel:
- `d_hist`, the cumulative mean of the first 30 valid samples, then an
  exponential average of weight 1/30, restarted at each alarm;
- `sigma_r = max(sqrt(pi/2) x running mean |d - d_hist|, sigma(d))`, each
  sample clipped at 3 `sigma_r`, the same memory, not learning for 30
  samples after an alarm; `sigma(z) = a + b z + c z^2` is the sensor's
  noise floor (`DepthNoise`, Femto Mega: b = 0.95 mm/m);
- `S+- = max(0, S+- +- r - k)` on `r = (d - d_hist) / sigma_r`; an alarm
  when either passes h, or one frame has `|r| > 6`; k 0.5, h 8;
- a camera's pixels test after its first 30 frames; an invalid sample
  changes nothing,
- but over no return: a pixel first valid once its camera tests had
  returned nothing, so its history lasts only while it returns. A lone
  invalid sample there is a dropout and changes nothing: the history, its
  sums and its count of valid samples go on as if it never happened. The
  second invalid sample in a row ends the history, and flags `Vanished` if
  it had three valid samples, a lone dropout skipped; the mask word's high
  30 bits carry the depth that left (0.1 mm), so a reader needs nothing
  else.

**Why.** Change is a camera disagreeing with its own past; comparing a
camera with itself cancels its calibration error. Measured on the rig's
static takes with injected changes (CPU, both takes, four cameras):
- the pixel's own `sigma_r` is what keeps false alarms rare: 550x fewer
  than the sensor model alone at the same k and h, and a static edge that
  always flickers learns a large one and stays quiet with no edge term;
- one false alarm per interior pixel in about 39 minutes, 15 rays a camera
  frame (0.006%), 90x fewer resets than an earlier prototype's spatial test;
- steps of 40 mm alarm in the frame they happen (98%), 20 mm 81%, 10 mm in
  2 frames (median), 5 mm in 5; a silhouette 0.5 m nearer on the frame it
  reaches a pixel (95.7%), the rest the next;
- without the freeze, sustained motion trains `sigma_r` up and doubles the
  lag at 10 mm a frame.
h is the one knob: h 5 is 1-2 frames faster on 5-10 mm steps and resets
0.145% of rays a frame. As measured, the floor is taken at the sample's
depth, and a restarted history is tested from its next sample at the
pixel's `sigma_r`, not inflated by the one-sample history's own error: 5.9%
of alarms follow another on the same pixel within 3 frames, part of the
rates above; inflating by sqrt(1 + 1/n) is untried. The warm-up counts the
camera's frames, as measured, not each pixel's samples: counted per pixel, a
pixel a mover first made valid over a background with no return never left
its warm-up before the mover did, so its later intermittent returns flagged
nothing, and a Dynamic replay of the 1 m/s sphere kept 382 cm² of ghost
there against main's 85 (take1, lab RTX 4090).

**Over no return.** Invalid is not empty, so an invalid sample alone never
flags. But where nothing returned before a surface came, nothing returning
after it is that surface leaving, and no other evidence of it will come:
with the camera's warm-up fixed, Dynamic still kept 102 / 121 cm² of the
1 m/s / 0.1 m/s sphere where no camera returns (main: 85 / 68), and 1.27 m²
of a walking column (main: 2.16 m²; take1 and take3, lab RTX 4090). The
three-sample condition is what keeps the rule quiet on still scenes: a
background's sporadic returns end silently. On the recorded takes'
validity (four cameras, sets 60 on), with a history ended on its first
invalid sample, three valid samples in a row flagged 14-23 times a second
(take3, take1), three in total 1.8-2.6k, two in a row 210-319 and five
none; over the 1 m/s sphere's replay three in a row flagged 6.6k times a
second, of which two in a row would miss 6%.

**A dropout is not a departure** (the owner's call, measured on Dynamic in
the next entry). Ended on its first invalid sample, a still surface over no
return re-learned after each dropout, and each one after three valid
samples flagged `Vanished`, 30 p (1 - p)^3 times a pixel a second at
dropout rate p (2.2 at 10%), each resetting its ray's band under Dynamic: a
column standing over no return with 15% i.i.d. dropouts was flagged 41k
times a second, pinholes spread over its face. So the second invalid sample
in a row ends the history, and a lone one is as if it never happened. A
history then lasts (1 + p) / p^2 frames on average and flags if two more
valid samples came first, each with probability (1 - p)(1 + p): p^2
(1 - p)^3 (1 + p)^2 flags a pixel a frame, 0.26 a second at 10% (tested)
and 0.55 at 15%, 8 and 5 times fewer. A real departure is flagged one
frame (33 ms) later. The mark takes a bit, so the history's three counts
are 9 bits each and `memory`, like `freeze`, is at most 511 samples (17 s;
the default is 30). On the takes' validity, the rule flags 42-69 times a
second on the still takes (14-23 before: sporadic returns a frame apart now
add up), against 1.8-3.0k ordinary false alarms; the walking column
110-113k times a second while it walks, without and with 15% dropouts on
it (112k before, without them); the standing column with them 8.1k (41k
before). Restarting
the count at a lone dropout instead (three valid samples literally in a
row) flags 13-21 times a second still and 6.2k on the standing column, but
misses a departure wherever one of its last three samples dropped out,
1 - (1 - p)^3 of the pixels: the walking column with 15% dropouts then
flagged 86k times a second against 110k without them. Eviction comes
first, so the count skips the dropout.

**Cost.** 24 bytes a pixel per camera, 8.8 MB at 640 x 576; one dispatch a
camera a frame.

**Tests.** `recon_tsdf_change_detector`: synthetic white Gaussian noise
scattered per pixel around the floor as the rig's interior pixels are.
False alarms per pixel per second 3.7e-4 (ToF-like, 2 m) and 5.1e-4
(stereo-like, `c z^2` from 1 to 3 m, the same at 1-2 m and 2-3 m), against
the rig's 4.3e-4; the step, ramp and sweep delays within the rig's; dropouts
change nothing (a twin that skipped them matches bit for bit); a camera that
leaves and returns compares with its own history. Over no return, a mover's
departure flags `Vanished` on every pixel with its depth (to the noise of
the history's mean, under 7 mm), a step
while it is there notwithstanding; a 2-sample sporadic return ends
silently, a 3-sample one flags; single-frame dropouts there change
nothing (a twin that skipped them matches bit for bit, through a step and
the departure, flagged on its second frame); a still surface there with
10% dropouts flags 0.256 times a pixel a second (the rule's 0.265; with a
lone dropout ending the history, 2.15, which fails, as do the twin and the
departure's frame); a depth past the word's 30 bits keeps the largest they
hold (107 km). The noise freeze holds a 10 mm a frame
ramp's drift before its history restarts to 30 mm at p95 (bound 60 mm;
80-90 mm with the freeze off, which fails).

### 2026-10-29 — Dynamic fusion evicts on change and averages conflict (supersedes the 2026-10-08 Dynamic set entry).

**The rule.** A Dynamic set, one instant, three passes a frame:
1. **Detect.** Each frame's camera, by its slot (`FrameInput::slot`), runs
   the integrator's `ChangeDetector` on its depth (the entry above).
2. **Reset.** On a ray its camera flagged -- read at a voxel's nearest
   pixel -- every voxel in front of the new sample and in its band goes
   back to unobserved (tsdf, weight and colour 0); history behind the band,
   which the new surface occludes, stays. A `Vanished` ray has no new
   sample: it resets up to the band of the surface that left, the depth its
   mask word carries, the set after the surface left (the entry above).
   Every frame's reset runs before any frame's fuse.
3. **Fuse,** in slot order. Free space past the band averages in as +trunc
   at a tenth of a sample's weight, geometry only (no colour), and only into
   a voxel that already holds weight.

**Why.** The former Dynamic cleared a voxel whenever any camera's one
sample put it past the band. That let noise near the band's front edge
flicker the surface, let cameras erase each other at rims, and never
cleared debris within 4 cm of a surface after an object left (measured:
the moving sphere's 80 / 72 cm² ghost, fast / slow, stays). Change is a
camera disagreeing with its own past; conflict is cameras disagreeing with
each other. Only change evicts; conflict averages.
- **Reset rule:** an earlier prototype's, tested there (an object that
  approaches and recedes leaves no older layer; occluded history survives).
- **Free space only into weighted voxels.** It wears away a surface no
  change removed, at a tenth of a sample's weight, so a camera seeing past
  another's rim shifts that surface by about a tenth of the band at equal
  depth, more when the camera seeing past is nearer (weights go as 1/z^2):
  one 1.6 times nearer or more carves the other's rim, as the former clear
  did (the shader's arithmetic run to equilibrium). Never
  making an unobserved voxel observed keeps space a reset cleared, and a
  mover's blocks, unobserved, so the free pass frees them, as before. The
  A/B's other arm fused free space into every voxel (known-empty at a tenth
  of a sample's weight). On the box that enters, stays and leaves (take3,
  lab RTX 4090) it evicted the same -- no ghost, cleared in the set it left
  -- with 0.3 points fewer silhouette holes (2.50% against 2.83%), but 20%
  more bitrate (13.7 against 11.4 Mbps) and 33 MB more allocator memory, as
  free space then holds weight wherever a camera looks. Unknown it stays.
- **Slot order.** Whether one camera's free space meets a voxel another
  camera's band first weights in the same set depends on the order; in slot
  order the grid is the same bit for bit whatever order the frames come in.
  The dependence on the slots themselves stays, in a voxel's first weighted
  set only; removing it would take a weight snapshot or a free-space pass of
  its own.
- **Noise.** `sender::fuse_inputs` gives every camera the default
  `DepthNoise` (the Femto's): the sensor package's frame does not carry its
  noise yet, which is a sensor change.

**Measured** on replays of the rig's recorded takes with synthetic movers
drawn into the depth (lab RTX 4090, against main):
- Still scenes: silhouette holes 3.9% to 2.7% (take1) and 4.5% to 2.9%
  (take3), edge flicker 11.5% to 1.1% and 9.2% to 0.85%, the stream 74.8 to
  15.0 and 64.9 to 11.3 Mbps; 1.8-3.0k rays a second reset over four
  cameras, 0.007-0.010% of valid samples.
- A box that enters, stays 10 s and leaves (take3): whole in the set it
  arrives, and nothing of it left the set it goes (main: a 169 cm² ring at
  its foot, 9 sets to clear).
- A sphere at 0.1 m/s (take1): 38 cm² of ghost stays (main 68). At 1 m/s:
  46.5 cm² while it moves, 50.2 the set after it leaves, 46.4 for good
  (main 77.9 / 81.3 / 84.8); shown while moving 98.6% (main 90.5%).
- A walking column that vanishes mid-walk (take3): 0.48 m² stays along its
  last pose and in slivers at its first (main 2.16 m²; 1.27 m² before the
  `Vanished` reset).
- `Vanished` on the second invalid sample in a row (the entry above), at the
  final code against the build before it: the vanishing column leaves
  0.474 m² the set after it goes (0.468), a sheet at its last pose that the
  next set clears, then 0.467 (0.467) and 0.465 (0.466) two and five sets
  on; while it walks, 0.455 m² (0.444). The 1 m/s sphere and the box are
  unchanged (46.6 / 50.2 / 46.4 cm²; nothing left, cleared in one set).
  Rays reset outside the movers rise 2-4% (55-102 a second).
- A column that walks in and stands still, its pixels dropping out i.i.d. at
  15% (take3): while it stands, flagged 8.1k times a second (41k when a lone
  dropout ended a history; 14 without dropouts), and 97.4% of its surface
  shown at 4.3 mm rms (97.0% and 4.4 mm before; main with the same dropouts
  97.5% and 4.8 mm; without them 97.5% and 4.1 mm). The lit crops show a
  few scattered pinholes where there were many; main's sit at the rim. It
  leaves a little more of its walk-in behind than before: 0.66 m² of ghost
  while it stands (0.60; main 2.93), level throughout, and background missing
  near its path 64 cm² on average (47.5; main 28), nearly all during the
  walk-in, where it comes back a few sets later. On the walking column
  without dropouts the missing background is unchanged (80 cm²); the cause
  is not traced.
- A moving surface shows 99% of its visible area while it moves (main 90%).

**Cost.** Release, take1 replay (four Femto cameras, 1 cm), sets 40-119,
interleaved with main: integrate 1.62-1.63 ms a set on the RTX 4090 against
main's 1.54-1.55 (the change tests 0.07 ms, the resets 0.63 ms in place of
the former clears), and 4.62-4.63 ms on the M5 Max against 4.58-4.70 (0.11
and 1.46 ms; testing a voxel's weight before projecting it, as the former
clear did, took the resets from 2.02). The dropout mark costs nothing
measurable: the change tests 0.073 ms against 0.072, integrate 1.688-1.692
against 1.686-1.689 ms (RTX 4090, interleaved with the build before it).
Memory: the detector's 35.4 MB for the rig. Classic is unchanged (40 take1
sets: the codec stream byte-identical, the set-39 mesh the same
triangles).

**Tests.** `recon_tsdf_dynamic_change` fuses two cameras that disagree by
8 mm on a wall through the `Fuser`: no voxel's weight falls while the
scene is still or one camera misses a set; a card 2.5 cm in front of the
wall, inside its band, and a box 0.5 m in front, leave no triangle in the
set they leave, and the wall meshes there at once; the box meshes in the set
it arrives and the wall it hides keeps every voxel's weight.
A second scene puts the box where nothing returns behind most of it, the
wall only on the left: the part a camera sees the wall past goes in the set
the box goes, flagged `Farther`; where nothing returns behind it, that set
changes nothing, as a single-frame dropout there does not, and in the next
nothing of the box meshes, by its cameras' `Vanished` rays alone; no weight
falls in any other set.
`recon_tsdf_integrate_dynamic_set` gives the grazing camera a colour of its
own, which the frontal camera's band must not take, and requires that band
to keep at least the weight the frontal camera alone gives it (the former
clear left one sample's).
`recon_tsdf_integrate_set` now compares a Dynamic set with its reverse bit
for bit. Run on the M5 Max (MoltenVK) and the RTX 4090, with validation and
synchronization validation.

**Open.** A surface that leaves to no return flags nothing where its pixel
returned anything in its camera's warm-up (a surface there from the start,
or a background's sporadic return) or where it returned for under three
samples (`TODO(tsdf)`); partial evidence lowering the weight cap so slow
motion slides; the noise model's edge and incidence terms; and two dropouts
in a row on a still surface over no return, which still flag and reset
(the standing column's 8.1k a second; a real sensor's dropouts cluster on
hair and silhouettes, so there they come in a row more often than
i.i.d.).

### 2026-10-29 — Near an outline a voxel takes the true distance, in both modes; under Dynamic not where its camera saw a change, and a departure takes what it left.

**The rule.** A voxel whose depth sample is not a clean bilinear
interpolation inside the band -- a jump or a gap across its taps, past the
band, or no return -- and whose footprint window (+-1 to +-3 pixels, half a
voxel's footprint) holds a return and a gap or a depth range past the band
takes the Euclidean distance to the nearest back-projected return within
+-2 to +-6 pixels (the footprint + 1), kept only under `trunc_dist`. Its
sign is its own pixel's: negative behind that pixel's surface (faded as a
band sample), positive in front of it, beside it, or over a gap (a quarter
of a sample's weight). A voxel more than a voxel behind its nearest return,
beside it, is in the shadow behind the outline and keeps the projective
sample. A positive one brings no colour. A pass a frame over its pixels
finds where its outlines are and the depth range of the returns around each
pixel, so a voxel tests its window with one read and skips a search that
would find nothing. Under Dynamic, two more rules:
1. **No rim distance within the reach of a change.** Each camera's change
   test also marks its reach: every pixel within 6 -- the widest search --
   of a pixel it flagged this set, with a bit near a departure (`Farther`
   or `Vanished`). A voxel there is fused from its projective sample
   alone.
2. **A departure takes what it left beside it.** Within the reach of a
   departure, the reset also clears a positive voxel whose own ray sees
   past its band or into a gap and whose nearest return is gone from the
   band or is a pixel that changed, whoever fused it: a rim distance, or
   the front of another camera's band, which that camera fuses again the
   same set at one sample's weight. A gap counts at once there: within the
   reach of a departure, pixels that return nothing around a voxel are not
   taken for a dropout, as a lone invalid sample is elsewhere (the detector
   flags `Vanished` on the second). An arrival clears nothing.

**Why.** Beside a silhouette the projective distance is the background's,
so marching cubes steps the outline a cell at a time and leaves thin parts
open: on take1 3.9% of the pixels at depth edges show no surface, and on
the ground-truth bench today's Dynamic rims lie 2.6 mm inside the true
silhouette with a 4.8 mm sawtooth. An earlier study's true rim distance
was its best still-scene fix (holes 3.9% to 0.65%), but on its own, under
the former Dynamic, it shredded a sphere moving at 1 m/s and left more
debris (131 against 80 cm²): next to a moving outline, distances to where
the surface was or is going averaged into its fresh samples.
- **Rule 1** keeps a moving surface's own samples whole. Without it, the
  test's plate moving 3 cm a set kept 42 voxels beside its moving sides
  holding distances in the set it moved.
- **Rule 2.** A beside voxel lies on a ray that does not hit the object, so
  the reset along changed rays (the entry above) never reaches it when
  the object leaves
  (the test's sphere: 779 weighted voxels off the wall in the set it
  left). Clearing on any change instead let an arriving box clear one
  camera's barely-past-band view of the other camera's wall band (653
  weights fell); departures only, with the nearest-return test, clear none
  there. A camera clears only beside a departure it saw itself, and the
  resets run before any fuse, so another camera's band fuses again in the
  same set, though at one sample's weight where it had up to the cap: on
  rays its camera did not flag, so whether that keeps "cameras never evict
  each other" is open (below).
- **That rule otherwise,** with its fix for walls behind rims (the
  shadow rule) and its gap weight.

**Measured** on replays of the rig's recorded takes with synthetic movers
drawn in (lab RTX 4090), against main and against the entry above alone
("change-driven"):
- **Still scenes** (take1 / take3): silhouette holes 3.9 / 4.5% on main,
  2.7 / 2.9% change-driven, 0.80 / 0.26% now; edge flicker 11.5 / 9.2, 1.1 / 0.85,
  2.2 / 1.2%; the stream 74.8 / 64.9, 15.0 / 11.3, 24.7 / 18.1 Mbps.
  Classic, take1: holes 0.96% (2.6% before), edge flicker 1.0% (0.83%),
  20.2 Mbps (17.0).
- **Against the ground-truth bench** (realistic noise, against main): Dynamic
  rims lie 1.2 mm inside the true silhouette (2.6 before), the surface
  within 2 cm of a rim is 97.4% covered (92.5%), and 0.77% of triangles
  are flipped (1.09%); the sawtooth stays 4.6 mm (4.8). Classic: a 2.3 mm
  sawtooth (3.0) and 2.3 cm² of fattening (13.7), but rims 1.5 mm inside
  (0.7) and 19.6 cm² of sheets more than 1 cm inside the objects (0.5).
- **Movers.** What a mover leaves where nothing is, the set after it goes
  (main / change-driven / now): the 1 m/s sphere 81 / 50 / 4.4 cm², gone in one set
  now and never before; the 0.1 m/s sphere 81 / 44 / 0.6 cm²; the walking
  column 2.23 / 0.47 / 0.05 m²; the box 169 / 0 / 0 cm². The moving surface
  shows 99% of its visible area while it moves, as before. Replayed again
  with the detector's second-invalid rule (the entries above), the 1 m/s
  sphere gives the same: 4.5 cm² left the set after it goes, gone in one
  set, 99% shown while it moves; the still take1 0.80% holes, 2.3% edge
  flicker, 25.1 Mbps.
- **What it costs movers:** background beside a mover's path goes missing
  while it moves. Each run above is scored against its own still run, and
  against this mode's that counts the outline surface rule 1 withholds near
  a change (means 25, 104 and 362 cm² for the fast sphere, the slow one and
  the column). The same meshes scored against the change-driven still run
  give means of 10, 44 and 182 cm² (the fast sphere 11 with the
  second-invalid rule), of which the two still runs' own difference there
  is 6, 6 and 108; change-driven, 5, 14 and 80. That is
  about change-driven's loss for the fast sphere and the column and about
  two and a half times it for the slow one, with higher peaks (75, 86 and
  951 cm² against 25, 34 and 644); the cause is not isolated, as no rule
  was taken out on the replays. By the end of each run no more is missing
  than the still runs' difference, within 1 cm². Against its own still run
  19-29 cm² reads as missing at the end; for the box that is surface this
  mode's still run holds in the footprint (25-29 cm², more than 2 cm from
  the change-driven still surface), which the box run does not grow back
  once the box leaves, its cause not isolated either. Edge flicker at the
  still sets after a mover is 2.7-4.3%, against 1.2-2.2% on the still run.

**Cost.** Release, take1 replay (four Femto cameras, 1 cm), sets 40-119,
interleaved: integrate a set takes 2.79 ms on the RTX 4090 against 1.63
change-driven (Classic 1.93 against 0.98): about 0.3 ms the rim map and change reach
passes, the rest the nearest-return search beside outlines. The first
cut, which scanned each voxel's window and searched wherever a window held
an outline, took 3.81 ms; reading the rim map and skipping searches that
find nothing changes no result (take1, 120 sets, the same triangles in
both modes). Memory: 23.6 MB more for the rig, a rim map, return range and
change reach a camera. The M5 Max is not measured yet.

**Tests.** `recon_tsdf_rim_distance` fuses noise-free ray-cast depth from a
Femto-like camera (640x576, fx 504) into 1 cm voxels through the `Fuser`.
In both modes, a plate turned 20 degrees in its plane meshes its outline
within 2 mm of the true edge on average and 6 mm anywhere (projective:
2.9 / 8.1 mm Classic, 6.5 / 12.8 mm Dynamic), a voxel beside its edge
holds its Euclidean distance to within the pixel pitch, and a rod 1.6 cm
across meshes along its whole length. Under Dynamic, a plate moving 3 cm a
set leaves no weight beside its moving sides in a set it moves, and takes
its distances in the first set it is still; a sphere at 1 m/s past a wall,
seen by three cameras, is 97% covered in every set it moves, 99.5% once it
stops, and leaves nothing off the wall's band in the set it goes. Each rule
was taken out in turn, and its check failed. In the change-driven test's
box over no return, the set the box goes clears it also within 6 pixels of
where a camera sees the wall past it (to 4 cm past where nothing returns
behind it), and the rest the next set, as before. Run on the RTX 4090 and
the M4 Mac mini with validation (the full suite) and synchronization
validation (the tsdf and sender tests); the rim test prints the same
figures on both, on the M5 Max (at the first cut), and before and after
the rim map.

**Open.** Rule 2 clears the front of another camera's band beside a
departure, on rays that camera did not flag, and that camera fuses it again
at one sample's weight: whether that keeps cameras from evicting each other
is open. Narrowing rule 2 to voxels a rim distance wrote, at a bit a voxel,
would keep that band, but must keep the movers' figures above:
change-driven alone, with no rim distance, leaves 46-50 cm² behind the fast
sphere, and part of what clears it here may be rule 2, which a narrowed rule
would lose. The extra background
missing beside a slow mover (above). Classic's sheets inside objects. The
M5 Max's cost, and a nearest-return search that stops early (PERF.md, P11).
The rim test's footprint and search are in pixels of one voxel
size; a finer grid changes them with it, unmeasured. A static outline
within 6 pixels of a change takes no rim distance that set, so it shows
its projective steps while something moves beside it.

### 2026-10-29 — fuse_realsense is an example of its own on fuse_orbbec's run, and a core profile of its own.

**Context.** sensor 0.4.0 brought `RealSenseSensor`, a RealSense D400
camera's own depth (its projector on) and its colour camera's YUYV on the
host, as an `IRgbdSensor`; the owner's plan (the RealSense and stereo-depth
design, 2026-10-29) fuses it live with `fuse_orbbec`'s loop, and later
depth from its infrared pair by stereo matching, as one more depth source.

**Decision.**
- One example per source, so `fuse_realsense`, not a `fuse_orbbec` option.
  Its run is `fuse_orbbec`'s, moved whole into
  `examples/common/live_fusion.hpp` (`run_live_fusion`), which takes the
  program's camera opener and each camera's counters: one live loop, not a
  copy. The array report and clock that loop needs are `live_sets.hpp`'s,
  on the array alone, so `rig_viewer` keeps them too.
- The camera is a `SensorArray` of one, which the array accepts with no
  sync role or rig clock, so prep, fusion, the per-set table and the
  counters are the rig's, and a set is the camera's frame.
- Its scenario is core's `FuseRealsense` profile (core 0.31.0), since every
  desktop program loads a core profile and takes only `--config` and
  `--help` (core's 2026-10-11 entry): the camera's serial, modes and depth
  gate, `fuse_orbbec`'s fusion keys, the PLY and the run length. No codec,
  outputs beyond the PLY, stream operation or diagnostics until a use needs
  them: the readback reads no host picture, and the dump writes JPEGs.
- Colour at auto exposure, with no key for a manual one: with a manual
  exposure the D455's colour drifts from its depth by up to half a frame
  (16.7 ms at 30 fps; sensor #6's review), which misplaces colour on a
  moving subject.
- The opener is where a depth source is chosen. Depth from the infrared
  pair (`StereoDepthSensor` over the camera's stereo source) joins there as
  another `IRgbdSensor`, with a scenario key to choose it, the loop
  unchanged.

**Validation.** On the lab RTX 4090 (Ubuntu 26.04, Release, the Orbbec,
RealSense and video drivers): the full suite, 44 tests, under the
validation layer, and core's 528 with the profile and its consumer check.
Three planted bugs were killed: the counters' order, a clock check that
ignores a camera's clock, and the options' colour width. Checked by hand,
not a test:
- `fuse_realsense` on the D455 (1 cm voxels): 900 frames in 31.7 s, 30 fps
  after the camera's first 1.6 s; 37 start-up framesets refused, none
  dropped. At 30 fps the GPU is about 20% busy and idles in P8, its clocks
  sampled every 100 ms at 450 MHz SM and 405 MHz memory against a boost of
  2595 and 10501 MHz. A frame's 1.20 ms of prep and 5.05 ms of integration
  on the device are therefore its cost at idle clocks: not a budget, and not
  comparable with the rig's below. On the host it costs 0.22 ms, and 7.2 ms
  from arrival to published, the latency at 30 fps as it runs; the process
  peaked at 895 MiB of GPU memory.
- `fuse_orbbec` on the four-Femto rig, 25 sets, twice each against main:
  prep 0.42 against 0.40 ms and integration 1.33 against 1.34 ms on the
  device (the means of both runs), the same rate, and meshes within 0.2% in
  the second pair. 25 sets stay inside the change detector's 30-frame
  warm-up, so Dynamic's test and reset ran only on the D455. In the first
  pair one camera's H.265 joined a second before its first key frame (24
  lost), which left the branch's mesh 11% smaller; the driver is unchanged
  by the re-pin.

### 2026-10-29 — fuse_realsense takes FFS depth from the camera's infrared pair, through the same opener and loop, only where sensor builds sensor_ffs.

**Context.** The owner keeps both RealSense depths: the camera's own with
the projector on, and Fast-FoundationStereo (FFS) on its infrared pair with
it off, FFS on NVIDIA only. sensor 0.8.0 brought `RealSenseStereo` and
`sensor_ffs`'s `StereoDepthSensor`, which rectifies any calibrated pair and
runs FFS on a thread of its own as an `IRgbdSensor`; core 0.32.0 the
`FuseRealsense` keys that choose it.

**Decision.**
- One program, `fuse_realsense`, its depth the scenario's `source.depth`,
  chosen in its opener alone (the entry above): the camera's own is a
  `RealSenseSensor`, FFS a `StereoDepthSensor` over a `RealSenseStereo`.
  Either is a `SensorArray` of one, so prep, fusion, the per-set table and
  the counters are the same; FFS's depth stays on the device, where the prep
  reads it in place.
- The pair is rectified to the depth size as luma at alpha 0: luma carries
  no coverage, so at alpha 0 no border is matched as picture. The infrared
  cameras run at the driver's own mode (1280x720), which the rectifier
  resamples, and FFS's range is fusion's depth gate.
- FFS builds where sensor builds `sensor_ffs` (`VS_WITH_FFS`: CUDA and
  TensorRT), and `VR_EXAMPLE_FFS` turns it on in the example. A build
  without it refuses an FFS scenario as a scenario is refused (exit 2),
  saying how to build it, before any GPU or camera opens; the Mac never
  builds it.
- CI's CUDA leg (24.04) installs TensorRT 11.4.0.106, every package pinned
  as sensor's CI pins it, and builds `VS_WITH_FFS`; the other legs check the
  refusal. The plan cache and the model are the scenario's, with no
  default: a model is exported for one size, and a plan takes minutes to
  build.

**Validation.** On the lab RTX 4090 (Ubuntu 26.04, Release, warnings as
errors, the Orbbec, RealSense and video drivers and `VS_WITH_FFS`, TensorRT
11.4): the suite, 45 tests, under the validation layer and synchronization
validation; reconfigured without `VS_WITH_FFS`, the refusal's tests pass.
Eight planted bugs were killed: either projector dropped, the infrared
cameras at the depth size, `Rgba8` views, the colour size rectified, FFS's
near gate dropped, and FFS refused with `sensor_ffs` or let through without
it. Checked by hand, not a test, on the D455 (firmware 5.17.0.10,
USB 3.2), 900 frames at 1 cm voxels, the box to itself:

| | FFS, 960x544, projector off | the camera's own, 1280x720, projector on |
| --- | --- | --- |
| frames | 900 in 31.7 s; 30.0 fps after the first frame (1.7 s) | 900 in 31.6 s; 30 fps after the first (1.6 s) |
| camera | 940 received, 2 dropped, 38 failed at start-up | 937 received, 0 dropped, 37 failed at start-up |
| GPU state | P2, SM 2565 MHz median (2235–2760), memory 10251 MHz, PCIe 4.0 x16, 84% busy | P8, SM 540 MHz, memory 405 MHz, 26% busy (idle clocks) |
| FFS on the device (Nsight Systems, 450 frames) | 27.77 ms median, 27.93 p95: the network 27.76, its input and depth kernels 0.007 and 0.003 | - |
| prep / integration, device | 0.109 / 0.536 ms | 1.16 / 6.94 ms at idle clocks |
| arrival / capture to published, p50 | 29.8 / 45.6 ms | 9.4 / 25.3 ms |
| GPU memory, process peak | 2604 MiB | 1377 MiB |
| mesh | 251,974 triangles; walls flat | 632,966 triangles; walls rough |

- At 960x544 a frame's GPU work, FFS's 27.8 ms, the rectifier's 0.1 ms
  (sensor #8 and #10's measure, not repeated here), prep and integration,
  is about 28.5 ms of the camera's 33.3 ms, so 30 fps holds with fusion
  running, and FFS's time is the same as on an idle box (27.7 ms, sensor
  #10). FFS adds about 20 ms to the latency, its network's run.
- FFS keeps the GPU at its working clocks, which the camera's own depth
  never asks for: their device rows differ by the clocks, not the work.

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
