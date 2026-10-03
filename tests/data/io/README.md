# Mesh input regression fixtures

These small authored assets belong to the repository (MIT). They exercise
`io::load_mesh` without external datasets or a Vulkan device. All glTF buffers
are embedded as base64 data URIs; no downloaded or generated binary artifact
is required at test time.

- `triangle.{obj,ply,gltf}` describe the same oriented triangle at `(1,2,3)`,
  `(5,2,3)`, `(1,8,3)`. PLY puts its properties in a different order and adds
  colour. OBJ/PLY declare no physical unit; loading preserves coordinates.
- `quad.ply` is a +Z-facing 4-by-6 rectangle at z=30, supplied as one quad.
- `seams.obj` is an outward 2-by-3-by-5 box with normal/UV seams, three
  materials and one relative-index face. Its texture path intentionally does
  not exist: only geometry is requested. After exact position joining it has
  8 vertices, 12 triangles, 18 indexed edges and signed volume 30.
- `near_positions.obj` has two triangles sharing an exact origin (with signed
  zeros), but their other corners differ by 1e-6. It must retain 5 positions.
- `unrepaired.obj` retains duplicate and collinear triangles for the caller's
  topology audit; loading must not silently repair or delete geometry.
- `instances.gltf` references one triangle mesh three times under a parent
  Z rotation/translation, child translations and nonuniform scales. One
  instance has a negative determinant; another has two nested reflections.
  The resulting oriented triangles are listed independently in the test.
- `animated.gltf`, `morph.gltf` and `skinned.gltf` use valid minimal animation,
  morph-target and skin structures. Static-only loading must report
  `Unsupported`, rather than silently returning a different pose.
- `hidden_singular_node.gltf` is `triangle.gltf` plus a mesh-less node scaled
  to zero. Only transforms that place geometry are validated, so it loads.
- `triangle_zup_cm.dae` is the same triangle in a Z-up, centimetre Collada
  file. The importer's axis and unit conversions stay off (unit: Assimp 5.3+).
- `singular_transform.gltf` uses zero Y scale; `projective_transform.gltf`
  has a non-affine matrix. `overflow_transform.gltf` combines finite local
  translations that exceed the output float range. All must be refused.
- `nonfinite.gltf` has +infinity in the first position's X component. The
  buffer contains the IEEE-754 little-endian bytes, not an invalid JSON token.
- The remaining named assets exercise empty input, points/lines only,
  malformed syntax and indices outside the three-position array.
