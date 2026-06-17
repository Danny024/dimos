# Scene packages

A **scene package** is a self-contained directory holding everything a
simulator needs to load one 3D scene: visual meshes, collision geometry, a
per-object table, a scene-only MuJoCo file, and a list of dynamic entities.

The pipeline is **one source asset → one package → loaded by any sim, with any
robot.** The robot is never part of the package — the sim attaches it at load
time. The package is *cooked* once (offline, the expensive step) and *loaded*
cheaply at runtime.

## Lifecycle

```
1. Author    drop <scene>.cook.json next to the source mesh
2. Cook      python -m dimos.experimental.pimsim.scene.cook <source>   →  package dir
3. Compose   at runtime the sim loads the scene file, attaches the robot,
             and adds the entities as MuJoCo bodies
```

## Package layout

```text
data/scene_packages/<name>/
├── scene.meta.json          manifest: alignment, artifact paths, entities, stats
├── mujoco/<hash>/
│   ├── wrapper.xml          scene-only MJCF (no robot)
│   └── *.obj                static collision (primitives / convex hulls)
├── entities/<id>/
│   ├── visual.glb           per-entity visual, in the entity's local frame
│   └── mujoco_collision/    CoACD convex hulls (hull_000.obj, …)
└── browser/                 for the pimsim / Havok browser sim + lidar raycasts
    ├── visual.glb
    ├── collision.glb
    └── objects.json         per-prim semantic table (id, path, AABB)
```

Packages are **content-hash keyed** on (source mesh + alignment + sidecar +
schema version): change any input and the cooker writes a fresh package;
otherwise it reuses the cooked one.

## Authoring — the cook sidecar

`<scene>.cook.json`, next to the source asset, says how to cook it. Without one
you still get a valid package (auto-fit static collision, no entities).

```json
{
  "collision": {
    "default": "auto",
    "prim_overrides": {
      "Floor":    {"type": "plane"},
      "Wall_*":   {"type": "box"},
      "Stairs_*": {"type": "decompose", "max_hulls": 16}
    }
  },
  "interactables": [
    {"id": "chair_000", "source_prim_paths": ["Chair_000_*"], "mass": 8.0,
     "physics": {"shape": "mesh"}, "tags": ["chair"]}
  ]
}
```

**Static collision** — `prim_overrides` maps fnmatch patterns to a collision
type: `auto | box | sphere | cylinder | capsule | plane | hull | mesh |
decompose | skip`. Full policy in `simulation/mujoco/collision_spec.py`.

**Entities** — each `interactables[]` entry becomes a MuJoCo body `entity:<id>`.
There are three sources of entities:

- **Extracted** — modelled inside the source mesh. `source_prim_paths` (fnmatch)
  splits the prims out → per-entity GLB + CoACD hulls. *Requires Blender on PATH*
  (the only step that does).
- **Synthetic** — a primitive prop not in the source mesh: give it a `pose` and
  `physics.shape` / `extents`.
- **Scanned** — SAM3D photo→mesh props, injected into the package directly.
  Not in the sidecar, so `--rebake` drops them — re-run the injection after.

| Field | Notes |
|---|---|
| `id` | required; becomes the `entity:<id>` body name |
| `source_prim_paths` | fnmatch globs (extracted) — *or* `pose` (synthetic) |
| `kind` | `dynamic` (freejoint+mass) \| `kinematic` (RPC-driven) \| `static` (welded). Default `dynamic` |
| `mass` | kg; 0 forces kinematic. Default 1.0 |
| `physics.shape` | `mesh` \| `box` \| `sphere` \| `cylinder`. Default `box` |
| `physics.extents` | half-extents (box), `[r]` (sphere), `[r, half_h]` (cylinder) |
| `physics.friction` | scalar or `[slide, torsional, rolling]`. Default `[0.3, 0.05, 0.001]` |
| `visual.rgba` | `[r, g, b, a]` 0–1 |
| `spawn` | `initial` (at boot) \| `manual` (RPC only). Default `initial` |
| `tags` | free-form semantic labels |

## Cooking

```bash
python -m dimos.experimental.pimsim.scene.cook <source.glb> \
    --output-dir=data/scene_packages/<name>
```

The sidecar is auto-discovered next to the source (`--cook-spec` overrides).
Useful flags: `--rebake`, alignment (`--scale` / `--translation` /
`--rotation-zyx-deg` / `--no-y-up`), `--visual-optimizer {gltfpack,blender,copy}`
(gltfpack default; Blender only as a fallback for formats it can't read),
`--no-browser-collision`, `--no-mujoco`. The cooker is robot-agnostic — no robot
flag, ever.

## Loading into MuJoCo

`MujocoSimModule._compose_model` builds one `MjModel` from the package:

1. load `mujoco/<hash>/wrapper.xml` into an `MjSpec`
2. load the robot MJCF and `spec_scene.attach(spec_robot, prefix=robot_id)` —
   robot **first**, so it keeps the leading freejoint / qpos block
3. `add_entities_to_spec(spec, pkg.entities)` — each `spawn=="initial"` entity
   becomes `entity:<id>`: `dynamic` + positive mass gets a freejoint, everything
   else is welded. Entity geoms carry `priority=1` (so their friction wins the
   contact pair) and sit in geom group 3 (so depth-lidar sees them)
4. `spec.compile()` → scene + robot + entities in one model
5. spawn audit: any entity booting >2 cm inside the static scene is re-welded
   static instead of being ejected

**Why bake at all?** MuJoCo collides a mesh as its *convex hull*, so a raw
concave scene mesh would be one useless blob. The bake turns each prim into
MuJoCo-digestible geometry — a primitive when it fits, a convex hull, or a CoACD
decomposition for concave parts. Entity meshes are decomposed the same way at
cook time (deterministic, no runtime/per-machine cache).

Blueprint wiring:

```python
from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule
from dimos.simulation.scenes.catalog import resolve_scene_package

pkg = resolve_scene_package("dimos-office")
MujocoSimModule.blueprint(
    scene_xml=str(pkg.mujoco_scene_path),  # cooked scene-only wrapper
    robot_mjcf="path/to/g1.xml",           # any robot MJCF
    robot_meshdir="path/to/g1/assets",
    robot_id="",                           # body-name prefix ("" for a single robot)
    scene_entities=pkg.entities,
    spawn_z=0.793,
)
```

The robot MJCF must contain **no scene geometry** (floors/walls/furniture are
the package's), **no manipulation rigs** (author props as interactables), and
**no `<include>`s outside its own directory**.

## Reference

| File | Role |
|---|---|
| `dimos/experimental/pimsim/scene/cook.py` | cook entry point + CLI |
| `dimos/experimental/pimsim/scene/sidecar.py` | `<scene>.cook.json` schema |
| `dimos/experimental/pimsim/scene/plan.py` | sidecar → resolved cook plan (static vs entities) |
| `dimos/experimental/pimsim/scene/visual_glb.py` | static visual GLB (gltfpack) |
| `dimos/experimental/pimsim/scene/visual_blender.py` | per-entity visual GLBs (Blender) |
| `dimos/experimental/pimsim/scene/browser_collision.py` | fused collision GLB + `objects.json` |
| `dimos/experimental/pimsim/scene/entity_collision.py` | cook-time CoACD hulls per entity |
| `dimos/experimental/pimsim/scene/inspect.py` | source-asset stats |
| `dimos/simulation/scene_assets/spec.py` | `ScenePackage` + on-disk JSON shape |
| `dimos/simulation/scene_assets/mesh_scene.py` | glTF / USD → world-frame meshes |
| `dimos/simulation/mujoco/scene_mesh_to_mjcf.py` | the MuJoCo bake (mesh → MJCF) |
| `dimos/simulation/mujoco/collision_spec.py` | static-collision policy |
| `dimos/simulation/mujoco/entity_scene.py` | runtime entity composition + spawn audit |
| `dimos/simulation/scenes/catalog.py` | `resolve_scene_package(name \| path)` |

## Known gaps

- Scanned (SAM3D) entities live only in the cooked package, so `--rebake` drops
  them (see above).
- Articulation (doors, drawers) has no schema yet — `joints[]` on the entity
  spec is the next extension.
