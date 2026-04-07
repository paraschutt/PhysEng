# Adaptive Physics Core (APC)

**v0.1.0** — A header-only, deterministic rigid-body and articulated-body physics engine for C++17.

APC provides a self-contained collision detection and constraint solving pipeline with strict IEEE 754 floating-point compliance, making it suitable for real-time simulation, rollback netcode, and reproducible tooling.

## Features

- **Deterministic math** — `Vec3`, `Quat`, `Mat3` with IEEE 754 strict compliance (no FMA, no SIMD float reordering)
- **Platform FP enforcement** — Runtime and compile-time controls for x64 and ARM64
- **Collision pipeline** — Sweep-and-Prune broadphase, OBB tree midphase, GJK/EPA narrowphase
- **6 shape types** — Sphere, Box, ConvexPiece, Plane, Capsule, Cylinder
- **36-pair dispatch table** — Dedicated fast-paths for every shape combination
- **Sequential Impulse solver** — Coulomb friction, restitution, Baumgarte stabilization
- **Contact persistence** — Warm-starting through `ContactManager`
- **Articulated dynamics** — Featherstone ABA with per-bone physics blend modes
- **Skeletal FK** — Forward kinematics for articulated bodies
- **Convex decomposition** — V-HACD `.apccol` binary asset loading
- **Zero dynamic allocation in hot path** — No external dependencies

## Requirements

| Requirement       | Version          |
|-------------------|------------------|
| CMake             | 3.20+            |
| C++ compiler      | GCC 12+ / MSVC 2022+ |
| Dependencies      | None             |

## Building

```bash
mkdir build && cd build
cmake .. -DCMAKE_CXX_FLAGS="-Werror -Wall -Wextra"
cmake --build .
ctest --output-on-failure
```

## Integration

APC is a header-only library. To integrate it into your project:

1. **Add the source directory to your include path.** This lets you include headers as `apc_math/apc_vec3.h`, `apc_collision/apc_gjk.h`, etc.

2. **Compile and link `apc_core`** (the small static library that ships with APC). It contains the `.cpp` files for math primitives and platform FP-mode enforcement — the only compiled units in the library.

3. **Add the build directory to your include path** so that the generated `apc_config.h` is found.

With CMake, this looks like:

```cmake
add_subdirectory(path/to/apc-core)

target_link_libraries(your_target PRIVATE apc_core)
```

The include directories are exported automatically through the `apc_core` target.

## Quick Example

```cpp
#include "apc_collision/apc_collision_dispatch.h"

// Create shapes
apc::CollisionShape sphere = apc::CollisionShape::make_sphere(1.0f, {0, 5, 0});
apc::CollisionShape ground = apc::CollisionShape::make_plane({0, 0, 0}, {0, 1, 0});

// Detect collision
apc::ContactManifold manifold;
bool hit = apc::dispatch_detect(sphere, ground, 0, 1, manifold);

if (hit) {
    // manifold contains contact points, normals, and penetration depths
}
```

For solving rigid-body dynamics, include `apc_solver/apc_si_solver_3d.h` and create `RigidBody` instances from `apc_solver/apc_rigid_body.h`.

## Project Structure

```
apc-core/
  src/
    apc_math/        Vec3, Quat, Mat3, math common
    apc_collision/   Broadphase (SAP), GJK, EPA, dispatch, shapes, OBB tree
    apc_solver/      Rigid body, Sequential Impulse solver, contact manager
    apc_skeleton/    Skeletal types, forward kinematics, Featherstone ABA
    apc_containers/  FlatMap, SlotMap
    apc_platform/    FP mode enforcement (x64, ARM64)
  tests/
    correctness/     Unit tests for each module
    integration/     Pipeline and solver integration tests
    determinism/     Cross-platform determinism verification
  tools/             V-HACD CLI, CI reporting utilities
  cmake/             Compiler flag and determinism CMake modules
  docs/              Documentation
  ARCHITECTURE.md    Technical architecture and design vision
```

## Testing

17 test targets cover three categories:

- **Correctness** — Per-module unit tests (math types, GJK/EPA, OBB tree, broadphase, dispatch, FK, blend modes)
- **Integration** — Full pipeline tests (sphere-box, friction, pendulum, 3D bounce, solver)
- **Determinism** — Hash-verified simulations across platforms to guarantee bit-exact reproducibility

Run all tests with `ctest --output-on-failure` from the build directory.

## Compiler Flags

APC relies on specific compiler flags to enforce deterministic floating-point behavior. These are applied automatically to the `apc_core` target via `cmake/APCDeterminism.cmake`. If consuming APC outside of CMake, ensure the following:

- **GCC:** `-ffp-contract=off -fno-fast-math -mfpmath=sse` (x64)
- **MSVC:** `/fp:strict`

Disabling FMA and strict float contraction is mandatory for cross-platform determinism.

## License

Proprietary — internal use only.

## Links

- **Repository:** [github.com/paraschutt/PhysEng](https://github.com/paraschutt/PhysEng) (branch: `Opencode`)
- **Architecture:** See [ARCHITECTURE.md](ARCHITECTURE.md) for technical design, data flow, and future roadmap.
