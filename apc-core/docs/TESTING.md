# Testing — Adaptive Physics Core (APC)

This document describes the testing strategy, conventions, and tooling for the APC deterministic physics engine.

---

## 1. Test Framework

APC uses **no external test framework**. There is no Google Test, Catch2, or similar dependency. Every test is a standalone C++ executable following a minimal pattern:

```cpp
#include <cassert>

int main() {
    assert(condition);
    // more assertions...
    return 0;
}
```

If any `assert()` fires, the process aborts with a non-zero exit code. CTest detects this as a failure. This approach keeps the build simple, avoids framework versioning issues, and is sufficient for a deterministic engine where floating-point results are either exact or checked against known tolerances.

Tests are registered with CTest via `add_test()` in `tests/CMakeLists.txt` and discovered automatically by `ctest`.

---

## 2. Test Categories

All source lives under `tests/` and is organized into three categories.

### 2.1 Correctness Tests (`tests/correctness/`)

Unit-level tests that verify individual subsystems in isolation.

| Test | What it covers |
|---|---|
| `test_vec3` | Arithmetic, dot/cross product, `equals_exact` (bitwise `memcmp`), `equals_approx` (epsilon), zero-vector edge cases |
| `test_quat` | Construction, `from_axis_angle`, `from_rotation_matrix`, quaternion multiply, identity |
| `test_mat3` | `from_quat`, transpose, inverse, `transform_vec`, identity matrix |
| `test_skeleton_fk` | Forward-kinematics chain: joint transforms propagate correctly through a skeleton hierarchy |
| `test_obb` | OBB construction, separating-axis theorem (SAT) overlap tests |
| `test_gjk_epa` | GJK boolean query + EPA penetration depth for sphere-sphere, sphere-box, box-box, and other convex pairs |
| `test_broadphase` | Sweep-and-prune (SAP) pair generation, filtering, pair normalization (lower-id-first ordering) |
| `test_dispatch` | Full collision dispatch matrix for all 14 shape-pair combinations — AABB computation, manifold generation, filtering |
| `test_sprint4` | Capsule and cylinder support functions, AABB, sphere-capsule / capsule-capsule collision, plane-box and plane-convex, contact manager lifecycle (40+ assertions) |

### 2.2 Integration Tests (`tests/integration/`)

End-to-end tests that exercise the full pipeline: broadphase → narrowphase → solver → integration.

| Test | What it covers |
|---|---|
| `test_pipeline_sphere_box` | Complete sphere-box pipeline with mock collision shapes through dispatch, solver, and integration |
| `test_3d_sphere_bounce` | 3D sequential-impulse solver: sphere bouncing on a ground plane, stability over many frames |
| `test_apc_pendulum` | Articulated body algorithm (ABA): single-link pendulum under gravity, energy conservation |
| `test_blend_modes` | Per-bone physics blend modes: `ANIM`, `PHYSICS`, and `BLENDED` — verifies skeletal pose mixing |
| `test_friction` | Coulomb friction model integrated with surface contact resolution |
| `test_solver_v2` | Six sub-tests: multi-iteration convergence, restitution stability, configurable damping, `prepare_manifold` with multiple contacts, full 2-second pipeline, and bit-exact determinism (run-sim-twice, compare positions) |

### 2.3 Determinism Tests (`tests/determinism/`)

Verifies that the engine produces identical results across runs, platforms, and compilers. These tests output a hash digest that is compared against a golden reference.

| Test | What it covers |
|---|---|
| `det_main` | Entry point aggregating `det_vec3_ops`, `det_quat_ops`, `det_mat3_ops` — enforces deterministic FP mode, runs math operations, outputs FNV-1a combined hash |
| `det_trajectory_sim` | 10,000-frame trajectory simulation (sphere bouncing with solver) — hashes the full position history and compares against a known digest |

Use `tools/hash_compare.py` to validate determinism output across platforms.

---

## 3. Running Tests

### Build all targets

```bash
cmake --build build
```

### Run the full suite

```bash
cd build && ctest --output-on-failure
```

### Run by category

CTest test names follow a convention (e.g., `Vec3Correctness`, `PipelineSphereBox`, `MathOpsDeterminism`). Filter with regex:

```bash
ctest -R Correctness   --output-on-failure
ctest -R Integration   --output-on-failure
ctest -R Determinism   --output-on-failure
```

### Run a single test executable directly

```bash
./build/test_vec3
./build/test_dispatch
./build/test_solver_v2
./build/det_main
```

Running directly is useful during development — you get immediate assertion failures without CTest overhead.

---

## 4. Build Configuration

| Setting | Value |
|---|---|
| Language standard | C++17 |
| Supported compilers | GCC 14.2+, Clang (x64/ARM64), MSVC 2022+ |
| Warning flags (GCC/Clang) | `-Wall -Wextra -Werror -g` |
| Warning flags (MSVC) | `/W4 /WX /Z7` |
| Determinism flags | `-ffp-contract=off -fno-fast-math -fno-unsafe-math-optimizations -fno-tree-vectorize -fpack-struct=8` (plus compiler-specific additions) |
| Preprocessor defines | `APC_DETERMINISM_ENABLED=1`, `APC_STRICT_FP=1` |
| Library linkage | `apc_core` linked as a **STATIC** library to every test executable |
| Total test targets | 17 (10 correctness + 6 integration + 2 determinism; `det_main` is multi-file) |

Warnings are treated as errors. All tests must compile cleanly under the strict flag set defined in `cmake/APCCompilerFlags.cmake`.

---

## 5. Adding a New Test

1. **Create the test file** in the appropriate directory:
   - `tests/correctness/test_myfeature.cpp` for unit-level checks
   - `tests/integration/test_myfeature.cpp` for pipeline tests

2. **Write the test** using `int main()` with `assert()` calls:

   ```cpp
   #include "apc_math/apc_vec3.h"
   #include <cassert>

   int main() {
       apc::Vec3 a(1.0f, 2.0f, 3.0f);
       assert(a.x == 1.0f);
       return 0;
   }
   ```

3. **Register the target** in `tests/CMakeLists.txt`:

   ```cmake
   add_executable(test_myfeature correctness/test_myfeature.cpp)
   target_link_libraries(test_myfeature PRIVATE apc_core)
   add_test(NAME MyFeatureCorrectness COMMAND test_myfeature)
   ```

4. **Build and run**:

   ```bash
   cmake --build build && cd build && ctest -R MyFeature --output-on-failure
   ```

For multi-file tests (like `det_main`), list all source files in `add_executable()`.

---

## 6. Best Practices

### Test edge cases

Every subsystem test should cover degenerate inputs that are likely to appear in production:

- **Zero vectors** — normalize, dot, cross with `Vec3(0,0,0)`
- **Degenerate quaternions** — zero rotation, identity, near-zero axis
- **Zero-radius shapes** — sphere with radius 0, capsule with zero extent
- **Deep penetration** — shapes overlapping significantly, not just touching
- **Bitwise signed zero** — `+0.0f` vs `-0.0f` (APC distinguishes these in `equals_exact`)

### Test determinism

For integration and solver tests, run the same simulation twice within a single process and compare results with `memcmp` or an epsilon check. Tests like `test_solver_v2` (Test 6) and `det_trajectory_sim` demonstrate this pattern. Always call `apc::enforce_deterministic_fp_mode()` before any math in determinism tests.

### Verify normal conventions

APC collision normals point from body B to body A (`B→A`). Collision tests should assert the direction explicitly to catch regressions.

### Cover boundary values

- Distances at exactly `APC_EPSILON` from contact
- Maximum solver iterations (does it converge, not loop forever?)
- Minimum/maximum shape extents
- Identity transforms (no rotation, zero translation)

### Independence

Each test executable must be fully self-contained. No shared state, no global singletons carrying over between tests. This ensures tests can run in any order and in parallel (`ctest -j`).
