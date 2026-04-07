# Determinism Guarantees

> Determinism is a non-negotiable core requirement of the Adaptive Physics Core (APC).
> Every design decision in this engine is made with reproducibility as the first-class constraint.

---

## 1. Determinism Guarantee

APC guarantees **bit-exact frame reproduction** across runs on the same platform, with the same compiler, the same optimization flags, and the same inputs. Given an identical sequence of inputs (forces, collisions, parameters), every frame of every simulation step produces an identical result down to the last bit of every float.

This guarantee is the foundation that enables:

- **Rollback netcode** — resimulate past frames with corrected inputs and expect identical divergence points.
- **Frame-perfect replay** — record inputs only; reconstruct simulation state exactly from any saved frame.
- **Consistent AI training** — reward signals do not vary between training runs caused by nondeterministic physics jitter.
- **Cross-machine debug reproducibility** — a crash trace captured on one developer's machine can be reproduced on another, provided the target platform and compiler match.

If you are reading this and thinking "close enough is fine," this is not the engine for you. Bit-exact is the requirement, and every subsystem is built to enforce it.

---

## 2. Floating-Point Enforcement

Floating-point arithmetic is the single largest threat to deterministic simulation. APC eliminates every known source of nondeterminism at the hardware and source level.

### No FMA (Fused Multiply-Add)

FMA instructions collapse `a * b + c` into a single hardware operation with a single rounding step. While this improves accuracy in isolation, it **breaks associativity**: the result differs from a discrete multiply followed by a separate add, and availability varies across CPU generations and compilers. APC disables FMA entirely via compiler flags (`-ffp-contract=off` on GCC/Clang, `/fp:precise` on MSVC) and runtime FP mode enforcement. No code path in APC relies on FMA for correctness.

### No SIMD Float Reordering

SSE, AVX, and NEON instruction sets permit the compiler to reorder floating-point operations within SIMD vectors for throughput. APC restricts SIMD usage to integer and bitwise operations only. Floating-point math is always performed scalar with explicit evaluation order.

### Explicit Evaluation Order

All vector math uses strictly ordered operations:

- **Dot products**: `v.x * v.x + v.y * v.y + v.z * v.z` — always `xx + yy + zz`, never shuffled.
- **Cross products**: explicit element access (`a.y*b.z - a.z*b.y`, etc.), never pointer-based swizzling that could be reordered by the optimizer.
- **Matrix-vector multiply**: manual `Mat3*Vec3` implementation on hot paths, not operator overloading that might invite the compiler to rearrange terms.

### Platform-Consistent Math Functions

APC uses `std::sqrt` and `std::fabs` from `<cmath>`, which are mandated by the C++ standard to produce platform-consistent results for IEEE 754 arithmetic. Approximate reciprocal-square-root tricks (`rsqrt`, `_mm_rsqrt_ps`) are **prohibited** regardless of performance gains.

---

## 3. Platform FP Mode (`apc_platform/`)

APC provides a small platform abstraction layer in `apc_platform/` that establishes and guards the floating-point environment at runtime.

### `enforce_deterministic_fp_mode()` — call once at startup, before any simulation code

- **x64**: Sets **FTZ** (Flush-To-Zero) and **DAZ** (Denormals-Are-Zero) bits in the MXCSR register. This forces denormalized numbers to zero, eliminating the catastrophic performance penalties and platform-specific behavior of denormal arithmetic.
- **ARM64**: Sets the **FZ** (Flush-to-Zero) bit in the FPCR register, achieving the same guarantee on AArch64 targets.
- **Fail-fast**: If the FP mode cannot be set (e.g., the OS denies MXCSR writes), the function **crashes immediately**. A silent failure would be worse than a crash — it would mean the simulation silently diverges with no indication.

### `verify_fp_mode()` — debug check

Call this periodically (e.g., at the start of each frame in debug builds) to assert that the FP state has not been corrupted by third-party libraries, plugin code, or DLL boundary crossings that may alter the rounding mode or denormal handling.

### `restore_fp_mode()` — save/restore across boundaries

Some third-party libraries (audio middleware, physics plugins) change the FP rounding mode or MXCSR state. Use `restore_fp_mode()` with the saved state to restore APC's deterministic environment after returning from such code.

### `query_fp_capabilities()` — informational

Returns a struct describing what the current hardware supports: FTZ/DAZ availability, FMA presence, SIMD width, and whether the FP mode is currently in a known-deterministic state. Useful for logging at startup and diagnosing CI failures.

### Platform Defines (`apc_fp_mode.h`)

| Define | Meaning |
|---|---|
| `APC_PLATFORM_X64` | x86-64 target, MXCSR-based FP control |
| `APC_PLATFORM_ARM64` | AArch64 target, FPCR-based FP control |
| `APC_PLATFORM_CONSOLE` | Fixed-hardware console target with known FP behavior |

---

## 4. Sorted Deterministic Iteration

Nondeterminism hides in iteration order as often as it hides in floating-point arithmetic. APC enforces deterministic ordering on every collection:

| Subsystem | Ordering Rule |
|---|---|
| **Broadphase collision pairs** | `id_a < id_b` enforced lexicographically before any pair is processed |
| **SlotMap** | Insertion-order iteration; never pointer-chase order |
| **FlatMap** | Always sorted by key; insertion order is irrelevant |
| **EPA face selection** | First-index-wins on tied scores; never "best quality" heuristic |
| **GJK/EPA** | Index-order tie-breaking at every decision point in the expanding simplex |

The rule is simple: if two elements have equal priority for processing, the one with the lower index wins. This extends to the contact manifold ordering, constraint solver iteration, and island processing. There are no exceptions.

---

## 5. What Could Break Determinism

This is a non-exhaustive list of things that will silently destroy reproducibility. Treat each as a hard rule.

**FMA instructions** — whether emitted by the compiler (`-ffp-contract=fast`), enabled by a pragma, or present in hand-written intrinsics. One FMA in a hot loop and every subsequent frame diverges.

**`std::unordered_map`** — C++11 and later mandate hash-salt randomization for security. The iteration order of `unordered_map` is nondeterministic across runs. **Never use it.** Use `FlatMap` or a sorted `std::vector` of key-value pairs.

**Thread scheduling order** — if the solver or broadphase is parallelized without deterministic work assignment, different runs produce different accumulation order. APC currently runs single-threaded for this reason. Any future parallelism must use deterministic work-stealing or static partitioning.

**Compiler version or optimization level differences** — `-O2` and `-O3` may generate different instruction sequences. Debug and release builds will not produce identical results. All determinism guarantees apply per-build-configuration.

**Cross-architecture differences** — x64 and ARM64 have different floating-point semantics at the hardware level. Results will not match across architectures. If cross-platform determinism is required, use fixed-point arithmetic or software floating-point (not currently supported).

**Denormalized numbers** — handled by flushing to zero (see Section 3). If FP mode enforcement is bypassed, denormals introduce slow, platform-specific accumulation errors.

**Third-party libraries** — any code that calls `_controlfp`, `__builtin_set_rounding_mode`, or directly writes MXCSR/FPCR will corrupt APC's FP state. Always save and restore.

---

## 6. Testing for Determinism

APC includes a dedicated determinism test suite. Every test runs the same computation twice and compares the results via bitwise hash.

| Test | Description |
|---|---|
| `det_vec3_ops` | 10,000 vector add, subtract, scale, dot, and cross operations; hash compared across two runs |
| `det_quat_ops` | 10,000 quaternion multiply, normalize, slerp, and rotate operations; hash compared |
| `det_mat3_ops` | 10,000 matrix multiply, inverse, and transform operations; hash compared |
| `det_trajectory_sim` | Full sphere simulation (gravity + ground collision) over 10,000 frames; final position and velocity hash compared |

Run the full suite:

```bash
ctest -R Determinism --output-on-failure
```

CI integration tools:

- **`ci_report.py`** — Parses CTest output and generates a determinism pass/fail report for CI dashboards.
- **`hash_compare.py`** — Takes two hash dump files and produces a detailed byte-level diff report if any divergence is detected.

These tests run on every commit in CI. A single failure blocks the merge.

---

## 7. Guidelines for Maintaining Determinism

These rules are mandatory for anyone contributing to APC. There are no exceptions and no "just this once."

1. **Always use `Vec3::add`, `sub`, `scale`, `dot`, `cross`** on hot paths. Never use `operator+`, `operator*`, or other overloads that might invite the compiler to reorder terms.

2. **Never use `std::unordered_map`.** Use `FlatMap` or a sorted `std::vector`.

3. **Never enable `-ffast-math`, `/fp:fast`, or equivalent flags.** These flags silently enable FMA, reciprocal approximations, and associative reordering.

4. **Always compile with `-Werror -Wall -Wextra`.** Warnings about implicit float conversions, suspicious reordering, or signed overflow are early indicators of determinism bugs.

5. **Use `Vec3::equals_exact` (bitwise `memcmp`) for comparison in deterministic contexts.** Never use `operator==` with an epsilon tolerance, as different epsilon paths can diverge.

6. **Test determinism after any change to math, collision, or solver code.** Run the full determinism suite, not just the test you think is affected.

7. **Call `enforce_deterministic_fp_mode()` before any simulation code.** If this is not the first FP-related call in your process, something is wrong.

8. **Audit third-party integrations.** Any new library must be evaluated for FP mode side effects. If it touches MXCSR, FPCR, or rounding mode, wrap every call site in save/restore.

9. **Document deviations.** If a subsystem cannot meet bit-exact determinism (e.g., audio DSP), it must be isolated behind a documented interface that does not feed back into simulation state.

Determinism is not a feature — it is the foundation. Every line of code in APC exists within this constraint.
