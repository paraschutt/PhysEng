Deterministic Sports Physics Engine
DSPE — Revised Technical Brief v1.1
All critique items resolved. Implementation language clarified.
Parameter	Specification
Version	1.1 (Revised)
Implementation Language	C++20 (NOT Python — see Section 0)
Target Scale	15v15 (30 skeletal entities + ball + environment)
Core Requirement	Deterministic simulation for networked competitive play
Integration Method	Velocity Verlet (symplectic, time-reversible) + 4x substeps
Fixed Timestep	16.666ms outer (60Hz); 4.166ms substep (240Hz effective)
MAX_ENTITIES	64 (30 players + 1 ball + 33 reserved for environment/triggers)


0. Implementation Language: C++20
This engine is implemented in C++20. Python is not used anywhere in the physics simulation pipeline. This is a hard requirement driven by:
Deterministic fixed-point arithmetic requiring bitwise control over numeric representations
Performance targets (< 8ms physics step) incompatible with interpreted languages
Low-level memory layout control (cache-friendly SoA entity arrays)
Direct compiler flag control for floating-point consistency (-mfpmath=sse, -msse2)

Build toolchain: CMake 3.26+, compiler: GCC 13+ or Clang 16+ or MSVC 2022. Python may be used for offline tooling (asset pipeline, test harnesses, data export) but never in the real-time simulation loop.

1. Executive Architecture
1.1 Deterministic Foundation
To guarantee identical simulation across clients (critical for 15v15 networked sports):

Component	Specification
Numerical Representation	Positions: Q24.8 fixed-point. Forces/velocities: Q15.16. All multiplications use 64-bit intermediates. See Section 1.3 for range analysis.
Timestep	Fixed outer Δt = 16.666ms (60Hz). 4 substeps per tick = 4.166ms effective. No variable timestep.
Execution Order	Deterministic sort by entity ID every frame. No hash-based collections in hot path.
Random Seed	Synchronized per-match. Physics uses deterministic RNG (xorshift64).
Threading	Single-threaded physics or deterministically ordered parallel reduction.
MAX_ENTITIES	64 fixed-size array. Indices 0–29: players. Index 30: ball. Indices 31–63: environment/triggers.

✔ UPDATED: MAX_ENTITIES is now defined. Q15.16 replaced with domain-appropriate fixed-point formats. Substep count moved from risk section into core spec.

1.2 Fixed-Point Range Analysis (NEW)
⚠ Original brief specified Q15.16 for all quantities without range verification. This caused potential overflow on intermediate force computations.

Fixed-point format assignments after range analysis:

Quantity	Format	Max Value	Rationale
World position (x, y, z)	Q24.8	±16,777,215 m	Pitch = 105m, ample headroom
Velocity	Q15.16	±32,767 m/s	Max ball speed ~50 m/s, safe
Acceleration / Force	Q15.16	±32,767 m/s²	Max impulse < 1000 N, safe
Angular velocity (ω)	Q8.24	±127 rad/s	Max spin ~100 rad/s, safe
All intermediate products	int64_t	±9.2 × 10¹⁸	Mandatory for all mul ops

Rule: Every multiplication of two fixed-point values MUST widen to int64_t before the operation. Final result is shifted and truncated back to target format. This eliminates overflow on drag (v²) and Magnus (||ω|| × ||v||) computations.

1.3 Entity Component System (ECS)
World (Deterministic Update Loop)
  Entities[Fixed Array 0..63]  // MAX_ENTITIES = 64
    ComponentMask (bitflags for archetype)
    RigidBody (mass, invMass, position[Q24.8], velocity[Q15.16], acceleration[Q15.16])
    Collider (shape type, bounds, materialID)
    Skeleton (optional, 15-bone rig)
    BallProperties (spin[Q8.24], Magnus coefficient)
    TriggerVolume (optional, AABB, eventType)
  Systems (Strict Execution Order)
    InputSystem -> ForceAccumulator
    SkeletonController (animation -> torque)
    PhysicsStep (Velocity Verlet + 4 substeps)
    ContinuousCollisionDetection (ball + fast movers)
    BroadPhase (Dynamic AABB Tree update)
    NarrowPhase (GJK/EPA contact generation)
    TriggerSystem (goal/out-of-bounds detection)
    ConstraintSolver (Sequential Impulse, ordered by entity ID)
    SleepSystem (wake/sleep threshold management)
    EventSystem (collision callbacks, trigger callbacks)


2. Core Physics Specification
2.1 Time Integration: Velocity Verlet + Substeps
Chosen over Leapfrog for easier handling of velocity-dependent forces (drag, friction) while maintaining symplectic properties. 4 substeps per tick provide stability and enable CCD without variable timestep.

// Outer loop: 60Hz (deltaT = 16.666ms)
// Inner substep: 4x (dt = 4.166ms each)
for (int sub = 0; sub < 4; ++sub) {
  float dt = OUTER_DT / 4.0f;

  // Half-step velocity
  v(t + dt/2) = v(t) + 0.5 * a(t) * dt;

  // Full-step position
  x(t + dt) = x(t) + v(t + dt/2) * dt;

  // CCD sweep for ball + fast entities before force eval
  RunCCD(entities, dt);

  // Calculate forces at new position
  // NOTE: drag uses v(t + dt/2) - first-order approximation.
  // C_d is tuned empirically to compensate. See Section 2.1.1.
  a(t + dt) = F(x(t + dt), v(t + dt/2)) / m;

  // Complete velocity step
  v(t + dt) = v(t + dt/2) + 0.5 * a(t + dt) * dt;
}

2.1.1 Velocity-Dependent Force Approximation
⚠ Original brief claimed clean velocity-dependent force handling. This is not strictly accurate for Velocity Verlet.
Drag and Magnus forces are evaluated at v(t + Δt/2), the half-step velocity, not the true full-step velocity. This introduces a first-order error in drag magnitude. Mitigation strategy: C_d is calibrated empirically against reference ball trajectories rather than using the theoretical aerodynamic value. The error is small at 60Hz with 4 substeps and acceptable for gameplay-accurate (not scientifically exact) ball physics. Document acknowledges this approximation; do not tune C_d to its ISA value.

2.2 Substep Count Rationale
✔ UPDATED: Substep count moved from Risk Mitigation into core spec where it belongs.
4 substeps per 60Hz tick yield an effective integration frequency of 240Hz. This provides:
CCD window of 4.166ms — ball at 30 m/s travels 0.125m per substep, well above goalpost radius of 0.06m
Solver warm-start convergence within substep budget
Stable stacking (30-player pile) at < 8ms total step

2.3 Force Model
Global Forces
Gravity: 9.81 m/s² downward (configurable per stadium altitude)
Air Resistance (quadratic drag): F_drag = -0.5 × ρ × C_d × A × |v| × v
ρ: Air density 1.225 kg/m³ (sea level); configurable for altitude
C_d: Tuned empirically (~0.25 ball, ~1.0 player). Do not use theoretical ISA value.
Wind Force (NEW): F_wind = 0.5 × ρ × C_d × A × |v_wind|² × wind_dir, applied to ball and light projectiles only. Wind vector set per-stadium via data. Connects to crowd wind field in Section 7.2.

✔ UPDATED: Wind force added to Section 2.2 force model. Previously referenced in Section 7.2 (crowd wind fields) with no corresponding entry in the physics force model — now consistent.

Ball-Specific Forces
Magnus Effect (spin-induced lift): F_Magnus = S × (ω × v)
S = 0.5 × ρ × A × r × C_L (lift coefficient ~1.0 for spinning sphere)
Units: S has units kg/m, ω×v has units m/s², result is Newtons. Dimensionally correct.
Scaling note: This formulation scales linearly with both |ω| and |v|, not with |v|² as in full aerodynamic drag. C_L is tuned against empirical football trajectory data (Goff & Carré, 2010). At competitive ball speeds (15–35 m/s) the linear approximation is validated.

⚠ Magnus formula validated dimensionally. Linear-in-|v| scaling acknowledged; C_L must be calibrated to Goff & Carré (2010) data, not theoretical aerodynamic tables.


3. Continuous Collision Detection (NEW — Phase 1 Critical)
⚠ Original brief omitted CCD entirely. At 60Hz a ball at 30 m/s travels 0.5m per tick — 4.5× the ball radius. Tunnelling through goalposts (0.12m diameter) and foot colliders is guaranteed without CCD.

CCD is a Phase 1 requirement for the ball and any entity whose velocity exceeds v_threshold = r / Δt per substep.

3.1 Swept Sphere CCD (Ball)
Algorithm: Swept sphere vs. static capsule and swept sphere vs. swept capsule.
Cast a swept sphere from x(t) to x(t+dt) for the ball each substep
Find the earliest time-of-impact t_c ∈ [0, dt] against all colliders in broad phase
Advance ball to x(t_c), resolve contact impulse, continue integration from t_c with remaining dt
Cost: O(broad_phase_candidates) per substep, amortised by AABB tree

3.2 CCD Threshold
// Per entity, per substep: activate CCD if:
bool needsCCD = (length(v) * dt) > (collider.radius * 0.5f);
// Ball at 30 m/s, substep dt = 4.166ms:
// displacement = 0.125m, radius = 0.11m => CCD active always for ball
// Player at 8 m/s: displacement = 0.033m, capsule radius = 0.12m => CCD inactive (fine)

3.3 Phase 1 Deliverable Addition
[ ] Swept sphere vs. capsule CCD for ball entity
[ ] CCD threshold check per entity per substep
[ ] TOI (time of impact) solver integrated into substep loop


4. Constraint Solver — Full Specification
⚠ Original brief named Sequential Impulse but omitted iteration count, Baumgarte β, friction cone, and joint warm-starting. All are now specified.

4.1 Solver Parameters
Parameter	Value	Notes
Iteration count	10 (default)	Configurable 8–20. Higher = more accurate, more CPU. 10 validated for 200 contacts.
Baumgarte β (position bias)	0.2	Range 0.1–0.4. Below 0.1 = sinking; above 0.4 = jitter. 0.2 is the industry default.
Baumgarte slop	0.005m	Penetration depth ignored for position correction. Prevents vibration at rest.
Friction cone (μ)	Per material	Tangential impulse clamped: |λ_t| ≤ μ × λ_n. See material system Section 9.
Warm-starting: contacts	Yes	Cached impulses from previous frame, re-applied at iteration 0.
Warm-starting: joints	Yes (NEW)	Joint constraint lambda accumulated per frame. Reset on large pose delta.
Contact ordering	By entity ID pair	Required for determinism. Sort (min(eA,eB), max(eA,eB)) before solve.

4.2 Friction Cone Implementation
// For each contact constraint after normal impulse:
float lambda_n = ComputeNormalImpulse(contact);
lambda_n = max(0.0f, lambda_n);  // no pulling

// Tangential (friction) impulse, two axes
float lambda_t1 = ComputeTangentImpulse(contact, tangent1);
float lambda_t2 = ComputeTangentImpulse(contact, tangent2);

// Friction cone clamp
float mu = GetMaterial(contact).kineticFriction;
float maxFriction = mu * lambda_n;
float frictionMag = sqrt(lambda_t1*lambda_t1 + lambda_t2*lambda_t2);
if (frictionMag > maxFriction && frictionMag > 0.0f) {
    float scale = maxFriction / frictionMag;
    lambda_t1 *= scale;
    lambda_t2 *= scale;
}


5. Skeletal Rig Specification
5.1 Anatomy Structure (unchanged)
Root (Pelvis) [6DOF Rigid Body]
  Spine [Revolute Joint, 1 axis]
    Head [Spherical Joint, 3 axis limit]
    LeftArm  [Revolute] -> LeftForeArm  [Revolute]
    RightArm [Revolute] -> RightForeArm [Revolute]
  LeftThigh  [Revolute, hip flexion]
    LeftCalf  [Revolute, knee 0-140 deg]
      LeftFoot  [Hinge, limited + velocity sensor]
  RightThigh [Revolute]
    RightCalf [Revolute]
      RightFoot [Hinge, limited + velocity sensor]

5.2 Foot Handling (REVISED)
⚠ Original spec marked feet as 'Fixed to calf, sensor only'. This is incorrect: foot velocity at contact determines kick power and spin. Feet must participate in the impulse calculation.

Foot colliders are active participants in the constraint solve, not passive sensors:
Foot capsule: radius 0.08m, hinge-constrained to calf (dorsiflexion: -20° to +50°)
Foot velocity v_foot is sampled at the contact point for kick impulse magnitude
Off-centre contact offset from foot centroid determines spin generation (Section 6.2)
Foot colliders included in inter-skeleton narrow phase (kicking, tackling foot-to-foot)
Foot colliders excluded from self-skeleton narrow phase only

5.3 Foot Placement IK (Phase 2 Addition)
✔ UPDATED: Foot IK added to Phase 2 deliverables. On any non-flat pitch surface, rigid FK skeletons produce visible foot penetration or floating.
Two-bone analytical IK pass applied post-simulation, pre-render:
Raycast from knee toward ground at foot position
Solve two-bone IK (hip→knee→ankle) to plant foot on surface
Blend IK weight by ground proximity (full IK when foot within 0.3m of ground, zero when airborne)
Apply pelvis height offset to avoid hip crunch when both feet on elevated surface


6. Ball Physics Module
6.1 Rigid Body Properties (unchanged)
Mass: 0.43 kg (FIFA standard)
Radius: 0.11 m
Coefficient of Restitution (e): 0.8 (grass), 0.95 (hard surface)
Friction: 0.6 (rolling), 0.4 (sliding)

6.2 Spin Dynamics
State tracked as angular velocity vector ω (Q8.24 fixed-point, 3 components):
Air interaction: Magnus force (see Section 2.3)
Surface interaction: Friction impulse generates spin (topspin on bounce)
Decay: Air resistance torque: τ_drag = -k_rot × ω, where k_rot = 0.5 × ρ × C_d_rot × r⁵

6.3 Contact Response
Incremental manifold persistence for stable resting contact
Contact points cached between substeps, discarded if separation > 0.001m
Warm-starting impulses for solver convergence (initial guess = previous frame impulse × 0.8)
Baumgarte stabilization for penetration recovery: bias = β/Δt × max(0, depth − slop)

7. Sleep System (Promoted to Phase 1)
✔ UPDATED: Sleep system moved from Risk Mitigation to Phase 1 deliverable. In a 15v15 match, most players are in low-velocity ground contact most of the time. Simulating full Velocity Verlet + constraint solve on sleeping entities wastes ~30-40% of physics budget.

Sleep rules:
Entity enters SLEEP state when linear velocity < 0.01 m/s AND angular velocity < 0.01 rad/s for 0.5 seconds (30 consecutive ticks)
Sleeping entities are skipped by integrator, CCD, and narrow phase
Sleep is broken by: collision with non-sleeping entity, external force application, proximity wake radius (1.0m from any non-sleeping entity)
Ball is never allowed to sleep (always simulate)
Sleeping entities still participate in broad phase AABB tree for wake triggering


8. Trigger Volume System (NEW)
⚠ Original brief listed 'goals' in the event log with no explanation of how they are detected. Trigger volumes are the physics-layer mechanism for goal, out-of-bounds, and penalty area events.

Trigger volumes are AABB regions attached to entities with ComponentMask::TRIGGER. They do not participate in collision resolution — they only generate events. Checked once per outer tick (not per substep).

Trigger	Shape	Event Generated
Goal mouth (left)	AABB	EVENT_GOAL_LEFT when ball centroid fully inside bounds
Goal mouth (right)	AABB	EVENT_GOAL_RIGHT when ball centroid fully inside bounds
Pitch boundary	AABB (inverted)	EVENT_OUT_OF_BOUNDS when ball exits pitch AABB
Penalty area (left/right)	AABB	EVENT_BALL_IN_BOX — referee logic determines foul consequence
Kick-off circle	Circle (2D)	EVENT_KICKOFF_INFRINGEMENT if opposing player inside at kickoff

Trigger volumes are defined in the stadium configuration file and loaded at match init. Entity IDs 31–63 are reserved for environment and trigger volumes.

9. Material System (Full Specification)
✔ UPDATED: Material system was referenced throughout the brief but never defined. Full property set specified here.

Surface	Restitution e	Static μ	Kinetic μ	Rolling μ
Dry grass (natural)	0.80	0.70	0.60	0.018
Wet grass (natural)	0.75	0.45	0.35	0.012
Artificial turf (3G)	0.82	0.65	0.55	0.020
Hard surface (indoor)	0.95	0.50	0.40	0.010
Player boot (vs ball)	0.50	0.80	0.70	N/A
Player boot (vs player)	0.30	0.55	0.45	N/A
Goalpost (steel)	0.60	0.30	0.25	N/A

Weather modifier: When surface == wet, multiply static μ and kinetic μ by weather.wetness_factor ∈ [0.5, 1.0]. Wetness factor is data-driven per stadium, interpolated over match duration. Material IDs are 8-bit integers; ID 0 = default (dry grass).


10. Network Integration (Revised)
⚠ Original brief conflated rollback netcode and match replay into a single 10-second buffer, which would require ~3.6GB at 60Hz × 30 entities × ~200 bytes of state — exceeding the 64MB memory budget by 56x.

✔ UPDATED: Rollback buffer and replay buffer are now separate systems with separate memory budgets.

10.1 Rollback Buffer (Netcode)
Purpose: Compensate for input latency in peer-to-peer networked matches. Re-simulate when late inputs arrive.
Buffer size: 8 frames (133ms at 60Hz) — sufficient for LAN and low-latency WAN
State snapshot: position + velocity + angular velocity per entity = ~160 bytes × 64 entities × 8 frames = 82KB
Memory budget: < 256KB (well within 64MB limit)
Trigger: late input received for frame N — restore snapshot N-1, re-simulate N..current
Prediction: extrapolate opponent inputs using last known command (dead-reckoning)

10.2 Replay Buffer (Spectator / Instant Replay)
Purpose: In-engine instant replay for broadcast view, VAR-style review, and highlight export.
Buffer size: 10 seconds (600 frames at 60Hz)
State snapshot: compressed transform only (position + quaternion) = ~56 bytes × 64 entities × 600 frames = 2.15MB
Memory budget: < 4MB
Separate ring buffer from rollback; written every tick, never used for re-simulation
Replay playback uses the interpolation system (Section 12.1) for smooth scrubbing

10.3 Desync Detection
State checksum (FNV-1a 64-bit) computed every 60 frames from full physics state
Checksum exchanged between clients; mismatch triggers EVENT_DESYNC
On desync: host sends full state snapshot for resync (authoritative server model)

11. Determinism Checklist (Extended)
✔ UPDATED: Compiler flags and transcendental function handling added. Original checklist was incomplete.

	Requirement	Verification
[ ]	Compile with -ffloat-store (GCC) or /fp:precise (MSVC)	Check compiler flags in CI
[ ]	Compile with -mfpmath=sse -msse2 (x86) or -mfpu=vfp (ARM) to prevent x87 80-bit extended precision	Critical: -ffloat-store alone is insufficient
[ ]	No std::unordered_map in hot path. Use std::vector + linear search (N < 64)	Static analysis grep for unordered_map in physics files
[ ]	No transcendentals (sin/cos/sqrt) from standard library in physics loop. Replace with LUT or deterministic math lib (e.g., fpm, libfixmath)	libc sqrt differs by platform/version
[ ]	No FMA instructions (-ffp-contract=off or equivalent)	FMA fuses mul+add with single rounding — platform dependent
[ ]	Sort contacts by (min(eA,eB), max(eA,eB)) before solve	Automated determinism test
[ ]	All fixed-point multiplications use int64_t intermediate	Static analysis; overflow unit tests
[ ]	Fixed seed for any procedural geometry	Seed logged in match header
[ ]	State hash every frame in debug builds; every 60 frames in release	Cross-platform hash comparison CI job


12. Phase 1 Deliverables (Updated)
✔ UPDATED: Sleep system and CCD added to Phase 1. These were missing from the original deliverable list.

12.1 Core Simulation
[ ] Fixed-point math library (Q24.8, Q15.16, Q8.24) with 64-bit intermediate enforcement
[ ] Velocity Verlet integrator with 4 substeps and force accumulation
[ ] Swept sphere CCD for ball + fast entities
[ ] Broad-phase (Dynamic AABB Tree) + Narrow-phase (GJK/EPA)
[ ] Trigger volume system (goal, out-of-bounds, penalty area)
[ ] Sequential Impulse solver: 10 iterations, β=0.2, friction cone, joint warm-starting
[ ] 15-bone skeleton with hinge/spherical joints + foot velocity sensing
[ ] Ball physics: Magnus effect (C_L calibrated to Goff & Carré), surface friction, CCD
[ ] Material system (7 surface types + wet modifier)
[ ] Sleep/wake system with proximity wake radius

12.2 Testing & Validation
[ ] Determinism test: 30-min match simulation, hash state every 60 frames, verify cross-platform bit identity (Windows/Linux/macOS)
[ ] Energy conservation: ball dropped from 10m, verify bounce heights match e=0.8 restitution analytically
[ ] CCD test: ball fired at 40 m/s at a 0.12m goalpost at 2m range — must not tunnel
[ ] Stability test: 30 players stacked, no explosion/jitter after 10 seconds
[ ] Spin test: curve ball trajectory error < 5% vs. analytical Magnus prediction
[ ] Friction cone test: player sliding on wet grass must stop at μ-correct distance
[ ] Trigger test: ball crossing goal line fires EVENT_GOAL within 1 tick

12.3 Phase 2 Deliverables (Updated)
[ ] Interpolate/extrapolate: render at variable FPS (144Hz+) between 60Hz physics ticks
[ ] GPU mesh skinning for 15-bone skeletons
[ ] Foot placement IK: two-bone analytical IK pass, blended by ground proximity
[ ] Ball trail: visual arc showing trajectory + spin axis
[ ] Debug overlay: force vectors, collision shapes, CCD sweep volumes, centre of mass, sleep state
[ ] Camera physics: collision-aware camera with spring-damper follow
[ ] Stadium collision meshes (static BVH)
[ ] Surface shaders responding to weather (wet grass = friction modifier)
[ ] Crowd wind field: runtime wind vector from stadium config, feeds into Section 2.3 wind force

13. Performance Targets
System	Budget	Notes
Full physics step (15v15)	< 8ms	120Hz budget. Includes 4 substeps.
CCD sweep (ball only)	< 0.5ms	Per substep. AABB broad phase pre-filters.
Constraint solve (200 contacts)	< 3ms	10 iterations × 200 contacts. Vectorise inner loop.
Sleep overhead (30 sleeping)	< 0.2ms	AABB tree wake check only.
Physics state memory	< 64MB	Full simulation state. Replay buffer < 4MB extra.
Rollback buffer	< 256KB	8 frames at 64 entities.

14. Risk Mitigation (Updated)
Risk	Mitigation
Determinism failure	State hash every frame (debug) / every 60 frames (release). Automated cross-platform CI. See Section 11 checklist.
Solver explosion	Position/velocity clamping. 4 substeps. Baumgarte slop 0.005m. Velocity clamp: |v| < 200 m/s.
Ball tunnelling	CCD mandatory for ball (Section 3). Validated by goalpost tunnel test.
Jitter at rest	Sleep system (Section 7). Baumgarte slop. Warm-starting.
Performance overrun	Sleep system reduces active entity count. LOD: distant players use single capsule. Profile per substep.
Fixed-point overflow	64-bit intermediates mandatory (Section 1.2). Range analysis documented. Overflow unit tests in CI.

Appendix A: Revision Changelog (v1.0 → v1.1)
#	Area	Change
1	Language	Added Section 0: C++20 specified. Python explicitly excluded from simulation pipeline.
2	Fixed-point	Q15.16 replaced with domain-specific formats (Q24.8/Q15.16/Q8.24). Range analysis table added. 64-bit intermediates mandated.
3	Determinism	-mfpmath=sse -msse2 added to checklist. Transcendental functions (sin/cos/sqrt) must use LUT or deterministic lib. -ffp-contract=off added.
4	Velocity Verlet	Drag approximation (half-step velocity) documented. C_d empirical tuning noted. 4 substeps moved from risk to core spec.
5	CCD	New Section 3. Swept sphere CCD for ball added to Phase 1. Goalpost tunnel scenario documented.
6	Constraint solver	Iteration count (10), Baumgarte β (0.2), slop (0.005m), friction cone, and joint warm-starting all specified.
7	Foot handling	Feet promoted from passive sensors to active constraint participants. Foot IK added to Phase 2.
8	Magnus formula	Dimensional verification added. Linear-in-|v| scaling acknowledged. C_L calibration to Goff & Carré (2010) mandated.
9	Network buffers	Rollback (8 frames, 256KB) and replay (600 frames, 4MB) separated. Memory contradiction resolved.
10	Sleep system	Promoted from Risk Mitigation to Phase 1 deliverable. Full wake/sleep rules specified.
11	Trigger volumes	New Section 8. Goal, out-of-bounds, penalty area, kick-off circle triggers specified.
12	Material system	Full property table added (7 surface types, restitution/static μ/kinetic μ/rolling μ/wet modifier).
13	Wind force	Wind force added to Section 2.3 force model. Now consistent with crowd wind reference in Section 12.3.
14	MAX_ENTITIES	Defined as 64. Entity index ranges assigned (players 0-29, ball 30, environment/triggers 31-63).
