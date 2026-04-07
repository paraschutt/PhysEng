Adaptive Physics Core (APC) - Technical Architecture Document
Author: Lead Developer
Status: Initial Specification
Classification: Internal - Engineering

1. Executive Summary
We're not building a physics engine. We're building a physics behavior authoring system that happens to include a simulation core. The distinction matters—it shapes every architectural decision.

Off-the-shelf solutions fail us because they answer "what happens when objects collide?" when we need to answer "what should happen when a 220lb linebacker hits a receiver mid-leap, and how do we make that feel intentional rather than emergent?"

2. Core Technical Pillars
2.1 Determinism (Non-Negotiable)
Every frame, every platform, every replay. This enables:

Rollback netcode for "Impact League"
Frame-perfect replay systems
Consistent AI training in simulation mode
Debug reproducibility across dev machines
Implementation Strategy:

text

- Fixed timestep: 240Hz internal (4x visual frame)
- IEEE 754 strict compliance (no FMA, no SIMD float reordering)
- Platform-specific FP mode enforcement at startup
- Sorted iteration over all collections (ID-based, not pointer-based)
- No external library calls mid-simulation (sin/cos look-up tables)
Hard Rule: If it can't be made deterministic, it doesn't ship in the core.

2.2 Skeletal Body Dynamics (The Differentiator)
We're abandoning traditional ragdoll-as-afterthought. Instead:

text

TRADITIONAL APPROACH:
Animation → Visual Pose → [On Death/Hit] → Ragdoll Overlay → Jittery mess

APC APPROACH:
Animation → Pose Intent → Physics Resolution → Final Pose
                ↑                    ↓
          Style Parameters    Skeletal Constraints
Articulated Body Algorithm: Featherstone's Articulated Body Algorithm (ABA) with extensions:

Reduced coordinate representation (joint angles, not world positions)
Proper inertia propagation through kinematic chains
Support for loop closures (shoulder girdle, pelvis stability)
Per-Bone Physics Blend:

cpp

struct BonePhysicsState {
    uint8_t blend_mode;        // ANIM_DRIVEN, PHYSICS_DRIVEN, BLENDED
    float physics_weight;      // 0.0 = pure anim, 1.0 = pure physics
    float anim_weight;         // Inverse of above (cached for hot path)
    float stiffness;           // Spring constant back to anim target
    float damping;             // Velocity damping toward anim target
    float max_deviation;       // Clamp distance from anim pose
    uint8_t collision_participation; // Does this bone generate/resolve collisions?
};
This allows a character to be mostly animation-driven while their trailing arm gets knocked by a tackle—the arm resolves physically and springs back.

2.3 Mesh-Level Collision Resolution
We need more than primitives. A helmet hitting a knee needs actual surface interaction.

Three-Tier Collision Strategy:

Tier
Complexity
Use Case
Performance Budget
Broadphase      AABB/Sphere     Culling <0.1ms
Midphase        OBB Trees       Limb-level      <0.5ms
Narrowphase     SDF-GJK Hybrid  Contact generation      <2.0ms

SDF-GJK Hybrid (Our Innovation):

Precompute Signed Distance Fields for complex convex hulls
Use SDF for penetration depth estimation (faster than EPA)
Use GJK for contact normal accuracy
Combine: SDF depth + GJK normal = correct response, faster convergence
Mesh Decomposition Pipeline:

text

Source Mesh → V-HACD (Convex Decomposition) → Merge small pieces → 
Generate OBB hierarchy → Bake SDF per convex piece → Runtime asset
Target: 8-12 convex pieces per character limb, 60-80 per full character.

2.4 Stylized Physics Response
Realistic physics isn't fun physics. We need tunable "game feel."

Impact Response System:

cpp

struct ImpactStyleProfile {
    // Momentum transfer modifiers
    float base_momentum_transfer;    // 0.0 - 2.0 (1.0 = realistic)
    float vertical_bias;             // Exaggerate upward launches
    float spin_multiplier;           // More satisfying tumbling
    
    // Contact behavior
    float slide_friction_curve[8];   // Non-linear friction by angle
    float bounce_curve[8];           // Velocity-dependent restitution
    
    // "Game feel" hacks (named honestly)
    float hit_stop_duration_ms;      // Brief pause on big impacts
    float camera_shake_intensity;    // Hook for camera system
    float time_scale_on_impact;      // Brief slow-mo window
    
    // Outcome scripting
    ImpactOutcomeTable* outcome_lut; // Designer-authored results
};
Impact Outcome Tables:
Designers don't tweak coefficients—they define outcomes:

text

IF (tackle_speed > 15 m/s) AND (receiver_airborne) AND (contact_point = torso)
THEN (launch_angle = 45°, spin_rate = 720°/s, ragdoll_duration = 1.2s, outcome = DRAMATIC_FLYING)
Physics simulates toward these outcomes rather than away from them.

3. System Architecture
text

┌─────────────────────────────────────────────────────────────────┐
│                      GAME INTEGRATION LAYER                      │
│  (Sport-specific rules, "Impact League" game code, callbacks)   │
└─────────────────────────────────────────────────────────────────┘
                                │
                                ▼
┌─────────────────────────────────────────────────────────────────┐
│                    PHYSICS BEHAVIOR AUTHORMG                      │
│  Impact profiles, outcome tables, style curves, ragdoll presets  │
└─────────────────────────────────────────────────────────────────┘
                                │
                                ▼
┌─────────────────────────────────────────────────────────────────┐
│                      SKELETAL DYNAMICS CORE                      │
│  Articulated body solver, bone blend states, pose constraints   │
└─────────────────────────────────────────────────────────────────┐
                                │
              ┌─────────────────┼─────────────────┐
              ▼                 ▼                 ▼
┌──────────────────┐ ┌──────────────────┐ ┌──────────────────┐
│  COLLISION DETECT │ │  CONSTRAINT SOLVE │ │  INTEGRATION      │
│  Broad/Mid/Narrow │ │  Sequential Impulse│ │  Semi-implicit    │
│  SDF-GJK Hybrid   │ │  + Position Proj  │ │  Euler (Verlet)   │
└──────────────────┘ └──────────────────┘ └──────────────────┘
              │                 │                 │
              └─────────────────┼─────────────────┘
                                ▼
┌─────────────────────────────────────────────────────────────────┐
│                         MATH FOUNDATION                         │
│  Vec3/Mat3/Quat (deterministic), Fixed-point optional, LUTs     │
└─────────────────────────────────────────────────────────────────┘
                                │
                                ▼
┌─────────────────────────────────────────────────────────────────┐
│                      MEMORY & JOB SYSTEM                        │
│  SOA layout, frame-persistent allocators, job graph (fiber)     │
└─────────────────────────────────────────────────────────────────┘
4. Memory & Performance Strategy
4.1 Data Layout
Structure of Arrays for hot paths:

cpp

// NOT this:
struct RigidBody {
    Vec3 position;
    Vec3 velocity;
    Quat orientation;
    float mass;
    // ... 40 more fields
};
std::vector<RigidBody> bodies;

// THIS:
struct BodySoA {
    float* px; float* py; float* pz;        // Positions
    float* vx; float* vy; float* vz;        // Velocities  
    float* qx; float* qy; float* qz; float* qw;  // Orientations
    float* inv_mass;                          // Inverse masses
    float* inv_inertia[9];                   // Inertia tensors
    uint32_t* flags;                          // Static/active/sleeping
    uint32_t count;
};
Why: Cache-friendly iteration, enables SIMD even with strict FP mode (carefully), easier to parallelize.

4.2 Memory Budget (Per Frame, 22 Characters)
System
Budget
Notes
Collision pairs 128KB   Max ~4000 pairs
Contact data    64KB    4 contacts per pair max
Skeletal state  256KB   ~80 bones × 22 chars × full state
Solver scratch  512KB   Jacobian rows, intermediate
Total   ~1MB    Fits L2 comfortably

4.3 Threading Model
text

Frame Timeline (8.33ms for 120Hz render, 4.17ms for physics):

├─ Broadphase (parallel) .................. 0.3ms
├─ Midphase (parallel per-pair) ........... 0.8ms  
├─ Narrowphase (parallel per-pair) ......... 1.5ms
├─ Contact clustering ........................ 0.2ms
├─ Skeletal ABA (parallel per-skeleton) ... 1.0ms
├─ Constraint solve (4 iterations) ......... 1.2ms
│   └─ Sequential impulse, parallel within iteration
└─ Integration + output ........................ 0.2ms
                                  Total: ~5.2ms
Target: 5ms physics budget for "Impact League" (22 characters + ball + environment).

5. Development Phases
Phase 0: Foundation (Weeks 1-4)
Deliverable: Deterministic math lib + basic rigid body sim

text

MILESTONES:
□ IEEE 754 compliance tests passing on x64/ARM/Console
□ Vec3/Quat/Mat3 with 100% branch coverage
□ Fixed-point fallback path (optional, for future mobile)
□ Basic sphere-sphere collision
□ 10,000 frame determinism test across platforms
□ Performance baseline: 1000 spheres at 240Hz
Phase 1: Collision Core (Weeks 5-12)
Deliverable: Full collision pipeline + convex mesh support

text

MILESTONES:
□ AABB broadphase with sweep-and-prune
□ OBB tree midphase
□ GJK narrowphase for convex primitives
□ EPA for penetration resolution
□ Convex decomposition pipeline (V-HACD integration)
□ Mesh-mesh collision working
□ SDF generation and baking pipeline
□ Collision filtering system (layers, masks, callbacks)
Phase 2: Skeletal Dynamics (Weeks 13-24)
Deliverable: Articulated body simulation + animation blending

text

MILESTONES:
□ Featherstone ABA implementation
□ Skeletal asset format definition
□ Per-bone physics blend (ANIM/PHYSICS/BLENDED modes)
□ Spring-damper pose constraints
□ Loop closure constraints (closed kinematic chains)
□ Animation sample → physics intent pipeline
□ First "standing humanoid that gets knocked over" demo
Phase 3: Stylized Response (Weeks 25-32)
Deliverable: Impact profiles + outcome system

text

MILESTONES:
□ ImpactStyleProfile system
□ Non-linear friction/restitution curves
□ Hit-stop and time-scale hooks
□ ImpactOutcomeTable authoring and runtime
□ Designer-facing tuning tool (Unity/Unreal plugin or standalone)
□ "Tackle that feels good" vertical slice
Phase 4: Sports Physics (Sprints 13-16) ✅ COMPLETE
Deliverable: Multi-sport physics subsystem — ball physics, control, implements, contact sports, sport fields, rules
Commit: 1c8bf99 — 8 headers, 4 test files, 523 assertions, 29/29 tests pass

Sprint 13: Ball Physics Core
Files: apc_ball_physics.h
Tests: test_sprint13.cpp (12 tests, 201 asserts)

MILESTONES:
☑ BallShape enum (SPHERE, PROLATE) — round ball and oval ball support
☑ SurfaceType enum (14 surfaces: GRASS, ARTIFICIAL, CLAY, HARD_COURT, WOOD, CARPET, SAND, ICE, WATER, TRAMPOLINE, MAT, CONCRETE, CUSTOM_0/1)
☑ SpinAxis enum (TOPSPIN, BACKSPIN, SIDESPIN, HELICAL)
☑ BallConfig — shape, dimensions, mass, MOI, drag Cd, Magnus Cl, friction, restitution, get_effective_radius(), get_drag_force(), get_magnus_force(), to_rigid_body()
☑ BallState — runtime state with spin tracking, deformation, surface, knuckle intensity, update_spin_info()
☑ SurfaceBounceTable — per-surface restitution, friction, rolling resistance, spin damping, deformation, 14 entries
☑ AerodynamicModel — drag + Magnus force + spin decay, compute() and apply()
☑ BallBounce — resolve_surface() and resolve_body() with deformation and spin transfer
☑ BallFactory — 11 sport-specific presets (soccer, basketball, rugby, american_football, tennis, baseball, golf, volleyball, aussie_rules, cricket, handball)
☑ BallPhysicsWorld — multi-ball simulation (MAX_BALLS=32), step with gravity + aero + ground bounce
☑ Knuckle ball effect (reduced drag for unpredictable flight)
☑ Deformation and recovery system
☑ Contact sports: oval ball (PROLATE) with semi-major/semi-minor axes for rugby, football, aussie rules

Sprint 14: Ball Control, Passing & Kicking
Files: apc_ball_control.h, apc_pass_kick.h
Tests: test_sprint14.cpp (8 test groups, 61 asserts)

MILESTONES:
☑ ControlMethod enum (NONE, FOOT, HAND, BOTH, CATCH_ASSIST, HEAD, CHEST, EQUIPMENT)
☑ ControlState enum (FREE, DRIBBLED, CONTROLLED, CARRIED, TRAPPED, IN_FLIGHT, FUMBLING, DEAD)
☑ ControlPoint enum (8 body points: RIGHT/LEFT_FOOT, RIGHT/LEFT_HAND, BOTH_HANDS, CHEST, HEAD, THIGH, EQUIPMENT)
☑ AthleteBallControl — per-athlete dribble/trap/carry/catch parameters
☑ PossessionSystem — register/claim/release/fumble, MAX_RECORDS=64
☑ DribbleController — update_foot_dribble() and update_hand_dribble() with touch timing
☑ CatchController — attempt_catch() with magnetism assist, carry_ball()
☑ KickType enum (15 types: INSTEP, SIDE_FOOT, CHIP, CURVE, LOB, PUNT, DROP_KICK, PLACE_KICK, DROP_PUNT, TORPEDO, VOLLEY, HALF_VOLLEY, BICYCLE, PANNA, RABONA)
☑ ThrowType enum (14 types: CHEST_PASS, BOUNCE_PASS, OVERHEAD, ONE_HAND, SPIRAL, LOB_THROW, LATERAL, SHOT_PUT, POP_PASS, HAND_THROW, BASEBALL, UNDERHAND, SHOVEL, HOOK_PASS)
☑ KickProfile — per-kick power, accuracy, launch angle, spin factors; 7 factory presets
☑ ThrowProfile — per-throw parameters; 6 factory presets
☑ KickExecutor — execute() with power application, accuracy scatter, spin, launch angle
☑ ThrowExecutor — execute() with hand velocity contribution
☑ TrajectoryPrediction — predict_parabolic(), position_at(), time_to_height(), landing_position(), MAX_POINTS=64
☑ Contact sports: shoulder charge, screen/block, clothesline
☑ Controlling with feet (soccer dribble) and hands (basketball dribble) differentiated
☑ Passing mechanics (accuracy scatter, spin application)

Sprint 15: Implements & Contact Sports
Files: apc_implement.h, apc_contact_sport.h
Tests: test_sprint15.cpp (10 tests, 116 asserts)

MILESTONES:
☑ ImplementType enum (14 types: BASEBALL_BAT, CRICKET_BAT, TENNIS_RACKET, BADMINTON_RACKET, TABLE_TENNIS, SQUASH_RACKET, GOLF_DRIVER, GOLF_IRON, GOLF_WEDGE, GOLF_PUTTER, HOCKEY_STICK, FIELD_HOCKEY, LACROSSE, HURLING_HURLEY)
☑ ImplementShape enum (CYLINDER, FLAT_FACE, STRING_BED, NET_POCKET)
☑ SweetSpotZone enum (PERFECT, GOOD, AVERAGE, POOR, MIS_HIT, SHANK)
☑ ImplementProfile — mass, length, grip, hitting zone, sweet spot, face properties, string tension, flex; 7 factory presets
☑ SweetSpotModel — classify(), get_power/accuracy/spin_multiplier()
☑ ImplementHitResolver — sweet spot classification, restitution, flex bonus, loft angle, deflection, spin from friction+loft
☑ ImplementSwingModel — wind-up, acceleration, follow-through phases, get_tip_velocity()
☑ ContactType enum (15 types: TACKLE_FORM/SHOULDER/DIVE/STRIP, SACK, BLOCK, SHOULDER_CHARGE, SCREEN, ELBOW, HIP_CHECK, CLOTHESLINE, GRAPPLE_HOLD/THROW, PUSH, COLLISION)
☑ TackleType enum (11 types: NONE, LOW, HIGH, MID, DIVE_SLIDE, DIVE_HEADER, WRAP, ARM_TACKLE, DOUBLE, BEHIND, SIDE, FRONT)
☑ BlockType enum (9 types: NONE, PASS_BLOCK, RUN_BLOCK, SCREEN, BOX_OUT, PICK, BODY_CHECK, LEGAL_SHIELD, ILLEGAL)
☑ GrappleType enum (10 types: NONE, COLLAR_TIE, BODY_LOCK, SINGLE/DOUBLE_LEG, HEAD_LOCK, MAUL_BIND, RUCK_OVER, JUDO_HIP/SHOULDER)
☑ TackleProfile — success rate, power, momentum transfer, fumble chance, injury risk, recovery; 4 factory presets
☑ BlockProfile — success rate, impact absorption, deflection, push force; 4 factory presets
☑ ContactResolver — resolve_tackle(), resolve_block(), resolve_shoulder_charge()
☑ GrappleResolver — initiate(), update(), break, MAX_GRAPPLES=16
☑ ContactResult — full post-contact state: velocities, recovery, hit_stop, camera_shake, time_scale
☑ Bat/racket sports: baseball bat, cricket bat, tennis racket, badminton racket, golf clubs, hockey sticks
☑ Contact sports: form tackle, shoulder tackle, slide tackle, dive tackle, grapple mechanics

Sprint 16: Sport Environments & Integration
Files: apc_sport_field.h, apc_sport_rules.h, apc_sport_physics.h
Tests: test_sprint16.cpp (10 tests, 145 asserts)

MILESTONES:
☑ SportType enum (28 sports: SOCCER, BASKETBALL, AMERICAN_FOOTBALL, RUGBY_UNION/LEAGUE, AUSTRALIAN_RULES, TENNIS, VOLLEYBALL, HANDBALL, ICE_HOCKEY, FIELD_HOCKEY, CRICKET, BASEBALL, GOLF, BADMINTON, TABLE_TENNIS, LACROSSE, RUGBY_SEVENS, BEACH_VOLLEYBALL, FUTSAL, HURLING, GAELIC_FOOTBALL, WATER_POLO, BOXING, MMA, WRESTLING, CUSTOM_0/1)
☑ FieldType enum (RECTANGLE, OVAL, DIAMOND, CIRCLE, RINK, RING, COURSE, MAT)
☑ FieldGeometry — dimensions, goal dims, penalty area, 3pt line, key; 10 factory presets (soccer, basketball, american_football, rugby, tennis, ice_hockey, baseball_diamond, aussie_rules, volleyball, beach_volleyball)
☑ FieldZoneId enum (19 zones: FULL_FIELD, HALF_HOME/AWAY, PENALTY_AREA, GOAL_AREA, CENTER_CIRCLE, WINGS, END_ZONES, THREE_POINT, KEY_PAINT, etc.)
☑ FieldZone — zone_id, center, extents, surface, out-of-bounds, scoring; contains_point()
☑ GoalPost — ball-in-goal detection, post/crossbar hit detection
☑ BoundaryEventType enum (14 events: OUT_OF_BOUNDS, GOAL_SCORED, HIT_POST, HIT_CROSSBAR, HIT_NET, TOUCHDOWN, SAFETY, DEAD_BALL, GROUNDING, BEHIND, HOME_RUN, FOUL_BALL)
☑ SportField — bounds checking, surface lookup, goal checking, boundary events; MAX_ZONES=20, MAX_GOALS=4, MAX_EVENTS=32
☑ PlayState enum (27 states: KICKOFF, LIVE, DEAD_BALL, GOAL_SCORED, TIMEOUT, PERIOD_END, OVERTIME, SHOOTOUT, FREE_KICK, CORNER_KICK, PENALTY_KICK, THROW_IN, LINEOUT, SCRUM, FACEOFF, JUMP_BALL, POSSESSION, etc.)
☑ FoulType enum (27 types: TACKLE_FROM_BEHIND, HIGH_TACKLE, LATE_TACKLE, DANGEROUS_PLAY, HAND_BALL, OFFSIDE, CHARGING, HOLDING, BLOCKING_FOUL, PERSONAL/TECHNICAL/FLAGRANT_FOUL, etc.)
☑ CardType enum (NONE, YELLOW, RED, BLUE, WHITE)
☑ ScoringSystem — per-sport point values, add_score(), configure for soccer/basketball/rugby/american_football/aussie_rules
☑ DisciplineSystem — foul tracking, card determination, penalty assessment, suspension; MAX_FOULS=64, MAX_ATHLETES=64
☑ ClockSystem — game/period/shot/play clocks, timeouts, configure for soccer/basketball/american_football
☑ SportPhysicsWorld — unified integration manager owning all subsystems, step(), kick(), throw_ball(), tackle(), hit_with_implement()
☑ SportPhysicsFactory — create_soccer(), create_basketball(), create_american_football(), create_rugby(), create_tennis(), create_ice_hockey()
☑ SportConfig — per-sport physics configuration; 6 factory presets
☑ Full soccer integration test (kick → step → ball movement → boundary check)
☑ Full rugby tackle integration test (tackle → momentum exchange → fumble check)
☑ All 6 factory sports verified (soccer, basketball, american football, rugby, tennis, ice hockey)

Phase 4 Summary:
  Headers: 8 (apc_ball_physics.h, apc_ball_control.h, apc_pass_kick.h, apc_implement.h, apc_contact_sport.h, apc_sport_field.h, apc_sport_rules.h, apc_sport_physics.h)
  Tests: 4 files (test_sprint13.cpp, test_sprint14.cpp, test_sprint15.cpp, test_sprint16.cpp)
  Assertions: 523 total (201 + 61 + 116 + 145)
  Sports covered: 28 (6 with full factory presets)
  Ball types: 11 factory presets (round ball + oval ball)
  Implement types: 14 (bat/racket/stick variants)
  Kick types: 15 | Throw types: 14
  Field presets: 10 | Surface types: 14
  Contact types: 15 | Tackle types: 11 | Grapple types: 10
  All deterministic, header-only, zero dynamic allocation, C++17, apc:: namespace

Phase 5: Middleware Extraction (Weeks 45-60)
Deliverable: APC as licensable internal middleware

text

MILESTONES:
□ Clean API boundary (no game-specific code)
□ Engine-agnostic integration layer
□ Documentation (API docs, integration guides)
□ Test suite (unit, integration, cross-platform determinism)
□ Sample projects (basic, skeletal, sport)
□ Packaging and versioning system
6. Risk Register
Risk
Probability
Impact
Mitigation
Determinism breaks on console   Medium  Critical        Start console testing Week 2; fixed-point fallback
Skeletal sim too expensive      Medium  High    Profile early; fallback to simplified ragdoll per-bone
GJK/EPA edge cases cause explosions     High    Medium  Robust tolerances; fallback to separation-only mode
Designer can't tune "feel"      Medium  High    Invest heavily in Phase 3 tooling; daily playtests
Mesh collision too slow for 22 chars    Low     Critical        Aggressive LOD; simplified collision for distant pairs
Featherstone ABA unstable for complex poses     Medium  High    Position-level post-correction; iterative constraint solve
Team doesn't understand physics math    Medium  Medium  Documentation; pair programming; reference implementations

7. Team Structure
Core Team (Phase 0-4)
Role
Count
Focus
Lead Developer (me)     1       Architecture, skeletal dynamics, determinism
Physics Engineer        2       Collision, constraints, solver
Tools Engineer  1       Authoring tools, visual debugging, designer UX
Gameplay Engineer       1       "Impact League" integration, sport rules
QA/DevOps       1       Determinism testing, CI, performance benchmarks

Extended (Phase 5)
Role
Count
Focus
Documentation   1       API docs, integration guides
Support Engineer        1       Help other teams integrate APC

8. Success Metrics
Technical KPIs
Determinism: 100% frame-match across 1M frames, 3 platforms
Performance: 5ms budget at 22 characters, 240Hz internal
Stability: Zero "physics explosions" in 1000-hour soak test
Memory: <2MB total physics allocation per frame
Product KPIs ("Impact League")
Feel: 80%+ playtest rating on "impacts feel satisfying"
Consistency: <5% variance in tackle outcomes at same inputs
Visual: Zero obvious interpenetration in shipped footage
Middleware KPIs (Phase 5)
Integration Time: <2 weeks for new project
API Surface: <50 public functions
Build Time: <30 seconds from source
9. Open Technical Questions
These need resolution in first two weeks:

Fixed-point vs. Strict Float: Do we need fixed-point for mobile future, or is strict FP sufficient?
→ Prototype both, measure perf delta, decide Week 2
Solver Choice: Sequential Impulse (Box2D-style) vs. Direct (Jacobson)?
→ SI is more proven for games; direct is more accurate for skeletons. Likely hybrid.
Animation Pipeline Coupling: How tightly does APC hook into the animation system?
→ Need to define interface with anim team before Phase 2
Mesh LOD Strategy: When do we swap from full mesh to capsule?
→ Distance + velocity heuristic; need profiling to set thresholds
Ball Physics Exception: Sports balls need special handling (spin Magnus effect, "catch assist"). Separate subsystem or same solver?
→ Same solver, additional constraint types
10. Immediate Next Actions
text

TODAY:
□ Review this doc with team
□ Stakeholder sign-off on 60-week timeline
□ Secure console devkit access

THIS WEEK:
□ Set up CI with cross-platform determinism tests
□ Create repro repo with math lib skeleton
□ Schedule collision deep-dive with physics engineers

NEXT WEEK:
□ First determinism baseline (sphere sim)
□ Risk investigation: console FP behavior
□ Kick off V-HACD integration spike
This document will be updated weekly during Phase 0 and bi-weekly thereafter. All changes tracked in project wiki.

— Lead Developer, APC Project