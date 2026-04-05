# DSPE Bug Fix Task List

## Test Results Summary
**Failed Tests:** 4/8
**Passed Tests:** 4/8

---

## Critical Bugs (Phase 1 Blockers)

### 1. [CRITICAL] Energy Conservation - Ball Never Bounces
**Test:** `energy_conservation`
**File:** `tests/test_phase1.cpp:135`
**Symptom:** Ball dropped from 10m never bounces, `bounced` flag stays false

**Root Cause Analysis:**
- Ground plane created at y=-0.5m with top face at y=0 in `world.cpp:18-24`
- Ball starts at y=10m, falls under gravity
- Issue: Ground material is `MAT_DRY_GRASS` but ball uses `MAT_DRY_GRASS` too
- Combined restitution = sqrt(0.80 * 0.80) = 0.80 ✓ (correct)
- **Problem:** Narrow phase may not be detecting sphere-box contact correctly, OR constraint solver not applying bounce impulse

**Investigation Steps:**
1. Check if ball-ground contact manifold is being generated
2. Verify constraint solver applies normal impulse for bouncing
3. Check if ball is going to sleep before hitting ground
4. Verify ground box collider is marked as STATIC

**Files to Check:**
- `src/systems/narrow_phase.cpp` - sphere_box contact generation
- `src/systems/constraint_solver.cpp` - normal impulse application
- `src/world.cpp` - ground creation (line 18-24)

**Fix Priority:** P0 - Blocks all ground collision physics

---

### 2. [CRITICAL] CCD Tunnelling - Ball Passes Through Goalpost
**Test:** `ccd_no_tunnel`
**File:** `tests/test_phase1.cpp:179`
**Symptom:** Ball at 40 m/s tunnels through goalpost capsule, reaches x=2.488m (>2.2m threshold)

**Root Cause Analysis:**
- Goalpost: static box at x=2, y=[0,2.44], dimensions (0.06, 1.22, 0.06)
- Ball: starts at x=0, velocity 40 m/s in +X direction
- CCD should detect collision before ball passes post
- **Problem:** Test creates goalpost with `create_static_box()` which creates a BOX collider
- CCD sweep in `ccd.cpp` only handles CAPSULE and BOX for static objects
- `swept_sphere_box()` may have bug in slab intersection test

**Specific Issues Found:**
1. In `ccd.cpp:152-169`, `test_axis()` function has logic error:
   - Line 165: `(&normal.x)[axis] = vel.raw < 0 ? FpVel::one() : -FpVel::one();`
   - This writes to normal components using pointer arithmetic - error prone
2. Line 167: `t_max = t_max < t2 ? t_max : t2;` - should be `t_max = t_max > t2 ? t2 : t_max;`
3. CCD hit detection runs BEFORE integration but position correction happens AFTER
4. Ball entity ID is `BALL_ENTITY` (30), goalpost is in ENV range (31-63)

**Files to Fix:**
- `src/systems/ccd.cpp` - swept_sphere_box() slab test logic
- `src/world.cpp` - verify goalpost creation uses correct shape

**Fix Priority:** P0 - Critical for ball-goalpost, player-player fast collisions

---

### 3. [CRITICAL] Stability Stack - Solver Explosion
**Test:** `stability_stack`
**File:** `tests/test_phase1.cpp:227`
**Symptom:** Player positions explode to >20m after 10s simulation

**Root Cause Analysis:**
- 30 players stacked vertically at y = 0, 0.2, 0.4, ... 5.8m
- After 600 ticks (10s), max position extent = 2974.27m (way over 20m limit)
- **Problem:** Constraint solver producing explosive impulses

**Potential Causes:**
1. **Effective mass calculation error** - division by zero or tiny inv_mass
2. **Baumgarte bias too aggressive** - β=0.2 with dt_inv=240 creates huge position correction
3. **Warm-start scale wrong** - cached impulses amplifying instead of damping
4. **Joint constraint fighting contact constraints** - skeleton motors pushing against stack
5. **Friction cone not clamping properly** - tangential impulses unbounded

**Code Review Findings:**
- `constraint_solver.cpp:218`: `total_t_sq` computed but never used (compiler warning)
- Friction cone clamp at line 222-227 happens AFTER individual tangent clamps
- Joint constraints use stiffness=500 N·m/rad, damping=40 N·m·s/rad - may be too high

**Files to Fix:**
- `src/systems/constraint_solver.cpp` - effective mass, Baumgarte bias, friction cone
- `src/systems/skeleton.cpp` - motor torque limits

**Fix Priority:** P0 - 30-player stability is core requirement

---

### 4. [HIGH] Friction Cone - Player Never Stops Sliding
**Test:** `friction_cone`
**File:** `tests/test_phase1.cpp:322`
**Symptom:** Player sliding on wet grass (μ=0.35) never comes to stop

**Root Cause Analysis:**
- Test sets `wet.wetness = 0.0f` (fully wet) → selects MAT_WET_GRASS
- Expected kinetic μ = 0.35, stopping distance ~9.32m from 8 m/s
- Player slides indefinitely

**Root Cause Found in Code:**
In `integrator.cpp:282`:
```cpp
if (input.jump && sk.left_foot_grounded || sk.right_foot_grounded) {
```
**BUG:** Missing parentheses! Should be:
```cpp
if (input.jump && (sk.left_foot_grounded || sk.right_foot_grounded)) {
```
Current logic: `(input.jump && sk.left_foot_grounded) || sk.right_foot_grounded`
This means jump fires whenever right foot is grounded, regardless of input.jump

**Friction-Specific Issues:**
1. `integrator.cpp:256-260`: Surface material selection logic
   - Uses `surface.wetness < 0.5f` to choose MAT_WET_GRASS vs MAT_DRY_GRASS
   - But test sets wetness=0.0 expecting wet grass - this works correctly
2. Player capsule may not be contacting ground properly (see Bug #1)
3. Friction impulse may not be applied to non-ball entities

**Files to Fix:**
- `src/systems/integrator.cpp:282` - Add parentheses for jump condition
- `src/systems/integrator.cpp` - Verify friction force accumulation for players
- `src/systems/constraint_solver.cpp` - Verify friction cone applied to player-ground contacts

**Fix Priority:** P1 - Affects player movement realism

---

## Compiler Warnings to Fix

### 5. [LOW] Unused Variable in Constraint Solver
**File:** `src/systems/constraint_solver.cpp:218`
**Warning:** `variable 'total_t_sq' set but not used`
**Fix:** Remove line 218 or use the variable for debugging

### 6. [LOW] Operator Precedence Warning
**File:** `src/systems/integrator.cpp:282`
**Warning:** `suggest parentheses around '&&' within '||'`
**Fix:** Add parentheses as shown in Bug #4

---

## Test Coverage Gaps

### Missing Tests (Not in Brief but Recommended)
- [ ] Ground contact manifold verification
- [ ] Capsule-box narrow phase test
- [ ] Joint motor torque limit test
- [ ] Sleep/wake transition test
- [ ] Multiple substep convergence test

---

## Fix Implementation Order

1. **P0 - Fix integrator.cpp:282 parentheses** (5 min)
2. **P0 - Debug ground collision** - add debug prints to narrow_phase sphere_box
3. **P0 - Fix CCD swept_sphere_box** slab intersection logic
4. **P0 - Investigate constraint solver explosion** - check effective mass calculation
5. **P1 - Verify friction application** for player entities
6. **P2 - Clean up compiler warnings**

---

## Verification Checklist

After fixes, re-run:
```bash
cd build && make -j4 && ./dspe_tests
```

Expected output:
```
Results: 8/8 passed
```

All tests must pass with no compiler warnings.

---

## Notes

- CMake version changed from 3.26 to 3.25 for compatibility
- Build system working correctly
- Determinism test passes - good sign for fixed-point math
- Magnus effect test passes - ball spin physics working
- Trigger test passes - goal detection working