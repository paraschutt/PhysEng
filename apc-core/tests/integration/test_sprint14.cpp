// =============================================================================
// Sprint 14 Tests — Ball Control & Passing
// =============================================================================
//
// Tests for Phase 3 Sprint 14:
//   1. AthleteBallControl default construction
//   2. PossessionSystem: register, claim, release, fumble, update
//   3. DribbleController: foot and hand dribble
//   4. DribbleController::trap_ball
//   5. CatchController::attempt_catch
//   6. KickExecutor: instep, side_foot, chip, curve profiles
//   7. ThrowExecutor: basketball chest pass
//   8. TrajectoryPredictor: parabolic prediction, position_at, time_to_height
//
// =============================================================================

#include "apc_sport/apc_ball_physics.h"
#include "apc_sport/apc_ball_control.h"
#include "apc_sport/apc_pass_kick.h"
#include <cassert>
#include <cmath>
#include <cstdio>

int main() {
    printf("Sprint 14: Ball Control & Passing tests\n");
    printf("========================================\n\n");

    // =========================================================================
    // TEST 1: AthleteBallControl — default construction
    // =========================================================================
    {
        printf("  [Test 1] AthleteBallControl defaults...\n");

        apc::AthleteBallControl ctrl;

        assert(ctrl.foot_control_rating > 0.0f);
        assert(ctrl.hand_control_rating > 0.0f);
        assert(ctrl.catch_radius > 0.0f);
        assert(ctrl.control_radius > 0.0f);
        assert(ctrl.dribble_frequency > 0.0f);
        assert(ctrl.dribble_force > 0.0f);
        assert(ctrl.trap_strength > 0.0f);
        assert(ctrl.fumble_impact_threshold > 0.0f);
        assert(ctrl.fumble_base_probability >= 0.0f);

        printf("    [PASS] foot_ctrl=%.2f hand_ctrl=%.2f catch_r=%.2f ctrl_r=%.2f\n",
               ctrl.foot_control_rating, ctrl.hand_control_rating,
               ctrl.catch_radius, ctrl.control_radius);
    }

    // =========================================================================
    // TEST 2: PossessionSystem — register, claim, release, fumble, update
    // =========================================================================
    {
        printf("  [Test 2] PossessionSystem...\n");

        apc::PossessionSystem poss;

        // 2a. Register 2 balls
        uint32_t r0 = poss.register_ball(0);
        uint32_t r1 = poss.register_ball(1);
        assert(r0 != 0xFFFFFFFF);
        assert(r1 != 0xFFFFFFFF);
        assert(poss.record_count == 2);

        // 2b. Verify initial state is FREE
        const apc::PossessionRecord* rec0 = poss.get(0);
        assert(rec0 != nullptr);
        assert(rec0->state == apc::ControlState::FREE);
        assert(rec0->ball_id == 0);

        // 2c. Claim ball 0 with athlete 0, FOOT control
        bool claimed = poss.claim(0, 0, apc::ControlMethod::FOOT,
                                  apc::ControlPoint::RIGHT_FOOT, 0.5f);
        assert(claimed);
        assert(rec0->state == apc::ControlState::DRIBBLED ||
               rec0->state == apc::ControlState::CONTROLLED);
        assert(rec0->athlete_id == 0);
        assert(rec0->method == apc::ControlMethod::FOOT);
        assert(rec0->touch_count == 1);

        printf("    [PASS] Claimed ball 0: state=%d athlete=%u\n",
               (int)rec0->state, rec0->athlete_id);

        // 2d. Release ball 0 → IN_FLIGHT
        bool released = poss.release(0);
        assert(released);
        assert(rec0->state == apc::ControlState::IN_FLIGHT);
        assert(rec0->method == apc::ControlMethod::NONE);
        assert(rec0->control_strength == 0.0f);

        printf("    [PASS] Released ball 0: state=IN_FLIGHT\n");

        // 2e. Fumble check: set up controlled ball, apply high impact
        // Re-claim with low initial_strength so fumble is more likely
        poss.claim(1, 1, apc::ControlMethod::BOTH,
                   apc::ControlPoint::BOTH_HANDS, 0.1f);

        apc::AthleteBallControl ctrl;
        ctrl.fumble_impact_threshold = 5.0f;
        ctrl.fumble_base_probability = 0.1f;

        const apc::PossessionRecord* rec1 = poss.get(1);
        assert(rec1 != nullptr);
        assert(rec1->is_controlled());

        // High impact force should trigger fumble
        bool fumbled = poss.check_fumble(1, 100.0f, ctrl);
        assert(fumbled);

        printf("    [PASS] Fumble triggered at impact=100.0 (strength=%.2f)\n",
               rec1->control_strength);

        // Low impact should NOT trigger fumble
        bool no_fumble = poss.check_fumble(1, 3.0f, ctrl);
        assert(!no_fumble);

        printf("    [PASS] No fumble at impact=3.0 (below threshold 5.0)\n");

        // 2f. update() advances time and increases control_strength
        float t_before = poss.current_time;
        float str_before = rec1->control_strength;
        poss.update(0.5f);
        assert(poss.current_time > t_before);
        // Control strength should increase for controlled records
        // rec1 is still controlled (BOTH_HANDS claim)
        assert(poss.get(1)->control_strength >= str_before);

        printf("    [PASS] update(0.5): time=%.2f control_strength=%.2f->%.2f\n",
               poss.current_time, str_before, poss.get(1)->control_strength);
    }

    // =========================================================================
    // TEST 3: DribbleController — foot dribble
    // =========================================================================
    {
        printf("  [Test 3] DribbleController (foot)...\n");

        apc::BallState ball;
        ball.config = apc::BallFactory::make_soccer();
        ball.reset();
        ball.body.position = apc::Vec3(0.0f, ball.config.get_effective_radius(), 0.2f);
        ball.body.linear_velocity = apc::Vec3(0.0f, 0.0f, 0.0f);

        apc::AthleteBallControl ctrl;
        apc::DribbleState state;
        state.reset();

        apc::Vec3 athlete_pos(0.0f, 0.0f, 0.0f);
        apc::Vec3 athlete_vel(0.0f, 0.0f, 1.0f);   // Moving forward
        apc::Vec3 athlete_forward(0.0f, 0.0f, 1.0f);
        float dt = 1.0f / 60.0f;

        // Ball at (0, r, 0.2), athlete at origin. Horizontal dist = 0.2.
        // max_dist = control_radius * 2.0 = 0.6. 0.2 < 0.6 → in range.
        apc::DribbleController dribble;

        bool result = dribble.update_foot_dribble(
            ball, athlete_pos, athlete_vel, athlete_forward,
            ctrl, dt, state);
        assert(result); // Ball in range → returns true

        printf("    [PASS] Foot dribble: ball in range, result=%s\n",
               result ? "true" : "false");

        // Multiple updates to simulate dribble touches
        bool all_ok = true;
        for (int i = 0; i < 120; ++i) {
            bool r = dribble.update_foot_dribble(
                ball, athlete_pos, athlete_vel, athlete_forward,
                ctrl, dt, state);
            if (!r) { all_ok = false; break; }
        }
        assert(all_ok);

        // Ball should have moved (gained velocity from touches)
        float ball_speed = apc::Vec3::length(ball.body.linear_velocity);
        assert(ball_speed > 0.0f);

        printf("    [PASS] 120 dribble updates OK, ball speed=%.2f m/s\n", ball_speed);
    }

    // =========================================================================
    // TEST 3b: DribbleController — hand dribble
    // =========================================================================
    {
        printf("  [Test 3b] DribbleController (hand)...\n");

        apc::BallState ball;
        ball.config = apc::BallFactory::make_basketball();
        ball.reset();
        // Place ball near athlete (within 1.2m max hand dribble reach)
        ball.body.position = apc::Vec3(0.0f, ball.config.get_effective_radius(), 0.3f);
        ball.body.linear_velocity = apc::Vec3(0.0f, 0.0f, 0.0f);

        apc::AthleteBallControl ctrl;
        apc::DribbleState state;
        state.reset();

        apc::Vec3 athlete_pos(0.0f, 0.0f, 0.0f);
        apc::Vec3 athlete_vel(0.0f, 0.0f, 1.5f);
        float dt = 1.0f / 60.0f;

        apc::DribbleController dribble;

        bool result = dribble.update_hand_dribble(
            ball, athlete_pos, athlete_vel, ctrl, dt, state);
        assert(result); // Ball within 1.2m range

        // Multiple updates
        bool all_ok = true;
        for (int i = 0; i < 120; ++i) {
            bool r = dribble.update_hand_dribble(
                ball, athlete_pos, athlete_vel, ctrl, dt, state);
            if (!r) { all_ok = false; break; }
        }
        assert(all_ok);

        printf("    [PASS] Hand dribble: 120 updates OK\n");
    }

    // =========================================================================
    // TEST 4: DribbleController::trap_ball
    // =========================================================================
    {
        printf("  [Test 4] DribbleController::trap_ball...\n");

        apc::BallState ball;
        ball.config = apc::BallFactory::make_soccer();
        ball.reset();
        // Place ball near athlete with low speed (1 m/s)
        ball.body.position = apc::Vec3(0.2f, ball.config.get_effective_radius(), 0.1f);
        ball.body.linear_velocity = apc::Vec3(0.5f, 0.0f, 0.86f); // 1.0 m/s
        ball.body.angular_velocity = apc::Vec3(5.0f, 0.0f, 0.0f);

        apc::AthleteBallControl ctrl;
        apc::Vec3 athlete_pos(0.0f, 0.0f, 0.0f);
        apc::Vec3 athlete_forward(0.0f, 0.0f, 1.0f);

        apc::DribbleController dribble;

        // First trap call may not fully trap (ball still settling)
        // Call multiple times to allow ball speed to decrease below 0.5
        bool trapped = false;
        float speed_before = apc::Vec3::length(ball.body.linear_velocity);
        for (int i = 0; i < 10; ++i) {
            trapped = dribble.trap_ball(ball, athlete_pos, athlete_forward, ctrl);
            if (trapped) break;
        }

        float speed_after = apc::Vec3::length(ball.body.linear_velocity);

        // Either ball is fully trapped (returns true), or it made progress
        // (speed decreased significantly)
        bool progress = (trapped) || (speed_after < speed_before * 0.5f);
        assert(progress);

        printf("    [PASS] trap_ball: trapped=%s speed=%.2f->%.2f\n",
               trapped ? "true" : "false", speed_before, speed_after);
    }

    // =========================================================================
    // TEST 5: CatchController::attempt_catch
    // =========================================================================
    {
        printf("  [Test 5] CatchController::attempt_catch...\n");

        apc::BallState ball;
        ball.config = apc::BallFactory::make_basketball();
        ball.reset();
        // Place ball near hand position (within catch_radius = 0.5m)
        ball.body.position = apc::Vec3(0.1f, 1.0f, 0.1f);
        ball.body.linear_velocity = apc::Vec3(0.0f, 0.0f, 2.0f); // Low speed

        apc::AthleteBallControl ctrl;
        apc::Vec3 hand_pos(0.0f, 1.0f, 0.0f);
        apc::Vec3 hand_vel(0.0f, 0.0f, 0.5f); // Moving toward ball slightly

        apc::CatchController catcher;
        apc::CatchResult result = catcher.attempt_catch(
            ball, hand_pos, hand_vel, ctrl, 0.0f);

        // Ball is close (0.14m < catch_radius=0.5), low speed (2 m/s)
        // catch_prob = (0.9 - 0.02) * 0.8 = 0.704, plus distance bonus → > 0.5
        assert(result.caught);
        assert(result.catch_quality > 0.0f);

        // After catch, ball should be at hand position (with ball radius offset)
        float dist_to_hand = apc::Vec3::length(
            apc::Vec3::sub(ball.body.position, hand_pos));
        assert(dist_to_hand < 0.5f); // Reasonably close to hand

        printf("    [PASS] caught=%s quality=%.2f dist_to_hand=%.3f\n",
               result.caught ? "true" : "false", result.catch_quality, dist_to_hand);
    }

    // =========================================================================
    // TEST 6: KickExecutor — instep kick
    // =========================================================================
    {
        printf("  [Test 6] KickExecutor (instep)...\n");

        apc::BallState ball;
        ball.config = apc::BallFactory::make_soccer();
        ball.reset();
        // Ball at athlete's feet
        ball.body.position = apc::Vec3(0.0f, ball.config.get_effective_radius(), 0.3f);

        apc::KickProfile profile = apc::KickProfile::make_instep();
        apc::KickExecutor kicker;

        apc::Vec3 athlete_pos(0.0f, 0.0f, 0.0f);
        apc::Vec3 athlete_forward(0.0f, 0.0f, 1.0f);
        apc::Vec3 kick_dir(0.0f, 0.0f, 1.0f);

        apc::PassResult result = kicker.execute(
            ball, profile, athlete_pos, athlete_forward, kick_dir, 0.8f);

        assert(result.executed);
        assert(result.speed > 10.0f);

        // Verify ball has forward component
        float fwd = apc::Vec3::dot(ball.body.linear_velocity,
                                    apc::Vec3(0.0f, 0.0f, 1.0f));
        assert(fwd > 0.0f);

        printf("    [PASS] Instep: speed=%.2f m/s fwd=%.2f angle=%.3f rad\n",
               result.speed, fwd, result.launch_angle);
    }

    // =========================================================================
    // TEST 6b: KickExecutor — side_foot (more accurate than instep)
    // =========================================================================
    {
        printf("  [Test 6b] KickExecutor (side_foot vs instep)...\n");

        apc::BallState ball1, ball2;
        ball1.config = apc::BallFactory::make_soccer(); ball1.reset();
        ball2.config = apc::BallFactory::make_soccer(); ball2.reset();
        ball1.body.position = apc::Vec3(0.0f, 0.11f, 0.3f);
        ball2.body.position = apc::Vec3(0.0f, 0.11f, 0.3f);

        apc::KickProfile instep = apc::KickProfile::make_instep();
        apc::KickProfile side_foot = apc::KickProfile::make_side_foot();

        apc::KickExecutor kicker;
        apc::Vec3 pos(0, 0, 0);
        apc::Vec3 fwd(0, 0, 1);
        apc::Vec3 dir(0, 0, 1);

        apc::PassResult r1 = kicker.execute(ball1, instep, pos, fwd, dir, 0.5f);
        apc::PassResult r2 = kicker.execute(ball2, side_foot, pos, fwd, dir, 0.5f);

        assert(r1.executed);
        assert(r2.executed);

        // Side foot should have less accuracy error (more accurate)
        assert(r2.accuracy_error <= r1.accuracy_error);

        printf("    [PASS] side_foot error=%.4f <= instep error=%.4f\n",
               r2.accuracy_error, r1.accuracy_error);
    }

    // =========================================================================
    // TEST 6c: KickExecutor — chip has higher launch_angle than instep
    // =========================================================================
    {
        printf("  [Test 6c] KickExecutor (chip vs instep launch angle)...\n");

        apc::BallState ball1, ball2;
        ball1.config = apc::BallFactory::make_soccer(); ball1.reset();
        ball2.config = apc::BallFactory::make_soccer(); ball2.reset();
        ball1.body.position = apc::Vec3(0.0f, 0.11f, 0.3f);
        ball2.body.position = apc::Vec3(0.0f, 0.11f, 0.3f);

        apc::KickProfile instep = apc::KickProfile::make_instep();
        apc::KickProfile chip = apc::KickProfile::make_chip();

        apc::KickExecutor kicker;
        apc::Vec3 pos(0, 0, 0);
        apc::Vec3 fwd(0, 0, 1);
        apc::Vec3 dir(0, 0, 1);

        apc::PassResult r1 = kicker.execute(ball1, instep, pos, fwd, dir, 0.6f);
        apc::PassResult r2 = kicker.execute(ball2, chip, pos, fwd, dir, 0.6f);

        assert(r1.executed);
        assert(r2.executed);

        // Chip should have higher launch angle
        assert(r2.launch_angle > r1.launch_angle);

        printf("    [PASS] chip angle=%.3f rad > instep angle=%.3f rad\n",
               r2.launch_angle, r1.launch_angle);
    }

    // =========================================================================
    // TEST 6d: KickExecutor — curve has more spin than instep
    // =========================================================================
    {
        printf("  [Test 6d] KickExecutor (curve vs instep spin)...\n");

        apc::BallState ball1, ball2;
        ball1.config = apc::BallFactory::make_soccer(); ball1.reset();
        ball2.config = apc::BallFactory::make_soccer(); ball2.reset();
        ball1.body.position = apc::Vec3(0.0f, 0.11f, 0.3f);
        ball2.body.position = apc::Vec3(0.0f, 0.11f, 0.3f);

        apc::KickProfile instep = apc::KickProfile::make_instep();
        apc::KickProfile curve = apc::KickProfile::make_curve();

        apc::KickExecutor kicker;
        apc::Vec3 pos(0, 0, 0);
        apc::Vec3 fwd(0, 0, 1);
        apc::Vec3 dir(0, 0, 1);

        apc::PassResult r1 = kicker.execute(ball1, instep, pos, fwd, dir, 0.7f);
        apc::PassResult r2 = kicker.execute(ball2, curve, pos, fwd, dir, 0.7f);

        assert(r1.executed);
        assert(r2.executed);

        // Curve should apply more spin (spin_rate_factor: curve=2.5 vs instep=0.5)
        assert(r2.spin_rate > r1.spin_rate);

        printf("    [PASS] curve spin=%.2f rad/s > instep spin=%.2f rad/s\n",
               r2.spin_rate, r1.spin_rate);
    }

    // =========================================================================
    // TEST 7: ThrowExecutor — basketball chest pass
    // =========================================================================
    {
        printf("  [Test 7] ThrowExecutor (basketball)...\n");

        apc::BallState ball;
        ball.config = apc::BallFactory::make_basketball();
        ball.reset();

        apc::ThrowProfile profile = apc::ThrowProfile::make_chest_pass();
        apc::ThrowExecutor thrower;

        apc::Vec3 hand_pos(0.0f, 1.2f, 0.0f);
        apc::Vec3 hand_vel(1.0f, 0.0f, 0.0f);
        apc::Vec3 throw_dir(0.0f, 0.0f, 1.0f);

        apc::PassResult result = thrower.execute(
            ball, profile, hand_pos, hand_vel, throw_dir, 0.5f);

        assert(result.executed);
        assert(result.speed > 0.0f);

        // Ball should be at hand position after throw
        float pos_dist = apc::Vec3::length(
            apc::Vec3::sub(ball.body.position, hand_pos));
        assert(pos_dist < 1.0f); // Close to hand pos (set by executor)

        printf("    [PASS] Chest pass: speed=%.2f m/s pos_dist=%.3f\n",
               result.speed, pos_dist);
    }

    // =========================================================================
    // TEST 8: TrajectoryPredictor — parabolic trajectory
    // =========================================================================
    {
        printf("  [Test 8] TrajectoryPredictor (parabolic)...\n");

        apc::TrajectoryPrediction pred;
        apc::Vec3 pos(0.0f, 1.0f, 0.0f);
        apc::Vec3 vel(5.0f, 10.0f, 0.0f);
        float gravity = -9.81f;

        uint32_t count = pred.predict_parabolic(pos, vel, gravity, 5.0f, 0.05f);
        assert(count > 0);

        printf("    [PASS] predict_parabolic: %u points\n", count);

        // Landing position: y should be ≈ 0
        apc::Vec3 landing = pred.landing_position();
        assert(landing.y < 0.5f);  // Close to ground
        assert(landing.y >= 0.0f);

        printf("    [PASS] landing_position: (%.3f, %.3f, %.3f)\n",
               landing.x, landing.y, landing.z);

        // position_at() for t within range
        float mid_t = pred.times[count / 2];  // Mid-point time
        apc::Vec3 mid_pos = pred.position_at(mid_t);
        assert(mid_pos.y > 0.0f); // Should be above ground at mid-flight

        printf("    [PASS] position_at(t=%.3f): (%.3f, %.3f, %.3f)\n",
               mid_t, mid_pos.x, mid_pos.y, mid_pos.z);

        // time_to_height() for target_y below max height
        // Max height: y = 1 + 10^2 / (2*9.81) ≈ 6.1m
        float t_ground = pred.time_to_height(0.0f);
        assert(t_ground > 0.0f);

        float t_mid_height = pred.time_to_height(3.0f);
        assert(t_mid_height > 0.0f);
        assert(t_mid_height < t_ground); // Reaches 3m before hitting ground

        printf("    [PASS] time_to_height(0.0)=%.3f time_to_height(3.0)=%.3f\n",
               t_ground, t_mid_height);
    }

    printf("\nSprint 14: ALL TESTS PASSED\n");
    return 0;
}
