#pragma once
// =============================================================================
// Ball Control — Dribble, trap, catch, carry, possession system
// =============================================================================
//
// Manages the interaction between athletes and balls across different sports:
//
//   FOOT CONTROL (soccer, rugby, Aussie rules):
//     - Dribble: ball kept close with periodic touches
//     - Trap/Control: bring ball to rest at feet
//     - Shield: body between ball and opponent
//
//   HAND CONTROL (basketball, handball, American football, rugby):
//     - Catch: receive ball with magnetism assist
//     - Carry: ball held against body
//     - Handle/Palming: basketball dribble with hand
//
//   POSSESSION:
//     - State machine: FREE → CONTROLLED → CARRIED → THROWN → FREE
//     - Control strength (how securely the ball is held)
//     - Dispossess mechanics (tackle → fumble probability)
//
// Design:
//   - Header-only, apc:: namespace
//   - No dynamic allocation
//   - Deterministic: fixed-order updates
//   - C++17
//
// =============================================================================

#include "apc_sport/apc_ball_physics.h"
#include "apc_math/apc_vec3.h"
#include "apc_math/apc_quat.h"
#include "apc_math/apc_math_common.h"
#include <cstdint>
#include <cmath>

namespace apc {

// =============================================================================
// ControlMethod — How the athlete controls the ball
// =============================================================================
enum class ControlMethod : uint8_t {
    NONE       = 0,    // No control (ball is free)
    FOOT       = 1,    // Foot control (soccer dribble, rugby kick-chase)
    HAND       = 2,    // Hand control (basketball carry, handball)
    BOTH       = 3,    // Both hands (American football carry, rugby carry)
    CATCH_ASSIST = 4,  // Magnetic catch zone (gameplay assist)
    HEAD       = 5,    // Heading (soccer, volleyball)
    CHEST      = 6,    // Chest control (soccer)
    EQUIPMENT  = 7     // Via implement (bat, racket, stick)
};

// =============================================================================
// ControlState — Ball possession state machine
// =============================================================================
enum class ControlState : uint8_t {
    FREE       = 0,    // Ball is uncontrolled (in play, on ground, or in air)
    DRIBBLED   = 1,    // Ball is being dribbled (foot or hand)
    CONTROLLED = 2,    // Ball is held/stationary at athlete's control point
    CARRIED    = 3,    // Ball is carried against athlete's body
    TRAPPED    = 4,    // Ball is trapped/pinned (held on ground)
    IN_FLIGHT  = 5,    // Ball was thrown/kicked and is in flight (no control)
    FUMBLING   = 6,    // Ball is in process of being lost (tackle impact)
    DEAD       = 7     // Ball is out of play (out of bounds, whistle)
};

// =============================================================================
// ControlPoint — Where on the body the ball is controlled from
// =============================================================================
enum class ControlPoint : uint8_t {
    RIGHT_FOOT     = 0,
    LEFT_FOOT      = 1,
    RIGHT_HAND     = 2,
    LEFT_HAND      = 3,
    BOTH_HANDS     = 4,
    CHEST          = 5,
    HEAD           = 6,
    THIGH          = 7,   // Soccer thigh trap
    EQUIPMENT      = 8     // Bat, racket, stick
};

// =============================================================================
// AthleteBallControl — Per-athlete ball control parameters
// =============================================================================
struct AthleteBallControl {
    // --- Identity ---
    uint32_t athlete_id = 0;

    // --- Control capability ---
    float foot_control_rating = 0.8f;     // 0.0-1.0 (skill level)
    float hand_control_rating = 0.8f;
    float catch_radius = 0.5f;            // Catch magnetism radius (meters)
    float control_radius = 0.3f;          // Dribble control radius (meters)

    // --- Dribble parameters ---
    float dribble_frequency = 4.0f;       // Touches per second
    float dribble_force = 15.0f;          // Force of each touch (N)
    float dribble_height = 0.15f;         // Max bounce height during dribble (m)
    float dribble_direction_error = 0.05f;// Direction scatter (rad)

    // --- Trap parameters ---
    float trap_strength = 0.7f;           // How quickly ball comes to rest (0-1)
    float trap_max_speed = 15.0f;         // Max incoming ball speed for clean trap

    // --- Carry parameters ---
    float carry_offset_y = 0.5f;          // Ball height when carried (m)
    float carry_offset_z = 0.3f;          // Ball forward offset when carried
    float carry_angular_damping = 0.8f;   // Spin decay when carried

    // --- Catch parameters ---
    float catch_reaction_time = 0.15f;    // Seconds to complete catch
    float catch_success_base = 0.9f;      // Base catch probability
    float catch_speed_penalty = 0.01f;    // Probability drop per m/s of ball speed
    float drop_ball_speed_threshold = 30.0f; // Above this, catch always fails

    // --- Shield parameters ---
    float shield_strength = 0.7f;         // How well ball is protected
    float shield_radius = 0.6f;           // Effective shielding distance

    // --- Fumble parameters ---
    float fumble_base_probability = 0.1f; // Base fumble chance on big hit
    float fumble_impact_threshold = 5.0f; // Impact force to trigger fumble check
    float fumble_recovery_time = 0.5f;    // Seconds to recover a fumble
};

// =============================================================================
// PossessionRecord — Who controls which ball
// =============================================================================
struct PossessionRecord {
    uint32_t ball_id = 0xFFFFFFFF;
    uint32_t athlete_id = 0xFFFFFFFF;
    ControlState state = ControlState::FREE;
    ControlMethod method = ControlMethod::NONE;
    ControlPoint control_point = ControlPoint::RIGHT_FOOT;
    float control_strength = 0.0f;        // 0.0 = loose, 1.0 = locked
    float control_duration = 0.0f;        // How long this athlete has controlled
    float last_touch_time = 0.0f;         // Time since last meaningful touch
    uint32_t touch_count = 0;             // Total touches this possession

    bool is_controlled() const {
        return state == ControlState::DRIBBLED ||
               state == ControlState::CONTROLLED ||
               state == ControlState::CARRIED ||
               state == ControlState::TRAPPED;
    }

    bool is_free() const {
        return state == ControlState::FREE ||
               state == ControlState::IN_FLIGHT ||
               state == ControlState::DEAD;
    }
};

// =============================================================================
// PossessionSystem — Manages ball possession state machine
// =============================================================================
struct PossessionSystem {
    static constexpr uint32_t MAX_RECORDS = 64u;

    PossessionRecord records[MAX_RECORDS];
    uint32_t record_count = 0;
    float current_time = 0.0f;

    // --- Create a possession record for a ball ---
    uint32_t register_ball(uint32_t ball_id) {
        if (record_count >= MAX_RECORDS) return 0xFFFFFFFF;
        PossessionRecord& rec = records[record_count];
        rec.ball_id = ball_id;
        rec.athlete_id = 0xFFFFFFFF;
        rec.state = ControlState::FREE;
        rec.method = ControlMethod::NONE;
        rec.control_strength = 0.0f;
        rec.control_duration = 0.0f;
        rec.last_touch_time = current_time;
        rec.touch_count = 0;
        return record_count++;
    }

    // --- Get possession record for a ball ---
    PossessionRecord* get(uint32_t ball_id) {
        for (uint32_t i = 0; i < record_count; ++i) {
            if (records[i].ball_id == ball_id) return &records[i];
        }
        return nullptr;
    }

    const PossessionRecord* get(uint32_t ball_id) const {
        for (uint32_t i = 0; i < record_count; ++i) {
            if (records[i].ball_id == ball_id) return &records[i];
        }
        return nullptr;
    }

    // --- Claim possession of a ball ---
    bool claim(uint32_t ball_id, uint32_t athlete_id,
               ControlMethod method, ControlPoint point,
               float initial_strength = 0.5f)
    {
        PossessionRecord* rec = get(ball_id);
        if (!rec) return false;

        rec->athlete_id = athlete_id;
        rec->state = (method == ControlMethod::FOOT ||
                      method == ControlMethod::HEAD ||
                      method == ControlMethod::CHEST) ?
            ControlState::DRIBBLED : ControlState::CONTROLLED;
        rec->method = method;
        rec->control_point = point;
        rec->control_strength = initial_strength;
        rec->control_duration = 0.0f;
        rec->last_touch_time = current_time;
        ++rec->touch_count;
        return true;
    }

    // --- Release ball (pass, shoot, fumble) ---
    bool release(uint32_t ball_id, ControlState new_state = ControlState::IN_FLIGHT) {
        PossessionRecord* rec = get(ball_id);
        if (!rec) return false;

        rec->state = new_state;
        rec->method = ControlMethod::NONE;
        rec->control_point = ControlPoint::RIGHT_FOOT;
        rec->control_strength = 0.0f;
        return true;
    }

    // --- Check fumble on impact ---
    bool check_fumble(uint32_t ball_id, float impact_force,
                      const AthleteBallControl& control) const
    {
        const PossessionRecord* rec = get(ball_id);
        if (!rec || !rec->is_controlled()) return false;

        if (impact_force < control.fumble_impact_threshold) return false;

        // Fumble probability = base * (impact / threshold) * (1 - control_strength)
        float force_ratio = impact_force / control.fumble_impact_threshold;
        float strength_factor = 1.0f - rec->control_strength;
        float fumble_prob = control.fumble_base_probability * force_ratio * strength_factor;

        return fumble_prob > 0.5f; // Deterministic threshold
    }

    // --- Update possession timers ---
    void update(float dt) {
        current_time += dt;
        for (uint32_t i = 0; i < record_count; ++i) {
            PossessionRecord& rec = records[i];
            if (rec.is_controlled()) {
                rec.control_duration += dt;
                rec.control_strength = std::min(1.0f,
                    rec.control_strength + dt * 0.5f); // Strength grows over time
            }
        }
    }

    void reset() {
        record_count = 0;
        current_time = 0.0f;
    }
};

// =============================================================================
// DribbleState — Foot dribble simulation
// =============================================================================
struct DribbleState {
    float touch_timer = 0.0f;            // Time until next touch
    float touch_cooldown = 0.0f;         // Minimum time between touches
    Vec3 dribble_direction;              // Current dribble direction
    Vec3 target_direction;               // Athlete's movement direction
    float last_touch_force = 0.0f;

    void reset() {
        touch_timer = 0.0f;
        touch_cooldown = 0.0f;
        dribble_direction = Vec3(0.0f, 0.0f, 1.0f);
        target_direction = Vec3(0.0f, 0.0f, 1.0f);
        last_touch_force = 0.0f;
    }
};

// =============================================================================
// DribbleController — Handles foot and hand dribbling
// =============================================================================
struct DribbleController {
    // --- Foot dribble: apply periodic touches to keep ball close ---
    bool update_foot_dribble(BallState& ball, const Vec3& athlete_pos,
                              const Vec3& athlete_vel, const Vec3& athlete_forward,
                              const AthleteBallControl& control,
                              float dt, DribbleState& state)
    {
        // Compute distance from athlete to ball (horizontal)
        Vec3 to_ball = Vec3::sub(ball.body.position, athlete_pos);
        to_ball.y = 0.0f; // Ignore vertical
        float distance = Vec3::length(to_ball);

        // Control radius: ball should stay within this distance
        float max_dist = control.control_radius * 2.0f;
        if (distance > max_dist) return false; // Ball too far, lost control

        // Touch timer
        state.touch_timer -= dt;
        if (state.touch_timer > 0.0f) {
            // Between touches: apply gentle pull toward control position
            Vec3 control_pos = Vec3::add(athlete_pos,
                Vec3::scale(athlete_forward, control.carry_offset_z));
            control_pos.y = ball.config.get_effective_radius();

            Vec3 to_control = Vec3::sub(control_pos, ball.body.position);
            to_control.y = 0.0f;
            float control_dist = Vec3::length(to_control);

            if (control_dist > 0.01f) {
                Vec3 pull = Vec3::scale(
                    Vec3::scale(to_control, 1.0f / control_dist),
                    control.foot_control_rating * 5.0f * dt);
                ball.body.linear_velocity = Vec3::add(ball.body.linear_velocity, pull);
            }
            return true;
        }

        // Apply a touch
        state.touch_timer = 1.0f / control.dribble_frequency;

        // Touch direction: blend between current dribble and athlete movement
        float speed = Vec3::length(athlete_vel);
        if (speed > 0.5f) {
            state.target_direction = Vec3::scale(athlete_vel, 1.0f / speed);
        } else {
            state.target_direction = athlete_forward;
        }

        // Smooth direction change
        state.dribble_direction = Vec3::add(
            Vec3::scale(state.dribble_direction, 0.7f),
            Vec3::scale(state.target_direction, 0.3f));
        float dir_len = Vec3::length(state.dribble_direction);
        if (dir_len > APC_EPSILON) {
            state.dribble_direction = Vec3::scale(state.dribble_direction,
                1.0f / dir_len);
        }

        // Touch force
        float touch_force = control.dribble_force;

        // Increase force if ball is too far
        if (distance > control.control_radius) {
            float dist_ratio = distance / control.control_radius;
            touch_force *= dist_ratio;
        }

        // Apply direction error based on skill
        float error_angle = control.dribble_direction_error *
            (1.0f - control.foot_control_rating);
        // Rotate direction by error
        float cos_e = std::cos(error_angle);
        float sin_e = std::sin(error_angle);
        Vec3 touch_dir(
            state.dribble_direction.x * cos_e - state.dribble_direction.z * sin_e,
            0.0f,
            state.dribble_direction.x * sin_e + state.dribble_direction.z * cos_e
        );

        Vec3 impulse = Vec3::scale(touch_dir, touch_force * ball.config.mass);
        ball.body.linear_velocity = Vec3::add(ball.body.linear_velocity, impulse);

        state.last_touch_force = touch_force;
        return true;
    }

    // --- Hand dribble (basketball): bounce ball off ground with hand ---
    bool update_hand_dribble(BallState& ball, const Vec3& athlete_pos,
                              const Vec3& athlete_vel,
                              const AthleteBallControl& control,
                              float dt, DribbleState& state)
    {
        Vec3 to_ball = Vec3::sub(ball.body.position, athlete_pos);
        float distance = Vec3::length(to_ball);

        float max_dist = 1.2f; // Max hand dribble reach
        if (distance > max_dist) return false;

        state.touch_timer -= dt;
        if (state.touch_timer > 0.0f) {
            // Between bounces: ball is in the air or rolling
            // Apply very subtle guidance toward ideal bounce point
            Vec3 bounce_target = Vec3::add(athlete_pos,
                Vec3::scale(athlete_vel, 0.1f)); // Lead the bounce
            bounce_target.y = ball.config.get_effective_radius();

            Vec3 to_target = Vec3::sub(bounce_target, ball.body.position);
            float target_dist = Vec3::length(to_target);
            if (target_dist > 0.01f && target_dist < 1.0f) {
                Vec3 guide = Vec3::scale(
                    Vec3::scale(to_target, 1.0f / target_dist),
                    2.0f * dt * control.hand_control_rating);
                ball.body.linear_velocity.x += guide.x;
                ball.body.linear_velocity.z += guide.z;
            }
            return true;
        }

        // Apply hand push (ball bounces off ground and comes back up)
        state.touch_timer = 1.0f / (control.dribble_frequency * 0.6f); // Slower than foot

        // Push direction: slightly forward of athlete, with some speed
        Vec3 push_dir;
        float speed = Vec3::length(athlete_vel);
        if (speed > 1.0f) {
            push_dir = Vec3::scale(athlete_vel, 1.0f / speed);
        } else {
            push_dir = Vec3(0.0f, 0.0f, 1.0f);
        }

        // Push ball down and slightly forward for a clean bounce
        float push_force = 8.0f * ball.config.mass * control.hand_control_rating;
        Vec3 impulse(
            push_dir.x * push_force * 0.3f,
            -push_force * 0.5f, // Downward to bounce
            push_dir.z * push_force * 0.3f
        );
        ball.body.linear_velocity = Vec3::add(ball.body.linear_velocity, impulse);

        return true;
    }

    // --- Trap / First touch control: bring ball to rest ---
    bool trap_ball(BallState& ball, const Vec3& athlete_pos,
                    const Vec3& athlete_forward,
                    const AthleteBallControl& control)
    {
        float ball_speed = Vec3::length(ball.body.linear_velocity);
        if (ball_speed > control.trap_max_speed) return false;

        // Check proximity
        Vec3 to_ball = Vec3::sub(ball.body.position, athlete_pos);
        to_ball.y = 0.0f;
        float distance = Vec3::length(to_ball);
        if (distance > control.catch_radius) return false;

        // Apply deceleration based on trap strength and skill
        float decel_factor = 1.0f - control.trap_strength * control.foot_control_rating;
        ball.body.linear_velocity = Vec3::scale(ball.body.linear_velocity, decel_factor);

        // Kill spin
        ball.body.angular_velocity = Vec3::scale(ball.body.angular_velocity, 0.5f);

        // Move ball toward control position
        Vec3 control_pos = Vec3::add(athlete_pos,
            Vec3::scale(athlete_forward, 0.3f));
        control_pos.y = ball.config.get_effective_radius();

        Vec3 to_control = Vec3::sub(control_pos, ball.body.position);
        ball.body.position = Vec3::add(ball.body.position,
            Vec3::scale(to_control, 0.3f)); // Move 30% toward control position

        // If ball is slow enough, it's trapped
        if (Vec3::length(ball.body.linear_velocity) < 0.5f) {
            ball.body.linear_velocity = Vec3(
                athlete_forward.x * 0.5f,  // Slight forward momentum
                0.0f,
                athlete_forward.z * 0.5f
            );
            ball.body.angular_velocity = Vec3(0.0f, 0.0f, 0.0f);
            return true; // Successfully trapped
        }
        return false; // Still settling
    }
};

// =============================================================================
// CatchController — Hand catching with magnetism assist
// =============================================================================
struct CatchResult {
    bool caught = false;
    float catch_quality = 0.0f;   // 0.0-1.0 (perfect catch = 1.0)
    Vec3 catch_offset;            // Offset from ideal catch point
};

struct CatchController {
    // --- Attempt to catch a ball ---
    CatchResult attempt_catch(BallState& ball, const Vec3& hand_pos,
                               const Vec3& hand_vel,
                               const AthleteBallControl& control,
                               float current_time)
    {
        CatchResult result;
        result.caught = false;
        result.catch_quality = 0.0f;

        Vec3 to_ball = Vec3::sub(ball.body.position, hand_pos);
        float distance = Vec3::length(to_ball);

        // Out of catch range
        if (distance > control.catch_radius) return result;

        // Ball speed check
        float ball_speed = Vec3::length(ball.body.linear_velocity);
        if (ball_speed > control.drop_ball_speed_threshold) return result;

        // Catch success probability (deterministic)
        float speed_penalty = ball_speed * control.catch_speed_penalty;
        float catch_prob = control.catch_success_base - speed_penalty;
        catch_prob *= control.hand_control_rating;

        // Distance bonus: closer = easier
        if (distance < control.catch_radius * 0.5f) {
            catch_prob += 0.1f;
        }

        // Direction alignment: hand moving toward ball helps
        float hand_speed = Vec3::length(hand_vel);
        if (hand_speed > 0.5f) {
            Vec3 hand_dir = Vec3::scale(hand_vel, 1.0f / hand_speed);
            Vec3 ball_dir = Vec3::scale(to_ball, 1.0f / (distance + 0.01f));
            float alignment = Vec3::dot(hand_dir, ball_dir);
            if (alignment > 0.3f) {
                catch_prob += 0.1f * alignment;
            }
        }

        // Magnetism: if within close range, pull ball toward hand
        float magnetism_radius = control.catch_radius * 0.4f;
        if (distance < magnetism_radius && catch_prob > 0.3f) {
            // Smoothly guide ball to hand position
            float magnetism_strength = 1.0f - (distance / magnetism_radius);
            Vec3 pull = Vec3::scale(to_ball, -magnetism_strength * 20.0f *
                (1.0f / 240.0f)); // Frame-rate independent
            ball.body.linear_velocity = Vec3::add(ball.body.linear_velocity, pull);

            // Also dampen ball speed toward hand
            ball.body.linear_velocity = Vec3::scale(ball.body.linear_velocity,
                0.95f); // Quick dampening
        }

        // Deterministic catch check
        result.caught = catch_prob > 0.5f;
        if (result.caught) {
            result.catch_quality = std::min(1.0f, catch_prob);

            // Stop ball relative to hand
            ball.body.linear_velocity = Vec3::scale(hand_vel, 0.5f);
            ball.body.angular_velocity = Vec3::scale(ball.body.angular_velocity,
                control.carry_angular_damping);

            // Snap ball to hand position
            ball.body.position = Vec3::add(hand_pos,
                Vec3(0.0f, ball.config.get_effective_radius(), 0.0f));
        }

        return result;
    }

    // --- Carry ball: attach ball to athlete ---
    void carry_ball(BallState& ball, const Vec3& athlete_pos,
                     const Vec3& athlete_forward,
                     const AthleteBallControl& control)
    {
        // Position ball at carry offset
        Vec3 carry_pos = Vec3::add(athlete_pos, Vec3(
            0.0f,
            control.carry_offset_y,
            control.carry_offset_z
        ));

        // Smooth interpolation to carry position
        float blend = 10.0f * (1.0f / 240.0f); // Frame-rate independent
        ball.body.position = Vec3::add(
            Vec3::scale(ball.body.position, 1.0f - blend),
            Vec3::scale(carry_pos, blend)
        );

        // Match athlete velocity
        ball.body.linear_velocity = Vec3::scale(athlete_forward,
            Vec3::length(ball.body.linear_velocity) * 0.9f);

        // Kill spin when carrying
        ball.body.angular_velocity = Vec3::scale(ball.body.angular_velocity,
            control.carry_angular_damping);

        ball.on_ground = false;
    }
};

} // namespace apc
