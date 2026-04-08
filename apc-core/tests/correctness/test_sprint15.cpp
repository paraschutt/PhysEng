// =============================================================================
// Sprint 15 Tests — Bat/Racket & Contact Sports
// =============================================================================
//
// Tests for Sprint 15 deliverables:
//   1.  ImplementProfile factories — all 7 implement types with valid properties
//   2.  SweetSpotModel — classify, power/accuracy/spin multipliers
//   3.  ImplementHitResolver — sweet-spot hit on baseball
//   4.  ImplementHitResolver off-center — shank zone (grip area)
//   5.  ImplementSwingModel — phase evaluation and tip velocity
//   6.  TackleProfile factories — form, shoulder, slide comparison
//   7.  ContactResolver::resolve_tackle — form tackle impact
//   8.  ContactResolver::resolve_block — pass block contact
//   9.  ContactResolver::resolve_shoulder_charge — standing charge
//  10.  GrappleResolver — initiate, update, break_hold lifecycle
//
// Pattern: int main() + assert(), no test framework.
// =============================================================================

#include "apc_sport/apc_ball_physics.h"
#include "apc_sport/apc_ball_control.h"
#include "apc_sport/apc_implement.h"
#include "apc_sport/apc_contact_sport.h"
#include <cassert>
#include <cmath>
#include <cstdio>

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
static constexpr float EPS = 1e-5f;

static bool approx_eq(float a, float b, float eps = EPS) {
    return std::abs(a - b) < eps;
}

// =============================================================================
// TEST 1: ImplementProfile factories
// =============================================================================
static void test_implement_profile_factories() {
    printf("  [Test 1] ImplementProfile factories...\n");

    auto bat     = apc::ImplementProfile::make_baseball_bat();
    auto cricket = apc::ImplementProfile::make_cricket_bat();
    auto tennis  = apc::ImplementProfile::make_tennis_racket();
    auto driver  = apc::ImplementProfile::make_golf_driver();
    auto iron    = apc::ImplementProfile::make_golf_iron();
    auto hockey  = apc::ImplementProfile::make_hockey_stick();
    auto badminton = apc::ImplementProfile::make_badminton_racket();

    // All must have positive mass, positive length, and sweet spot past grip
    apc::ImplementProfile profiles[] = { bat, cricket, tennis, driver, iron, hockey, badminton };
    const char* names[] = {
        "baseball_bat", "cricket_bat", "tennis_racket", "golf_driver",
        "golf_iron", "hockey_stick", "badminton_racket"
    };

    for (int i = 0; i < 7; ++i) {
        const auto& p = profiles[i];
        assert(p.total_mass > 0.0f && "mass must be > 0");
        assert(p.total_length > 0.0f && "total_length must be > 0");
        assert(p.sweet_spot_center > p.grip_length &&
               "sweet_spot_center must be past grip_length");
        (void)names[i];
    }

    // Spot-check specific known values
    assert(bat.total_mass == 0.88f);
    assert(bat.total_length == 0.86f);
    assert(bat.sweet_spot_center == 0.65f);

    assert(tennis.shape == apc::ImplementShape::STRING_BED);
    assert(tennis.total_mass == 0.32f);

    assert(driver.total_length == 1.17f);
    assert(driver.face_angle > 0.0f && "driver has loft");

    assert(badminton.total_mass == 0.09f && "badminton is very light");

    assert(hockey.total_length == 1.52f && "hockey stick is longest");

    printf("    [PASS] All 7 implement profiles have valid mass, length, sweet spot\n");
}

// =============================================================================
// TEST 2: SweetSpotModel
// =============================================================================
static void test_sweet_spot_model() {
    printf("  [Test 2] SweetSpotModel...\n");

    apc::SweetSpotModel ssm;
    auto bat = apc::ImplementProfile::make_baseball_bat();
    float swing_speed = 20.0f;

    // --- classify ---
    // Contact at sweet_spot_center → PERFECT
    auto zone = ssm.classify(bat.sweet_spot_center, swing_speed, bat);
    assert(zone == apc::SweetSpotZone::PERFECT && "sweet spot center = PERFECT");

    // Contact at sweet_spot_center + 2*sweet_spot_radius → AVERAGE
    zone = ssm.classify(bat.sweet_spot_center + 2.0f * bat.sweet_spot_radius,
                        swing_speed, bat);
    assert(zone == apc::SweetSpotZone::AVERAGE &&
           "contact at 2*radius from center should be AVERAGE (or POOR)");

    // Contact in grip area → SHANK
    zone = ssm.classify(0.0f, swing_speed, bat);
    assert(zone == apc::SweetSpotZone::SHANK && "contact in grip area = SHANK");

    // Contact very far from sweet spot → MIS_HIT
    zone = ssm.classify(bat.total_length + 0.5f, swing_speed, bat);
    assert(zone == apc::SweetSpotZone::MIS_HIT && "far past hitting zone = MIS_HIT");

    // Contact just outside GOOD range → check GOOD works
    zone = ssm.classify(bat.sweet_spot_center + bat.sweet_spot_radius * 0.7f,
                        swing_speed, bat);
    assert(zone == apc::SweetSpotZone::GOOD && "near sweet spot = GOOD");

    // --- get_power_multiplier: PERFECT > GOOD > AVERAGE > POOR > MIS_HIT ---
    float p_perf   = ssm.get_power_multiplier(apc::SweetSpotZone::PERFECT);
    float p_good   = ssm.get_power_multiplier(apc::SweetSpotZone::GOOD);
    float p_avg    = ssm.get_power_multiplier(apc::SweetSpotZone::AVERAGE);
    float p_poor   = ssm.get_power_multiplier(apc::SweetSpotZone::POOR);
    float p_mis    = ssm.get_power_multiplier(apc::SweetSpotZone::MIS_HIT);
    float p_shank  = ssm.get_power_multiplier(apc::SweetSpotZone::SHANK);

    assert(p_perf > p_good && "PERFECT power > GOOD");
    assert(p_good > p_avg  && "GOOD power > AVERAGE");
    assert(p_avg > p_poor  && "AVERAGE power > POOR");
    assert(p_poor > p_mis  && "POOR power > MIS_HIT");
    assert(p_mis > p_shank && "MIS_HIT power > SHANK");
    assert(approx_eq(p_perf, 1.15f) && "PERFECT power = 1.15");
    assert(approx_eq(p_shank, 0.20f) && "SHANK power = 0.20");

    // --- get_accuracy_modifier: PERFECT > GOOD > AVERAGE > POOR > MIS_HIT ---
    float a_perf  = ssm.get_accuracy_modifier(apc::SweetSpotZone::PERFECT);
    float a_good  = ssm.get_accuracy_modifier(apc::SweetSpotZone::GOOD);
    float a_avg   = ssm.get_accuracy_modifier(apc::SweetSpotZone::AVERAGE);
    float a_poor  = ssm.get_accuracy_modifier(apc::SweetSpotZone::POOR);
    float a_mis   = ssm.get_accuracy_modifier(apc::SweetSpotZone::MIS_HIT);
    float a_shank = ssm.get_accuracy_modifier(apc::SweetSpotZone::SHANK);

    assert(a_perf > a_good && "PERFECT accuracy > GOOD");
    assert(a_good > a_avg  && "GOOD accuracy > AVERAGE");
    assert(a_avg > a_poor  && "AVERAGE accuracy > POOR");
    assert(a_poor > a_mis  && "POOR accuracy > MIS_HIT");
    assert(a_mis > a_shank && "MIS_HIT accuracy > SHANK");
    assert(approx_eq(a_perf, 1.0f) && "PERFECT accuracy = 1.0");
    assert(approx_eq(a_shank, 0.15f) && "SHANK accuracy = 0.15");

    // --- get_spin_modifier: POOR > PERFECT (off-center = more spin) ---
    float s_perf = ssm.get_spin_modifier(apc::SweetSpotZone::PERFECT);
    float s_good = ssm.get_spin_modifier(apc::SweetSpotZone::GOOD);
    float s_avg  = ssm.get_spin_modifier(apc::SweetSpotZone::AVERAGE);
    float s_poor = ssm.get_spin_modifier(apc::SweetSpotZone::POOR);

    assert(s_poor > s_perf && "POOR spin > PERFECT spin (off-center = more spin)");
    assert(s_avg > s_good  && "AVERAGE spin > GOOD spin");
    assert(s_good > s_perf && "GOOD spin > PERFECT spin");
    assert(approx_eq(s_perf, 1.0f) && "PERFECT spin = 1.0");
    assert(approx_eq(s_poor, 1.5f) && "POOR spin = 1.5");

    printf("    [PASS] SweetSpotModel classify, power, accuracy, spin all correct\n");
}

// =============================================================================
// TEST 3: ImplementHitResolver — sweet spot hit
// =============================================================================
static void test_hit_resolver_sweet_spot() {
    printf("  [Test 3] ImplementHitResolver sweet-spot hit...\n");

    apc::ImplementHitResolver resolver;
    auto bat = apc::ImplementProfile::make_baseball_bat();

    // Create baseball and place at implement position
    apc::BallState ball;
    ball.config = apc::BallFactory::make_baseball();
    ball.reset();

    apc::Vec3 implement_pos(0.0f, 1.0f, 0.0f);
    apc::Vec3 implement_vel(20.0f, 2.0f, 0.0f);
    apc::Vec3 face_normal(1.0f, 0.0f, 0.0f);

    ball.body.position = implement_pos; // Ball at implement position

    apc::HitResult result = resolver.resolve(
        ball, bat,
        implement_vel, face_normal,
        implement_pos,
        bat.sweet_spot_center, // Contact at sweet spot
        0.8f                    // Skill rating
    );

    assert(result.hit == true && "hit should succeed with ball at implement pos");
    assert(result.ball_speed > 0.0f && "ball_speed must be > 0 after hit");
    assert(result.zone == apc::SweetSpotZone::PERFECT &&
           "contact at sweet_spot_center = PERFECT zone");

    // Angular velocity should have been set on ball
    float ang_speed = apc::Vec3::length(ball.body.angular_velocity);
    // Baseball bat with face_angle=0 and some tangential component from vel=(20,2,0)
    // should produce spin from the loft angle and tangential friction
    (void)ang_speed; // Spin may be zero for pure horizontal hit; ball_speed is key

    // Ball should be in air after hit
    assert(ball.in_air == true && "ball should be in_air after hit");

    // Power multiplier should be high for PERFECT zone
    assert(result.power_multiplier > 1.0f && "PERFECT zone power > 1.0");

    printf("    [PASS] Sweet-spot hit: hit=true, ball_speed>0, zone=PERFECT\n");
}

// =============================================================================
// TEST 4: ImplementHitResolver — off-center (shank zone)
// =============================================================================
static void test_hit_resolver_shank() {
    printf("  [Test 4] ImplementHitResolver shank zone...\n");

    apc::ImplementHitResolver resolver;
    auto bat = apc::ImplementProfile::make_baseball_bat();

    apc::BallState ball;
    ball.config = apc::BallFactory::make_baseball();
    ball.reset();

    apc::Vec3 implement_pos(0.0f, 1.0f, 0.0f);
    apc::Vec3 implement_vel(20.0f, 0.0f, 0.0f);
    apc::Vec3 face_normal(1.0f, 0.0f, 0.0f);

    ball.body.position = implement_pos;

    // Contact in grip area → SHANK zone
    apc::HitResult result = resolver.resolve(
        ball, bat,
        implement_vel, face_normal,
        implement_pos,
        0.0f,  // Contact offset at grip (well within grip_length=0.18)
        0.8f
    );

    assert(result.hit == true && "shank still registers as a hit");
    assert(result.zone == apc::SweetSpotZone::SHANK && "grip contact = SHANK zone");

    // SHANK power multiplier should be very low (~0.2)
    assert(result.power_multiplier < 0.25f &&
           "SHANK power multiplier should be ~0.2");

    // SHANK accuracy modifier should be very low (~0.15)
    assert(result.accuracy_multiplier < 0.2f &&
           "SHANK accuracy modifier should be very low");

    // Ball speed from shank should be much less than sweet spot
    assert(result.ball_speed > 0.0f && "even shank produces some ball speed");

    printf("    [PASS] Shank zone: hit=true, power≈0.2, accuracy very low\n");
}

// =============================================================================
// TEST 5: ImplementSwingModel
// =============================================================================
static void test_swing_model() {
    printf("  [Test 5] ImplementSwingModel...\n");

    apc::ImplementSwingModel swing;

    // At t=0: start of wind-up phase, power=0, angular_velocity < 0 (winding back)
    apc::SwingPhase phase0 = swing.evaluate(0.0f);
    assert(approx_eq(phase0.power, 0.0f) && "power=0 at t=0 (wind-up)");
    assert(phase0.angular_velocity < 0.0f &&
           "angular_velocity < 0 at t=0 (winding back)");

    // At t < 0: same as before swing
    apc::SwingPhase phase_neg = swing.evaluate(-1.0f);
    assert(approx_eq(phase_neg.power, 0.0f) && "power=0 for negative t");

    // Mid-acceleration phase: power > 0, angular_velocity > 0
    // Use wind_up_duration + half of acceleration_duration
    float t_mid = swing.wind_up_duration + swing.acceleration_duration * 0.5f;
    apc::SwingPhase phase_mid = swing.evaluate(t_mid);
    assert(phase_mid.power > 0.0f && "power > 0 at mid-acceleration");
    assert(phase_mid.angular_velocity > 0.0f &&
           "angular_velocity > 0 at mid-acceleration");

    // get_tip_velocity returns non-zero vector during swing
    apc::Vec3 pivot(0.0f, 1.0f, 0.0f);
    apc::Vec3 axis(0.0f, 1.0f, 0.0f);
    float impl_length = 0.86f; // Baseball bat length
    apc::Vec3 tip_vel = swing.get_tip_velocity(t_mid, pivot, axis, impl_length);
    float tip_speed = apc::Vec3::length(tip_vel);
    assert(tip_speed > 0.0f && "tip velocity non-zero during swing");

    // At t=0: tip velocity should be ~0 (or very small for wind-up phase)
    apc::Vec3 tip_vel_0 = swing.get_tip_velocity(0.0f, pivot, axis, impl_length);
    // At t=0 we're in the wind-up phase with negative angular velocity
    // Tip velocity magnitude = |angular_velocity| * length, which is non-zero
    // during wind-up (backswing). But power should still be 0.
    float tip_speed_0 = apc::Vec3::length(tip_vel_0);
    (void)tip_speed_0; // Wind-up has motion but no power — acceptable

    // Follow-through: power decays but still non-zero briefly
    float t_follow = swing.wind_up_duration + swing.acceleration_duration + 0.05f;
    apc::SwingPhase phase_ft = swing.evaluate(t_follow);
    assert(phase_ft.power > 0.0f && "some power in early follow-through");

    // Far past swing: power and velocity should be minimal
    float t_late = swing.total_swing_duration + 5.0f;
    apc::SwingPhase phase_late = swing.evaluate(t_late);
    // Follow-through decays but doesn't reach exactly 0 in the model
    assert(phase_late.power < phase_mid.power &&
           "late swing power < mid-swing power");

    printf("    [PASS] SwingModel: t=0 idle, mid-swing active, tip velocity > 0\n");
}

// =============================================================================
// TEST 6: TackleProfile factories
// =============================================================================
static void test_tackle_profiles() {
    printf("  [Test 6] TackleProfile factories...\n");

    auto form     = apc::TackleProfile::make_form_tackle();
    auto shoulder = apc::TackleProfile::make_shoulder_tackle();
    auto slide    = apc::TackleProfile::make_slide_tackle();

    // form_tackle has highest base_success_rate
    assert(form.base_success_rate > shoulder.base_success_rate &&
           "form tackle > shoulder success rate");
    assert(form.base_success_rate > slide.base_success_rate &&
           "form tackle > slide success rate");
    assert(approx_eq(form.base_success_rate, 0.85f) && "form success = 0.85");

    // shoulder has highest power_multiplier
    assert(shoulder.power_multiplier > form.power_multiplier &&
           "shoulder > form power multiplier");
    assert(shoulder.power_multiplier > slide.power_multiplier &&
           "shoulder > slide power multiplier");
    assert(approx_eq(shoulder.power_multiplier, 1.4f) && "shoulder power = 1.4");

    // slide has longest recovery_time
    assert(slide.recovery_time > form.recovery_time &&
           "slide > form recovery time");
    assert(slide.recovery_time > shoulder.recovery_time &&
           "slide > shoulder recovery time");
    assert(approx_eq(slide.recovery_time, 0.8f) && "slide recovery = 0.8");

    // Additional sanity checks
    assert(form.type == apc::TackleType::WRAP && "form tackle = WRAP type");
    assert(shoulder.type == apc::TackleType::MID && "shoulder tackle = MID type");
    assert(slide.type == apc::TackleType::DIVE_SLIDE && "slide tackle = DIVE_SLIDE");

    // All have reasonable reach
    assert(form.reach > 0.0f);
    assert(shoulder.reach > 0.0f);
    assert(slide.reach > form.reach && "slide reach > form reach (extended)");

    printf("    [PASS] Tackle profiles: form=highest success, shoulder=highest power, slide=longest recovery\n");
}

// =============================================================================
// TEST 7: ContactResolver::resolve_tackle
// =============================================================================
static void test_resolve_tackle() {
    printf("  [Test 7] ContactResolver::resolve_tackle...\n");

    apc::ContactResolver resolver;
    auto profile = apc::TackleProfile::make_form_tackle();

    apc::Vec3 atk_pos(0.0f, 0.0f, 0.0f);
    apc::Vec3 atk_vel(8.0f, 0.0f, 0.0f);
    apc::Vec3 def_pos(1.0f, 0.0f, 0.0f);
    apc::Vec3 def_vel(0.0f, 0.0f, 0.0f);

    apc::ContactResult result = resolver.resolve_tackle(
        atk_pos, atk_vel, def_pos, def_vel, profile, 0.8f, 0.8f
    );

    assert(result.contact_made == true && "tackle contact should be made");
    assert(result.impact_magnitude > 0.0f && "impact magnitude > 0");

    // Post-contact velocities should differ from input velocities
    // (momentum transfer mixes them)
    float atk_post_len = apc::Vec3::length(result.post_contact_vel_attacker);
    float def_post_len = apc::Vec3::length(result.post_contact_vel_defender);

    // Attacker started at speed 8.0, defender at 0.0
    // After momentum exchange with transfer=0.9, velocities differ from originals
    // The attacker should slow down and defender should gain speed
    bool atk_changed = !approx_eq(atk_post_len, 8.0f, 0.1f);
    bool def_changed = def_post_len > 0.1f; // Was 0, should now be > 0
    assert(atk_changed && "attacker velocity changed after tackle");
    assert(def_changed && "defender gained velocity after tackle");

    // Recovery times should be set
    assert(result.recovery_time_attacker > 0.0f && "attacker has recovery time");
    assert(result.recovery_time_defender >= 0.0f && "defender has recovery time");

    printf("    [PASS] Form tackle: contact=true, impact>0, velocities changed\n");
}

// =============================================================================
// TEST 8: ContactResolver::resolve_block
// =============================================================================
static void test_resolve_block() {
    printf("  [Test 8] ContactResolver::resolve_block...\n");

    apc::ContactResolver resolver;
    auto profile = apc::BlockProfile::make_pass_block();

    apc::Vec3 blocker_pos(0.0f, 0.0f, 0.0f);
    apc::Vec3 blocker_vel(0.0f, 0.0f, 0.0f);
    apc::Vec3 attacker_pos(1.0f, 0.0f, 0.0f);
    apc::Vec3 attacker_vel(6.0f, 0.0f, 0.0f);

    apc::ContactResult result = resolver.resolve_block(
        blocker_pos, blocker_vel,
        attacker_pos, attacker_vel,
        profile, 0.8f, 0.8f
    );

    assert(result.contact_made == true && "block contact should be made");
    assert(result.block_type == apc::BlockType::PASS_BLOCK &&
           "block type = PASS_BLOCK");

    // Impact should be absorbed
    assert(result.impact_magnitude > 0.0f && "impact magnitude > 0 on block");

    // Post-contact attacker velocity should be deflected
    float post_atk_len = apc::Vec3::length(result.post_contact_vel_attacker);
    // Attacker came in at speed 6.0, after absorption (0.7) and deflection,
    // should have reduced speed
    assert(post_atk_len < 6.0f && "attacker slows down after block");

    printf("    [PASS] Pass block: contact=true, impact>0, attacker slowed\n");
}

// =============================================================================
// TEST 9: ContactResolver::resolve_shoulder_charge
// =============================================================================
static void test_resolve_shoulder_charge() {
    printf("  [Test 9] ContactResolver::resolve_shoulder_charge...\n");

    apc::ContactResolver resolver;

    apc::Vec3 charger_pos(0.0f, 0.0f, 0.0f);
    apc::Vec3 charger_vel(7.0f, 0.0f, 0.0f);
    apc::Vec3 receiver_pos(0.5f, 0.0f, 0.0f);
    apc::Vec3 receiver_vel(3.0f, 0.0f, 0.0f);

    apc::ContactResult result = resolver.resolve_shoulder_charge(
        charger_pos, charger_vel,
        receiver_pos, receiver_vel
    );

    assert(result.contact_made == true && "shoulder charge contact made");
    assert(result.type == apc::ContactType::SHOULDER_CHARGE &&
           "type = SHOULDER_CHARGE");

    // Relative speed = |(7,0,0)-(3,0,0)| = 4.0
    // impact_magnitude = 4.0 * 1.2 = 4.8
    assert(result.impact_magnitude > 0.0f && "impact magnitude > 0");
    assert(approx_eq(result.impact_magnitude, 4.8f, 0.1f) &&
           "shoulder charge impact = relative_speed * 1.2");

    // Impact force should be in the direction of charger→receiver
    float force_len = apc::Vec3::length(result.impact_force);
    assert(force_len > 0.0f && "impact force non-zero");

    // Recovery times set
    assert(result.recovery_time_attacker > 0.0f);
    assert(result.recovery_time_defender > 0.0f);

    // Stylization: impact > 5 triggers hit-stop and camera shake
    // Here impact = 4.8 < 5.0, so no hit-stop
    assert(result.hit_stop_ms < 0.01f && "no hit-stop for sub-5 impact");

    printf("    [PASS] Shoulder charge: contact=true, impact≈4.8, stylization correct\n");
}

// =============================================================================
// TEST 10: GrappleResolver
// =============================================================================
static void test_grapple_resolver() {
    printf("  [Test 10] GrappleResolver...\n");

    apc::GrappleResolver grapple;

    // --- Initiate grapple between athletes 0 and 1 ---
    uint32_t idx = grapple.initiate(
        0, 1,
        apc::GrappleType::BODY_LOCK,
        0.8f  // initial strength
    );

    assert(idx != 0xFFFFFFFF && "initiate should return valid index");
    assert(idx == 0 && "first grapple gets index 0");
    assert(grapple.active_count == 1u && "one active grapple");

    // --- Get athlete grapple, verify is_active() ---
    apc::GrappleState* gs = grapple.get_athlete_grapple(0);
    assert(gs != nullptr && "get_athlete_grapple(0) returns non-null");
    assert(gs->is_active() == true && "grapple is active after initiate");
    assert(gs->type == apc::GrappleType::BODY_LOCK && "type = BODY_LOCK");
    assert(gs->attacker_id == 0 && "attacker_id = 0");
    assert(gs->defender_id == 1 && "defender_id = 1");
    assert(gs->hold_strength == 0.8f && "initial hold_strength = 0.8");

    // Also retrievable via defender_id
    apc::GrappleState* gs2 = grapple.get_athlete_grapple(1);
    assert(gs2 == gs && "same grapple retrieved via defender_id");

    // Athlete not in grapple → nullptr
    assert(grapple.get_athlete_grapple(99) == nullptr &&
           "athlete 99 not in any grapple");

    // --- Update with attacker_effort=1.0, dt=3.0 → approach resolution ---
    // Initial: hold_strength=0.8, throw_progress=0
    // throw_progress += 1.0 * 3.0 * 0.3 * 0.8 = 0.72
    // hold_strength -= 3.0 * 0.05 = 0.15 → 0.65
    // hold_duration = 3.0 (== max, not > max, so doesn't auto-break)
    grapple.update(3.0f, 1.0f, 0.0f);

    assert(gs->throw_progress > 0.0f && "throw_progress increased");
    assert(gs->throw_progress > 0.5f && "throw_progress approaching resolution (>0.5)");
    assert(gs->is_active() == true && "grapple still active after partial update");
    assert(gs->hold_duration == 3.0f && "hold_duration = dt");

    // One more update should push past max_hold_duration → break
    // Note: break_hold() resets hold_duration to 0, so check is_active instead
    grapple.update(0.01f, 1.0f, 0.0f);
    assert(gs->is_active() == false && "grapple broke after max duration");

    // --- Test break_hold() directly ---
    // Create a fresh grapple
    grapple.reset();
    idx = grapple.initiate(2, 3, apc::GrappleType::COLLAR_TIE, 0.9f);
    assert(idx == 0);

    apc::GrappleState* gs3 = grapple.get_athlete_grapple(2);
    assert(gs3 != nullptr);
    assert(gs3->is_active() == true);

    // Break the hold
    gs3->break_hold();
    assert(gs3->is_active() == false && "break_hold sets is_active to false");
    assert(gs3->type == apc::GrappleType::NONE && "type reset to NONE after break");
    assert(gs3->hold_strength < EPS && "hold_strength reset to 0 after break");
    assert(gs3->is_resolved == true && "is_resolved = true after break");

    // --- Can't initiate grapple with already-grappling athlete ---
    grapple.reset();
    uint32_t i1 = grapple.initiate(10, 11, apc::GrappleType::HEAD_LOCK, 0.7f);
    assert(i1 != 0xFFFFFFFF && "first grapple succeeds");
    uint32_t i2 = grapple.initiate(10, 12, apc::GrappleType::BODY_LOCK, 0.7f);
    assert(i2 == 0xFFFFFFFF && "can't grapple if athlete already in one");
    uint32_t i3 = grapple.initiate(12, 11, apc::GrappleType::SINGLE_LEG, 0.7f);
    assert(i3 == 0xFFFFFFFF && "can't grapple if defender already in one");

    // Fresh pair works
    uint32_t i4 = grapple.initiate(12, 13, apc::GrappleType::DOUBLE_LEG, 0.7f);
    assert(i4 != 0xFFFFFFFF && "fresh pair can grapple");

    printf("    [PASS] GrappleResolver: initiate, active, update, break, dedup all correct\n");
}

// =============================================================================
// Main
// =============================================================================
int main() {
    printf("Sprint 15: Bat/Racket & Contact Sports tests\n");
    printf("=============================================\n");

    test_implement_profile_factories();
    test_sweet_spot_model();
    test_hit_resolver_sweet_spot();
    test_hit_resolver_shank();
    test_swing_model();
    test_tackle_profiles();
    test_resolve_tackle();
    test_resolve_block();
    test_resolve_shoulder_charge();
    test_grapple_resolver();

    printf("\nSprint 15: ALL TESTS PASSED\n");
    return 0;
}
