#include "Plane.h"

/*
  Runway centreline tracking for AUTO takeoff, start sanity check and
  below-V1 ground-roll abort. See runway_takeoff.h for an overview.
 */

#if AP_PLANE_RUNWAY_TAKEOFF_ENABLED

#include <stdarg.h>

// distance the A->B line is extended both ways for the L1 controller.
// Must be well beyond the L1 distance and the distance flown before the
// takeoff altitude is reached.
#define RWY_LINE_EXTEND_M 10000.0f

// minimum A->B length, so survey errors give a small bearing error
#define RWY_MIN_LINE_LENGTH_M 200.0f

// how far behind point A the aircraft may start the roll
#define RWY_MAX_START_BEHIND_A_M 100.0f

// minimum lookahead distance for the ground steering law
#define RWY_MIN_LOOKAHEAD_M 10.0f

// below this ground speed after an abort the steering is centred
#define RWY_ABORT_CENTRE_SPEED 2.0f

// ground speed at which the aircraft counts as rolling and the
// RWY_BLEND fade-in starts
#define RWY_BLEND_START_SPEED 1.0f

// number of consecutive navigation cycles (10Hz) an abort condition
// must be present before the abort is triggered
#define RWY_ABORT_DEBOUNCE_COUNT 3

// height above the launch baro altitude at which we assume we are
// airborne and stop considering an abort
#define RWY_ABORT_MAX_HEIGHT_M 5.0f

#define RWY_REPORT_INTERVAL_MS 2000U
#define RWY_REFUSE_MSG_INTERVAL_MS 3000U

// brake output, as a fraction of the way from SERVOn_TRIM to
// RWY_BRK_PWM, above which the brakes count as applied
#define RWY_BRAKE_ON_FRAC 0.1f

// throttle output at or below which the throttle counts as closed
#define RWY_THR_CLOSED_PCT 10.0f

// time for the steering lookahead to move fully between its normal
// value and RWY_LOOK_MAX when deceleration starts or stops
#define RWY_DECEL_LOOK_TIME_S 1.0f

// time for the brake value used to soften the steering to follow a
// full step in the brake output
#define RWY_BRK_SOFT_TIME_S 0.5f

// fraction of RWY_MAX_CORR allowed for the return to the centreline
// after an abort
#define RWY_ABORT_CORR_FRAC 0.5f

// the GPS position is trusted for steering and the start check when
// its horizontal accuracy is within this fraction of RWY_STRT_XTRK
#define RWY_GPS_ACC_FRAC 0.5f

// brakes-not-holding check while AUTO holds the takeoff. The brakes
// count as commanded on above this fraction of their travel
#define RWY_HOLD_BRAKE_FRAC 0.5f
// moving: GPS ground speed above this, and above this fraction of
// TKOFF_THR_MINSPD when that is set
#define RWY_HOLD_SPD_MAX 0.5f
#define RWY_HOLD_SPD_FRAC 0.3f
// accelerating: TECS longitudinal acceleration above this while the
// GPS ground speed is above the floor, which rejects idle vibration
#define RWY_HOLD_ACCEL 0.5f
#define RWY_HOLD_ACCEL_MIN_SPD 0.2f
// consecutive navigation cycles (10Hz) before the abort
#define RWY_HOLD_DEBOUNCE_COUNT 5

// RWY_WL_HGT: time constant of the filter that learns the rangefinder's
// reading on the runway during the ground phase, and the largest
// reading accepted on the ground at the start of the takeoff
#define RWY_RF_REF_TC_S 1.0f
#define RWY_RF_GROUND_MAX_M 2.0f

// runway landing: touchdown when the rangefinder is within this height of
// the runway, and the aircraft is not climbing faster than this, for this
// many navigation cycles (10Hz)
#define RWY_TD_HGT_M 0.15f
#define RWY_TD_CLIMB_MAX 0.2f
#define RWY_TD_DEBOUNCE_COUNT 3
// a bounce: the rangefinder this high above the runway again for this
// many navigation cycles after touchdown
#define RWY_BOUNCE_HGT_M 0.3f
#define RWY_BOUNCE_COUNT 2
// time to move the elevator from its value at touchdown to RWY_LND_ELEV
#define RWY_LND_ELEV_RAMP_S 1.0f
// ground speed at which the rollout is reported as done
#define RWY_LAND_STOP_SPEED 1.0f
// warn if the approach (previous waypoint to LAND) is further off the
// landing line than this
#define RWY_LAND_APPROACH_TOL_DEG 10.0f

// brake operating time warning: lead time before RWY_BRK_TMAX (at most
// half of it), and the repeat interval
#define RWY_BRK_WARN_LEAD_S 30.0f
#define RWY_BRK_WARN_INTERVAL_MS 10000U

const AP_Param::GroupInfo RunwayTakeoff::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Runway takeoff enable
    // @Description: Bit 0 enables runway centreline tracking for AUTO takeoffs. The runway line runs from the NAV_TAKEOFF item lat/lon (point A, surveyed centreline point at the start end) to the next mission waypoint (point B, surveyed centreline point at the departure end or beyond). When enabled, auto takeoff is refused if the runway is not defined or the start checks fail. Bit 1 holds the wings level on the ground: no bank demand from the start of the roll until TKOFF_ROTATE_SPD (RWY_V1 if that is 0) or, if a weight on wheels switch is configured, while it reads on the ground, and throughout an abort; then the stock takeoff roll limit is restored over RWY_WL_FADE. Bit 2 is the runway landing: on the rollout the steering tracks the line from the NAV_LAND lat/lon (the touchdown point on the centreline) to the next mission item, which must be a waypoint further along the centreline, with the lookahead held at RWY_LOOK_MAX and the steering softened under braking (RWY_BRK_GAIN). Touchdown is seen by the rangefinder (RWY_WL_HGT) or a weight on wheels switch, and only while not climbing; leaving the runway again is a bounce, flown as still airborne until the wheels are down again. RWY_LND_ELEV can hold the nose down on the rollout. A go-around commanded after touchdown is flown as a runway takeoff along the landing line. With bits 1 and 2 the wings are also held level near and on the runway on landing. The other bits do nothing unless bit 0 is set.
    // @Bitmask: 0:Runway takeoff,1:Wings level on the ground,2:Runway landing
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLE", 1, RunwayTakeoff, enable, 0, AP_PARAM_FLAG_ENABLE),

    // index 2 was HDG, removed: the runway direction now comes from A->B

    // @Param: STRT_XTRK
    // @DisplayName: Max start cross-track error
    // @Description: Takeoff is refused if the aircraft is further than this from the runway centreline when the takeoff roll would start, or if the GPS horizontal accuracy is not available or worse than half of this.
    // @Units: m
    // @Range: 0.5 20
    // @User: Advanced
    AP_GROUPINFO("STRT_XTRK", 3, RunwayTakeoff, start_xtrk_max, 3),

    // @Param: STRT_HDG
    // @DisplayName: Max start heading error
    // @Description: Takeoff is refused if the aircraft heading differs from the A->B direction by more than this when the takeoff roll would start.
    // @Units: deg
    // @Range: 1 30
    // @User: Advanced
    AP_GROUPINFO("STRT_HDG", 4, RunwayTakeoff, start_hdg_max, 5),

    // @Param: STRT_DIST
    // @DisplayName: Min runway remaining at start
    // @Description: If non-zero, takeoff is refused unless point B (the waypoint after NAV_TAKEOFF) is at least this far ahead along the runway. Most useful when B is the departure end of the runway.
    // @Units: m
    // @Range: 0 5000
    // @User: Advanced
    AP_GROUPINFO("STRT_DIST", 5, RunwayTakeoff, start_dist_min, 0),

    // @Param: ABT_XTRK
    // @DisplayName: Abort cross-track limit
    // @Description: If non-zero, the takeoff is aborted (engine stopped, brakes applied) if the cross-track error exceeds this while below RWY_V1. Set this from the runway width and gear track, near where the main gear would leave the runway; a high speed abort is itself a risk.
    // @Units: m
    // @Range: 0 30
    // @User: Advanced
    AP_GROUPINFO("ABT_XTRK", 6, RunwayTakeoff, abort_xtrk, 0),

    // @Param: ABT_HDG
    // @DisplayName: Abort heading error limit
    // @Description: If non-zero, the takeoff is aborted if the heading error relative to the A->B direction exceeds this while below RWY_V1.
    // @Units: deg
    // @Range: 0 45
    // @User: Advanced
    AP_GROUPINFO("ABT_HDG", 7, RunwayTakeoff, abort_hdg, 0),

    // @Param: V1
    // @DisplayName: Takeoff decision speed
    // @Description: Decision speed: airspeed if the AHRS is using an airspeed sensor, otherwise GPS ground speed. Below this speed an abort condition stops the engine and applies full brakes. At or above it the takeoff continues. Required if RWY_ABT_XTRK or RWY_ABT_HDG is set, and must not exceed TKOFF_ROTATE_SPD.
    // @Units: m/s
    // @Range: 0 60
    // @User: Advanced
    AP_GROUPINFO("V1", 8, RunwayTakeoff, v1, 0),

    // @Param: BRK_CHAN
    // @DisplayName: Wheel brake servo output
    // @Description: Servo output number (1-based) driving the wheel brakes. On abort this output is held at RWY_BRK_PWM, overriding whatever function the output has (including RC passthrough). The output is also read, against its SERVOn_TRIM (brakes off) and RWY_BRK_PWM (full brake), to see how much brake is applied, for the steering, the brakes-not-holding check and the brake time warning. 0 disables brake control.
    // @Range: 0 32
    // @User: Advanced
    AP_GROUPINFO("BRK_CHAN", 9, RunwayTakeoff, brake_chan, 0),

    // @Param: BRK_PWM
    // @DisplayName: Wheel brake full PWM
    // @Description: PWM sent to RWY_BRK_CHAN for full braking during an abort.
    // @Units: PWM
    // @Range: 800 2200
    // @User: Advanced
    AP_GROUPINFO("BRK_PWM", 10, RunwayTakeoff, brake_pwm, 1900),

    // @Param: BLEND
    // @DisplayName: Steering fade-in time
    // @Description: Time over which the runway steering correction is faded in, to avoid a step in the steering demand. The fade-in starts when the aircraft starts rolling (1 m/s ground speed), not when the takeoff is triggered. On a runway landing the stock rollout steering is faded into the runway steering over this time from touchdown.
    // @Units: s
    // @Range: 0 10
    // @User: Advanced
    AP_GROUPINFO("BLEND", 11, RunwayTakeoff, blend_time, 2),

    // @Param: MAX_CORR
    // @DisplayName: Max steering correction
    // @Description: Limits the heading demand; the steering deflection this allows falls with speed.
    // @Units: deg
    // @Range: 2 30
    // @User: Advanced
    AP_GROUPINFO("MAX_CORR", 12, RunwayTakeoff, max_corr, 10),

    // @Param: LOOKAHEAD
    // @DisplayName: Steering lookahead time
    // @Description: The runway steering aims at a point on the centreline this many seconds of travel ahead (minimum 10 m). Larger values correct cross-track more gently.
    // @Units: s
    // @Range: 1 10
    // @User: Advanced
    AP_GROUPINFO("LOOKAHEAD", 13, RunwayTakeoff, lookahead, 3),

    // @Param: BRK_RAMP
    // @DisplayName: Abort brake ramp time
    // @Description: Time over which the brake output is ramped from its value at the abort to RWY_BRK_PWM. 0 applies full brakes at once.
    // @Units: s
    // @Range: 0 5
    // @User: Advanced
    AP_GROUPINFO("BRK_RAMP", 14, RunwayTakeoff, brake_ramp, 1),

    // @Param: WL_FADE
    // @DisplayName: Wings level hand-back time
    // @Description: With RWY_ENABLE bit 1 set, the time over which the roll limit is raised from zero back to the stock takeoff roll limit (LEVEL_ROLL_LIMIT, then the TKOFF_LVL_ALT scaling) once the ground phase ends. With RWY_WL_HGT set this is the fastest the limit may rise, and the height above the runway sets how far. 0 restores it at once.
    // @Units: s
    // @Range: 0 5
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("WL_FADE", 15, RunwayTakeoff, wl_fade, 1),

    // @Param: LOOK_MAX
    // @DisplayName: Steering lookahead limit
    // @Description: If non-zero, the upper limit on the runway steering lookahead distance (RWY_LOOKAHEAD seconds of travel, minimum 10 m), so the cross-track correction keeps its authority at high speed. While decelerating the lookahead moves to this value, and on a runway landing rollout it is held at this value. 0 means no limit (and after an abort or on a landing rollout, the runway direction is held instead of returning to the line).
    // @Units: m
    // @Range: 0 200
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("LOOK_MAX", 16, RunwayTakeoff, look_max, 0),

    // @Param: BRK_GAIN
    // @DisplayName: Steering gain at full brake
    // @Description: Runway steering response to heading error at full brake, relative to unbraked. Braking makes the nosewheel more effective, so the response is reduced in proportion to the brake applied (from the brake output against its SERVOn_TRIM and RWY_BRK_PWM), down to this at full brake. Applies to any braking during the roll, an abort or a runway landing rollout. 1.0 leaves the steering unchanged.
    // @Range: 0.1 1
    // @Increment: 0.05
    // @User: Advanced
    AP_GROUPINFO("BRK_GAIN", 17, RunwayTakeoff, brk_gain, 1),

    // @Param: BRK_TMAX
    // @DisplayName: Brake operating time limit
    // @Description: Assigned operating time of the wheel brakes. The time the brakes have been applied continuously (from the RWY_BRK_CHAN output, whatever drives it, armed or disarmed) is compared with this, and the GCS is warned from 30 s before it (half of it if shorter) every 10 s, with a critical message once it is reached, so the brakes can be released or the engine shut down before the brakes switch themselves off. Warning only: the brakes are not changed. 0 disables the warning.
    // @Units: s
    // @Range: 0 600
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("BRK_TMAX", 18, RunwayTakeoff, brk_tmax, 90),

    // @Param: WL_HGT
    // @DisplayName: Wings level hand-back height
    // @Description: With RWY_ENABLE bit 1 set, if non-zero, the roll limit is handed back by height above the runway instead of by time alone: after rotate speed it rises in proportion to the downward rangefinder's height above its reading on the runway, reaching the stock takeoff roll limit at this height, and drops back if the aircraft settles or bounces. It still rises no faster than RWY_WL_FADE allows. Uses the rangefinder set by RNGFND_LND_ORNT (pointing down by default); takeoff is refused if it is not working, and if it fails after rotate speed the hand-back continues by time. On a runway landing (RWY_ENABLE bits 1 and 2) the limit falls in proportion to the height above the runway in the same way, and touchdown is when the height is under 0.15 m while not climbing (over 0.3 m again is a bounce); the runway level learned on takeoff is used, or RNGFND1_GNDCLR if there was none. 0 hands back by time only.
    // @Units: m
    // @Range: 0 10
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("WL_HGT", 19, RunwayTakeoff, wl_hgt, 0),

    // @Param: LND_ELEV
    // @DisplayName: Landing rollout elevator
    // @Description: On a runway landing (RWY_ENABLE bit 2), the elevator held on the rollout from a confirmed touchdown instead of the stock flare pitch demand, as a percentage of full travel; negative is down elevator, holding the nosewheel on the runway for steering. It is reached over 1 s from the elevator at touchdown, and released at once on a bounce or go-around. The landing counterpart of TKOFF_TDRAG_ELEV; as there, too much down elevator can make the steering unstable. 0 keeps the stock flare pitch demand.
    // @Units: %
    // @Range: -100 100
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("LND_ELEV", 20, RunwayTakeoff, lnd_elev, 0),

    AP_GROUPEND
};

RunwayTakeoff::RunwayTakeoff(void)
{
    AP_Param::setup_object_defaults(this, var_info);
    state.steer_soft = 1;
    land.wl_factor = 1;
}

/*
  set up the runway line from the NAV_TAKEOFF item (A) and the next
  navigation item, which must be a waypoint (B)
 */
void RunwayTakeoff::init(const AP_Mission::Mission_Command &cmd)
{
    // reset per-takeoff state. The abort latch deliberately survives
    // a mission restart; it is only cleared in update_outputs()
    state.line_valid = false;
    state.roll_started = false;
    state.go_around = false;
    state.past_v1 = false;
    state.abort_count = 0;
    state.max_abs_xtrk = 0;
    state.last_steer_err = 0;
    state.last_refuse_msg_ms = 0;

    if (!enabled()) {
        return;
    }

    const Location &a = cmd.content.location;
    if (a.lat == 0 && a.lng == 0) {
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "RWY: TAKEOFF item has no lat/lon (point A)");
        return;
    }

    Location b;
    if (!waypoint_after(cmd.index, b)) {
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "RWY: need WAYPOINT after TAKEOFF (point B)");
        return;
    }
    const float len = a.get_distance(b);
    if (len < RWY_MIN_LINE_LENGTH_M) {
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "RWY: A to B only %.0fm, need %.0fm",
                      (double)len, (double)RWY_MIN_LINE_LENGTH_M);
        return;
    }

    state.point_a = a;
    state.point_b = b;
    state.length_m = len;
    state.bearing_deg = wrap_360(degrees(a.get_bearing(b)));
    state.l1_start = a;
    state.l1_end = b;
    state.l1_start.offset_bearing(wrap_360(state.bearing_deg + 180), RWY_LINE_EXTEND_M);
    state.l1_end.offset_bearing(state.bearing_deg, RWY_LINE_EXTEND_M);
    state.line_valid = true;

    float xtrk, along;
    get_line_errors(xtrk, along);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "RWY: A->B brg %.1f len %.0fm",
                  (double)state.bearing_deg, (double)state.length_m);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "RWY: now xt %.1fm hdg err %.1f rem %.0fm",
                  (double)xtrk, (double)heading_error_deg(), (double)(state.length_m - along));
}

/*
  start-of-roll sanity checks. Returns false (with a rate limited
  message) if the takeoff must not start
 */
bool RunwayTakeoff::launch_allowed(bool in_auto)
{
    if (state.aborted) {
        refuse("RWY: abort latched, disarm to clear");
        return false;
    }
    if (!enabled()) {
        return true;
    }
    if (!in_auto) {
        refuse("RWY: runway takeoff needs AUTO (or RWY_ENABLE=0)");
        return false;
    }
    if (!state.line_valid) {
        refuse("RWY: no runway, check TAKEOFF and next WP");
        return false;
    }
    if (abort_enabled()) {
        if (v1 <= 0) {
            refuse("RWY: abort enabled but RWY_V1 not set");
            return false;
        }
        if (plane.g.takeoff_rotate_speed > 0 && v1 > plane.g.takeoff_rotate_speed) {
            refuse("RWY: RWY_V1 %.1f > TKOFF_ROTATE_SPD %.1f",
                   (double)v1.get(), (double)plane.g.takeoff_rotate_speed.get());
            return false;
        }
    }
    if (option_set(Option::WINGS_LEVEL) && wl_height_mode()) {
        float hgt;
        if (!rangefinder_height(hgt) || hgt > RWY_RF_GROUND_MAX_M) {
            refuse("RWY: RWY_WL_HGT needs a working down rangefinder");
            return false;
        }
    }
    if (option_set(Option::WINGS_LEVEL)) {
        if (!wl_height_mode() && wow_available()) {
#if AP_LANDINGGEAR_ENABLED
            if (plane.g2.landing_gear.get_wow_state() != AP_LandingGear::LG_WOW) {
                refuse("RWY: weight on wheels switch reads airborne");
                return false;
            }
#endif
        } else if (plane.g.takeoff_rotate_speed <= 0 && v1 <= 0) {
            // nothing would end the ground phase
            refuse("RWY: wings level needs TKOFF_ROTATE_SPD or RWY_V1");
            return false;
        }
    }

    // the start cross-track check is only as good as the position
    float hacc;
    if (!plane.gps.horizontal_accuracy(hacc)) {
        refuse("RWY: GPS accuracy not available");
        return false;
    }
    const float hacc_max = start_xtrk_max * RWY_GPS_ACC_FRAC;
    if (hacc > hacc_max) {
        refuse("RWY: GPS accuracy %.2fm > %.2fm", (double)hacc, (double)hacc_max);
        return false;
    }

    float xtrk, along;
    get_line_errors(xtrk, along);
    const float hdg_err = heading_error_deg();

    if (fabsf(xtrk) > start_xtrk_max) {
        refuse("RWY: start xt %.1fm > %.1fm", (double)xtrk, (double)start_xtrk_max.get());
        return false;
    }
    if (fabsf(hdg_err) > start_hdg_max) {
        refuse("RWY: start hdg err %.1f > %.1f", (double)hdg_err, (double)start_hdg_max.get());
        return false;
    }
    const float remaining = state.length_m - along;
    if (remaining <= 0) {
        refuse("RWY: point B %.0fm behind aircraft", (double)-remaining);
        return false;
    }
    if (along < -RWY_MAX_START_BEHIND_A_M) {
        refuse("RWY: %.0fm behind point A", (double)-along);
        return false;
    }
    if (start_dist_min > 0 && remaining < start_dist_min) {
        refuse("RWY: runway rem %.0fm < %.0fm", (double)remaining, (double)start_dist_min.get());
        return false;
    }
    return true;
}

void RunwayTakeoff::launch_started()
{
    if (!active()) {
        return;
    }
    state.roll_started = true;
    state.past_v1 = false;
    state.past_rotate = false;
    state.rf_ref_valid = false;
    state.rf_lost = false;
    state.abort_count = 0;
    state.max_abs_xtrk = 0;
    state.blend_start_ms = 0;
    state.blend = 0;
    state.launch_ms = AP_HAL::millis();
    state.last_report_ms = state.launch_ms;

    // steer along the runway from the start of the roll. A non -1
    // hold_course_cd selects calc_nav_yaw_course(), which takes its
    // error from ground_steering_error()
    plane.steer_state.hold_course_cd = runway_heading_cd();

    float xtrk, along;
    get_line_errors(xtrk, along);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "RWY: roll xt %.1fm hdg err %.1f rem %.0fm",
                  (double)xtrk, (double)heading_error_deg(), (double)(state.length_m - along));
    if (abort_enabled()) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "RWY: V1 %.1f abort xt %.1fm hdg %.0f",
                      (double)v1.get(), (double)abort_xtrk.get(), (double)abort_hdg.get());
        if (brake_chan <= 0) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "RWY: no brake output set (RWY_BRK_CHAN)");
        }
    } else {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "RWY: abort disabled");
    }
}

int32_t RunwayTakeoff::runway_heading_cd() const
{
    return wrap_360_cd(int32_t(state.bearing_deg * 100));
}

void RunwayTakeoff::update_l1()
{
    plane.nav_controller->update_waypoint(state.l1_start, state.l1_end);
}

/*
  track the ground phase of the takeoff: from the start of the roll
  until rotate speed (or, with a weight on wheels switch and no
  RWY_WL_HGT, while it reads on the ground), and throughout an abort.
  Sets the roll limit hand-back factor: 0 in the ground phase, then
  rising to 1 no faster than RWY_WL_FADE allows and, with RWY_WL_HGT,
  no further than the height above the runway allows
 */
void RunwayTakeoff::update_ground_phase()
{
    const uint32_t now_us = AP_HAL::micros();
    const float dt = constrain_float((now_us - state.last_gp_us) * 1.0e-6f, 0, 0.1f);
    state.last_gp_us = now_us;

    if (!active() || plane.control_mode != &plane.mode_auto ||
        !(state.roll_started || state.aborted)) {
        state.ground_phase = false;
        state.wl_factor = 1;
        return;
    }

    // rotate speed latches: speed alone can't detect a bounce
    if (!state.past_rotate && reached_rotate_speed()) {
        state.past_rotate = true;
    }

    bool on_ground = !state.past_rotate;
#if AP_LANDINGGEAR_ENABLED
    if (!wl_height_mode() && wow_available()) {
        // a bounce returns to the ground phase
        on_ground = plane.g2.landing_gear.get_wow_state() == AP_LandingGear::LG_WOW;
    }
#endif
    if (state.aborted) {
        on_ground = true;
    }
    state.ground_phase = on_ground;

    float hgt;
    state.rf_ok = wl_height_mode() && rangefinder_height(hgt);
    if (on_ground) {
        state.wl_factor = 0;
        if (state.rf_ok) {
            // learn the rangefinder's reading on the runway, including
            // the gear extending as the wing takes the weight
            if (!state.rf_ref_valid) {
                state.rf_ref = hgt;
                state.rf_ref_valid = true;
            } else {
                state.rf_ref += (hgt - state.rf_ref) * constrain_float(dt / RWY_RF_REF_TC_S, 0, 1);
            }
        }
        state.rf_above = 0;
        state.rf_factor = 0;
    } else {
        float target = 1;
        if (wl_height_mode()) {
            if (state.rf_ok && state.rf_ref_valid) {
                // in proportion to height, so wingtip clearance grows with
                // bank, and a bounce takes the limit back down
                state.rf_above = MAX(hgt - state.rf_ref, 0);
                state.rf_factor = constrain_float(state.rf_above / wl_hgt, 0, 1);
                target = state.rf_factor;
            } else if (!state.rf_lost) {
                state.rf_lost = true;
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "RWY: rangefinder lost, wings level by time");
            }
        }
        if (target < state.wl_factor) {
            // down at once: a settle or bounce
            state.wl_factor = target;
        } else if (is_positive(wl_fade)) {
            state.wl_factor = MIN(target, state.wl_factor + dt / wl_fade);
        } else {
            state.wl_factor = target;
        }
    }
    if (!option_set(Option::WINGS_LEVEL)) {
        state.wl_factor = 1;
    }
}

int32_t RunwayTakeoff::ground_roll_limit_cd(int32_t stock_limit_cd)
{
    update_ground_phase();
    // scaling the stock limit leaves no step at the hand-back, and
    // follows the TKOFF_LVL_ALT scaling once it applies
    return int32_t(stock_limit_cd * state.wl_factor);
}

/*
  ground steering law. Aim at a point on the centreline a lookahead
  distance ahead, cap the correction, and fade it in from the start of
  the roll. While decelerating the lookahead moves to RWY_LOOK_MAX.
  After an abort, steer back towards the centreline with a smaller
  correction (or hold the runway direction if RWY_LOOK_MAX is 0 or the
  GPS can't be trusted), and centre the steering once nearly stopped.
  Result is a heading error (desired - current), centidegrees.
 */
void RunwayTakeoff::ground_steering_error(int32_t &bearing_error_cd)
{
    if (landing_rollout()) {
        landing_steering_error(bearing_error_cd);
        return;
    }
    if (!active() || plane.control_mode != &plane.mode_auto ||
        !(state.roll_started || state.aborted)) {
        return;
    }

    const float cap = constrain_float(max_corr, 1, 45);
    const float gspd = plane.ahrs.groundspeed();
    float xtrk, along;
    get_line_errors(xtrk, along);

    float look = lookahead_m(gspd);
    if (is_positive(look_max)) {
        // while decelerating, move (at a limited rate) to the longest
        // lookahead, for gentle cross-track correction
        look += (MAX(look_max, RWY_MIN_LOOKAHEAD_M) - look) * state.decel_frac;
    }
    state.last_look = look;

    float err;
    if (state.aborted) {
        if (!is_positive(look_max) || !gps_position_ok()) {
            // no line to return to without RWY_LOOK_MAX, or once the
            // position can't be trusted: hold the runway direction for
            // the rest of the stop, so a position jump can't steer us
            state.abort_hold_hdg = true;
        }
        if (gspd < RWY_ABORT_CENTRE_SPEED) {
            err = 0;
        } else if (state.abort_hold_hdg) {
            err = wrap_180(state.bearing_deg - plane.ahrs.get_yaw_deg());
        } else {
            // steer back towards the centreline with a reduced
            // correction limit. The correction depends only on the
            // current offset, so nothing winds up during the stop
            const float abort_cap = cap * RWY_ABORT_CORR_FRAC;
            const float xt_corr = constrain_float(-degrees(atan2f(xtrk, look)), -abort_cap, abort_cap);
            err = wrap_180(state.bearing_deg + xt_corr - plane.ahrs.get_yaw_deg());
        }
    } else {
        const float xt_corr = constrain_float(-degrees(atan2f(xtrk, look)), -cap, cap);
        err = wrap_180(state.bearing_deg + xt_corr - plane.ahrs.get_yaw_deg());

        // fade in once the aircraft is actually rolling, not from when
        // the takeoff was triggered
        const uint32_t now_ms = AP_HAL::millis();
        if (state.blend_start_ms == 0 && gspd >= RWY_BLEND_START_SPEED) {
            state.blend_start_ms = MAX(now_ms, 1U);
        }
        float blend = 0;
        if (state.blend_start_ms != 0) {
            blend = 1;
            if (is_positive(blend_time)) {
                blend = constrain_float((now_ms - state.blend_start_ms) * 0.001f / blend_time, 0, 1);
            }
        }
        state.blend = blend;
        err *= blend;
    }

    err = constrain_float(err, -cap, cap);

    // under braking, reduce the response to heading error (RWY_BRK_GAIN).
    // Scaling the error leaves the integrator's stored value alone
    err *= state.steer_soft;

    state.last_steer_err = err;
    bearing_error_cd = int32_t(err * 100);
}

void RunwayTakeoff::update_monitor()
{
    if (!active()) {
        return;
    }
    if (!hal.util->get_soft_armed()) {
        // disarmed: this takeoff attempt is over. Stops the abort
        // checks, which fired while the aircraft was pushed back
        state.roll_started = false;
        state.hold_count = 0;
        return;
    }
    // before the roll, with AUTO holding the takeoff
    const bool holding = !state.roll_started;

    const uint32_t now_ms = AP_HAL::millis();
    float xtrk, along;
    get_line_errors(xtrk, along);
    const float hdg_err = heading_error_deg();
    const float spd = speed_for_v1();

    update_ground_phase();
    state.max_abs_xtrk = MAX(state.max_abs_xtrk, fabsf(xtrk));
    write_log(xtrk, along, hdg_err, spd);

    if (holding) {
        if (!state.aborted) {
            check_brakes_holding(xtrk, hdg_err);
        }
        return;
    }

    if (now_ms - state.last_report_ms >= RWY_REPORT_INTERVAL_MS) {
        state.last_report_ms = now_ms;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "RWY: %s xt %.1fm hdg err %.1f spd %.1f",
                      state.aborted ? "ABORTED" : "roll",
                      (double)xtrk, (double)hdg_err, (double)spd);
    }

    if (state.aborted) {
        return;
    }

    if (!state.past_v1 && abort_enabled()) {
        if (spd >= v1) {
            state.past_v1 = true;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "RWY: V1 %.1f, continuing takeoff (xt %.1fm)",
                          (double)spd, (double)xtrk);
        } else if (plane.barometer.get_altitude() - plane.auto_state.baro_takeoff_alt > RWY_ABORT_MAX_HEIGHT_M) {
            state.past_v1 = true;
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "RWY: airborne below V1, abort disabled");
        }
    }

    if (!abort_enabled() || state.past_v1) {
        return;
    }

    const char *reason = nullptr;
    if (abort_xtrk > 0 && fabsf(xtrk) > abort_xtrk) {
        reason = "xtrack";
    } else if (abort_hdg > 0 && fabsf(hdg_err) > abort_hdg) {
        reason = "heading";
    } else if (plane.gps.status() < AP_GPS::GPS_OK_FIX_3D) {
        reason = "GPS fix lost";
    }

    if (reason == nullptr) {
        state.abort_count = 0;
        return;
    }
    if (++state.abort_count >= RWY_ABORT_DEBOUNCE_COUNT) {
        trigger_abort(reason, xtrk, hdg_err, spd);
    }
}

/*
  while AUTO holds the takeoff with the brakes commanded on and the
  engine running, abort if the aircraft starts to move or accelerate:
  the brakes are not holding, and TKOFF_THR_MINSPD would otherwise start
  the takeoff at full power
 */
void RunwayTakeoff::check_brakes_holding(float xtrk, float hdg_err)
{
    // only on the ground with the throttle held back for the takeoff,
    // never if a mission restarts at the TAKEOFF item in flight
    if (!plane.throttle_suppressed || plane.is_flying() ||
        state.brake_frac < RWY_HOLD_BRAKE_FRAC || !engine_running()) {
        state.hold_count = 0;
        return;
    }

    const float gspd = plane.gps.ground_speed();
    float spd_lim = RWY_HOLD_SPD_MAX;
    if (is_positive(plane.g.takeoff_throttle_min_speed)) {
        spd_lim = MIN(spd_lim, plane.g.takeoff_throttle_min_speed * RWY_HOLD_SPD_FRAC);
    }
    const bool moving = gspd > spd_lim;
    const bool accelerating = gspd > RWY_HOLD_ACCEL_MIN_SPD &&
                              plane.TECS_controller.get_VXdot() > RWY_HOLD_ACCEL;
    if (!moving && !accelerating) {
        state.hold_count = 0;
        return;
    }
    if (++state.hold_count >= RWY_HOLD_DEBOUNCE_COUNT) {
        trigger_abort("brakes slip", xtrk, hdg_err, gspd);
    }
}

void RunwayTakeoff::takeoff_complete()
{
    if (!active() || !state.roll_started) {
        return;
    }
    state.roll_started = false;
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "RWY: takeoff complete, max xt %.1fm",
                  (double)state.max_abs_xtrk);
}

void RunwayTakeoff::trigger_abort(const char *reason, float xtrk, float hdg_err, float spd)
{
    state.aborted = true;

    state.abort_ms = AP_HAL::millis();
    GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "RWY: ABORT %s xt %.1f he %.1f v %.1f",
                  reason, (double)xtrk, (double)hdg_err, (double)spd);

    // the steering integrator is left alone: it holds the crosswind
    // correction learned on the roll, which the stop still needs. It
    // returns to zero below STEER2SRV_MINSPD

#if AP_ICENGINE_ENABLED
    // ignition off
    plane.g2.ice_control.engine_control(0, 0, 0, 0);
#endif

    // the emergency stop forces throttle, starter and engine run
    // enable outputs to their stopped values and stops the ICE
    // controller from restarting the engine
    if (!SRV_Channels::get_emergency_stop()) {
        SRV_Channels::set_emergency_stop(true);
        state.we_set_estop = true;
    }

    if (brake_chan > 0 && brake_chan <= NUM_SERVO_CHANNELS) {
        hal.rcout->enable_ch(brake_chan - 1);
        // ramp from whatever the brake output was at the abort
        state.brake_start_pwm = hal.rcout->read(brake_chan - 1);
    }
    state.brake_out_pwm = 0;
    state.abort_hold_hdg = false;
}

/*
  run every loop after the servo outputs have been written but before
  they are pushed to the hardware
 */
void RunwayTakeoff::update_outputs()
{
    update_brakes();

    if (!state.aborted) {
        return;
    }

    if (brake_chan > 0 && brake_chan <= NUM_SERVO_CHANNELS) {
        uint16_t pwm = brake_pwm;
        const float elapsed = (AP_HAL::millis() - state.abort_ms) * 0.001f;
        if (is_positive(brake_ramp) && elapsed < brake_ramp && state.brake_start_pwm != 0) {
            const float frac = elapsed / brake_ramp;
            pwm = uint16_t(state.brake_start_pwm + (brake_pwm - state.brake_start_pwm) * frac);
        }
        hal.rcout->write(brake_chan - 1, pwm);
        state.brake_out_pwm = pwm;
    }

    if (hal.util->get_soft_armed()) {
        return;
    }

    // disarmed. Only clear once the engine start switch (if any) is
    // not asking for the engine to run, so the engine cannot crank
    // as soon as the emergency stop is released
    const uint32_t now_ms = AP_HAL::millis();
    if (engine_switch_in_run()) {
        if (now_ms - state.last_clear_msg_ms > 5000U) {
            state.last_clear_msg_ms = now_ms;
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "RWY: set engine switch to stop to clear abort");
        }
        return;
    }

    state.aborted = false;
    state.roll_started = false;
    if (state.we_set_estop) {
        SRV_Channels::set_emergency_stop(false);
        state.we_set_estop = false;
    }
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "RWY: abort cleared, brakes released");
}

/*
  brake output and deceleration state, run every loop. Deceleration is
  taken from the commanded inputs (brakes, throttle closed or engine
  off, abort) rather than measured, so it is known at once and is not
  fooled by runway bumps, slope or gusts
 */
void RunwayTakeoff::update_brakes()
{
    const uint32_t now_us = AP_HAL::micros();
    const float dt = constrain_float((now_us - state.last_brake_us) * 1.0e-6f, 0, 0.1f);
    state.last_brake_us = now_us;

    if (!enabled()) {
        state.brake_frac = 0;
        state.decel_flags = 0;
        state.decel_frac = 0;
        state.brake_soft = 0;
        state.steer_soft = 1;
        state.brake_on_ms = 0;
        state.brake_time_s = 0;
        state.brake_warn_level = 0;
        return;
    }

    state.brake_frac = brake_fraction();

    // continuous brake time, armed or disarmed, in any mode
    const uint32_t now_ms = AP_HAL::millis();
    if (state.brake_frac >= RWY_BRAKE_ON_FRAC) {
        if (state.brake_on_ms == 0) {
            state.brake_on_ms = MAX(now_ms, 1U);
        }
        state.brake_time_s = (now_ms - state.brake_on_ms) * 0.001f;
    } else {
        state.brake_on_ms = 0;
        state.brake_time_s = 0;
        state.brake_warn_level = 0;
    }
    check_brake_time(now_ms);

    // steering softening under braking. The brake value used follows
    // the output at a limited rate, so there is no step in the steering
    // when the brakes come on or go off
    const float bstep = dt / RWY_BRK_SOFT_TIME_S;
    state.brake_soft += constrain_float(state.brake_frac - state.brake_soft, -bstep, bstep);
    state.steer_soft = 1 - (1 - constrain_float(brk_gain, 0.1, 1)) * state.brake_soft;

    uint8_t flags = 0;
    if (state.brake_frac >= RWY_BRAKE_ON_FRAC) {
        flags |= uint8_t(DecelFlag::BRAKES);
    }
    if (throttle_closed_or_engine_off()) {
        flags |= uint8_t(DecelFlag::THROTTLE);
    }
    if (state.aborted) {
        flags |= uint8_t(DecelFlag::ABORT);
    }
    state.decel_flags = flags;

    if (!(state.roll_started || state.aborted)) {
        // the runway steering is not running
        state.decel_frac = 0;
    } else {
        // rate limited, so a brake tap gives no step in the steering
        const float target = flags != 0 ? 1 : 0;
        const float step = dt / RWY_DECEL_LOOK_TIME_S;
        state.decel_frac += constrain_float(target - state.decel_frac, -step, step);
    }
}

/*
  brake applied, 0 to 1, from the brake output against its SERVOn_TRIM
  and RWY_BRK_PWM. Covers whatever drives the output: RC passthrough
  or the abort override
 */
float RunwayTakeoff::brake_fraction() const
{
    if (brake_chan <= 0 || brake_chan > NUM_SERVO_CHANNELS) {
        return 0;
    }
    const SRV_Channel *c = SRV_Channels::srv_channel(brake_chan - 1);
    if (c == nullptr) {
        return 0;
    }
    uint16_t pwm = 0;
    if (state.aborted && state.brake_out_pwm != 0) {
        // the abort override is written straight to the output
        pwm = state.brake_out_pwm;
    } else if (!SRV_Channels::get_output_pwm_chan(brake_chan - 1, pwm)) {
        return 0;
    }
    const float trim = c->get_trim();
    const float span = brake_pwm - trim;
    if (pwm == 0 || fabsf(span) < 1) {
        return 0;
    }
    return constrain_float((pwm - trim) / span, 0, 1);
}

/*
  warn before the brakes reach their assigned operating time
  (RWY_BRK_TMAX), so they can be released, or the engine shut down,
  before the brakes switch themselves off. 30 s gives three warnings
  before a 90 s limit: time to notice, to shut an engine down and
  confirm it has stopped if the aircraft must not be left unbraked
  with it running, and then release. Warning only
 */
void RunwayTakeoff::check_brake_time(uint32_t now_ms)
{
    if (!is_positive(brk_tmax) || state.brake_on_ms == 0) {
        return;
    }
    const float t = state.brake_time_s;
    const float lead = MIN(RWY_BRK_WARN_LEAD_S, brk_tmax * 0.5f);
    uint8_t level = 0;
    if (t >= brk_tmax) {
        level = 2;
    } else if (t >= brk_tmax - lead) {
        level = 1;
    }
    if (level == 0) {
        return;
    }
    if (level <= state.brake_warn_level &&
        now_ms - state.last_brake_warn_ms < RWY_BRK_WARN_INTERVAL_MS) {
        return;
    }
    state.brake_warn_level = level;
    state.last_brake_warn_ms = now_ms;
    if (level == 2) {
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "RWY: brakes on %.0fs, %.0fs limit reached",
                      (double)t, (double)brk_tmax.get());
    } else {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "RWY: brakes on %.0fs of %.0fs, release or shut down",
                      (double)t, (double)brk_tmax.get());
    }
}

// true if the engine is running, for the brakes-not-holding check
bool RunwayTakeoff::engine_running() const
{
    if (SRV_Channels::get_emergency_stop()) {
        return false;
    }
#if AP_ICENGINE_ENABLED
    const AP_ICEngine::ICE_State ice = plane.g2.ice_control.get_state();
    if (ice != AP_ICEngine::ICE_DISABLED) {
        return ice == AP_ICEngine::ICE_RUNNING;
    }
#endif
    // no ICE controller: a motor that is being driven
    return SRV_Channels::get_output_scaled(SRV_Channel::k_throttle) > 0;
}

bool RunwayTakeoff::throttle_closed_or_engine_off() const
{
    if (SRV_Channels::get_emergency_stop()) {
        return true;
    }
#if AP_ICENGINE_ENABLED
    const AP_ICEngine::ICE_State ice = plane.g2.ice_control.get_state();
    if (ice != AP_ICEngine::ICE_DISABLED && ice != AP_ICEngine::ICE_RUNNING) {
        return true;
    }
#endif
    return SRV_Channels::get_output_scaled(SRV_Channel::k_throttle) <= RWY_THR_CLOSED_PCT;
}

// true if the GPS position is good enough to steer and check against
bool RunwayTakeoff::gps_position_ok() const
{
    float hacc;
    return plane.gps.status() >= AP_GPS::GPS_OK_FIX_3D &&
           plane.gps.horizontal_accuracy(hacc) &&
           hacc <= start_xtrk_max * RWY_GPS_ACC_FRAC;
}

/*
  runway landing (RWY_ENABLE bit 2). The rollout line runs from the
  NAV_LAND lat/lon (A, the touchdown point on the centreline) to the next
  navigation item, which must be a NAV_WAYPOINT (B) further along the
  centreline. Called from do_land()
 */
void RunwayTakeoff::init_landing(const AP_Mission::Mission_Command &cmd)
{
    land.line_valid = false;
    land.rollout = false;
    land.hold_hdg = false;
    land.td_count = 0;
    land.max_abs_xtrk = 0;
    land.done = false;
    land.rf_lost = false;
    land.wl_factor = 1;
    land.blend = 0;
    land.last_steer_err = 0;
    land.rf_above = -1;
    land.touched = false;
    land.air_count = 0;
    land.bounces = 0;
    land.elev_start_valid = false;
    land.elev_out = 0;
    end_go_around();

    if (!option_set(Option::RUNWAY_LANDING)) {
        return;
    }

    if (wl_height_mode()) {
        if (!state.rf_ref_valid) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "RWY: land runway level from GNDCLR %.2fm",
                          (double)landing_rf_ref());
        }
    } else if (!wow_available()) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "RWY: landing needs RWY_WL_HGT or WoW");
    }

    const Location &a = cmd.content.location;
    Location b;
    if ((a.lat == 0 && a.lng == 0) || !waypoint_after(cmd.index, b)) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "RWY: no WAYPOINT after LAND, stock rollout");
        return;
    }
    const float len = a.get_distance(b);
    if (len < RWY_MIN_LINE_LENGTH_M) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "RWY: LAND to next WP only %.0fm, need %.0fm",
                      (double)len, (double)RWY_MIN_LINE_LENGTH_M);
        return;
    }
    land.point_a = a;
    land.bearing_deg = wrap_360(degrees(a.get_bearing(b)));
    land.line_valid = true;
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "RWY: land line brg %.1f len %.0fm",
                  (double)land.bearing_deg, (double)len);

    // the approach (previous waypoint to LAND) should lie along the line
    if (plane.prev_WP_loc.get_distance(a) > RWY_MIN_LINE_LENGTH_M) {
        const float app_err = wrap_180(degrees(plane.prev_WP_loc.get_bearing(a)) - land.bearing_deg);
        if (fabsf(app_err) > RWY_LAND_APPROACH_TOL_DEG) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "RWY: approach %.0f deg off land line", (double)app_err);
        }
    }
    if (plane.mission.continue_after_land()) {
        // the mission would continue to point B once stopped
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "RWY: MIS_OPTIONS continue after land flies to B");
    }
}

// true on the landing rollout, from touchdown until the landing ends
bool RunwayTakeoff::landing_rollout() const
{
    return land.rollout && option_set(Option::RUNWAY_LANDING) &&
           plane.control_mode == &plane.mode_auto &&
           plane.flight_stage == AP_FixedWing::FlightStage::LAND;
}

// rangefinder reading on the runway: learned on this flight's takeoff,
// otherwise the rangefinder's configured ground clearance
float RunwayTakeoff::landing_rf_ref() const
{
    if (state.rf_ref_valid) {
        return state.rf_ref;
    }
#if AP_RANGEFINDER_ENABLED
    return plane.rangefinder.ground_clearance_orient(plane.rangefinder_orientation());
#else
    return 0;
#endif
}

// RWY_WL_HGT: rangefinder height above the runway; 0 below its minimum range
bool RunwayTakeoff::landing_height_above(float &above) const
{
    if (!wl_height_mode()) {
        return false;
    }
    float hgt;
    bool below_min;
    if (!rangefinder_height(hgt, &below_min)) {
        return false;
    }
    above = below_min ? 0 : MAX(hgt - landing_rf_ref(), 0);
    return true;
}

/*
  called at the navigation rate while the mission is on NAV_LAND:
  touchdown and bounce detection, rollout reporting and logging
 */
void RunwayTakeoff::update_landing()
{
    if (!option_set(Option::RUNWAY_LANDING) || plane.control_mode != &plane.mode_auto) {
        return;
    }
    if (plane.flight_stage != AP_FixedWing::FlightStage::LAND || !hal.util->get_soft_armed()) {
        // go-around, or the landing is over
        land.rollout = false;
        land.td_count = 0;
        land.air_count = 0;
        return;
    }

    float xtrk = 0, along = 0, hdg_err = 0;
    if (land.line_valid) {
        line_errors(land.point_a, land.bearing_deg, xtrk, along);
        hdg_err = wrap_180(plane.ahrs.get_yaw_deg() - land.bearing_deg);
    }
    float above;
    const bool have_hgt = landing_height_above(above);
    land.rf_above = have_hgt ? above : -1;
    const float gspd = plane.ahrs.groundspeed();

    if (!land.rollout) {
        // touchdown: the rangefinder at runway level, or weight on wheels,
        // and not climbing, so a bounce on its way up doesn't count
        bool on_ground = false;
        if (have_hgt) {
            on_ground = above < RWY_TD_HGT_M;
#if AP_LANDINGGEAR_ENABLED
        } else if (wow_available()) {
            on_ground = plane.g2.landing_gear.get_wow_state() == AP_LandingGear::LG_WOW;
#endif
        }
        const bool climbing = plane.auto_state.sink_rate < -RWY_TD_CLIMB_MAX;
        if (!on_ground || climbing) {
            land.td_count = 0;
        } else if (++land.td_count >= RWY_TD_DEBOUNCE_COUNT) {
            land.rollout = true;
            land.touched = true;
            land.td_ms = AP_HAL::millis();
            land.air_count = 0;
            land.elev_start_valid = false;
            if (land.bounces == 0) {
                land.max_abs_xtrk = fabsf(xtrk);
            }
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "RWY: touchdown xt %.1fm hdg err %.1f spd %.1f",
                          (double)xtrk, (double)hdg_err, (double)gspd);
        }
    } else {
        // bounce: off the runway again. Flown as still airborne (stock
        // flare steering and pitch, LEVEL_ROLL_LIMIT) until the wheels
        // are down again
        bool airborne = false;
        if (have_hgt) {
            airborne = above > RWY_BOUNCE_HGT_M;
#if AP_LANDINGGEAR_ENABLED
        } else if (wow_available()) {
            airborne = plane.g2.landing_gear.get_wow_state() == AP_LandingGear::LG_NO_WOW;
#endif
        }
        if (!airborne) {
            land.air_count = 0;
        } else if (++land.air_count >= RWY_BOUNCE_COUNT) {
            land.rollout = false;
            land.td_count = 0;
            land.air_count = 0;
            land.bounces++;
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "RWY: bounce %.1fm spd %.1f, flare until down",
                          (double)(have_hgt ? above : 0), (double)gspd);
        }
        land.max_abs_xtrk = MAX(land.max_abs_xtrk, fabsf(xtrk));
        if (land.rollout && !land.done && gspd < RWY_LAND_STOP_SPEED) {
            land.done = true;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "RWY: rollout done xt %.1fm, max %.1fm",
                          (double)xtrk, (double)land.max_abs_xtrk);
        }
    }

    if (plane.landing.is_flaring() || land.rollout || land.wl_factor < 1) {
        write_landing_log(xtrk, along, hdg_err, gspd);
    }
}

/*
  landing rollout steering. Same law as the takeoff roll, with the
  lookahead held at RWY_LOOK_MAX as the whole rollout is deceleration,
  cross-faded in from the stock steering over RWY_BLEND from touchdown,
  and softened under braking (RWY_BRK_GAIN). Without a line only the
  softening is applied to the stock steering
 */
void RunwayTakeoff::landing_steering_error(int32_t &bearing_error_cd)
{
    float err = bearing_error_cd * 0.01f;

    if (land.line_valid) {
        const float cap = constrain_float(max_corr, 1, 45);
        const float gspd = plane.ahrs.groundspeed();
        float xtrk, along;
        line_errors(land.point_a, land.bearing_deg, xtrk, along);

        if (!is_positive(look_max) || !gps_position_ok()) {
            // as after an abort: hold the runway direction for the rest
            // of the rollout, so a position jump can't steer us
            land.hold_hdg = true;
        }
        float law;
        if (gspd < RWY_ABORT_CENTRE_SPEED) {
            law = 0;
        } else if (land.hold_hdg) {
            law = wrap_180(land.bearing_deg - plane.ahrs.get_yaw_deg());
        } else {
            const float look = MAX(look_max, RWY_MIN_LOOKAHEAD_M);
            const float xt_corr = constrain_float(-degrees(atan2f(xtrk, look)), -cap, cap);
            law = wrap_180(land.bearing_deg + xt_corr - plane.ahrs.get_yaw_deg());
        }
        law = constrain_float(law, -cap, cap);

        float blend = 1;
        if (is_positive(blend_time)) {
            blend = constrain_float((AP_HAL::millis() - land.td_ms) * 0.001f / blend_time, 0, 1);
        }
        land.blend = blend;
        err += (law - err) * blend;
    }

    err *= state.steer_soft;
    land.last_steer_err = err;
    bearing_error_cd = int32_t(err * 100);
}

/*
  called from the AUTO NAV_LAND roll calculation with the stock landing
  roll limit. With RWY_ENABLE bits 1 and 2, the limit falls in
  proportion to the height above the runway (RWY_WL_HGT), or with a
  weight on wheels switch, and is zero from touchdown to the stop. In
  a bounce it is the stock limit (LEVEL_ROLL_LIMIT in the flare) until
  the wheels are down again. It rises no faster than RWY_WL_FADE allows
 */
int32_t RunwayTakeoff::landing_roll_limit_cd(int32_t stock_limit_cd)
{
    const uint32_t now_us = AP_HAL::micros();
    const float dt = constrain_float((now_us - land.last_wl_us) * 1.0e-6f, 0, 0.1f);
    land.last_wl_us = now_us;

    if (!option_set(Option::RUNWAY_LANDING) || !option_set(Option::WINGS_LEVEL) ||
        plane.control_mode != &plane.mode_auto ||
        plane.flight_stage != AP_FixedWing::FlightStage::LAND) {
        land.wl_factor = 1;
        return stock_limit_cd;
    }

    float target = 1;
    float above;
    if (land.rollout) {
        target = 0;
    } else if (land.touched) {
        // a bounce: still airborne, with the stock flare limit
        target = 1;
    } else if (landing_height_above(above)) {
        target = constrain_float(above / wl_hgt, 0, 1);
    } else {
        if (wl_height_mode() && !land.rf_lost) {
            land.rf_lost = true;
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "RWY: rangefinder lost, landing wings level off");
        }
#if AP_LANDINGGEAR_ENABLED
        if (wow_available() && plane.g2.landing_gear.get_wow_state() == AP_LandingGear::LG_WOW) {
            target = 0;
        }
#endif
    }

    if (target < land.wl_factor) {
        land.wl_factor = target;
    } else if (is_positive(wl_fade)) {
        land.wl_factor = MIN(target, land.wl_factor + dt / wl_fade);
    } else {
        land.wl_factor = target;
    }
    return int32_t(stock_limit_cd * land.wl_factor);
}

/*
  called from stabilize_pitch(). On a runway landing rollout, from a
  confirmed touchdown, hold RWY_LND_ELEV elevator instead of the stock
  flare pitch demand, reached over RWY_LND_ELEV_RAMP_S from the elevator
  at touchdown. Returns false when the stock pitch control applies
 */
bool RunwayTakeoff::landing_elevator(float &elevator_cd)
{
    if (lnd_elev == 0 || !landing_rollout()) {
        land.elev_out = 0;
        return false;
    }
    const uint32_t now_ms = AP_HAL::millis();
    if (!land.elev_start_valid) {
        land.elev_start = SRV_Channels::get_output_scaled(SRV_Channel::k_elevator);
        land.elev_start_valid = true;
        land.elev_ms = now_ms;
    }
    // a percentage of travel to the -4500..4500 scaled output, as for
    // TKOFF_TDRAG_ELEV
    const float hold = 45.0f * lnd_elev;
    const float frac = constrain_float((now_ms - land.elev_ms) * 0.001f / RWY_LND_ELEV_RAMP_S, 0, 1);
    elevator_cd = land.elev_start + (hold - land.elev_start) * frac;
    land.elev_out = elevator_cd;
    return true;
}

/*
  called from set_flight_stage() as a landing abort starts, before the
  stock go-around is flown. After a confirmed touchdown it becomes a
  runway takeoff
 */
void RunwayTakeoff::landing_aborted()
{
    if (land.rollout && option_set(Option::RUNWAY_LANDING) &&
        plane.control_mode == &plane.mode_auto && hal.util->get_soft_armed()) {
        start_go_around();
    }
    land.rollout = false;
}

/*
  a go-around commanded after a confirmed touchdown (DO_GO_AROUND, or
  the throttle stick with LAND_ABORT_THR) is flown as a runway takeoff
  along the landing line: the takeoff steering law, wings level until
  rotate speed, and the stock rotate-speed gate on the pitch. The
  below-V1 abort is not armed: a go-around must never stop the engine
 */
void RunwayTakeoff::start_go_around()
{
    if (!land.line_valid) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "RWY: go-around, no landing line, stock");
        return;
    }
    const uint32_t now_ms = AP_HAL::millis();
    state.point_a = land.point_a;
    state.point_b = land.point_a;
    state.point_b.offset_bearing(land.bearing_deg, RWY_LINE_EXTEND_M);
    state.bearing_deg = land.bearing_deg;
    state.length_m = RWY_LINE_EXTEND_M;
    state.l1_start = land.point_a;
    state.l1_start.offset_bearing(wrap_360(land.bearing_deg + 180), RWY_LINE_EXTEND_M);
    state.l1_end = state.point_b;
    state.line_valid = true;
    state.roll_started = true;
    state.go_around = true;
    state.past_v1 = true;
    state.past_rotate = false;
    state.abort_count = 0;
    state.max_abs_xtrk = 0;
    state.rf_lost = false;
    state.launch_ms = now_ms;
    state.last_report_ms = now_ms;
    // the steering is already on the line: no fade-in, and start from the
    // rollout's long lookahead so neither steps
    state.blend_start_ms = MAX(now_ms - uint32_t(MAX(blend_time.get(), 0.0f) * 1000), 1U);
    state.blend = 1;
    state.decel_frac = 1;

    // the rotate-speed gate and the end of the wings level ground phase
    // count speed from now, not from the original takeoff
    plane.auto_state.highest_airspeed = 0;
    plane.auto_state.rotation_complete = false;
    plane.steer_state.hold_course_cd = runway_heading_cd();

    float xtrk, along;
    get_line_errors(xtrk, along);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "RWY: go-around from rollout xt %.1fm, as takeoff",
                  (double)xtrk);
}

/*
  called at the navigation rate during a landing abort, after the stock
  go-around navigation. While a go-around from the rollout is flown as
  a runway takeoff, track the landing line rather than the approach
  heading, and log
 */
void RunwayTakeoff::update_go_around()
{
    if (!state.go_around) {
        return;
    }
    if (plane.control_mode != &plane.mode_auto || !hal.util->get_soft_armed() ||
        plane.flight_stage != AP_FixedWing::FlightStage::ABORT_LANDING ||
        plane.mission.get_current_nav_cmd().id != MAV_CMD_NAV_LAND ||
        plane.adjusted_relative_altitude_cm() > plane.auto_state.takeoff_altitude_rel_cm) {
        // climb-out done (the landing sequence restarts), or abandoned
        end_go_around();
        return;
    }

    update_l1();

    float xtrk, along;
    get_line_errors(xtrk, along);
    const float hdg_err = heading_error_deg();
    update_ground_phase();
    state.max_abs_xtrk = MAX(state.max_abs_xtrk, fabsf(xtrk));
    write_log(xtrk, along, hdg_err, speed_for_v1());
}

void RunwayTakeoff::end_go_around()
{
    if (!state.go_around) {
        return;
    }
    state.go_around = false;
    state.roll_started = false;
    state.line_valid = false;
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "RWY: go-around climb-out done, max xt %.1fm",
                  (double)state.max_abs_xtrk);
}

void RunwayTakeoff::write_landing_log(float xtrk, float along, float hdg_err, float gspd)
{
#if HAL_LOGGING_ENABLED
    // @LoggerMessage: RWYL
    // @Description: Runway landing (RWY_ENABLE bit 2)
    // @Field: TimeUS: Time since system startup
    // @Field: XT: cross-track error from the landing line (LAND point to the next waypoint), +ve right
    // @Field: AT: along-track distance past the LAND point
    // @Field: HE: heading error relative to the landing line, +ve nose right
    // @Field: SE: rollout steering heading error demand, +ve turn right
    // @Field: Spd: ground speed
    // @Field: TD: true from touchdown (rollout)
    // @Field: H: rangefinder height above the runway, -1 if not available
    // @Field: WL: roll limit factor applied, 0 wings held level to 1 stock limit
    // @Field: Bl: fade from the stock steering to the rollout law, 0 to 1
    // @Field: SF: steering softening factor applied under braking, 1 is none
    // @Field: Bn: bounces so far on this landing
    // @Field: EL: elevator held on the rollout (RWY_LND_ELEV), percent; 0 when not held
    AP::logger().WriteStreaming("RWYL", "TimeUS,XT,AT,HE,SE,Spd,TD,H,WL,Bl,SF,Bn,EL", "smmddn-m----%", "F000000000000",
                                "QfffffBffffBf",
                                AP_HAL::micros64(),
                                (double)xtrk,
                                (double)along,
                                (double)hdg_err,
                                (double)land.last_steer_err,
                                (double)gspd,
                                uint8_t(land.rollout),
                                (double)land.rf_above,
                                (double)land.wl_factor,
                                (double)land.blend,
                                (double)state.steer_soft,
                                land.bounces,
                                (double)(land.elev_out / 45.0f));
#endif
}

void RunwayTakeoff::get_line_errors(float &xtrk, float &along) const
{
    line_errors(state.point_a, state.bearing_deg, xtrk, along);
}

// position relative to a line through a on bearing_deg. xtrk +ve to the
// right of the line looking along it, along = metres past a
void RunwayTakeoff::line_errors(const Location &a, float bearing_deg, float &xtrk, float &along) const
{
    const Vector2f d = a.get_distance_NE(plane.current_loc);
    const float hdg = radians(bearing_deg);
    const Vector2f u{cosf(hdg), sinf(hdg)};
    along = d.x * u.x + d.y * u.y;
    xtrk = u.x * d.y - u.y * d.x;
}

/*
  the next navigation command after index, if it is a NAV_WAYPOINT. The
  mission state is not changed (get_next_nav_cmd() can modify DO_JUMP
  counters)
 */
bool RunwayTakeoff::waypoint_after(uint16_t index, Location &loc) const
{
    AP_Mission::Mission_Command next {};
    const uint16_t num_cmds = plane.mission.num_commands();
    for (uint16_t i = index + 1; i < num_cmds; i++) {
        if (!plane.mission.read_cmd_from_storage(i, next)) {
            return false;
        }
        if (AP_Mission::is_nav_cmd(next)) {
            if (next.id != MAV_CMD_NAV_WAYPOINT) {
                return false;
            }
            loc = next.content.location;
            return true;
        }
    }
    return false;
}

// +ve when the nose is right of the A->B direction
float RunwayTakeoff::heading_error_deg() const
{
    return wrap_180(plane.ahrs.get_yaw_deg() - state.bearing_deg);
}

float RunwayTakeoff::speed_for_v1() const
{
    // only a measured airspeed: the synthetic estimate used without a
    // sensor was off by up to 1.8 m/s on the roll
    float aspd;
    if (plane.ahrs.using_airspeed_sensor() && plane.ahrs.airspeed_EAS(aspd)) {
        return aspd;
    }
    return plane.gps.ground_speed();
}

bool RunwayTakeoff::abort_enabled() const
{
    return abort_xtrk > 0 || abort_hdg > 0;
}

/*
  steering lookahead distance: RWY_LOOKAHEAD seconds of travel, at least
  RWY_MIN_LOOKAHEAD_M, and no more than RWY_LOOK_MAX if that is set.
  Staying time-based below the limit keeps the cross-track damping
 */
float RunwayTakeoff::lookahead_m(float gspd) const
{
    float look = MAX(lookahead * gspd, RWY_MIN_LOOKAHEAD_M);
    if (is_positive(look_max)) {
        look = MIN(look, MAX(look_max, RWY_MIN_LOOKAHEAD_M));
    }
    return look;
}

/*
  end of the ground phase by speed: TKOFF_ROTATE_SPD on the same speed
  the stock takeoff rotates on, or RWY_V1 if no rotate speed is set
 */
bool RunwayTakeoff::reached_rotate_speed() const
{
    if (plane.g.takeoff_rotate_speed > 0) {
        return plane.auto_state.highest_airspeed >= plane.g.takeoff_rotate_speed;
    }
    if (v1 > 0) {
        return speed_for_v1() >= v1;
    }
    return false;
}

/*
  height of the landing rangefinder (RNGFND_LND_ORNT) above the ground,
  corrected for attitude. Below its minimum range, as on the wheels, the
  minimum range is returned; above its maximum, the maximum. False if
  it can't be used
 */
bool RunwayTakeoff::rangefinder_height(float &hgt, bool *below_min) const
{
    if (below_min != nullptr) {
        *below_min = false;
    }
#if AP_RANGEFINDER_ENABLED
    const Rotation orient = plane.rangefinder_orientation();
    const RangeFinder &rf = plane.rangefinder;
    switch (rf.status_orient(orient)) {
    case RangeFinder::Status::OutOfRangeLow:
        hgt = rf.min_distance_orient(orient);
        if (below_min != nullptr) {
            *below_min = true;
        }
        return true;
    case RangeFinder::Status::OutOfRangeHigh:
        hgt = rf.max_distance_orient(orient);
        return true;
    case RangeFinder::Status::Good: {
        Vector3f v{rf.distance_orient(orient), 0, 0};
        v.rotate(orient);
        v = plane.ahrs.get_rotation_body_to_ned() * v;
        if (!is_positive(v.z)) {
            // not pointing at the ground
            return false;
        }
        hgt = v.z;
        return true;
    }
    default:
        return false;
    }
#else
    return false;
#endif
}

// true if a weight on wheels switch is configured
bool RunwayTakeoff::wow_available()
{
#if AP_LANDINGGEAR_ENABLED
    return plane.g2.landing_gear.get_wow_state() != AP_LandingGear::LG_WOW_UNKNOWN;
#else
    return false;
#endif
}

/*
  true if an ICE start/stop RC switch is configured and is (or may be)
  asking for the engine to run. With no switch configured (engine
  started and stopped by MAVLink commands) this is always false, so the
  abort clears on disarm.
 */
bool RunwayTakeoff::engine_switch_in_run() const
{
#if AP_ICENGINE_ENABLED
    RC_Channel *c = rc().find_channel_for_option(RC_Channel::AUX_FUNC::ICE_START_STOP);
    if (c == nullptr) {
        return false;
    }
    if (!rc().has_valid_input()) {
        // can't see the switch: stay latched
        return true;
    }
    return c->get_aux_switch_pos() == RC_Channel::AuxSwitchPos::HIGH;
#else
    return false;
#endif
}

void RunwayTakeoff::refuse(const char *fmt, ...)
{
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - state.last_refuse_msg_ms < RWY_REFUSE_MSG_INTERVAL_MS) {
        return;
    }
    state.last_refuse_msg_ms = now_ms;
    va_list ap;
    va_start(ap, fmt);
    gcs().send_textv(MAV_SEVERITY_WARNING, fmt, ap);
    va_end(ap);
}

void RunwayTakeoff::write_log(float xtrk, float along, float hdg_err, float spd)
{
#if HAL_LOGGING_ENABLED
    // @LoggerMessage: RWYT
    // @Description: Runway takeoff tracking
    // @Field: TimeUS: Time since system startup
    // @Field: XT: cross-track error from runway centreline, +ve right
    // @Field: AT: along-track distance past point A towards point B
    // @Field: HE: heading error relative to the A->B direction, +ve nose right
    // @Field: Spd: speed used for the V1 decision
    // @Field: SE: runway steering heading error demand, +ve turn right
    // @Field: V1: true once past V1 (or airborne)
    // @Field: Abt: true once aborted
    // @Field: GP: true in the ground phase (on the runway, or aborted)
    // @Field: WL: roll limit hand-back factor applied, 0 wings held level to 1 stock limit
    // @Field: LA: steering lookahead distance used
    // @Field: DC: decelerating, and why: bit 0 brakes, bit 1 throttle closed or engine off, bit 2 abort
    // @Field: BF: brake applied, 0 to 1
    // @Field: SF: steering softening factor applied under braking, 1 is none
    // @Field: Bl: steering fade-in (RWY_BLEND) factor applied, 0 to 1
    // @Field: BT: time the brakes have been applied continuously
    AP::logger().WriteStreaming("RWYT", "TimeUS,XT,AT,HE,Spd,SE,V1,Abt,GP,WL,LA,DC,BF,SF,Bl,BT", "smmdnd----m----s", "F000000000000000", "QfffffBBBffBffff",
                                AP_HAL::micros64(),
                                (double)xtrk,
                                (double)along,
                                (double)hdg_err,
                                (double)spd,
                                (double)state.last_steer_err,
                                uint8_t(state.past_v1),
                                uint8_t(state.aborted),
                                uint8_t(state.ground_phase),
                                (double)state.wl_factor,
                                (double)state.last_look,
                                state.decel_flags,
                                (double)state.brake_frac,
                                (double)state.steer_soft,
                                (double)state.blend,
                                (double)state.brake_time_s);

    if (wl_height_mode()) {
        // @LoggerMessage: RWYH
        // @Description: Runway takeoff wings level hand-back by height above the runway (RWY_WL_HGT)
        // @Field: TimeUS: Time since system startup
        // @Field: H: rangefinder height above the runway, 0 in the ground phase
        // @Field: Ref: rangefinder reading on the runway, learned in the ground phase
        // @Field: HF: roll limit factor the height allows, 0 to 1
        // @Field: RFok: true if the rangefinder could be used
        AP::logger().WriteStreaming("RWYH", "TimeUS,H,Ref,HF,RFok", "smm--", "F0000", "QfffB",
                                    AP_HAL::micros64(),
                                    (double)state.rf_above,
                                    (double)state.rf_ref,
                                    (double)state.rf_factor,
                                    uint8_t(state.rf_ok));
    }
#endif

    // live values for GCS graphing at 5Hz
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - state.last_log_ms >= 200U) {
        state.last_log_ms = now_ms;
        gcs().send_named_float("RWY_XT", xtrk);
        gcs().send_named_float("RWY_HE", hdg_err);
    }
}

#endif // AP_PLANE_RUNWAY_TAKEOFF_ENABLED
