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

// the RWY_BLEND fade-in starts when the ground speed first passes this
#define RWY_BLEND_START_SPD 1.0f

// below this ground speed after an abort the steering is centred
#define RWY_ABORT_CENTRE_SPEED 2.0f

// number of consecutive navigation cycles (10Hz) an abort condition
// must be present before the abort is triggered
#define RWY_ABORT_DEBOUNCE_COUNT 3

// height above the launch baro altitude at which we assume we are
// airborne and stop considering an abort
#define RWY_ABORT_MAX_HEIGHT_M 5.0f

// slip compensation: the EKF ground track is only used above this
// ground speed (the same as GPS_GND_CRS_MIN_SPD), and the filtered slip
// is limited to this
#define RWY_SLIP_MIN_SPD 5.0f
#define RWY_SLIP_MAX_DEG 10.0f

// weight on wheels must hold a new state this long before the ground
// phase ends or restarts
#define RWY_WOW_DEBOUNCE_MS 500U

#define RWY_REPORT_INTERVAL_MS 2000U
#define RWY_REFUSE_MSG_INTERVAL_MS 3000U

const AP_Param::GroupInfo RunwayTakeoff::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Runway takeoff enable
    // @Description: Enables runway centreline tracking for AUTO takeoffs. The runway line runs from the NAV_TAKEOFF item lat/lon (point A, surveyed centreline point at the start end) to the next mission waypoint (point B, surveyed centreline point at the departure end or beyond). When enabled, auto takeoff is refused if the runway is not defined or the start checks fail.
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLE", 1, RunwayTakeoff, enable, 0, AP_PARAM_FLAG_ENABLE),

    // index 2 was HDG, removed: the runway direction now comes from A->B

    // @Param: STRT_XTRK
    // @DisplayName: Max start cross-track error
    // @Description: Takeoff is refused if the aircraft is further than this from the runway centreline when the takeoff roll would start.
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
    // @Description: Decision speed (equivalent airspeed estimate, or GPS ground speed if no airspeed estimate is available). Below this speed an abort condition stops the engine and applies full brakes. At or above it the takeoff continues. Required if RWY_ABT_XTRK or RWY_ABT_HDG is set, and must not exceed TKOFF_ROTATE_SPD.
    // @Units: m/s
    // @Range: 0 60
    // @User: Advanced
    AP_GROUPINFO("V1", 8, RunwayTakeoff, v1, 0),

    // @Param: BRK_CHAN
    // @DisplayName: Wheel brake servo output
    // @Description: Servo output number (1-based) driving the wheel brakes. On abort this output is held at RWY_BRK_PWM, overriding whatever function the output has (including RC passthrough). 0 disables brake control.
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
    // @Description: Time over which the runway steering correction is faded in, to avoid a step in the steering demand. The fade starts when the aircraft starts moving (ground speed above 1 m/s) after the roll is triggered.
    // @Units: s
    // @Range: 0 10
    // @User: Advanced
    AP_GROUPINFO("BLEND", 11, RunwayTakeoff, blend_time, 2),

    // @Param: MAX_CORR
    // @DisplayName: Max steering correction
    // @Description: Largest heading correction the runway steering will command on the ground, so no position jump or large error can command a hard turn. The steering rate demand is this divided by STEER2SRV_TCONST.
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

    // @Param: ABT_ISCL
    // @DisplayName: Steering integrator kept at abort
    // @Description: Fraction of the steering integrator kept when a takeoff abort starts. The integrator holds the crosswind correction built up on the roll, which is still needed during the stop. 1 keeps all of it, 0 clears it.
    // @Range: 0 1
    // @Increment: 0.05
    // @User: Advanced
    AP_GROUPINFO("ABT_ISCL", 15, RunwayTakeoff, abt_iscl, 1),

    // @Param: ABT_GAIN
    // @DisplayName: Steering gain while aborted
    // @Description: Multiplies the heading error the steering acts on while stopping after an abort. Braking loads the nosewheel and makes it more effective, so a value below 1 keeps the steering loop stable under braking.
    // @Range: 0.1 1
    // @Increment: 0.05
    // @User: Advanced
    AP_GROUPINFO("ABT_GAIN", 16, RunwayTakeoff, abt_gain, 1),

    // @Param: WINGS_LVL
    // @DisplayName: Wings level on the ground
    // @Description: Holds the wings level (zero roll demand) on the takeoff roll and while stopping after an abort, until the aircraft leaves the ground. The ground phase ends on weight on wheels (LGR_WOW_PIN) when fitted, otherwise when the airspeed reaches TKOFF_ROTATE_SPD (RWY_V1 if that is 0). The takeoff roll limit then fades in over RWY_WL_DELAY and RWY_WL_FADE. When disabled, the L1 roll demand is used on the ground, limited to LEVEL_ROLL_LIMIT.
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("WINGS_LVL", 17, RunwayTakeoff, wings_lvl, 1),

    // @Param: WL_DELAY
    // @DisplayName: Roll limit fade delay
    // @Description: Time after the aircraft leaves the ground (weight on wheels, or rotate speed reached) before the takeoff roll limit starts fading in from zero.
    // @Units: s
    // @Range: 0 5
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("WL_DELAY", 18, RunwayTakeoff, wl_delay, 0),

    // @Param: WL_FADE
    // @DisplayName: Roll limit fade time
    // @Description: Time over which the takeoff roll limit fades from zero to the normal takeoff limit (LEVEL_ROLL_LIMIT, then the TKOFF_LVL_ALT scaling) after RWY_WL_DELAY. 0 restores it at once.
    // @Units: s
    // @Range: 0 10
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("WL_FADE", 19, RunwayTakeoff, wl_fade, 1),

    // @Param: RUD_MAX
    // @DisplayName: Rudder share of ground steering
    // @Description: When the nosewheel has its own Ground Steering output, the rudders normally get the coordinated flight output on the ground. If non-zero, during the ground phase of a runway takeoff or abort (see RWY_WINGS_LVL) the rudders instead get this fraction of the steering output, scheduled with speed from RWY_RUD_SPD1 to RWY_RUD_SPD2, and fade back to the coordinated output with the roll limit after liftoff. 0 disables this.
    // @Range: 0 1
    // @Increment: 0.05
    // @User: Advanced
    AP_GROUPINFO("RUD_MAX", 20, RunwayTakeoff, rud_max, 0),

    // @Param: RUD_SPD1
    // @DisplayName: Rudder share start speed
    // @Description: At or below this speed (airspeed sensor if in use, else GPS ground speed) the rudder share of the steering is zero, so the rudders are centred.
    // @Units: m/s
    // @Range: 0 30
    // @Increment: 0.5
    // @User: Advanced
    AP_GROUPINFO("RUD_SPD1", 21, RunwayTakeoff, rud_spd1, 5),

    // @Param: RUD_SPD2
    // @DisplayName: Rudder share full speed
    // @Description: At or above this speed the rudder share of the steering is RWY_RUD_MAX. Between RWY_RUD_SPD1 and this it rises linearly.
    // @Units: m/s
    // @Range: 0 40
    // @Increment: 0.5
    // @User: Advanced
    AP_GROUPINFO("RUD_SPD2", 22, RunwayTakeoff, rud_spd2, 15),

    // @Param: XT_I
    // @DisplayName: Cross-track integral gain
    // @Description: Gain of a slow integral of the cross-track error added to the runway steering heading demand, to remove a standing offset from the centreline. Degrees of heading demand per second, per metre of cross-track error. 0 disables it.
    // @Units: deg/m/s
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("XT_I", 23, RunwayTakeoff, xt_i, 0),

    // @Param: XT_IMAX
    // @DisplayName: Cross-track integral limit
    // @Description: Largest heading demand the cross-track integral can add.
    // @Units: deg
    // @Range: 0 10
    // @Increment: 0.5
    // @User: Advanced
    AP_GROUPINFO("XT_IMAX", 24, RunwayTakeoff, xt_imax, 3),

    // @Param: XT_ISPD
    // @DisplayName: Cross-track integral min speed
    // @Description: The cross-track integral only builds above this ground speed.
    // @Units: m/s
    // @Range: 0 30
    // @Increment: 0.5
    // @User: Advanced
    AP_GROUPINFO("XT_ISPD", 25, RunwayTakeoff, xt_ispd, 8),

    // @Param: LOOK_MAX
    // @DisplayName: Max steering lookahead distance
    // @Description: If non-zero, caps the RWY_LOOKAHEAD distance (minimum 10 m). Below the cap the lookahead stays time based. 0 means no cap.
    // @Units: m
    // @Range: 0 200
    // @Increment: 5
    // @User: Advanced
    AP_GROUPINFO("LOOK_MAX", 26, RunwayTakeoff, look_max, 0),

    // @Param: SLIP_TC
    // @DisplayName: Slip compensation time constant
    // @Description: If non-zero, the runway steering compares the A->B direction with yaw plus the low-pass filtered difference between the EKF ground track and yaw, using this time constant. The steering keeps the quick yaw response but holds the ground track to the line, which removes a yaw estimate bias and tyre slip in crosswind. Used above 5 m/s ground speed. 0 disables it.
    // @Units: s
    // @Range: 0 5
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("SLIP_TC", 27, RunwayTakeoff, slip_tc, 0),

    AP_GROUPEND
};

RunwayTakeoff::RunwayTakeoff(void)
{
    AP_Param::setup_object_defaults(this, var_info);
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
    state.past_v1 = false;
    state.abort_count = 0;
    state.max_abs_xtrk = 0;
    state.last_steer_err = 0;
    state.last_refuse_msg_ms = 0;
    state.on_ground = false;
    state.gnd_src = GroundSource::NONE;
    state.release_ms = 0;
    state.xt_i = 0;
    state.slip_f = 0;

    if (!enabled()) {
        return;
    }

    const Location &a = cmd.content.location;
    if (a.lat == 0 && a.lng == 0) {
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "RWY: TAKEOFF item has no lat/lon (point A)");
        return;
    }

    // find the next navigation command without changing mission state
    // (get_next_nav_cmd() can modify DO_JUMP counters)
    AP_Mission::Mission_Command next {};
    bool found = false;
    const uint16_t num_cmds = plane.mission.num_commands();
    for (uint16_t i = cmd.index + 1; i < num_cmds; i++) {
        if (!plane.mission.read_cmd_from_storage(i, next)) {
            break;
        }
        if (AP_Mission::is_nav_cmd(next)) {
            found = true;
            break;
        }
    }
    if (!found || next.id != MAV_CMD_NAV_WAYPOINT) {
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "RWY: need WAYPOINT after TAKEOFF (point B)");
        return;
    }
    const Location &b = next.content.location;
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
    state.abort_count = 0;
    state.max_abs_xtrk = 0;
    state.launch_ms = AP_HAL::millis();
    state.last_report_ms = state.launch_ms;
    state.last_steer_ms = state.launch_ms;
    state.xt_i = 0;
    state.last_slip_ms = state.launch_ms;
    state.slip_f = 0;
    state.moving_ms = 0;
    state.blend = 0;

    // steer along the runway from the start of the roll. A non -1
    // hold_course_cd selects calc_nav_yaw_course(), which takes its
    // error from ground_steering_error()
    plane.steer_state.hold_course_cd = runway_heading_cd();

    // on the ground until weight on wheels or rotate speed says otherwise
    state.gnd_src = ground_source();
    state.on_ground = state.gnd_src != GroundSource::NONE;
    state.release_ms = 0;

    float xtrk, along;
    get_line_errors(xtrk, along);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "RWY: roll xt %.1fm hdg err %.1f rem %.0fm",
                  (double)xtrk, (double)heading_error_deg(), (double)(state.length_m - along));
    if (state.gnd_src == GroundSource::NONE && (wings_lvl != 0 || is_positive(rud_max))) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "RWY: no WoW or rotate speed, wings level and rudder share off");
    }
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
  ground steering law. Aim at a point on the centreline a lookahead
  distance ahead, cap the correction, and fade it in from the start of
  the roll. After an abort, hold the runway direction instead of
  chasing the line, and centre the steering once nearly stopped.
  Result is a heading error (desired - current), centidegrees.
 */
void RunwayTakeoff::ground_steering_error(int32_t &bearing_error_cd)
{
    if (!active() || plane.control_mode != &plane.mode_auto ||
        !(state.roll_started || state.aborted)) {
        return;
    }

    const float cap = constrain_float(max_corr, 1, 45);
    const float gspd = plane.ahrs.groundspeed();
    // yaw plus the filtered slip: follows yaw for fast changes and the
    // EKF ground track for slow ones (0 slip when RWY_SLIP_TC is 0)
    const float yaw_ref = plane.ahrs.get_yaw_deg() + state.slip_f;
    float err;

    if (state.aborted) {
        if (gspd < RWY_ABORT_CENTRE_SPEED) {
            err = 0;
        } else {
            // hold a track parallel to the runway. Braking makes the
            // nosewheel more effective, so steer more gently
            err = wrap_180(state.bearing_deg - yaw_ref) * constrain_float(abt_gain, 0.1, 1);
        }
    } else {
        float xtrk, along;
        get_line_errors(xtrk, along);
        const uint32_t now_ms = AP_HAL::millis();

        // fade in once the aircraft starts moving, not from the launch
        // trigger, which can come while it is still held on the brakes
        if (state.moving_ms == 0 && gspd > RWY_BLEND_START_SPD) {
            state.moving_ms = MAX(now_ms, uint32_t(1));
        }
        float blend = 1;
        if (is_positive(blend_time)) {
            if (state.moving_ms == 0) {
                blend = 0;
            } else {
                blend = constrain_float((now_ms - state.moving_ms) * 0.001f / blend_time, 0, 1);
            }
        }
        state.blend = blend;

        // slow crosstrack integral for a standing offset the lookahead
        // law leaves. Only once fully faded in and while the last demand
        // was inside the cap, so it can't wind up
        const float dt = MIN((now_ms - state.last_steer_ms) * 0.001f, 0.1f);
        state.last_steer_ms = now_ms;
        if (!is_positive(xt_i)) {
            state.xt_i = 0;
        } else if (gspd > xt_ispd && blend >= 1 && fabsf(state.last_steer_err) < cap) {
            const float imax = MAX(xt_imax.get(), 0.0f);
            state.xt_i = constrain_float(state.xt_i - xtrk * xt_i * dt, -imax, imax);
        }

        // time based lookahead, optionally capped in distance so a
        // heading error turns into less cross-track at high speed
        float look = MAX(lookahead * gspd, RWY_MIN_LOOKAHEAD_M);
        if (is_positive(look_max)) {
            look = MIN(look, MAX(look_max.get(), RWY_MIN_LOOKAHEAD_M));
        }
        state.look_m = look;
        const float xt_corr = constrain_float(-degrees(atan2f(xtrk, look)) + state.xt_i, -cap, cap);
        err = wrap_180(state.bearing_deg + xt_corr - yaw_ref);
        err *= blend;
    }

    err = constrain_float(err, -cap, cap);
    state.last_steer_err = err;
    bearing_error_cd = int32_t(err * 100);
}

void RunwayTakeoff::update_monitor()
{
    if (!active() || !state.roll_started) {
        return;
    }

    const uint32_t now_ms = AP_HAL::millis();
    float xtrk, along;
    get_line_errors(xtrk, along);
    const float hdg_err = heading_error_deg();
    const float spd = speed_for_v1();

    update_ground_phase();
    update_slip(now_ms);

    state.max_abs_xtrk = MAX(state.max_abs_xtrk, fabsf(xtrk));
    write_log(xtrk, along, hdg_err, spd);

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
        } else if (plane.barometer.get_altitude() - plane.auto_state.baro_takeoff_alt > RWY_ABORT_MAX_HEIGHT_M ||
                   (state.gnd_src == GroundSource::WOW && !state.on_ground)) {
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

void RunwayTakeoff::takeoff_complete()
{
    if (!active() || !state.roll_started) {
        return;
    }
    state.roll_started = false;
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "RWY: takeoff complete, max xt %.1fm",
                  (double)state.max_abs_xtrk);
}

/*
  low-pass filtered slip (EKF ground track - yaw), at the navigation
  rate. Held below RWY_SLIP_MIN_SPD, where the track is poorly defined,
  so an abort slowing through that speed gets no step
 */
void RunwayTakeoff::update_slip(uint32_t now_ms)
{
    const float dt = MIN((now_ms - state.last_slip_ms) * 0.001f, 0.2f);
    state.last_slip_ms = now_ms;
    if (!is_positive(slip_tc)) {
        state.slip_f = 0;
        return;
    }
    const Vector2f &v = plane.ahrs.groundspeed_vector();
    if (v.length() <= RWY_SLIP_MIN_SPD) {
        return;
    }
    const float track = degrees(atan2f(v.y, v.x));
    const float slip = wrap_180(track - plane.ahrs.get_yaw_deg());
    state.slip_f += (slip - state.slip_f) * dt / (dt + slip_tc);
    state.slip_f = constrain_float(state.slip_f, -RWY_SLIP_MAX_DEG, RWY_SLIP_MAX_DEG);
}

// speed at which the aircraft is taken to leave the ground when there
// is no weight on wheels sensor
float RunwayTakeoff::rotate_speed() const
{
    if (is_positive(plane.g.takeoff_rotate_speed)) {
        return plane.g.takeoff_rotate_speed;
    }
    return v1;
}

RunwayTakeoff::GroundSource RunwayTakeoff::ground_source()
{
#if AP_LANDINGGEAR_ENABLED
    if (plane.g2.landing_gear.get_wow_state() != AP_LandingGear::LG_WOW_UNKNOWN) {
        return GroundSource::WOW;
    }
#endif
    if (is_positive(rotate_speed())) {
        return GroundSource::ROTATE_SPEED;
    }
    return GroundSource::NONE;
}

/*
  decide whether the aircraft is still on the ground. Called at the
  navigation rate from update_monitor(); takeoff_calc_roll() and
  stabilize_yaw() only read the result
 */
void RunwayTakeoff::update_ground_phase()
{
    const bool was_on_ground = state.on_ground;
    state.gnd_src = ground_source();

    switch (state.gnd_src) {
#if AP_LANDINGGEAR_ENABLED
    case GroundSource::WOW:
        // debounced both ways, so a bounce goes back to the ground phase
        if (plane.g2.landing_gear.get_wow_state_duration_ms() >= RWY_WOW_DEBOUNCE_MS) {
            state.on_ground = plane.g2.landing_gear.get_wow_state() == AP_LandingGear::LG_WOW;
        }
        break;
#endif
    case GroundSource::ROTATE_SPEED:
        // the speed stock uses to rotate. highest_airspeed only grows,
        // so this latches
        state.on_ground = plane.auto_state.highest_airspeed < rotate_speed();
        break;
    case GroundSource::NONE:
    default:
        state.on_ground = false;
        break;
    }

    if (was_on_ground && !state.on_ground && !state.aborted) {
        state.release_ms = MAX(AP_HAL::millis(), uint32_t(1));
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "RWY: off the ground (%s), roll limit fading in",
                      state.gnd_src == GroundSource::WOW ? "WoW" : "rotate speed");
    } else if (!was_on_ground && state.on_ground) {
        // weight back on the wheels: wings level again, and the fade
        // restarts at the next release
        state.release_ms = 0;
    }
}

bool RunwayTakeoff::ground_phase() const
{
    if (!active() || plane.control_mode != &plane.mode_auto ||
        !(state.roll_started || state.aborted)) {
        return false;
    }
    return state.aborted || state.on_ground;
}

float RunwayTakeoff::handover_factor() const
{
    if (ground_phase()) {
        return 0;
    }
    if (!active() || state.release_ms == 0) {
        // no ground phase on this takeoff
        return 1;
    }
    const float t = (AP_HAL::millis() - state.release_ms) * 0.001f - MAX(wl_delay.get(), 0.0f);
    if (t <= 0) {
        return 0;
    }
    if (!is_positive(wl_fade)) {
        return 1;
    }
    return MIN(t / wl_fade, 1.0f);
}

/*
  rudder output while ground steering with a separate nosewheel output.
  During the ground phase the rudders take a speed scheduled share of
  the steering output instead of the coordinated flight output, then
  fade back to it with the roll limit after liftoff
 */
float RunwayTakeoff::ground_rudder(float steering_out, float coordinated_out)
{
    state.rud_share = 0;
    if (!is_positive(rud_max) || !active() || plane.control_mode != &plane.mode_auto) {
        return coordinated_out;
    }
    const float f = handover_factor();
    if (f >= 1) {
        return coordinated_out;
    }
    const float share = linear_interpolate(0, constrain_float(rud_max, 0, 1),
                                           speed_for_v1(),
                                           rud_spd1, MAX(rud_spd2.get(), rud_spd1.get()));
    state.rud_share = (1 - f) * share;
    return state.rud_share * steering_out + f * coordinated_out;
}

void RunwayTakeoff::trigger_abort(const char *reason, float xtrk, float hdg_err, float spd)
{
    state.aborted = true;

    state.abort_ms = AP_HAL::millis();
    GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "RWY: ABORT %s xt %.1f he %.1f v %.1f",
                  reason, (double)xtrk, (double)hdg_err, (double)spd);

    // the steering integrator holds the crosswind correction built up
    // on the roll, which is still needed during the stop. Keep the
    // configured fraction of it
    plane.steerController.scale_I(constrain_float(abt_iscl, 0, 1));

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
}

/*
  run every loop after the servo outputs have been written but before
  they are pushed to the hardware
 */
void RunwayTakeoff::update_outputs()
{
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

void RunwayTakeoff::get_line_errors(float &xtrk, float &along) const
{
    const Vector2f d = state.point_a.get_distance_NE(plane.current_loc);
    const float hdg = radians(state.bearing_deg);
    const Vector2f u{cosf(hdg), sinf(hdg)};
    along = d.x * u.x + d.y * u.y;
    // +ve to the right of the centreline looking in the takeoff direction
    xtrk = u.x * d.y - u.y * d.x;
}

// +ve when the nose is right of the A->B direction
float RunwayTakeoff::heading_error_deg() const
{
    return wrap_180(plane.ahrs.get_yaw_deg() - state.bearing_deg);
}

float RunwayTakeoff::speed_for_v1() const
{
    float aspd;
    if (plane.ahrs.airspeed_EAS(aspd)) {
        return aspd;
    }
    return plane.gps.ground_speed();
}

bool RunwayTakeoff::abort_enabled() const
{
    return abort_xtrk > 0 || abort_hdg > 0;
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
    AP::logger().WriteStreaming("RWYT", "TimeUS,XT,AT,HE,Spd,SE,V1,Abt", "smmdnd--", "F0000000", "QfffffBB",
                                AP_HAL::micros64(),
                                (double)xtrk,
                                (double)along,
                                (double)hdg_err,
                                (double)spd,
                                (double)state.last_steer_err,
                                uint8_t(state.past_v1),
                                uint8_t(state.aborted));
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
