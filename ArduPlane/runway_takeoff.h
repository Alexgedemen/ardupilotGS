#pragma once

/*
  Runway centreline tracking for AUTO takeoff, with a start sanity
  check and a below-V1 ground-roll abort.

  The runway line is defined by two surveyed centreline points carried
  in the mission, mirroring how a landing uses the waypoint before
  NAV_LAND:
    - A: the lat/lon of the NAV_TAKEOFF item (start end of the runway)
    - B: the next navigation item, which must be a NAV_WAYPOINT
         (departure end, or further out on the extended centreline)

  From the start of the roll the nosewheel steers along A->B using a
  simple lookahead law (RWY_LOOKAHEAD), faded in over RWY_BLEND and
  capped at RWY_MAX_CORR. The L1 controller is fed the same line so that
  after liftoff the roll demand follows the extended centreline until
  the takeoff altitude is reached.

  While decelerating (brakes, throttle closed or engine off, abort) the
  lookahead moves smoothly to RWY_LOOK_MAX. After an abort the steering
  returns gently to the centreline with that lookahead and a reduced
  correction limit, or holds the runway direction if RWY_LOOK_MAX is 0
  or the GPS accuracy is poor, and the brakes are ramped to full over
  RWY_BRK_RAMP.

  While AUTO holds the takeoff, the takeoff is aborted if the aircraft
  moves with the brakes on and the engine running, and the GCS is warned
  before the brakes reach their operating time limit (RWY_BRK_TMAX).

  RWY_ENABLE bit 1 holds the wings level while on the runway, including
  during an abort, and hands back to the stock roll limit over
  RWY_WL_FADE once rotate speed is reached; with RWY_WL_HGT, in
  proportion to the downward rangefinder's height above the runway.

  RWY_ENABLE bit 2 tracks the runway centreline on the landing rollout,
  along the line from the NAV_LAND lat/lon to the next waypoint, with
  the lookahead held at RWY_LOOK_MAX and the steering softened under
  braking; with bit 1 the wings are held level near and on the runway.

  With RWY_ENABLE=0 the stock takeoff behaviour is unchanged.
 */

#ifndef AP_PLANE_RUNWAY_TAKEOFF_ENABLED
#define AP_PLANE_RUNWAY_TAKEOFF_ENABLED 1
#endif

#if AP_PLANE_RUNWAY_TAKEOFF_ENABLED

#include <AP_Param/AP_Param.h>
#include <AP_Common/Location.h>
#include <AP_Mission/AP_Mission.h>

class RunwayTakeoff
{
public:
    RunwayTakeoff(void);

    CLASS_NO_COPY(RunwayTakeoff);

    static const struct AP_Param::GroupInfo var_info[];

    // RWY_ENABLE bits
    enum class Option : uint8_t {
        RUNWAY_TAKEOFF = (1U << 0),
        WINGS_LEVEL    = (1U << 1),
        RUNWAY_LANDING = (1U << 2),
    };

    // bit 0 enables the feature; the other bits only act with it set
    bool enabled() const { return (uint8_t(enable.get()) & uint8_t(Option::RUNWAY_TAKEOFF)) != 0; }
    bool option_set(Option opt) const { return enabled() && (uint8_t(enable.get()) & uint8_t(opt)) != 0; }

    // called from do_takeoff() with the NAV_TAKEOFF mission item
    void init(const AP_Mission::Mission_Command &cmd);

    // true when a valid runway line has been set up for this takeoff
    bool active() const { return enabled() && state.line_valid; }

    // true from the start of the roll until takeoff complete
    bool roll_started() const { return state.roll_started; }

    // gate called from auto_takeoff_check() before the throttle is
    // released. Returns true if it is OK to start the takeoff roll.
    bool launch_allowed(bool in_auto);

    // called when the takeoff roll has been triggered
    void launch_started();

    // runway (A->B) bearing in centidegrees, for steer_state.hold_course_cd
    int32_t runway_heading_cd() const;

    // feed the L1 controller with the runway line (roll demand)
    void update_l1();

    // called from takeoff_calc_roll() with the stock takeoff roll
    // limit. Returns the limit to apply: zero while on the runway
    // (RWY_ENABLE bit 1), then faded back to the stock limit
    int32_t ground_roll_limit_cd(int32_t stock_limit_cd);

    // called from calc_nav_yaw_course(). Replaces the L1 bearing error
    // with the runway ground steering error while on the takeoff roll,
    // stopping after an abort, or on a runway landing rollout.
    void ground_steering_error(int32_t &bearing_error_cd);

    // called at the navigation rate from verify_takeoff() for
    // monitoring, abort checks, GCS reporting and logging
    void update_monitor();

    // called when the takeoff altitude has been reached
    void takeoff_complete();

    // true once an abort has been triggered (latched until disarmed)
    bool aborted() const { return state.aborted; }

    // called every loop from servos_output() after the outputs have
    // been calculated. Applies the brake override while aborted and
    // clears the abort latch when safe.
    void update_outputs();

    // runway landing (RWY_ENABLE bit 2). init_landing() is called from
    // do_land() with the NAV_LAND item; update_landing() at the
    // navigation rate while on NAV_LAND
    void init_landing(const AP_Mission::Mission_Command &cmd);
    void update_landing();

    // called from the AUTO NAV_LAND roll calculation with the stock
    // landing roll limit. Returns the limit to apply
    int32_t landing_roll_limit_cd(int32_t stock_limit_cd);

private:
    // parameters
    AP_Int8  enable;
    AP_Float start_xtrk_max;   // m
    AP_Float start_hdg_max;    // deg
    AP_Float start_dist_min;   // m
    AP_Float abort_xtrk;       // m, 0 disables
    AP_Float abort_hdg;        // deg, 0 disables
    AP_Float v1;               // m/s
    AP_Int8  brake_chan;       // servo output number, 0 disables
    AP_Int16 brake_pwm;        // PWM for full brakes
    AP_Float blend_time;       // s
    AP_Float max_corr;         // deg
    AP_Float lookahead;        // s
    AP_Float brake_ramp;       // s
    AP_Float wl_fade;          // s
    AP_Float look_max;         // m, 0 disables
    AP_Float brk_gain;         // steering gain at full brake, 1 disables
    AP_Float brk_tmax;         // s, brake operating time warning, 0 disables
    AP_Float wl_hgt;           // m, wings level hand-back by height, 0 disables

    struct {
        Location point_a;      // TAKEOFF item: start of the runway line
        Location point_b;      // next waypoint: far end of the line
        Location l1_start;     // A->B extended behind A, for L1
        Location l1_end;       // A->B extended beyond B, for L1
        float bearing_deg;     // A->B
        float length_m;        // A->B
        bool line_valid;
        bool roll_started;
        bool past_v1;
        bool aborted;
        bool we_set_estop;
        uint8_t abort_count;   // consecutive cycles with an abort condition
        float max_abs_xtrk;
        float last_steer_err;  // deg, for logging
        uint32_t launch_ms;
        uint32_t abort_ms;
        uint16_t brake_start_pwm;
        uint32_t last_report_ms;
        uint32_t last_refuse_msg_ms;
        uint32_t last_log_ms;
        uint32_t last_clear_msg_ms;
        bool ground_phase;     // on the runway: wings held level
        bool past_rotate;      // rotate speed reached on this roll
        uint32_t last_gp_us;   // last update_ground_phase() call
        float wl_factor;       // roll limit hand-back, 0 (level) to 1 (stock)
        bool rf_ok;            // RWY_WL_HGT: rangefinder usable
        bool rf_ref_valid;
        bool rf_lost;          // rangefinder failure reported on this roll
        float rf_ref;          // m, rangefinder reading on the runway
        float rf_above;        // m, height above the runway
        float rf_factor;       // roll limit factor the height allows
        float last_look;       // m, steering lookahead used, for logging
        float brake_frac;      // brake applied, 0 to 1
        uint16_t brake_out_pwm; // abort brake override last written, 0 if none
        uint8_t decel_flags;   // DecelFlag bits
        float decel_frac;      // 0 normal lookahead to 1 RWY_LOOK_MAX
        uint32_t last_brake_us;
        bool abort_hold_hdg;   // latched: hold the runway direction for the rest of the stop
        float brake_soft;      // brake applied, rate limited, for steer_soft
        float steer_soft;      // steering softening factor applied, RWY_BRK_GAIN to 1
        uint32_t blend_start_ms; // when the aircraft started rolling, 0 before
        float blend;           // RWY_BLEND fade-in factor applied, 0 to 1
        uint8_t hold_count;    // consecutive cycles moving on the brake hold
        uint32_t brake_on_ms;  // when the brakes were applied, 0 if released
        float brake_time_s;    // continuous brake time
        uint8_t brake_warn_level; // 0 none, 1 warning, 2 limit reached
        uint32_t last_brake_warn_ms;
    } state;

    // runway landing
    struct {
        Location point_a;      // LAND item: touchdown point on the centreline
        float bearing_deg;     // LAND point to the next waypoint
        bool line_valid;
        bool rollout;          // from touchdown until the landing ends
        bool hold_hdg;         // latched: hold the runway direction
        bool done;             // rollout done reported
        bool rf_lost;          // rangefinder failure reported
        uint8_t td_count;      // consecutive cycles at runway level
        uint32_t td_ms;        // touchdown time
        uint32_t last_wl_us;
        float wl_factor;       // roll limit factor, 0 (level) to 1 (stock)
        float blend;           // fade from the stock steering, 0 to 1
        float last_steer_err;  // deg, for logging
        float max_abs_xtrk;
        float rf_above;        // m, rangefinder height above the runway, -1 if none
    } land;

    // causes of deceleration, logged in RWYT.DC
    enum class DecelFlag : uint8_t {
        BRAKES   = (1U << 0),
        THROTTLE = (1U << 1),  // throttle closed or engine off
        ABORT    = (1U << 2),
    };

    // geometry relative to the runway line. xtrk +ve = right of the
    // centreline looking from A to B, along = metres past A towards B
    void get_line_errors(float &xtrk, float &along) const;
    void line_errors(const Location &a, float bearing_deg, float &xtrk, float &along) const;
    bool waypoint_after(uint16_t index, Location &loc) const;
    float heading_error_deg() const;
    float speed_for_v1() const;
    bool abort_enabled() const;
    bool reached_rotate_speed() const;
    bool wl_height_mode() const { return is_positive(wl_hgt); }
    bool rangefinder_height(float &hgt, bool *below_min = nullptr) const;
    float lookahead_m(float gspd) const;
    void update_brakes();
    float brake_fraction() const;
    bool throttle_closed_or_engine_off() const;
    bool gps_position_ok() const;
    void check_brakes_holding(float xtrk, float hdg_err);
    void check_brake_time(uint32_t now_ms);
    bool engine_running() const;
    bool wow_available();
    void update_ground_phase();

    void trigger_abort(const char *reason, float xtrk, float hdg_err, float spd);
    bool engine_switch_in_run() const;
    void refuse(const char *fmt, ...) FMT_PRINTF(2, 3);
    void write_log(float xtrk, float along, float hdg_err, float spd);

    bool landing_rollout() const;
    float landing_rf_ref() const;
    bool landing_height_above(float &above) const;
    void landing_steering_error(int32_t &bearing_error_cd);
    void write_landing_log(float xtrk, float along, float hdg_err, float gspd);
};

#endif // AP_PLANE_RUNWAY_TAKEOFF_ENABLED
