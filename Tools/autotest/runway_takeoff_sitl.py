#!/usr/bin/env python3
"""
SITL scenarios for the runway centreline takeoff (RWY_*) feature.

Starts arduplane SITL directly (no MAVProxy), drives it over MAVLink with
pymavlink and reports the cross-track error during the takeoff roll.

usage:
  python3 Tools/autotest/runway_takeoff_sitl.py [scenario ...]

scenarios: stock, track, refuse, nodef, nowp, abort, abort_steer, past_v1,
           mission, all (default)
steering scenarios (checked against the DataFlash log, PASS/FAIL):
           hold, abort_i, wl_rotate, wl_wow, rudder, iref, lookmax, slip,
           disarm, steering (all of these)

environment:
  RWY_SPEEDUP   SITL speedup (default 5)
  RWY_INSTANCE  SITL instance, so several copies can run at once (default 0)
  RWY_BINARY    arduplane binary to run (default build/sitl/bin/arduplane)
  RWY_LENIENT_PARAMS=1  skip parameters the binary doesn't have, e.g. to
                run the same scenarios on an older build for comparison

The runway is synthetic, heading RWY_HDG_DEG true near CMAC. Point A (the
TAKEOFF item) is the threshold, point B (the next waypoint) is B_DIST_M
down the runway. The aircraft is placed a few metres right of the
centreline and yawed off the runway heading, with a crosswind, to make
the stock heading-hold drift.

Limits of the plane-ice-steering model: on the ground it holds roll and
side velocity at zero, steers by yaw rate only and has no brakes or
nosewheel loading. The steering scenarios check logic, outputs and logs,
not the braked or wheel-lift dynamics the changes are for.
"""

import bisect
import glob
import math
import os
import subprocess
import sys
import time

from pymavlink import mavutil

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
BINARY = os.environ.get("RWY_BINARY", os.path.join(ROOT, "build", "sitl", "bin", "arduplane"))
INSTANCE = int(os.environ.get("RWY_INSTANCE", "0"))
SPEEDUP = float(os.environ.get("RWY_SPEEDUP", "5"))

THRESH_LAT = -35.363261
THRESH_LON = 149.165230
THRESH_ALT = 584
RWY_HDG_DEG = 90.0
B_DIST_M = 1500.0

EARTH_R = 6378137.0


def offset(lat, lon, bearing_deg, dist_m):
    b = math.radians(bearing_deg)
    dn = math.cos(b) * dist_m
    de = math.sin(b) * dist_m
    lat2 = lat + math.degrees(dn / EARTH_R)
    lon2 = lon + math.degrees(de / (EARTH_R * math.cos(math.radians(lat))))
    return lat2, lon2


B_LAT, B_LON = offset(THRESH_LAT, THRESH_LON, RWY_HDG_DEG, B_DIST_M)


def line_errors(lat, lon):
    """xtrk (+ve right) and along-track (m, relative to threshold)"""
    dn = math.radians(lat - THRESH_LAT) * EARTH_R
    de = math.radians(lon - THRESH_LON) * EARTH_R * math.cos(math.radians(THRESH_LAT))
    h = math.radians(RWY_HDG_DEG)
    along = dn * math.cos(h) + de * math.sin(h)
    xtrk = math.cos(h) * de - math.sin(h) * dn
    return xtrk, along


BASE_PARAMS = {
    # ground steering on SERVO5 for the plane-ice-steering SITL frame
    "SERVO5_FUNCTION": 26,
    "GROUND_STEER_ALT": 5,
    "STEER2SRV_P": 3.0,
    "STEER2SRV_I": 0.0,
    "STEER2SRV_D": 0.0,
    "STEER2SRV_MINSPD": 2,
    # takeoff
    "TKOFF_ROTATE_SPD": 15,
    "TKOFF_THR_MAX": 45,   # slow acceleration -> long ground roll
    "TKOFF_THR_MINSPD": 0,
    "TKOFF_THR_MINACC": 0,
    "TKOFF_THR_DELAY": 0,
    "TKOFF_GND_PITCH": 0,
    "LEVEL_ROLL_LIMIT": 5,
    # brakes on SERVO9 as RC9 passthrough
    "SERVO9_FUNCTION": 59,
    # crosswind from the right
    "SIM_WIND_SPD": 6,
    "SIM_WIND_DIR": 180,
    "SIM_WIND_TURB": 1,
    "LOG_DISARMED": 0,
    "RC_OVERRIDE_TIME": -1,   # overrides never time out (test determinism)
    "RC11_OPTION": 0,         # no engine RC switch (engine by MAVLink command)
    "STICK_MIXING": 1,
}

RWY_PARAMS = {
    "RWY_ENABLE": 1,
    "RWY_STRT_XTRK": 3,
    "RWY_STRT_HDG": 5,
    "RWY_STRT_DIST": 1000,
    "RWY_ABT_XTRK": 0,
    "RWY_ABT_HDG": 0,
    "RWY_V1": 0,
    "RWY_BRK_CHAN": 9,
    "RWY_BRK_PWM": 1900,
    "RWY_BRK_RAMP": 1,
    "RWY_BLEND": 2,
    "RWY_MAX_CORR": 10,
    "RWY_LOOKAHEAD": 3,
}


class Sim:
    def __init__(self, start_offset_m, start_yaw_err_deg, instance=INSTANCE, home_alt=THRESH_ALT):
        lat, lon = offset(THRESH_LAT, THRESH_LON, RWY_HDG_DEG + 90, start_offset_m)
        yaw = (RWY_HDG_DEG + start_yaw_err_deg) % 360
        home = "%.8f,%.8f,%.1f,%.1f" % (lat, lon, home_alt, yaw)
        self.workdir = "/tmp/rwy_sitl_%d" % instance
        os.makedirs(self.workdir, exist_ok=True)
        for f in os.listdir(self.workdir):
            if f.endswith(".bin") or f == "eeprom.bin":
                os.unlink(os.path.join(self.workdir, f))
        for f in glob.glob(os.path.join(self.workdir, "logs", "*")):
            os.unlink(f)
        defaults = ",".join([
            os.path.join(ROOT, "Tools/autotest/models/plane.parm"),
            os.path.join(ROOT, "Tools/autotest/default_params/plane-ice.parm"),
            # sets RC11_OPTION 0 at boot: engine by MAVLink command only
            os.path.join(ROOT, "Tools/autotest/runway_takeoff_sitl.parm"),
        ])
        cmd = [BINARY, "--model", "plane-ice-steering", "--home", home,
               "--defaults", defaults, "--speedup", "%g" % SPEEDUP, "-I%d" % instance, "-w"]
        self.proc = subprocess.Popen(cmd, cwd=self.workdir,
                                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(2)
        self.mav = mavutil.mavlink_connection("tcp:127.0.0.1:%d" % (5760 + 10 * instance),
                                              source_system=255)
        self.mav.wait_heartbeat(timeout=30)
        self.texts = []
        self.track = []      # (t, xtrk, along, spd, alt)
        self.brake_pwm = []  # (t, pwm9, pwm3)
        self.att = []        # (t, yaw_deg, yawrate_dps)
        self.last_rc = {}
        for stream in (mavutil.mavlink.MAV_DATA_STREAM_ALL,):
            self.mav.mav.request_data_stream_send(1, 1, stream, 20, 1)

    def close(self):
        try:
            self.proc.terminate()
            self.proc.wait(5)
        except Exception:
            self.proc.kill()

    def pump(self, duration=0.0):
        end = time.time() + duration
        while True:
            m = self.mav.recv_match(blocking=True, timeout=0.05)
            if m is not None:
                self.handle(m)
            if time.time() >= end:
                break

    def handle(self, m):
        t = m.get_type()
        if t == "STATUSTEXT":
            self.texts.append(m.text)
            print("    [GCS] %s" % m.text)
        elif t == "GLOBAL_POSITION_INT":
            x, a = line_errors(m.lat * 1e-7, m.lon * 1e-7)
            spd = math.hypot(m.vx, m.vy) * 0.01
            self.track.append((time.time(), x, a, spd, m.relative_alt * 0.001, m.lat * 1e-7, m.lon * 1e-7))
        elif t == "SERVO_OUTPUT_RAW":
            self.brake_pwm.append((time.time(), m.servo9_raw, m.servo3_raw, m.time_usec * 1e-6))
        elif t == "ATTITUDE":
            self.att.append((time.time(), math.degrees(m.yaw), math.degrees(m.yawspeed)))

    def set_param(self, name, value):
        for _ in range(5):
            self.mav.param_set_send(name, float(value))
            t0 = time.time()
            while time.time() - t0 < 1.5:
                m = self.mav.recv_match(blocking=True, timeout=0.1)
                if m is None:
                    continue
                if m.get_type() == "PARAM_VALUE" and m.param_id == name:
                    if abs(m.param_value - float(value)) < 1e-3:
                        return
                else:
                    self.handle(m)
        if os.environ.get("RWY_LENIENT_PARAMS"):
            print("  (no parameter %s in this build, skipped)" % name)
            return
        raise RuntimeError("failed to set %s" % name)

    def set_params(self, d):
        for k, v in d.items():
            self.set_param(k, v)

    def get_param(self, name):
        for _ in range(5):
            self.mav.mav.param_request_read_send(1, 1, name.encode(), -1)
            t0 = time.time()
            while time.time() - t0 < 1.5:
                m = self.mav.recv_match(blocking=True, timeout=0.1)
                if m is None:
                    continue
                if m.get_type() == "PARAM_VALUE" and m.param_id == name:
                    return m.param_value
                self.handle(m)
        raise RuntimeError("failed to read %s" % name)

    def rc_override(self, chans):
        self.last_rc.update(chans)
        vals = [65535] * 18
        for ch, pwm in self.last_rc.items():
            vals[ch - 1] = pwm
        self.mav.mav.rc_channels_override_send(1, 1, *vals)

    def upload_mission(self, ref=True, b_cmd=None, items_extra=None):
        wp = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
        rel = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
        items = []
        # 0: home
        items.append((wp, 0, 0, 0, 0, THRESH_LAT, THRESH_LON, 0, mavutil.mavlink.MAV_FRAME_GLOBAL))
        # 1: takeoff, lat/lon = point A (runway threshold on the centreline)
        items.append((mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 10, 0, 0, 0,
                      THRESH_LAT if ref else 0, THRESH_LON if ref else 0, 40, rel))
        # 2: point B, departure end of the runway on the centreline
        items.append((b_cmd if b_cmd is not None else wp, 0, 0, 0, 0, B_LAT, B_LON, 60, rel))
        if items_extra is not None:
            items.extend(items_extra)
        else:
            far_lat, far_lon = offset(THRESH_LAT, THRESH_LON, RWY_HDG_DEG, 4000)
            items.append((wp, 0, 0, 0, 0, far_lat, far_lon, 80, rel))
            items.append((mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM, 0, 0, 0, 0, far_lat, far_lon, 80, rel))
        self.mav.mav.mission_count_send(1, 1, len(items))
        t0 = time.time()
        while time.time() - t0 < 20:
            m = self.mav.recv_match(type=["MISSION_REQUEST", "MISSION_REQUEST_INT", "MISSION_ACK"],
                                    blocking=True, timeout=1)
            if m is None:
                continue
            if m.get_type() == "MISSION_ACK":
                if m.type != 0:
                    raise RuntimeError("mission ack %d" % m.type)
                return
            cmd, p1, p2, p3, p4, lat, lon, alt, frame = items[m.seq]
            self.mav.mav.mission_item_int_send(1, 1, m.seq, frame, cmd, 0, 1, p1, p2, p3, p4,
                                               int(lat * 1e7), int(lon * 1e7), alt,
                                               mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
        raise RuntimeError("mission upload timeout")

    def wait_ready(self):
        # wait for EKF to be using GPS
        t0 = time.time()
        while time.time() - t0 < 120:
            m = self.mav.recv_match(type=["EKF_STATUS_REPORT", "STATUSTEXT", "GPS_RAW_INT"],
                                    blocking=True, timeout=1)
            if m is None:
                continue
            if m.get_type() == "STATUSTEXT":
                self.handle(m)
            if m.get_type() == "EKF_STATUS_REPORT" and (m.flags & 0x10) and (m.flags & 0x08) \
                    and time.time() - t0 > 20:
                return
        raise RuntimeError("EKF not ready")

    def set_mode(self, name):
        mode_id = self.mav.mode_mapping()[name]
        self.mav.set_mode(mode_id)
        self.pump(1)

    def arm(self):
        self.mav.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                                       1, 2989, 0, 0, 0, 0, 0)
        self.pump(1.5)

    def disarm(self):
        self.mav.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                                       0, 21196, 0, 0, 0, 0, 0)
        self.pump(1.5)

    def start_engine(self):
        # no engine RC switch: start with MAV_CMD_DO_ENGINE_CONTROL,
        # allowing a start while disarmed (flags bit 0)
        self.rc_override({9: 1100, 3: 1000})
        self.mav.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_DO_ENGINE_CONTROL, 0,
                                       1, 0, 0, 1, 0, 0, 0)
        self.pump(8)

    def run_takeoff(self, seconds):
        t_end = time.time() + seconds
        while time.time() < t_end:
            self.rc_override({})   # keep overrides alive
            self.pump(0.5)

    def roll_stats(self, max_alt=1.0):
        pts = [p for p in self.track if p[3] > 1.0 and p[4] < max_alt]
        if not pts:
            return None
        xs = [p[1] for p in pts]
        return {
            "n": len(pts),
            "start_xt": xs[0],
            "end_xt": xs[-1],
            "max_abs_xt": max(abs(x) for x in xs),
            "max_spd": max(p[3] for p in pts),
            "dist": pts[-1][2] - pts[0][2],
        }

    def yaw_rate_stats(self, t_from, t_to):
        """max |yaw rate| between two wall-clock times"""
        rates = [abs(r) for (t, _, r) in self.att if t_from <= t <= t_to]
        return max(rates) if rates else None


def scenario(name, params, start_offset, yaw_err, run_s, extra=None, ref=True, b_cmd=None, items_extra=None,
             home_alt=THRESH_ALT, after_ready=None):
    print("\n=== %s: start offset %.1fm right, yaw err %+.1f deg ===" % (name, start_offset, yaw_err))
    sim = Sim(start_offset, yaw_err, home_alt=home_alt)
    res = {}
    try:
        sim.set_params(BASE_PARAMS)
        sim.set_params(params)
        if os.environ.get("RWY_EXTRA_PARAMS"):
            import json
            sim.set_params(json.loads(os.environ["RWY_EXTRA_PARAMS"]))
        sim.wait_ready()
        if after_ready is not None:
            after_ready(sim)
        for attempt in range(3):
            try:
                sim.upload_mission(ref=ref, b_cmd=b_cmd, items_extra=items_extra)
                break
            except RuntimeError as ex:
                print("  mission upload retry: %s" % ex)
                sim.pump(2)
        sim.set_mode("AUTO")
        sim.arm()
        t_roll = time.time()
        # SITL only drives the starter when armed, so start after arming
        sim.start_engine()
        if extra is not None:
            extra(sim)
        else:
            sim.run_takeoff(run_s)
        st = sim.roll_stats()
        air = [p for p in sim.track if p[4] > 30]
        res.update({"stats": st, "texts": list(sim.texts),
                    "airborne_xt": air[0][1] if air else None,
                    "final_alt": sim.track[-1][4] if sim.track else None,
                    "brake": list(sim.brake_pwm),
                    "max_yaw_rate_roll": sim.yaw_rate_stats(t_roll, t_roll + 25)})
        print("  roll stats: %s" % st)
        # cross-track profile against distance down the runway
        prof = []
        nxt = 0
        for (_, x, a, spd, alt, _, _) in sim.track:
            if a >= nxt and spd > 0.5:
                prof.append("%4.0fm:%+5.1f%s" % (a, x, "*" if alt > 1 else ""))
                nxt += 50
            if a > 1200:
                break
        print("  xt profile (along:xt, *=airborne): " + " ".join(prof))
        if air:
            print("  xt when passing 30m AGL: %.1fm" % air[0][1])
        if res["max_yaw_rate_roll"] is not None:
            print("  max |yaw rate| in first 25 s of roll: %.1f deg/s" % res["max_yaw_rate_roll"])
        res["extra"] = getattr(sim, "extra_result", None)
        return res
    finally:
        sim.close()
        res["log"] = newest_log(sim.workdir)


def report_brake_ramp(sim):
    """time for SERVO9 to go from its value at the abort to full"""
    t_abort = None
    for (_, pwm, _, tb) in sim.brake_pwm:
        if pwm > 1110 and t_abort is None:
            t_abort = tb
        if t_abort is not None and pwm >= 1895:
            print("  brake ramp: SERVO9 reached %d %.2f s (sim time) after it started rising" % (pwm, tb - t_abort))
            return
    print("  brake ramp: full brake not seen")


def abort_extra(sim):
    # run until abort, check engine/brakes, then check the engine stays
    # stopped and the abort clears on disarm
    t0 = time.time()
    while time.time() - t0 < 60 and not any("ABORT" in t for t in sim.texts):
        sim.rc_override({})
        sim.pump(0.3)
    sim.run_takeoff(12)
    print("  last SERVO3 (thr) %s SERVO9 (brake) %s" %
          (sim.brake_pwm[-1][2], sim.brake_pwm[-1][1]))
    report_brake_ramp(sim)
    # a start command while the abort is latched must not start the engine
    sim.mav.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_DO_ENGINE_CONTROL, 0,
                                  1, 0, 0, 0, 0, 0, 0)
    sim.run_takeoff(6)
    sim.disarm()
    for _ in range(8):
        sim.rc_override({})
        sim.pump(0.5)
    print("  after disarm SERVO9 %s" % sim.brake_pwm[-1][1])


def abort_steer_extra(sim):
    """steering working; a pilot rudder kick pushes it off the line at
    speed, the abort fires, then check the stop stays straight"""
    t0 = time.time()
    kicked = False
    while time.time() - t0 < 60 and not any("ABORT" in t for t in sim.texts):
        spd = sim.track[-1][3] if sim.track else 0
        if not kicked and spd > 10:
            kicked = True
            # about 1 s of sim time of right rudder
            sim.rc_override({4: 1650})
            sim.pump(1.0 / float(os.environ.get("RWY_SPEEDUP", "5")))
            sim.rc_override({4: 1500})
        sim.rc_override({})
        sim.pump(0.3)
    t_abort = time.time()
    sim.run_takeoff(15)
    after = [p for p in sim.track if p[0] >= t_abort]
    if after:
        xs = [p[1] for p in after]
        print("  after abort: xt from %+.1f to %+.1f m, extremes %+.1f / %+.1f m" %
              (xs[0], xs[-1], min(xs), max(xs)))
    yaws = [((y - RWY_HDG_DEG + 180) % 360) - 180 for (t, y, _) in sim.att if t >= t_abort]
    if yaws:
        print("  after abort: heading error extremes %+.1f / %+.1f deg, final %+.1f" %
              (min(yaws), max(yaws), yaws[-1]))
    report_brake_ramp(sim)


def mission_items_other_runway():
    """after takeoff, fly to a second runway 2 km north of the first,
    heading 180, and land on it (WP before LAND -> LAND on its centreline)"""
    wp = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
    rel = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
    r2_lat, r2_lon = offset(THRESH_LAT, THRESH_LON, 0, 2000)        # runway 2 threshold
    app_lat, app_lon = offset(r2_lat, r2_lon, 0, 1200)              # approach point on extended centreline
    land_lat, land_lon = offset(r2_lat, r2_lon, 180, 150)           # touchdown point
    far_lat, far_lon = offset(THRESH_LAT, THRESH_LON, RWY_HDG_DEG, 3000)
    turn_lat, turn_lon = offset(app_lat, app_lon, 90, 1500)
    return [
        (wp, 0, 0, 0, 0, far_lat, far_lon, 80, rel),
        (wp, 0, 0, 0, 0, turn_lat, turn_lon, 80, rel),
        (wp, 0, 0, 0, 0, app_lat, app_lon, 40, rel),
        (mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, land_lat, land_lon, 0, rel),
    ], (r2_lat, r2_lon)


def mission_extra(sim):
    t_end = time.time() + 600 / float(os.environ.get("RWY_SPEEDUP", "5"))
    while time.time() < t_end and not any("Throttle disarmed" in t for t in sim.texts):
        sim.rc_override({})
        sim.pump(0.5)
    # landing rollout against runway 2's centreline (heading 180 through its threshold)
    _, (r2_lat, r2_lon) = mission_items_other_runway()
    roll = [p for p in sim.track if p[4] < 1.0 and p[3] > 1.0 and
            abs(p[5] - r2_lat) * 111000 < 1500]
    if roll:
        xs = []
        for p in roll:
            de = math.radians(p[6] - r2_lon) * EARTH_R * math.cos(math.radians(r2_lat))
            xs.append(-de)   # heading 180: right of the line is west
        print("  landing rollout on runway 2: %d samples, xt from %+.1f to %+.1f m, max |xt| %.1f m" %
              (len(xs), xs[0], xs[-1], max(abs(x) for x in xs)))
    else:
        print("  landing rollout on runway 2: not seen")


#
# steering scenarios, checked against the DataFlash log
#

STEER_I = 0.2       # STEER2SRV_I for the scenarios that need an integrator
CHECKS = []         # (name, passed, detail)
STEERING = ("hold", "abort_i", "wl_rotate", "wl_wow", "rudder", "iref", "lookmax", "slip", "disarm")


def check(name, ok, detail):
    CHECKS.append((name, bool(ok), detail))
    print("  %s %s: %s" % ("PASS" if ok else "FAIL", name, detail))


def newest_log(workdir):
    bins = glob.glob(os.path.join(workdir, "logs", "*.BIN"))
    return max(bins, key=os.path.getmtime) if bins else None


class Log:
    """messages of the given types from a DataFlash log, as dicts in time
    order, plus the parameters"""

    def __init__(self, path, types):
        self.msgs = {t: [] for t in types}
        self.params = {}
        if path is not None:
            mlog = mavutil.mavlink_connection(path)
            while True:
                m = mlog.recv_match(type=list(types) + ["PARM"])
                if m is None:
                    break
                if m.get_type() == "PARM":
                    self.params[m.Name] = m.Value
                elif m.get_type() == "GPS" and m.I != 0:
                    continue
                elif m.get_type() == "XKF1" and m.C != 0:
                    continue
                else:
                    self.msgs[m.get_type()].append(m.to_dict())
        self.times = {t: [d["TimeUS"] for d in v] for t, v in self.msgs.items()}

    def before(self, mtype, t_us):
        """last message of mtype at or before t_us"""
        i = bisect.bisect_right(self.times[mtype], t_us) - 1
        return self.msgs[mtype][i] if i >= 0 else None

    def after(self, mtype, t_us):
        """first message of mtype after t_us"""
        i = bisect.bisect_right(self.times[mtype], t_us)
        return self.msgs[mtype][i] if i < len(self.msgs[mtype]) else None

    def gspd(self, t_us):
        """EKF ground speed, which the steering uses (GPS lags it)"""
        k = self.before("XKF1", t_us)
        return math.hypot(k["VN"], k["VE"]) if k is not None else 0.0

    def track(self, t_us):
        """EKF ground track, deg"""
        k = self.before("XKF1", t_us)
        return math.degrees(math.atan2(k["VE"], k["VN"])) if k is not None else None


def wrap180(a):
    return ((a + 180) % 360) - 180


def mean(xs):
    return sum(xs) / len(xs) if xs else float("nan")


def max_desroll_below_rotate(log):
    """max |ATT.DesRoll| while the RWYT speed is below TKOFF_ROTATE_SPD,
    before any abort: the roll demand on the ground roll"""
    vr = log.params.get("TKOFF_ROTATE_SPD", 15)
    worst = 0.0
    for a in log.msgs["ATT"]:
        r = log.before("RWYT", a["TimeUS"])
        if r is None or r["Abt"] or r["Spd"] >= vr:
            continue
        worst = max(worst, abs(a["DesRoll"]))
    return worst


def hold_extra(sim):
    # the start has been refused since arming (RWY_HACC_MAX 0.1 with the
    # SITL GPS reporting 0.5 m). Hold a little longer, then let it go
    sim.run_takeoff(30 / SPEEDUP)
    sim.set_param("SIM_GPS1_ACC", 0.05)
    sim.run_takeoff(60 / SPEEDUP)


def check_hold(res):
    texts = res["texts"]
    refused = [i for i, t in enumerate(texts) if "RWY: GPS hacc" in t]
    rolled = [i for i, t in enumerate(texts) if "RWY: roll xt" in t]
    check("9b start refused on GPS accuracy, then allowed",
          refused and rolled and refused[0] < rolled[0],
          "refusals %d, roll started %s" % (len(refused), bool(rolled)))
    log = Log(res["log"], ["PIDS", "XKF1", "RWYT"])
    minspd = log.params.get("STEER2SRV_MINSPD", 2)
    worst = 0.0
    n = 0
    for p in log.msgs["PIDS"]:
        if log.gspd(p["TimeUS"]) >= minspd - 0.2:
            continue
        n += 1
        worst = max(worst, abs(p["I"]))
    check("1 steering integrator zero below STEER2SRV_MINSPD", n > 0 and worst < 1e-3,
          "%d PIDS samples below %.1f m/s, max |I| %.4f" % (n, minspd, worst))
    rw = log.msgs["RWYT"]
    first = next((r for r in rw if r["Bl"] > 0), None)
    early = [r for r in rw if r["Bl"] > 0 and log.gspd(r["TimeUS"]) < 0.9]
    if first is None or not rw:
        check("8 blend starts when moving", False, "no RWYT samples with Bl > 0")
    else:
        check("8 blend starts when moving", not early,
              "blend first > 0 at EKF speed %.2f m/s, %.1f s after the roll started, %d samples below 0.9 m/s" %
              (log.gspd(first["TimeUS"]), (first["TimeUS"] - rw[0]["TimeUS"]) * 1e-6, len(early)))


def check_abort_i(res, iscl, gain):
    log = Log(res["log"], ["PIDS", "RWYT", "XKF1"])
    rw = log.msgs["RWYT"]
    i_ab = next((i for i, r in enumerate(rw) if r["Abt"]), None)
    if i_ab is None or i_ab == 0:
        check("2/3 abort happened", False, "no abort in the log")
        return None
    # RWYT is written before the abort check, so the abort came just
    # after the last sample without it. SITL time doesn't advance within
    # a loop, so take PIDS from the loops strictly before and after
    t_ab = rw[i_ab - 1]["TimeUS"]
    pb = log.before("PIDS", t_ab - 1)
    pa = log.after("PIDS", t_ab)
    ib, ia = pb["I"], pa["I"]
    # the nosewheel trim offset makes the integrator hold a correction
    if iscl >= 1:
        check("2 RWY_ABT_ISCL 1: no integrator step at the abort", abs(ib) > 0.5 and abs(ia - ib) < 0.3,
              "I %.2f -> %.2f" % (ib, ia))
    else:
        check("2 RWY_ABT_ISCL 0: integrator cleared at the abort", abs(ib) > 0.5 and abs(ia) < 0.3,
              "I %.2f -> %.2f" % (ib, ia))
    cap = log.params.get("RWY_MAX_CORR", 10)
    worst = 0.0
    n = 0
    for r in rw[i_ab:]:
        if log.gspd(r["TimeUS"]) < 2.5 or abs(gain * r["HE"]) > cap - 0.1:
            continue
        n += 1
        worst = max(worst, abs(r["SE"] + gain * r["HE"]))
    check("3 aborted steering error = -RWY_ABT_GAIN x heading error (gain %.1f)" % gain,
          n > 0 and worst < 0.15, "%d samples, max |SE + gain x HE| %.3f deg" % (n, worst))
    xs = [r["XT"] for r in rw[i_ab:]]
    hes = [r["HE"] for r in rw[i_ab:] if log.gspd(r["TimeUS"]) > 2.5]
    print("  after the abort: xt %+.2f -> %+.2f m (extremes %+.2f / %+.2f), heading error extremes %+.1f / %+.1f deg" %
          (xs[0], xs[-1], min(xs), max(xs), min(hes) if hes else 0, max(hes) if hes else 0))
    return {"xt_growth": max(abs(x - xs[0]) for x in xs)}


def compress(seq):
    out = []
    for s in seq:
        if not out or out[-1] != s:
            out.append(s)
    return out


def check_wings_level(res, src):
    """src: 1 weight on wheels, 2 rotate speed"""
    log = Log(res["log"], ["RWYT", "ATT"])
    rw = log.msgs["RWYT"]
    seq = compress([int(r["WL"]) for r in rw])
    check("4 wings level state %d, then handover, then off" % src, seq[:3] == [src, 3, 0],
          "WL sequence %s" % seq)
    msg = "WoW" if src == 1 else "rotate speed"
    check("4 liftoff message (%s)" % msg, any(("liftoff (%s)" % msg) in t for t in res["texts"]), "")
    i_rel = next((i for i, r in enumerate(rw) if r["WL"] == 3), None)
    if i_rel is None:
        return
    delay = log.params.get("RWY_WL_DELAY", 0)
    fade = log.params.get("RWY_WL_FADE", 1)
    t_rel = rw[i_rel - 1]["TimeUS"]
    r_pos = next((r for r in rw[i_rel:] if r["HF"] > 0), None)
    r_one = next((r for r in rw[i_rel:] if r["HF"] >= 0.999), None)
    if r_pos is None or r_one is None:
        check("4 roll limit fades in", False, "HF never reached 1 in the log")
    else:
        t_pos = (r_pos["TimeUS"] - t_rel) * 1e-6
        t_one = (r_one["TimeUS"] - t_rel) * 1e-6
        check("4 roll limit fade: delay %.1f s then %.1f s" % (delay, fade),
              delay - 0.05 <= t_pos <= delay + 0.25 and delay + fade - 0.15 <= t_one <= delay + fade + 0.25,
              "HF first > 0 at %.2f s, reaches 1 at %.2f s after release" % (t_pos, t_one))
    worst0 = 0.0
    n0 = 0
    for a in log.msgs["ATT"]:
        rb = log.before("RWYT", a["TimeUS"])
        ra = log.after("RWYT", a["TimeUS"])
        if rb is None or ra is None or rb["WL"] == 0 or rb["HF"] > 0 or ra["HF"] > 0:
            continue
        n0 += 1
        worst0 = max(worst0, abs(a["DesRoll"]))
    check("4 roll demand zero on the ground and during the delay", n0 > 0 and worst0 < 0.05,
          "%d ATT samples, max |DesRoll| %.3f deg" % (n0, worst0))
    print("  max |DesRoll| below rotate speed: %.2f deg" % max_desroll_below_rotate(log))


def servo_norm(params, ch, pwm):
    trim = params.get("SERVO%d_TRIM" % ch, 1500)
    lo = params.get("SERVO%d_MIN" % ch, 1100)
    hi = params.get("SERVO%d_MAX" % ch, 1900)
    v = (pwm - trim) / (hi - trim) if pwm >= trim else (pwm - trim) / (trim - lo)
    return -v if params.get("SERVO%d_REVERSED" % ch, 0) else v


def rudder_kick_extra(sim):
    """a pilot rudder input at speed, mixed into the steering output,
    so the rudder share is checked at a real deflection"""
    t0 = time.time()
    while time.time() - t0 < 60 and (not sim.track or sim.track[-1][3] < 9):
        sim.rc_override({})
        sim.pump(0.3)
    sim.rc_override({4: 1700})
    sim.pump(1.5 / SPEEDUP)
    sim.rc_override({4: 1500})
    sim.run_takeoff(10)


def check_rudder(res, liftoff):
    log = Log(res["log"], ["RWYT", "RCOU"])
    p = log.params
    rmax = p.get("RWY_RUD_MAX", 0)
    s1 = p.get("RWY_RUD_SPD1", 5)
    s2 = p.get("RWY_RUD_SPD2", 15)
    rw = log.msgs["RWYT"]
    worst = 0.0
    n = 0
    for r in rw:
        if r["WL"] not in (1, 2):
            continue
        frac = min(max((r["Spd"] - s1) / (s2 - s1), 0.0), 1.0)
        n += 1
        worst = max(worst, abs(r["RSh"] - rmax * frac))
    check("4b rudder share follows the speed schedule", n > 0 and worst < 0.04,
          "%d ground samples, max |RSh - schedule| %.3f" % (n, worst))
    worst = 0.0
    n = 0
    big = 0
    for c in log.msgs["RCOU"]:
        rb = log.before("RWYT", c["TimeUS"])
        ra = log.after("RWYT", c["TimeUS"])
        if rb is None or ra is None or rb["WL"] not in (1, 2) or ra["WL"] not in (1, 2):
            continue
        rud = servo_norm(p, 4, c["C4"])
        steer = servo_norm(p, 5, c["C5"])
        sh = 0.5 * (rb["RSh"] + ra["RSh"])
        err = abs(rud - sh * steer)
        n += 1
        if abs(steer) > 0.05 and sh > 0.1:
            big += 1
        worst = max(worst, err - 0.1 * abs(steer))
    if not liftoff:
        check("4b rudder output = share x steering output on the ground", big > 0 and worst < 0.02,
              "%d RCOU samples (%d with real deflection), worst excess %.3f" % (n, big, worst))
    else:
        after = [r for r in rw if r["WL"] == 0 and r["HF"] >= 0.999]
        check("4b rudder share 0 after the handover", after and all(r["RSh"] == 0 for r in after),
              "%d samples" % len(after))


def steady(log, along_min=500, along_max=2500, spd_min=10):
    return [r for r in log.msgs["RWYT"]
            if along_min < r["AT"] < along_max and r["Spd"] >= spd_min and not r["Abt"]]


def roll_summary(res):
    log = Log(res["log"], ["RWYT", "ATT", "XKF1"])
    st = steady(log)
    xs = [r["XT"] for r in st]
    out = {
        "n": len(st),
        "mean_abs_xt": mean([abs(x) for x in xs]),
        "mean_xt": mean(xs),
        "sd_xt": math.sqrt(mean([(x - mean(xs)) ** 2 for x in xs])) if xs else float("nan"),
        "mean_abs_se": mean([abs(r["SE"]) for r in st]),
        "xti": st[-1].get("XTI", 0) if st else float("nan"),
        "slp": mean([r.get("Slp", 0) for r in st]),
    }
    # EKF track minus yaw on the steady part of the roll
    trk = []
    for r in st:
        t = log.track(r["TimeUS"])
        a = log.before("ATT", r["TimeUS"])
        if t is not None and a is not None and log.gspd(r["TimeUS"]) > 5:
            trk.append(wrap180(t - a["Yaw"]))
    out["track_minus_yaw"] = mean(trk)
    print("  steady roll (%d samples, 500-2500 m, >= 10 m/s): mean xt %+.2f m, mean |xt| %.2f m, sd %.2f m, "
          "mean |SE| %.2f deg, XTI %.2f deg, Slp %+.2f deg, EKF track - yaw %+.2f deg" %
          (out["n"], out["mean_xt"], out["mean_abs_xt"], out["sd_xt"], out["mean_abs_se"], out["xti"],
           out["slp"], out["track_minus_yaw"]))
    return out


def check_lookmax(res):
    log = Log(res["log"], ["RWYT", "XKF1"])
    p = log.params
    la_t = p.get("RWY_LOOKAHEAD", 3)
    cap = p.get("RWY_LOOK_MAX", 0)
    worst = 0.0
    below = above = 0
    for r in log.msgs["RWYT"]:
        if r["Abt"] or r["Bl"] <= 0:
            continue
        gs = log.gspd(r["TimeUS"])
        expect = min(max(la_t * gs, 10.0), max(cap, 10.0))
        if la_t * gs < cap - 3:
            below += 1
        elif la_t * gs > cap + 3:
            above += 1
        else:
            continue
        worst = max(worst, abs(r["LA"] - expect))
    check("7 lookahead = RWY_LOOKAHEAD x speed below the cap, RWY_LOOK_MAX above",
          below > 0 and above > 0 and worst < 1.5,
          "%d samples below the cap, %d above, max |LA - expected| %.2f m" % (below, above, worst))


def slip_after_ready(sim):
    # bias the yaw estimate: declination 0.1 rad (about 6 deg) off its
    # automatic value, which is only set once the GPS has a fix. The EKF
    # learns part of it back from the GPS velocity while accelerating
    dec = sim.get_param("COMPASS_DEC")
    sim.set_param("COMPASS_AUTODEC", 0)
    sim.set_param("COMPASS_DEC", dec + 0.1)
    print("  COMPASS_DEC %.3f -> %.3f rad" % (dec, dec + 0.1))
    sim.pump(30 / SPEEDUP)


def disarm_extra(sim):
    t0 = time.time()
    while time.time() - t0 < 60 and (not sim.track or sim.track[-1][3] < 6):
        sim.rc_override({})
        sim.pump(0.3)
    sim.disarm()
    n = len(sim.texts)
    # any cross-track now exceeds the abort limit
    sim.set_param("RWY_ABT_XTRK", 0.01)
    sim.run_takeoff(40 / SPEEDUP)
    after = sim.texts[n:]
    sim.extra_result = {
        "aborts_after_disarm": [t for t in after if "ABORT" in t],
        "reports_after_disarm": [t for t in after if "RWY: roll" in t],
    }


def check_disarm(res):
    ex = res["extra"] or {}
    ab = ex.get("aborts_after_disarm", ["(no result)"])
    check("9a no abort after disarm", not ab, "%d abort messages after disarm %s" % (len(ab), ab[:1]))


def main():
    want = sys.argv[1:] or ["all"]
    results = {}

    def on(n):
        return "all" in want or n in want or (n in STEERING and "steering" in want)

    if on("stock"):
        results["stock"] = scenario("stock (RWY_ENABLE=0)", {"RWY_ENABLE": 0}, 2.0, 3.0, 45)
    if on("track"):
        results["track"] = scenario("runway tracking", RWY_PARAMS, 2.0, 3.0, 45)
    if on("refuse"):
        p = dict(RWY_PARAMS)
        results["refuse"] = scenario("start check refuses (5m off)", p, 5.0, 0.0, 15)
    if on("nodef"):
        results["nodef"] = scenario("no point A in TAKEOFF item", RWY_PARAMS, 0.0, 0.0, 15, ref=False)
    if on("nowp"):
        results["nowp"] = scenario("item after TAKEOFF is not a waypoint", RWY_PARAMS, 0.0, 0.0, 15,
                                   b_cmd=mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM)
    if on("abort"):
        # steering removed so the crosswind/yaw error pushes it off the line
        p = dict(RWY_PARAMS, RWY_ABT_XTRK=3, RWY_V1=13, SERVO5_FUNCTION=0, KFF_RDDRMIX=0,
                 RWY_STRT_HDG=5)
        results["abort"] = scenario("abort below V1 (no steering)", p, 0.0, 3.0, 0, extra=abort_extra)
    if on("abort_steer"):
        # steering working, pilot kick at 10 m/s pushes it off the line
        p = dict(RWY_PARAMS, RWY_ABT_XTRK=2, RWY_V1=13)
        results["abort_steer"] = scenario("abort after a pilot kick (steering on)", p, 0.0, 0.0, 0,
                                          extra=abort_steer_extra)
    if on("mission"):
        items, _ = mission_items_other_runway()
        p = dict(RWY_PARAMS, TKOFF_THR_MAX=100)
        results["mission"] = scenario("take off runway 1, land on runway 2", p, 1.0, 2.0, 0,
                                      extra=mission_extra, items_extra=items)
    if on("past_v1"):
        p = dict(RWY_PARAMS, RWY_ABT_XTRK=3, RWY_V1=8, SERVO5_FUNCTION=0, KFF_RDDRMIX=0)
        results["past_v1"] = scenario("deviation after V1 continues", p, 0.0, 3.0, 45)

    # steering scenarios
    if on("hold"):
        # changes 1, 8, 9b: start refused on GPS accuracy, then released
        p = dict(RWY_PARAMS, STEER2SRV_I=STEER_I, RWY_HACC_MAX=0.1, SIM_GPS1_ACC=0.5)
        results["hold"] = scenario("held by a refused start, then released", p, 1.0, 2.0, 0, extra=hold_extra)
        check_hold(results["hold"])
    if on("abort_i"):
        # changes 2, 3: pilot kick at 10 m/s, abort, keep or clear the
        # integrator. The nosewheel is 30 us off trim, so the integrator
        # holds a correction when the abort comes
        base = dict(RWY_PARAMS, RWY_ABT_XTRK=1, RWY_V1=13, STEER2SRV_I=STEER_I, SERVO5_TRIM=1530)
        for iscl, gain in ((1, 0.6), (0, 1.0)):
            p = dict(base, RWY_ABT_ISCL=iscl, RWY_ABT_GAIN=gain)
            k = "abort_i%d" % iscl
            results[k] = scenario("abort, RWY_ABT_ISCL %d RWY_ABT_GAIN %.1f" % (iscl, gain), p, 0.0, 0.0, 0,
                                  extra=abort_steer_extra)
            check_abort_i(results[k], iscl, gain)
    if on("wl_rotate"):
        # change 4, rotate speed trigger
        p = dict(RWY_PARAMS, TKOFF_THR_MAX=100, RWY_WL_DELAY=0.5, RWY_WL_FADE=1)
        results["wl_rotate"] = scenario("wings level, rotate speed trigger", p, 2.0, 3.0, 40)
        check_wings_level(results["wl_rotate"], 2)
    if on("wl_wow"):
        # change 4, weight on wheels. SITL's WoW reads below 0.01 m above
        # sea level, and the plane sits 0.1 m above home on its gear
        p = dict(RWY_PARAMS, TKOFF_THR_MAX=100, RWY_WL_DELAY=0.5, RWY_WL_FADE=1,
                 SIM_WOW_PIN=8, LGR_WOW_PIN=8, LGR_WOW_POL=1)
        results["wl_wow"] = scenario("wings level, weight on wheels trigger", p, 2.0, 3.0, 40, home_alt=-0.1)
        check_wings_level(results["wl_wow"], 1)
    if on("rudder"):
        # change 4b: rudder on SERVO4, nosewheel on SERVO5. A slow roll
        # with a pilot rudder kick for a real deflection, then a takeoff
        # for the handover back to the coordinated output
        p = dict(RWY_PARAMS, RWY_RUD_MAX=0.5, RWY_RUD_SPD1=5, RWY_RUD_SPD2=12)
        results["rudder"] = scenario("rudders take a share of the steering", p, 2.0, 3.0, 0,
                                     extra=rudder_kick_extra)
        check_rudder(results["rudder"], False)
        results["rudder_tkoff"] = scenario("rudder share, takeoff", dict(p, TKOFF_THR_MAX=100), 2.0, 3.0, 40)
        check_rudder(results["rudder_tkoff"], True)
    if on("iref"):
        # changes 5, 6: long roll at about 12 m/s in the crosswind. With
        # the test's STEER2SRV_P 3 the SITL heading loop (time constant
        # about 4 s) is slower than the 3 s lookahead, and stacking both
        # integrators on it oscillates, so these runs use P 6
        base = dict(RWY_PARAMS, STEER2SRV_P=6, STEER2SRV_I=STEER_I)
        sums = {}
        for k, extra_p in (("iref_off", {"STEER2SRV_IREF": 0, "RWY_XT_I": 0}),
                           ("iref_on", {"STEER2SRV_IREF": 12, "RWY_XT_I": 0.15})):
            results[k] = scenario("crosswind roll, %s" % extra_p, dict(base, **extra_p), 2.0, 3.0, 45)
            sums[k] = roll_summary(results[k])
        a, b = sums["iref_off"], sums["iref_on"]
        check("5/6 standing cross-track with STEER2SRV_IREF 12 and RWY_XT_I 0.15",
              b["mean_abs_xt"] < 0.5 and b["mean_abs_xt"] <= a["mean_abs_xt"] + 0.05 and b["sd_xt"] < 0.3,
              "mean |xt| %.2f m -> %.2f m, sd %.2f m, mean |SE| %.2f -> %.2f deg" %
              (a["mean_abs_xt"], b["mean_abs_xt"], b["sd_xt"], a["mean_abs_se"], b["mean_abs_se"]))
    if on("lookmax"):
        # change 7: cap 30 m, so it engages at 10 m/s with the 3 s lookahead
        p = dict(RWY_PARAMS, RWY_LOOK_MAX=30)
        results["lookmax"] = scenario("lookahead capped at 30 m", p, 1.0, 2.0, 30)
        check_lookmax(results["lookmax"])
    if on("slip"):
        # change 7b: yaw estimate biased by the declination, no wind. P 6
        # as for iref, and the start heading check opened up for the bias
        base = dict(RWY_PARAMS, STEER2SRV_P=6, STEER2SRV_I=STEER_I, SIM_WIND_SPD=0, SIM_WIND_TURB=0,
                    RWY_STRT_HDG=10)
        sums = {}
        for tc in (0, 1):
            k = "slip%d" % tc
            results[k] = scenario("yaw bias, RWY_SLIP_TC %d" % tc, dict(base, RWY_SLIP_TC=tc), 0.0, 0.0, 45,
                                  after_ready=slip_after_ready)
            sums[k] = roll_summary(results[k])
        a, b = sums["slip0"], sums["slip1"]
        check("7b slip compensation removes the standing offset from the yaw bias",
              abs(a["mean_xt"]) > 0.15 and abs(b["mean_xt"]) < 0.5 * abs(a["mean_xt"]),
              "mean xt %+.2f m -> %+.2f m, sd %.2f -> %.2f m (EKF track - yaw %+.2f deg)" %
              (a["mean_xt"], b["mean_xt"], a["sd_xt"], b["sd_xt"], a["track_minus_yaw"]))
        check("7b filtered slip matches the EKF track - yaw in the log",
              abs(b["slp"] - b["track_minus_yaw"]) < 1.0,
              "Slp %+.2f deg, EKF track - yaw %+.2f deg" % (b["slp"], b["track_minus_yaw"]))
    if on("disarm"):
        # change 9a
        p = dict(RWY_PARAMS, RWY_ABT_XTRK=3, RWY_V1=13)
        results["disarm"] = scenario("disarm on the roll, then a cross-track limit of 0.01 m", p, 0.0, 0.0, 0,
                                     extra=disarm_extra)
        check_disarm(results["disarm"])

    print("\n=== SUMMARY ===")
    for k, r in results.items():
        st = r["stats"]
        aborted = any("ABORT" in t for t in r["texts"])
        refused = any("RWY: start" in t or "RWY: no runway" in t for t in r["texts"])
        landed = any("Landed" in t or "Flare" in t for t in r["texts"])
        print("%-11s max|xt| roll %s  max yaw rate %s  xt@30m %s  final alt %s  aborted=%s refused=%s landed=%s" % (
            k,
            "%.2f" % st["max_abs_xt"] if st else "-",
            "%.1f" % r["max_yaw_rate_roll"] if r["max_yaw_rate_roll"] is not None else "-",
            "%.2f" % r["airborne_xt"] if r["airborne_xt"] is not None else "-",
            "%.0f" % r["final_alt"] if r["final_alt"] is not None else "-",
            aborted, refused, landed))
    if CHECKS:
        print("\n=== CHECKS ===")
        for name, ok, detail in CHECKS:
            print("%s  %s: %s" % ("PASS" if ok else "FAIL", name, detail))
        print("%d/%d passed" % (sum(1 for c in CHECKS if c[1]), len(CHECKS)))
        if not all(c[1] for c in CHECKS):
            sys.exit(1)


if __name__ == "__main__":
    main()
