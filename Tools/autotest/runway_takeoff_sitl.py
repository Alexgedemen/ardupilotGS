#!/usr/bin/env python3
"""
SITL scenarios for the runway centreline takeoff (RWY_*) feature.

Starts arduplane SITL directly (no MAVProxy), drives it over MAVLink with
pymavlink and reports the cross-track error during the takeoff roll.

usage:
  python3 Tools/autotest/runway_takeoff_sitl.py [scenario ...]

scenarios: stock, track, refuse, nodef, nowp, abort, abort_steer, past_v1,
           mission, all (default)
           wl, field, abort_return, abort_soft, brake_slip, brake_free,
           brake_time, gps_acc, abort_hdg, abort_hdg_return, wl_rf,
           wl_rf_none, land_stock, land_rwy, land_bounce, land_ga_stock,
           land_ga_rwy (not in all)

After each scenario the newest dataflash log is summarised (RWYT, ATT,
PIDS, GPS). RWY_BINARY overrides the SITL binary, for A/B runs.

The runway is synthetic, heading RWY_HDG_DEG true near CMAC. Point A (the
TAKEOFF item) is the threshold, point B (the next waypoint) is B_DIST_M
down the runway. The aircraft is placed a few metres right of the
centreline and yawed off the runway heading, with a crosswind, to make
the stock heading-hold drift.
"""

import math
import os
import subprocess
import sys
import time

from pymavlink import mavutil

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
BINARY = os.environ.get("RWY_BINARY", os.path.join(ROOT, "build", "sitl", "bin", "arduplane"))

THRESH_LAT = -35.363261
THRESH_LON = 149.165230
THRESH_ALT = 584
RWY_HDG_DEG = 90.0
B_DIST_M = 1500.0
LAND_DIST_M = 300.0

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
    def __init__(self, start_offset_m, start_yaw_err_deg, instance=0, boot_params=None):
        lat, lon = offset(THRESH_LAT, THRESH_LON, RWY_HDG_DEG + 90, start_offset_m)
        yaw = (RWY_HDG_DEG + start_yaw_err_deg) % 360
        home = "%.8f,%.8f,%.1f,%.1f" % (lat, lon, THRESH_ALT, yaw)
        self.workdir = "/tmp/rwy_sitl_%d" % instance
        os.makedirs(self.workdir, exist_ok=True)
        for f in os.listdir(self.workdir):
            if f.endswith(".bin") or f == "eeprom.bin":
                os.unlink(os.path.join(self.workdir, f))
        defaults = ",".join([
            os.path.join(ROOT, "Tools/autotest/models/plane.parm"),
            os.path.join(ROOT, "Tools/autotest/default_params/plane-ice.parm"),
            # sets RC11_OPTION 0 at boot: engine by MAVLink command only
            os.path.join(ROOT, "Tools/autotest/runway_takeoff_sitl.parm"),
        ])
        if boot_params:
            # parameters that only take effect at boot (e.g. RNGFND1_TYPE)
            boot = os.path.join(self.workdir, "boot.parm")
            with open(boot, "w") as f:
                for k, v in boot_params.items():
                    f.write("%s %s\n" % (k, v))
            defaults += "," + boot
        cmd = [BINARY, "--model", "plane-ice-steering", "--home", home,
               "--defaults", defaults, "--speedup", os.environ.get("RWY_SPEEDUP", "5"), "-I%d" % instance, "-w"]
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
        self.navroll = []    # (t, nav_roll_deg)
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
        elif t == "NAV_CONTROLLER_OUTPUT":
            self.navroll.append((time.time(), m.nav_roll, m.nav_pitch))
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
        raise RuntimeError("failed to set %s" % name)

    def set_params(self, d):
        for k, v in d.items():
            self.set_param(k, v)

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

    def start_engine(self, brake_pwm=1100):
        # no engine RC switch: start with MAV_CMD_DO_ENGINE_CONTROL,
        # allowing a start while disarmed (flags bit 0)
        self.rc_override({9: brake_pwm, 3: 1000})
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


def newest_log(workdir):
    logdir = os.path.join(workdir, "logs")
    if not os.path.isdir(logdir):
        return None
    bins = [os.path.join(logdir, f) for f in os.listdir(logdir) if f.upper().endswith(".BIN")]
    return max(bins, key=os.path.getmtime) if bins else None


def log_report(path, minspd):
    """summarise the RWYT, ATT, PIDS and GPS messages of a dataflash log"""
    from pymavlink import DFReader
    if path is None:
        print("  log: none")
        return {}
    log = DFReader.DFReader_binary(path)
    rwyt, att, pids, gps, rwyh, rwyl = [], [], [], [], [], []
    while True:
        m = log.recv_match(type=["RWYT", "ATT", "PIDS", "GPS", "RWYH", "RWYL"])
        if m is None:
            break
        t = m.get_type()
        if t == "RWYT":
            rwyt.append(m)
        elif t == "ATT":
            att.append((m.TimeUS, m.DesRoll, m.Roll))
        elif t == "PIDS":
            pids.append((m.TimeUS, m.I))
        elif t == "GPS" and getattr(m, "I", 0) == 0:
            gps.append((m.TimeUS, m.Spd))
        elif t == "RWYH":
            rwyh.append(m)
        elif t == "RWYL":
            rwyl.append(m)

    def gps_spd(tus):
        best = None
        for (t, s) in gps:
            if t > tus:
                break
            best = s
        return best

    rep = {}
    # change 1: steering integrator below STEER2SRV_MINSPD
    gi = 0
    slow_i = []
    for (t, i) in pids:
        while gi + 1 < len(gps) and gps[gi + 1][0] <= t:
            gi += 1
        if gps and gps[gi][1] < minspd:
            slow_i.append(abs(i))
    if pids:
        rep["max_I_below_minspd"] = max(slow_i) if slow_i else None
        rep["max_I"] = max(abs(i) for (_, i) in pids)
        print("  log PIDS: max |I| %.3f, max |I| below MINSPD %s" % (
            rep["max_I"], "%.3f" % rep["max_I_below_minspd"] if slow_i else "-"))
    if rwyl:
        # runway landing: touchdown, rollout steering and wings level
        td = [r for r in rwyl if r.TD]
        if td:
            t_td = td[0].TimeUS
            print("  log RWYL: touchdown at %.1f s, XT %+.2f m, HE %+.1f deg, Spd %.1f m/s; rollout max |XT| %.2f m, "
                  "end XT %+.2f m; min SF %.2f; Bl reached %.2f" % (
                      t_td * 1e-6, td[0].XT, td[0].HE, td[0].Spd, max(abs(r.XT) for r in td), td[-1].XT,
                      min(r.SF for r in td), max(r.Bl for r in td)))
            if "Bn" in rwyl[0].get_fieldnames():
                # rollout elevator, and the TD / WL sequence through any bounce
                seq = []
                for r in rwyl:
                    if r.TimeUS >= t_td and (not seq or seq[-1][1] != r.TD):
                        seq.append((r.TimeUS, r.TD))
                print("  log RWYL: bounces %d; TD changes at %s; EL %.0f%% to %.0f%% on the rollout; "
                      "max WL off the runway after touchdown %.2f" % (
                          max(r.Bn for r in rwyl), ", ".join("%.1fs:%d" % (t * 1e-6, d) for (t, d) in seq),
                          min(r.EL for r in td), max(r.EL for r in td),
                          max([r.WL for r in rwyl if r.TimeUS > t_td and not r.TD] or [0])))
        else:
            print("  log RWYL: no touchdown seen")
        low = [r.TimeUS for r in rwyl if r.TD or 0 <= r.H < 0.5]
        near = [abs(d) for (t, d, _) in att if low and low[0] <= t <= rwyl[-1].TimeUS]
        if near:
            print("  log RWYL: max |DesRoll| from 0.5 m above the runway to the end %.2f deg (WL min %.2f)" % (
                max(near), min(r.WL for r in rwyl)))
    if not rwyt:
        print("  log RWYT: none")
        return rep
    fields = rwyt[0].get_fieldnames()
    has = set(fields)
    # change 3: desired roll in the ground phase
    if "GP" in has:
        gp_t = [(r.TimeUS, r.GP, r.WL) for r in rwyt]
        ri = 0
        des_gp, des_hand = [], []
        for (t, des, roll) in att:
            while ri + 1 < len(gp_t) and gp_t[ri + 1][0] <= t:
                ri += 1
            if gp_t[ri][0] > t or t - gp_t[ri][0] > 200000:
                continue
            if gp_t[ri][1]:
                des_gp.append((abs(des), abs(roll)))
            elif gp_t[ri][2] < 1:
                des_hand.append(abs(des))
        end_gp = [r.TimeUS for i, r in enumerate(rwyt[1:]) if rwyt[i].GP and not r.GP]
        wl_full = [r.TimeUS for i, r in enumerate(rwyt[1:]) if rwyt[i].WL < 1 and r.WL >= 1]
        if des_gp:
            print("  log ground phase: %d ATT samples, max |DesRoll| %.2f, max |Roll| %.2f deg" % (
                len(des_gp), max(d for d, _ in des_gp), max(r for _, r in des_gp)))
            rep["max_desroll_gp"] = max(d for d, _ in des_gp)
        if end_gp:
            msg = "  log ground phase ended at %.1f s" % (end_gp[0] * 1e-6)
            if wl_full:
                msg += ", WL reached 1 after %.2f s" % ((wl_full[0] - end_gp[0]) * 1e-6)
            if des_hand:
                msg += ", max |DesRoll| during hand-back %.2f deg" % max(des_hand)
            print(msg)
    else:
        # older RWYT: desired roll while rolling on the ground
        on_ground = [abs(d) for (t, d, _) in att if rwyt[0].TimeUS <= t <= rwyt[-1].TimeUS]
        if on_ground:
            print("  log takeoff: max |DesRoll| %.2f deg over the logged takeoff" % max(on_ground))
    if rwyh and "GP" in has:
        # RWY_WL_HGT: desired roll near the runway after rotate speed
        end = [r.TimeUS for i, r in enumerate(rwyt[1:]) if rwyt[i].GP and not r.GP]
        if end:
            t_end = end[0]
            lift = [h.TimeUS for h in rwyh if h.TimeUS >= t_end and h.H > 0.1]
            low = [h.TimeUS for h in rwyh if h.TimeUS >= t_end and h.H < 0.1]
            near = [abs(d) for (t, d, _) in att
                    if t >= t_end and any(abs(t - tl) < 100000 for tl in low)]
            print("  log RWYH: ref %.2f m, rangefinder usable %d/%d samples; height passed 0.1 m %s after rotate "
                  "speed; max |DesRoll| below 0.1 m after rotate speed %s" % (
                      rwyh[-1].Ref, sum(h.RFok for h in rwyh), len(rwyh),
                      "%.2f s" % ((lift[0] - t_end) * 1e-6) if lift else "never",
                      "%.2f deg" % max(near) if near else "-"))
            wl_at = [(h.TimeUS, h.H, h.HF) for h in rwyh if h.TimeUS >= t_end][:40:4]
            print("  log RWYH after rotate speed (t, H, HF): " + " ".join(
                "%.1f:%.2f:%.2f" % (t * 1e-6, hh, hf) for (t, hh, hf) in wl_at))
    if "LA" in has:
        print("  log LA: %.1f to %.1f m" % (min(r.LA for r in rwyt), max(r.LA for r in rwyt)))
    if "DC" in has:
        flags = 0
        for r in rwyt:
            flags |= r.DC
        print("  log DC seen: 0x%x, max BF %.2f, min SF %.2f, max BT %.1f s" % (
            flags, max(r.BF for r in rwyt),
            min(r.SF for r in rwyt) if "SF" in has else 1,
            max(r.BT for r in rwyt) if "BT" in has else 0))
    if "Bl" in has:
        started = [r for r in rwyt if r.Bl > 0]
        if started:
            s = gps_spd(started[0].TimeUS)
            print("  log blend: first Bl>0 at GPS speed %s m/s" % ("%.2f" % s if s is not None else "-"))
            rep["blend_start_spd"] = s
    return rep


def scenario(name, params, start_offset, yaw_err, run_s, extra=None, ref=True, b_cmd=None, items_extra=None,
             hold_brake=False, boot_params=None):
    print("\n=== %s: start offset %.1fm right, yaw err %+.1f deg ===" % (name, start_offset, yaw_err))
    sim = Sim(start_offset, yaw_err, boot_params=boot_params)
    try:
        sim.set_params(BASE_PARAMS)
        sim.set_params(params)
        if os.environ.get("RWY_EXTRA_PARAMS"):
            import json
            sim.set_params(json.loads(os.environ["RWY_EXTRA_PARAMS"]))
        sim.wait_ready()
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
        sim.start_engine(1900 if hold_brake else 1100)
        if extra is not None:
            extra(sim)
        else:
            sim.run_takeoff(run_s)
        st = sim.roll_stats()
        air = [p for p in sim.track if p[4] > 30]
        res = {"stats": st, "texts": list(sim.texts),
               "airborne_xt": air[0][1] if air else None,
               "final_alt": sim.track[-1][4] if sim.track else None,
               "brake": list(sim.brake_pwm),
               "max_yaw_rate_roll": sim.yaw_rate_stats(t_roll, t_roll + 25)}
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
        sim.close()
        minspd = dict(BASE_PARAMS, **params).get("STEER2SRV_MINSPD", 1)
        res["log"] = log_report(newest_log(sim.workdir), minspd)
        return res
    finally:
        sim.close()


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
            sim.rc_override({4: int(os.environ.get("RWY_KICK_PWM", "1650"))})
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


def brake_hold_extra(sim):
    """wait on the brake hold (brakes as set at engine start) until the
    takeoff triggers or aborts. The sim has no brakes, so ICE idle
    throttle makes the aircraft creep as if the brakes had released"""
    t0 = time.time()
    while time.time() - t0 < 40 and not any(("ABORT" in t or "Triggered" in t) for t in sim.texts):
        sim.rc_override({})
        sim.pump(0.3)
    sim.run_takeoff(4)
    spds = [p[3] for p in sim.track]
    print("  max GPS ground speed seen %.2f m/s; aborted=%s triggered=%s" % (
        max(spds) if spds else 0,
        any("ABORT" in t for t in sim.texts), any("Triggered" in t for t in sim.texts)))


def brake_time_test():
    """brake operating time warning, disarmed: hold RC9 at full brake"""
    print("\n=== brake operating time warning (disarmed, RWY_BRK_TMAX 20) ===")
    sim = Sim(0.0, 0.0)
    try:
        sim.set_params(BASE_PARAMS)
        sim.set_params(dict(RWY_PARAMS, RWY_BRK_TMAX=20))
        speedup = float(os.environ.get("RWY_SPEEDUP", "5"))
        sim.rc_override({9: 1900, 3: 1000})
        t0 = time.time()
        while time.time() - t0 < 32 / speedup + 3:
            sim.rc_override({})
            sim.pump(0.2)
        warn = [t for t in sim.texts if "brakes on" in t]
        print("  messages while held: %s" % warn)
        # release and re-apply: the timer restarts, so no message for 10 s
        sim.rc_override({9: 1100})
        sim.pump(1)
        n = len(sim.texts)
        sim.rc_override({9: 1900})
        t0 = time.time()
        while time.time() - t0 < 5 / speedup:
            sim.rc_override({})
            sim.pump(0.2)
        again = [t for t in sim.texts[n:] if "brakes on" in t]
        print("  messages in the first 5 s after re-applying: %s" % again)
        return {"stats": None, "texts": list(sim.texts), "airborne_xt": None, "final_alt": None,
                "max_yaw_rate_roll": None}
    finally:
        sim.close()


def mission_items_circuit():
    """after takeoff, fly a circuit and land back on the same runway,
    heading RWY_HDG_DEG, touching down LAND_DIST_M past the threshold.
    The waypoint after LAND is further along the centreline: the
    runway landing (RWY_ENABLE bit 2) tracks LAND -> that waypoint"""
    wp = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
    rel = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT

    def pt(east, north):
        la, lo = offset(THRESH_LAT, THRESH_LON, RWY_HDG_DEG, east)
        return offset(la, lo, RWY_HDG_DEG - 90, north)
    items = []
    # RWY_APPROACH_OFFSET puts the approach waypoint off the centreline (m, left)
    app_n = float(os.environ.get("RWY_APPROACH_OFFSET", "0"))
    for (e, n, alt) in [(3000, 0, 80), (3000, 1200, 80), (-2500, 1200, 60), (-1500, app_n, 40)]:
        la, lo = pt(e, n)
        items.append((wp, 0, 0, 0, 0, la, lo, alt, rel))
    la, lo = pt(LAND_DIST_M, 0)
    items.append((mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, la, lo, 0, rel))
    la, lo = pt(LAND_DIST_M + 900, 0)
    items.append((wp, 0, 0, 0, 0, la, lo, 60, rel))
    return items


def land_extra(sim, action=None):
    """fly the circuit; apply the RC brakes from touchdown; report the
    landing rollout against the runway centreline. action "bounce" fakes
    a bounce at the rangefinder (for ~1 s of
    sim time, by moving the simulated rangefinder up 0.8 m) just after
    touchdown; "goaround" sends DO_GO_AROUND ~1 s
    of sim time after touchdown"""
    speedup = float(os.environ.get("RWY_SPEEDUP", "5"))
    t_end = time.time() + 1500 / speedup
    braked = False
    acted = False

    def finished():
        return any("Throttle disarmed" in t or "climb-out done" in t for t in sim.texts)
    while time.time() < t_end and not finished():
        if not braked and any(("touchdown" in t or "Flare" in t) for t in sim.texts) and \
                sim.track and sim.track[-1][4] < 0.5 and action != "goaround":
            braked = True
            sim.rc_override({9: 1900})
        # touchdown: the runway landing's message, or (stock) on the runway after the flare
        landed = any("RWY: touchdown" in t for t in sim.texts) or (
            any("Flare" in t for t in sim.texts) and sim.track and sim.track[-1][4] < 0.2)
        if not acted and action is not None and landed:
            acted = True
            sim.pump(1.0 / speedup)
            if action == "bounce":
                # raise the simulated rangefinder 0.8 m: it reads as a bounce
                sim.set_param("SIM_SONAR_POS_Z", -0.8)
                sim.pump(1.0 / speedup)
                sim.set_param("SIM_SONAR_POS_Z", 0)
            elif action == "goaround":
                sim.t_goaround = time.time()
                sim.mav.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_DO_GO_AROUND, 0,
                                              0, 0, 0, 0, 0, 0, 0)
        sim.rc_override({})
        sim.pump(0.2 if action is not None else 0.5)
    if action == "goaround":
        t0 = getattr(sim, "t_goaround", None)
        if t0 is None:
            print("  go-around: not sent (no touchdown seen)")
            return
        after = [p for p in sim.track if p[0] >= t0]
        gnd = [p for p in after if p[4] < 1.0]
        t_air = next((p[0] for p in after if p[4] > 1.0), None)
        nav_gnd = [(r, pt) for (t, r, pt) in sim.navroll if t >= t0 and (t_air is None or t < t_air)]
        if gnd:
            print("  go-around ground roll: %d samples, xt %+.2f -> %+.2f m (max |xt| %.2f), speed %.1f -> %.1f m/s" % (
                len(gnd), gnd[0][1], gnd[-1][1], max(abs(p[1]) for p in gnd), gnd[0][3], gnd[-1][3]))
        if nav_gnd:
            print("  go-around on the ground: max |nav_roll| %.2f deg, nav_pitch %.1f to %.1f deg" % (
                max(abs(r) for (r, _) in nav_gnd), min(pt for (_, pt) in nav_gnd), max(pt for (_, pt) in nav_gnd)))
        air = [p for p in after if p[4] > 1.0]
        if air:
            print("  go-around: airborne at %.1f m/s, xt %+.2f m; reached %.0f m AGL" % (
                air[0][3], air[0][1], max(p[4] for p in after)))
        else:
            print("  go-around: not airborne")
        return
    top = max(range(len(sim.track)), key=lambda i: sim.track[i][4]) if sim.track else 0
    roll = [p for p in sim.track[top:] if p[4] < 0.5 and p[3] > 0.5]
    if roll:
        t_low = [p[0] for p in sim.track[top:] if p[4] < 0.5]
        near = [abs(r) for (t, r, _) in sim.navroll if t_low and t >= t_low[0]]
        if near:
            print("  landing: max |nav_roll| from 0.5 m above the runway to the end %.2f deg" % max(near))
        xs = [p[1] for p in roll]
        print("  landing rollout: %d samples, xt at touchdown %+.2f m, max |xt| %.2f m, end %+.2f m, "
              "%.0f m past the threshold at the end" % (len(xs), xs[0], max(abs(x) for x in xs), xs[-1], roll[-1][2]))
    else:
        print("  landing rollout: not seen")


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


def main():
    want = sys.argv[1:] or ["all"]
    results = {}

    def on(n):
        return "all" in want or n in want

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

    # scenarios for the November changes; run by name, not part of "all"
    def named(n):
        return n in want
    if named("wl"):
        p = dict(RWY_PARAMS, RWY_ENABLE=3, RWY_V1=13, TKOFF_THR_MAX=100)
        results["wl"] = scenario("wings level on the ground", p, 2.0, 3.0, 45)
    if named("wl_rf"):
        # RWY_WL_HGT with a simulated downward rangefinder
        p = dict(RWY_PARAMS, RWY_ENABLE=3, RWY_V1=13, TKOFF_THR_MAX=100, RWY_WL_HGT=2)
        results["wl_rf"] = scenario("wings level, hand-back by rangefinder height", p, 2.0, 3.0, 45,
                                    boot_params={"RNGFND1_TYPE": 100, "RNGFND1_ORIENT": 25,
                                                 "RNGFND1_MIN": 0.3, "RNGFND1_MAX": 50})
    if named("wl_rf_none"):
        # RWY_WL_HGT set but no rangefinder: the takeoff must be refused
        p = dict(RWY_PARAMS, RWY_ENABLE=3, RWY_V1=13, TKOFF_THR_MAX=100, RWY_WL_HGT=2)
        results["wl_rf_none"] = scenario("RWY_WL_HGT without a rangefinder", p, 0.0, 0.0, 15)
    if named("land_bounce") or named("land_ga_stock") or named("land_ga_rwy"):
        rf = {"RNGFND1_TYPE": 100, "RNGFND1_ORIENT": 25, "RNGFND1_MIN": 0.3, "RNGFND1_MAX": 50}
        p = dict(RWY_PARAMS, RWY_ENABLE=7, RWY_V1=13, TKOFF_THR_MAX=100, RWY_WL_HGT=2, RWY_LOOK_MAX=50,
                 RWY_BRK_GAIN=0.6, STEER2SRV_I=0.2, RWY_LND_ELEV=-20, TKOFF_GND_PITCH=0)
        if named("land_bounce"):
            results["land_bounce"] = scenario("runway landing, sensor bounce after touchdown", p, 0.0, 0.0, 0,
                                              extra=lambda sim: land_extra(sim, "bounce"),
                                              items_extra=mission_items_circuit(), boot_params=rf)
        if named("land_ga_stock"):
            results["land_ga_stock"] = scenario("go-around after touchdown, stock (RWY_ENABLE 3)",
                                                dict(p, RWY_ENABLE=3), 0.0, 0.0, 0,
                                                extra=lambda sim: land_extra(sim, "goaround"),
                                                items_extra=mission_items_circuit(), boot_params=rf)
        if named("land_ga_rwy"):
            results["land_ga_rwy"] = scenario("go-around after touchdown, runway takeoff (RWY_ENABLE 7)", p,
                                              0.0, 0.0, 0, extra=lambda sim: land_extra(sim, "goaround"),
                                              items_extra=mission_items_circuit(), boot_params=rf)
    if named("land_stock") or named("land_rwy"):
        # take off, fly a circuit and land back on the runway in the crosswind
        rf = {"RNGFND1_TYPE": 100, "RNGFND1_ORIENT": 25, "RNGFND1_MIN": 0.3, "RNGFND1_MAX": 50}
        p = dict(RWY_PARAMS, RWY_ENABLE=3, RWY_V1=13, TKOFF_THR_MAX=100, RWY_WL_HGT=2, RWY_LOOK_MAX=50,
                 RWY_BRK_GAIN=0.6, STEER2SRV_I=0.2, RWY_LND_ELEV=float(os.environ.get("RWY_LND_ELEV", "0")))
        if named("land_stock"):
            results["land_stock"] = scenario("circuit, stock landing (RWY_ENABLE 3)", p, 0.0, 0.0, 0,
                                             extra=land_extra, items_extra=mission_items_circuit(),
                                             boot_params=rf)
        if named("land_rwy"):
            results["land_rwy"] = scenario("circuit, runway landing (RWY_ENABLE 7)", dict(p, RWY_ENABLE=7), 0.0, 0.0,
                                           0, extra=land_extra, items_extra=mission_items_circuit(),
                                           boot_params=rf)
    if named("wl_off"):
        p = dict(RWY_PARAMS, RWY_V1=13, TKOFF_THR_MAX=100)
        results["wl_off"] = scenario("wings level off (RWY_ENABLE 1)", p, 2.0, 3.0, 45)
    if named("field"):
        # field values from the brief, with a steering integrator
        p = dict(RWY_PARAMS, RWY_ENABLE=3, RWY_V1=13, RWY_LOOK_MAX=50, RWY_BRK_GAIN=0.6,
                 STEER2SRV_IREF=12, STEER2SRV_I=0.2, TKOFF_THR_MAX=100)
        results["field"] = scenario("field settings", p, 2.0, 3.0, 45)
    if named("abort_return"):
        p = dict(RWY_PARAMS, RWY_ABT_XTRK=2, RWY_V1=13, RWY_LOOK_MAX=50)
        results["abort_return"] = scenario("abort after a pilot kick, return to the line", p, 0.0, 0.0, 0,
                                           extra=abort_steer_extra)
    if named("abort_soft"):
        p = dict(RWY_PARAMS, RWY_ABT_XTRK=2, RWY_V1=13, RWY_LOOK_MAX=50, RWY_BRK_GAIN=0.6,
                 STEER2SRV_I=0.2, RWY_ENABLE=3)
        results["abort_soft"] = scenario("abort after a pilot kick, field settings", p, 0.0, 0.0, 0,
                                         extra=abort_steer_extra)
    if named("abort_hdg") or named("abort_hdg_return"):
        # a small kick and a tight heading limit: the abort fires near the
        # line with a small heading error. Compare the stop with the runway
        # direction held (RWY_LOOK_MAX 0) and with the return to the line
        os.environ.setdefault("RWY_KICK_PWM", "1600")
        p = dict(RWY_PARAMS, RWY_ABT_HDG=3, RWY_V1=13, STEER2SRV_I=0.2)
        if named("abort_hdg"):
            results["abort_hdg"] = scenario("heading abort, runway direction held", p, 0.0, 0.0, 0,
                                            extra=abort_steer_extra)
        if named("abort_hdg_return"):
            results["abort_hdg_return"] = scenario("heading abort, return to the line", dict(p, RWY_LOOK_MAX=50),
                                                   0.0, 0.0, 0, extra=abort_steer_extra)
    if named("brake_slip"):
        p = dict(RWY_PARAMS, TKOFF_THR_MINSPD=4, ICE_IDLE_PCT=25)
        results["brake_slip"] = scenario("brakes on but not holding on the hold", p, 0.0, 0.0, 0,
                                         extra=brake_hold_extra, hold_brake=True)
    if named("brake_free"):
        p = dict(RWY_PARAMS, TKOFF_THR_MINSPD=4, ICE_IDLE_PCT=25)
        results["brake_free"] = scenario("brakes released on the hold (control)", p, 0.0, 0.0, 0,
                                         extra=brake_hold_extra, hold_brake=False)
    if named("brake_time"):
        results["brake_time"] = brake_time_test()
    if named("gps_acc"):
        p = dict(RWY_PARAMS, SIM_GPS1_ACC=2.0)
        results["gps_acc"] = scenario("GPS accuracy worse than half RWY_STRT_XTRK", p, 0.0, 0.0, 15)

    print("\n=== SUMMARY ===")
    for k, r in results.items():
        st = r["stats"]
        aborted = any("ABORT" in t for t in r["texts"])
        refused = any("RWY: start" in t or "RWY: no runway" in t or "RWY: GPS" in t or "RWY: RWY_WL_HGT" in t
                      for t in r["texts"])
        landed = any("Landed" in t or "Flare" in t for t in r["texts"])
        print("%-11s max|xt| roll %s  max yaw rate %s  xt@30m %s  final alt %s  aborted=%s refused=%s landed=%s" % (
            k,
            "%.2f" % st["max_abs_xt"] if st else "-",
            "%.1f" % r["max_yaw_rate_roll"] if r["max_yaw_rate_roll"] is not None else "-",
            "%.2f" % r["airborne_xt"] if r["airborne_xt"] is not None else "-",
            "%.0f" % r["final_alt"] if r["final_alt"] is not None else "-",
            aborted, refused, landed))


if __name__ == "__main__":
    main()
