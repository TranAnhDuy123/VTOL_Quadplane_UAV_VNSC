#!/usr/bin/env python3
"""Fly stock ArduPlane SystemID chirps to measure control effectiveness G.

    python sim/fly_sysid.py --phase hover   # QLOITER at 40 m, mixer roll/pitch/yaw
    python sim/fly_sysid.py --phase fw      # LOITER (r=300 m) at 3 airspeeds, FW mixer roll/pitch

SystemID adds a chirp r(t) straight onto the mixer input (SID_AXIS 10/11/12 for
the lift motors, 22/23 for aileron/elevator), on top of the stock PID output.
The chirp is independent of the aircraft state, which is what lets
analysis/identify_G.py separate the plant from the controller.

Stick inputs go in as RC overrides from system id 255 (= MAV_GCS_SYSID).
"""
import argparse
import threading
import time

from pymavlink import mavutil

R = mavutil.mavlink


class Vehicle:
    def __init__(self, connect):
        self.m = mavutil.mavlink_connection(connect, source_system=255)
        if self.m.wait_heartbeat(timeout=60) is None:
            raise SystemExit("no heartbeat")
        self.rc = [1500, 1500, 1000, 1500, 0, 0, 0, 0]
        self.stop = threading.Event()
        threading.Thread(target=self._hb, daemon=True).start()

    def _hb(self):
        # GCS heartbeat + RC override at 5 Hz for the whole flight.
        while not self.stop.is_set():
            try:
                self.m.mav.heartbeat_send(R.MAV_TYPE_GCS, R.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                self.m.mav.rc_channels_override_send(self.m.target_system,
                                                     self.m.target_component, *self.rc)
            except Exception:
                pass
            self.stop.wait(0.2)

    def text(self, timeout=0.5):
        x = self.m.recv_match(type='STATUSTEXT', blocking=True, timeout=timeout)
        if x is not None:
            print("  STATUSTEXT:", x.text)
            return x.text
        return None

    def wait_text(self, prefix, timeout):
        t0 = time.time()
        while time.time() - t0 < timeout:
            s = self.text(1.0)
            if s and s.startswith(prefix):
                return True
        return False

    def param(self, name, value):
        for _ in range(5):
            self.m.mav.param_set_send(self.m.target_system, self.m.target_component,
                                      name.encode(), float(value), R.MAV_PARAM_TYPE_REAL32)
            x = self.m.recv_match(type='PARAM_VALUE', blocking=True, timeout=2)
            if x is not None and x.param_id == name and abs(x.param_value - value) < 1e-3:
                return
        raise SystemExit("could not set %s" % name)

    def mode(self, name):
        self.m.mav.set_mode_send(self.m.target_system, R.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                                 self.m.mode_mapping()[name])
        time.sleep(1)

    def cmd(self, c, *p):
        p = list(p) + [0] * (7 - len(p))
        self.m.mav.command_long_send(self.m.target_system, self.m.target_component, c, 0, *p)

    def arm(self):
        for _ in range(5):
            self.cmd(R.MAV_CMD_COMPONENT_ARM_DISARM, 1)
            t0 = time.time()
            while time.time() - t0 < 5:
                x = self.m.recv_match(type=['HEARTBEAT', 'STATUSTEXT'], blocking=True, timeout=1)
                if x is None:
                    continue
                if x.get_type() == 'STATUSTEXT':
                    print("  STATUSTEXT:", x.text)
                elif x.base_mode & R.MAV_MODE_FLAG_SAFETY_ARMED:
                    return
        raise SystemExit("arm failed")

    def alt(self):
        x = self.m.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=2)
        return None if x is None else x.relative_alt * 1e-3

    def airspeed(self):
        x = self.m.recv_match(type='VFR_HUD', blocking=True, timeout=2)
        return None if x is None else x.airspeed

    def wait_gps(self):
        t0 = time.time()
        while time.time() - t0 < 120:
            x = self.m.recv_match(type='GPS_RAW_INT', blocking=True, timeout=2)
            if x is not None and x.fix_type >= 3:
                time.sleep(5)
                return
        raise SystemExit("no GPS")

    def sweep(self, axis, magnitude, f0, f1, t_rec):
        """One chirp on one axis via SID_* + aux function 184 (SYSTEMID)."""
        self.param("SID_AXIS", axis)
        self.param("SID_MAGNITUDE", magnitude)
        self.param("SID_F_START_HZ", f0)
        self.param("SID_F_STOP_HZ", f1)
        self.param("SID_T_FADE_IN", 2)
        self.param("SID_T_REC", t_rec)
        self.param("SID_T_FADE_OUT", 1)
        print("sweep axis %d mag %.3f %.1f-%.1f Hz %ds" % (axis, magnitude, f0, f1, t_rec))
        self.cmd(R.MAV_CMD_DO_AUX_FUNCTION, 184, 2)      # HIGH = start
        t_end = time.time() + t_rec + 2 + 1 + 2
        while time.time() < t_end:
            self.text(0.5)
        self.cmd(R.MAV_CMD_DO_AUX_FUNCTION, 184, 0)      # LOW
        time.sleep(3)


def upload(v, items):
    m = v.m
    m.mav.mission_clear_all_send(m.target_system, m.target_component)
    m.recv_match(type='MISSION_ACK', blocking=True, timeout=5)
    m.mav.mission_count_send(m.target_system, m.target_component, len(items))
    sent = 0
    t0 = time.time()
    while sent < len(items) and time.time() - t0 < 30:
        x = m.recv_match(type=['MISSION_REQUEST_INT', 'MISSION_REQUEST', 'MISSION_ACK'],
                         blocking=True, timeout=3)
        if x is None:
            continue
        if x.get_type() == 'MISSION_ACK':
            break
        m.mav.mission_item_int_send(m.target_system, m.target_component, *items[x.seq])
        sent += 1
    m.recv_match(type='MISSION_ACK', blocking=True, timeout=5)
    if sent < len(items):
        raise SystemExit("mission upload failed")


def hover(v, a):
    v.mode("QHOVER")
    v.rc[2] = 1000
    v.arm()
    print("climbing...")
    v.rc[2] = 1850
    while (v.alt() or 0) < a.alt:
        pass
    v.rc[2] = 1500
    v.mode("QLOITER")
    time.sleep(8)
    for axis, mag in ((10, a.mag), (11, a.mag), (12, a.mag_yaw)):
        v.sweep(axis, mag, a.f0, a.f1, a.t_rec)
    print("landing...")
    v.mode("QLAND")
    t0 = time.time()
    while time.time() - t0 < 180:
        x = v.m.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
        if x is not None and x.get_srcSystem() == v.m.target_system and \
                not (x.base_mode & R.MAV_MODE_FLAG_SAFETY_ARMED):
            print("disarmed")
            return


def fixed_wing(v, a):
    lat = -35.3632621
    lon = 149.1652374
    items = [
        (0, R.MAV_FRAME_GLOBAL, R.MAV_CMD_NAV_WAYPOINT, 0, 1, 0, 0, 0, 0,
         int(lat * 1e7), int(lon * 1e7), 584.0),
        (1, R.MAV_FRAME_GLOBAL_RELATIVE_ALT, R.MAV_CMD_NAV_VTOL_TAKEOFF, 0, 1, 0, 0, 0, 0,
         int(lat * 1e7), int(lon * 1e7), a.alt),
        (2, R.MAV_FRAME_GLOBAL_RELATIVE_ALT, R.MAV_CMD_NAV_WAYPOINT, 0, 1, 0, 0, 0, 0,
         int((lat + 0.02) * 1e7), int(lon * 1e7), a.alt),
    ]
    upload(v, items)
    v.param("WP_LOITER_RAD", 300)
    v.mode("QHOVER")
    v.arm()
    time.sleep(2)
    v.mode("AUTO")
    if not v.wait_text("Transition done", 120):
        raise SystemExit("no transition")
    time.sleep(5)
    v.mode("LOITER")
    for spd in a.speeds:
        v.param("AIRSPEED_CRUISE", spd)
        time.sleep(20)
        print("airspeed now %.1f (target %.1f)" % (v.airspeed() or -1, spd))
        for axis in (22, 23):
            v.sweep(axis, a.mag_fw, a.f0, a.f1_fw, a.t_rec)
    print("QLAND...")
    v.mode("QLAND")
    t0 = time.time()
    while time.time() - t0 < 240:
        x = v.m.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
        if x is not None and x.get_srcSystem() == v.m.target_system and \
                not (x.base_mode & R.MAV_MODE_FLAG_SAFETY_ARMED):
            print("disarmed")
            return


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--connect", default="tcp:127.0.0.1:5760")
    ap.add_argument("--phase", choices=["hover", "fw"], required=True)
    ap.add_argument("--alt", type=float, default=40.0)
    ap.add_argument("--mag", type=float, default=0.05, help="hover roll/pitch mixer chirp (0-1)")
    ap.add_argument("--mag-yaw", type=float, default=0.10)
    ap.add_argument("--mag-fw", type=float, default=5.0, help="FW mixer chirp, deg of +-45 travel")
    ap.add_argument("--f0", type=float, default=0.5)
    ap.add_argument("--f1", type=float, default=20.0)
    ap.add_argument("--f1-fw", type=float, default=15.0)
    ap.add_argument("--t-rec", type=int, default=40)
    ap.add_argument("--speeds", type=float, nargs="+", default=[18.5, 20.0, 21.5],
                    help="LOITER airspeeds; keep well above Q_ASSIST_SPEED (16.7) or assist blocks SystemID")
    a = ap.parse_args()

    v = Vehicle(a.connect)
    v.wait_gps()
    try:
        hover(v, a) if a.phase == "hover" else fixed_wing(v, a)
    finally:
        v.stop.set()


if __name__ == "__main__":
    main()
