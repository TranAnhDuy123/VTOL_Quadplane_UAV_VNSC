#!/usr/bin/env python3
"""Hover over the origin in QLOITER, push with a constant torque, land.

    python3 sim/fly_hover_disturbance.py --nm 1.44 --status-out status.json

The test that shows whether INDI or PID owns the hover rate loop:

  1. arm in QLOITER (position and height hold) at the take-off point,
     climb with the throttle stick (RC override) to HOLD_ALT_M, centre it;
  2. settle SETTLE_S;
  3. roll torque +M for PUSH_S, then REST_S;
  4. pitch torque +M for PUSH_S, then REST_S;
  5. QLAND.

Under a constant torque PID needs a rate error and then its integrator to
cancel the moment, so the attitude deviation lasts; INDI measures the extra
angular acceleration and adds the missing command in a few cycles, so its
effector state (INDI.RU0 / PU0) steps by M / (G I) almost at once.
Times are on the vehicle clock (ATTITUDE.time_boot_ms = DataFlash TimeUS).
SERVO9 marks the torque in the log (1900 roll, 1100 pitch, 1500 off).
"""
import argparse
import json
import os
import subprocess
import threading
import time

from pymavlink import mavutil

HOLD_ALT_M = 20.0
SETTLE_S, PUSH_S, REST_S = 15.0, 5.0, 15.0
THR_LOW, THR_MID, THR_UP = 1100, 1500, 1800
MARK_CH, PWM_ROLL, PWM_PITCH, PWM_OFF = 9, 1900, 1100, 1500


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--connect", default="tcp:127.0.0.1:5760")
    ap.add_argument("--nm", type=float, default=1.44, help="torque size, N.m")
    ap.add_argument("--status-out", default=None)
    ap.add_argument("--timeout", type=float, default=600.0)
    a = ap.parse_args()

    status = dict(test="hover_disturbance", nm=a.nm, armed=False, landed=False, completed=False,
                  reason="", pulses=[], max_alt=0.0)
    m = mavutil.mavlink_connection(a.connect, source_system=255)
    if m.wait_heartbeat(timeout=60) is None:
        raise SystemExit("no heartbeat")
    R = mavutil.mavlink

    rc = [THR_LOW]              # throttle; roll/pitch/yaw held centred
    stop = threading.Event()

    def background():
        # GCS heartbeat (failsafe) and RC override (expires after RC_OVERRIDE_TIME)
        while not stop.is_set():
            try:
                m.mav.heartbeat_send(R.MAV_TYPE_GCS, R.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                m.mav.rc_channels_override_send(m.target_system, m.target_component,
                                                1500, 1500, rc[0], 1500,
                                                65535, 65535, 65535, 65535)
            except Exception:
                pass
            stop.wait(0.1)
    threading.Thread(target=background, daemon=True).start()

    def cmd(c, *p):
        p = list(p) + [0] * (7 - len(p))
        m.mav.command_long_send(m.target_system, m.target_component, c, 0, *p)

    for msg_id, hz in ((R.MAVLINK_MSG_ID_ATTITUDE, 20), (R.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 10)):
        cmd(R.MAV_CMD_SET_MESSAGE_INTERVAL, msg_id, 1e6 / hz)

    state = dict(tb=0.0, att=(0.0, 0.0, 0.0), alt=0.0, armed=False)

    def pump(seconds=None, until=None):
        """Read messages for `seconds` of vehicle time, or until until() is true."""
        t_end = state["tb"] + seconds if seconds is not None else None
        wall0 = time.time()
        while time.time() - wall0 < a.timeout:
            msg = m.recv_match(blocking=True, timeout=0.05)
            if msg is not None:
                t = msg.get_type()
                if t == "ATTITUDE":
                    state["tb"] = msg.time_boot_ms * 1e-3
                    state["att"] = (msg.roll, msg.pitch, msg.yaw)
                elif t == "GLOBAL_POSITION_INT":
                    state["alt"] = msg.relative_alt * 1e-3
                    status["max_alt"] = max(status["max_alt"], state["alt"])
                elif t == "HEARTBEAT" and msg.get_srcSystem() == m.target_system:
                    state["armed"] = bool(msg.base_mode & R.MAV_MODE_FLAG_SAFETY_ARMED)
                elif t == "STATUSTEXT":
                    txt = msg.text if isinstance(msg.text, str) else msg.text.decode(errors="replace")
                    print("  STATUSTEXT:", txt)
            if t_end is not None and state["tb"] >= t_end:
                return True
            if until is not None and until():
                return True
        return False

    print("waiting for GPS...")
    while True:
        g = m.recv_match(type="GPS_RAW_INT", blocking=True, timeout=2)
        if g is not None and g.fix_type >= 3:
            break
    time.sleep(5)
    pump(2)

    m.mav.set_mode_send(m.target_system, R.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, m.mode_mapping()["QLOITER"])
    pump(2)
    for _ in range(3):
        cmd(R.MAV_CMD_COMPONENT_ARM_DISARM, 1)
        if pump(4, until=lambda: state["armed"]) and state["armed"]:
            break
    if not state["armed"]:
        status["reason"] = "arm rejected"
        _write(a.status_out, status)
        raise SystemExit(status["reason"])
    status["armed"] = True

    # Start the torque helper now: it blocks until Gazebo's wrench topic is
    # connected, and the vehicle clock runs on meanwhile. Starting it right
    # before a push made the logged start time stale by that wait.
    here = os.path.dirname(os.path.abspath(__file__))
    proc = subprocess.Popen(["/usr/bin/python3", os.path.join(here, "disturb.py")],
                            stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True)
    if proc.stdout.readline().strip() != "ready":
        raise SystemExit("disturb.py did not start")

    print("climbing to %.0f m..." % HOLD_ALT_M)
    rc[0] = THR_UP
    pump(until=lambda: state["alt"] >= HOLD_ALT_M - 1.0)
    rc[0] = THR_MID
    print("holding at %.1f m, settling %.0f s" % (state["alt"], SETTLE_S))
    pump(SETTLE_S)


    def send(line):
        proc.stdin.write(line + "\n")
        proc.stdin.flush()
        return proc.stdout.readline().strip() == "ok"

    def mark(pwm):
        cmd(R.MAV_CMD_DO_SET_SERVO, MARK_CH, pwm)

    for axis in ("roll", "pitch"):
        L, M = (a.nm, 0.0) if axis == "roll" else (0.0, a.nm)
        ok = send("body %f %f 0 %f %f %f" % ((L, M) + tuple(state["att"])))
        mark(PWM_ROLL if axis == "roll" else PWM_PITCH)
        status["pulses"].append(dict(window="hover_" + axis, action=axis, ok=ok,
                                     tb=round(state["tb"], 3), duration_s=PUSH_S))
        print("  %s torque %.2f N.m on at t=%.2f %s" % (axis, a.nm, state["tb"], "ok" if ok else "FAILED"))
        pump(PUSH_S)
        ok = send("clear")
        mark(PWM_OFF)
        status["pulses"].append(dict(window="hover_" + axis, action="off", ok=ok, tb=round(state["tb"], 3)))
        print("  %s torque off at t=%.2f" % (axis, state["tb"]))
        pump(REST_S)
    send("clear")
    proc.stdin.close()
    proc.wait(timeout=5)

    m.mav.set_mode_send(m.target_system, R.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, m.mode_mapping()["QLAND"])
    print("landing...")
    if pump(until=lambda: not state["armed"]):
        status["landed"] = status["completed"] = True
        status["reason"] = "disarmed after landing"
    else:
        status["reason"] = "timeout"
    stop.set()
    print(json.dumps({k: v for k, v in status.items() if k != "pulses"}))
    _write(a.status_out, status)


def _write(path, status):
    if path:
        with open(path, "w") as fh:
            json.dump(status, fh, indent=1)


if __name__ == "__main__":
    main()
