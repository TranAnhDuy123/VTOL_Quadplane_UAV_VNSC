"""Put two runs of the same mission on one clock so their pulses line up.

The moment pulses are triggered by mission events and positions (take-off,
transition, corners, leg midpoints, landing), so the same window starts at a
different time in the PID and the INDI run. For plotting, the other run's
time is mapped onto the reference run's clock:

  * each pulse window present in both runs is an anchor; inside the window
    (HOLD_S after its start: roll 1 s, rest 2 s, pitch 1 s, 0.5 s gap) the
    mapping is a pure shift, so the response keeps its true time scale;
  * between windows the time is stretched or compressed linearly;
  * with no common pulse window (calm runs) the phase changes are the anchors.

np.interp over the knot list gives exactly this piecewise-linear map.
"""
import numpy as np

HOLD_S = 4.5
PHASE_ANCHORS = ("transition", "fixed_wing", "back_transition", "hover_land")


def run_clock(status, t0, t_arm):
    """Pulse windows and pulses of one run, in seconds since arming."""
    windows = {w["window"]: w["start_tb"] - t0 - t_arm for w in (status or {}).get("windows", [])}
    pulses = [(p["tb"] - t0 - t_arm, p["action"], p["window"])
              for p in (status or {}).get("pulses", []) if p.get("action") in ("roll", "pitch")]
    return windows, pulses


def warp(ref_windows, other_windows, ref_phases=None, other_phases=None):
    """Function mapping the other run's time (since arming) to the reference clock.

    ref_phases / other_phases: {phase: start time since arming}, used only when
    the runs share no pulse window."""
    pairs = sorted((other_windows[k], ref_windows[k], HOLD_S)
                   for k in set(ref_windows) & set(other_windows))
    if not pairs and ref_phases and other_phases:
        pairs = sorted((other_phases[k], ref_phases[k], 0.0)
                       for k in PHASE_ANCHORS if k in ref_phases and k in other_phases)
    ko, kr = [0.0], [0.0]
    for o, r, hold in pairs:
        # keep the map monotonic in both clocks
        if o <= ko[-1] or r <= kr[-1]:
            continue
        ko += [o, o + hold] if hold else [o]
        kr += [r, r + hold] if hold else [r]
    ko.append(ko[-1] + 1e6)
    kr.append(kr[-1] + 1e6)
    ko, kr = np.array(ko), np.array(kr)
    return lambda t: np.interp(t, ko, kr)


def shade_pulses(ax, pulses_by_ctrl, colors, axis=None, alpha=0.22, common_color="#8c8b86"):
    """Shade 1 s pulses on an axis. pulses_by_ctrl: {ctrl: [(t_aligned, axis, window)]}.

    A window both runs received is shaded once in neutral grey (the pulses
    coincide after alignment); a window only one run received is shaded in
    that run's colour. axis='roll'/'pitch' shades only that axis' pulses."""
    windows = {c: {w for _, a, w in p if axis in (None, a)} for c, p in pulses_by_ctrl.items()}
    common = set.intersection(*windows.values()) if len(windows) > 1 else set()
    done = set()
    for ctrl, pulses in pulses_by_ctrl.items():
        for t, a, w in pulses:
            if axis not in (None, a):
                continue
            key = (w, a)
            if w in common:
                if key in done:
                    continue
                done.add(key)
                ax.axvspan(t, t + 1.0, color=common_color, alpha=alpha, lw=0)
            else:
                ax.axvspan(t, t + 1.0, color=colors[ctrl], alpha=alpha, lw=0)
