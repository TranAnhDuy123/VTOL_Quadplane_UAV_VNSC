# QuadPlane INDI — rework

Clean restart. Stock ArduPlane PID first, then an INDI body-rate loop
(lift motors in hover and transition, control surfaces in fixed-wing
flight), then a PID-vs-INDI comparison in Gazebo + SITL. The comparison
covers calm air, 6 m/s and 9 m/s wind, and roll/pitch moment pulses.

Status (2026-09-25):
- **Steps 1–7 are done:** baseline, simulation setup, reference, identification,
  INDI, gains, and calm checks.
- **Step 8 (the 18-run campaign) is done:** all 18 missions landed. Results are below and in `runs/compare/`.

The thesis Methodology has the same steps as a plain log
(`paper&thesis/thesis.tex`, subsection *Step-by-step procedure*).

## Layout

```
rework/
  ardupilot/                  ArduPilot clone, branch `indi` (git-ignored; see Build)
  patches/                    the 3 commits on `indi`, as git format-patch files
  sim/
    run_square_mission.sh     one run: start sim, fly square.plan, stop, save log
    run_campaign.sh           {pid,indi} x {calm,w6,w9} x seeds -> runs/<ctrl>_<case>_s<seed>
    run_sysid.sh              one SystemID flight (hover | fw) -> runs/<name>
    run_gazebo_sitl.sh        start Gazebo + SITL only (manual / QGC use)
    stop_gazebo_sitl.sh       stop them
    fly_plan_mission.py       uploads a .plan, flies AUTO, fires moment pulses (--disturb-nm)
    fly_sysid.py              SystemID chirps: hover mixer roll/pitch/yaw, FW aileron/elevator
    disturb.py                pushes a torque onto the airframe in Gazebo (system python)
    square.plan               VTOL takeoff -> 250 m square at 35 m -> VTOL land
    stock_pid.param           two-column params for stock ArduPlane (PID runs)
    indi.param                Q_INDI_* overlay, appended for INDI runs
    alti_transition_quad.param  original QGC export (5 columns, SITL can't read it)
    worlds/quadplane_runway.sdf.in  world template (wind, turbulence, wrench system)
    gazebo_models/            the quadplane model (VTOL_Quadplane)
    ardupilot_gazebo/         Gazebo<->ArduPilot JSON plugin (built)
  analysis/
    validate_log.py           one log -> validation.md/.json/.png (phases, errors, INDI time)
    real_reference.py         real flight log -> reference table (context only)
    identify_G.py             SystemID logs -> G, tau, T per axis
    indi_margins.py           gain choice from a discrete loop model (GM/PM)
    compare_controllers.py    campaign runs -> runs/compare/summary.md, csv, plots
  runs/<name>/                one directory per run
  references/                 INDI surveys, NASA eVTOL NDI paper, Beard & McLain
```

## Build (once)

```bash
cd rework
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
cd ardupilot
git checkout -b indi 9f648ccabc && git am ../patches/*.patch
/usr/bin/python3 ./waf configure --board sitl   # system python has empy; the .venv doesn't
/usr/bin/python3 ./waf plane
```

The three commits:
1. SITL: the JSON backend uses the reported wind when it computes airspeed.
2. INDI rate loop, `Q_INDI_*` parameters, and the `INDI`/`INDF` logs.
3. INDI uses the measured loop period.

With `Q_INDI_ENABLE 0` (the default) the stock code runs unchanged, so PID and
INDI fly the same binary.

**Don't rebuild while a simulation is running.** Replacing the binary under a
running SITL stopped one run. Also keep the strings `gz sim` and
`build/sitl/bin/arduplane` out of any command line that wraps these scripts:
the launcher's `pkill -f` will match it and kill the wrapper.

## Run

```bash
PY=/home/trananhduy/quadplane-indi/.venv/bin/python
bash sim/run_square_mission.sh pid_calm_01                     # one calm PID run (~3.5 min)
WIND_E=9 TURB_MAG=0.3 SEED=1 DISTURB_NM=3.24 bash sim/run_square_mission.sh w9_test
$PY analysis/validate_log.py runs/pid_calm_01/flight.BIN

bash sim/run_sysid.sh sysid_hover hover                         # Step 4 flights
bash sim/run_sysid.sh sysid_fw fw
$PY analysis/identify_G.py --hover runs/sysid_hover/flight.BIN --fw runs/sysid_fw/flight.BIN --fit-fmin 3
$PY analysis/indi_margins.py --fs 333 --params sim/stock_pid.param sim/indi.param

bash sim/run_campaign.sh 1 2 3                                  # Step 8, 18 runs (~75 min)
$PY analysis/compare_controllers.py runs
```

Environment variables for the run scripts:

| variable | effect |
|---|---|
| `PARAMS` | parameter file |
| `WIND_E` / `WIND_N` | steady wind, m/s |
| `TURB_MAG` / `TURB_DIR` / `TURB_VERT` | turbulence |
| `SEED` | Gazebo random seed |
| `DISTURB_NM` | moment pulse size, N·m |
| `GUI=1` | show Gazebo |
| `MAVPROXY=1` | MAVProxy to UDP 14550, for QGC |

## Step 1 — Stock PID baseline

Two calm runs (`pid_calm_01`, `pid_calm_02`) gave the same event times within
0.2 s. Mission timeline (s after arming):

| event | time (s) |
|---|---|
| transition starts | 16.9 |
| transition done | 40.0 |
| back-transition | 64.9 |
| landed | 137.7 |

| metric | run 01 | run 02 |
|---|---|---|
| hover attitude error RMS, roll / pitch / yaw (deg), takeoff | 0.01 / 0.53 / 0.05 | 0.02 / 0.53 / 0.05 |
| hover attitude error RMS, roll / pitch / yaw (deg), landing | 0.04 / 0.48 / 0.08 | 0.03 / 0.49 / 0.08 |
| hover rate error RMS, roll / pitch / yaw (deg/s), landing | 0.25 / 2.28 / 0.23 | 0.23 / 2.32 / 0.20 |
| hover rate content above 5 Hz (RMS, deg/s) | 0.20 | 0.21 |
| fixed-wing VTOL assist (% of time) | 0.0 | 0.0 |
| fixed-wing airspeed mean (m/s) | 20.3 | 20.3 |
| fixed-wing nav-roll tracking RMS / max (deg) | 11.4 / 46.0 | 10.9 / 44.2 |
| fixed-wing TECS height error RMS (m) | 6.8 | 6.8 |
| fixed-wing throttle at 100 % (% of time) | 58 | 57 |

**Hover is healthy.** The 12–17 Hz spectral peaks carry only about 0.2 deg/s,
which is noise level, not a limit cycle.

**Fixed-wing is thrust-limited in the Gazebo model.** Throttle sits at 100 %
for most of the segment, and the aircraft loses about 10 m of height per turn.

**Fixed-wing roll lags by about 1–1.5 s on turn entry**, with the ailerons at
only about ±6 % of travel. That lag comes from the angle loop, not from a lack
of authority.

Fixes that were needed to get this far:
- **Parameter file.** Converted to two columns, with `Q_OPTIONS 128 -> 0` and
  `SIM_WIND_SPD 0`.
- **Fresh run directory** for every run, so no stale `eeprom.bin`.
- **GCS heartbeat** now uses system 255 (it was 250, which caused a failsafe
  QLAND; kept as `runs/failed_gcs_failsafe_01`).
- **Launcher paths** now point into `rework/`.

## Step 2 — Wind and disturbances

- **SITL airspeed fix (commit 1).** Stock JSON SITL computed airspeed with
  wind = 0.
- **Turbulence seed.** `SEED` is passed as `gz sim --seed`, so a PID run and an
  INDI run with the same seed see the same gusts.
- **Wind.** From the west at 6 or 9 m/s, with turbulence 0.3 m/s. That is a
  crosswind on the N/S legs and head/tailwind on the E/W legs.
- **Moment pulses** (`sim/disturb.py`, Gazebo `ApplyLinkWrench`):
  - Size is `M = ½ρ u_g² · S · b/4` with S = 0.2044 m² and b = 1.30 m (OpenVSP).
    That gives 1.44 N·m at 6 m/s and 3.24 N·m at 9 m/s.
  - Each set is roll 1 s, pause 4 s, pitch 1 s.
  - A set fires in four places: hover climb, transition, cruise leg, and
    landing hover. Each start is a fixed delay after an autopilot message.
  - SERVO9 = 1900 (roll) / 1100 (pitch) marks each pulse in `RCOU.C9`.
- **Checks:**
  - A calm rerun matches Step 1.
  - At 9 m/s, ground speed ran 5.7–29.3 m/s at 17.5 m/s airspeed.
  - The first 0.3 s of every pulse rotated the aircraft about the right axis,
    at every heading.
  - In cruise, a 3.24 N·m roll pulse rolled the stock-PID aircraft by about 75°
    and triggered VTOL assist.

## Step 3 — Real flight (reference only)

`runs/reference/real_reference.md` (from `analysis/real_reference.py`). The
real flight is manual QHOVER plus 8.4 s of FBWA at about 4 m/s, and it was
never wing-borne, so it is **not compared** with the simulation. It only shows
what differs:

| item | real aircraft | SITL |
|---|---|---|
| hover throttle | 0.41 | 0.30 |
| loop rate | 300 Hz | 400 Hz |
| gyro filter | 42 Hz + harmonic notch | 20 Hz, no notch |
| rate gains | different (e.g. roll P 0.16) | roll P 0.7 |

## Step 4 — Control effectiveness (SystemID)

Chirps (0.5–20 Hz) come from the stock SystemID, added to the mixer input.
The estimate is `H = S_r,ω̇ / S_r,u`, with the chirp r as the instrument (with
the loop closed, the plain `S_u,ω̇/S_u,u` returns the inverse of the
controller). The model is `ω̇/u = G e^{-sT}/(1+sτ)`, fitted above 3 Hz where
the coherence is at least 0.6.

| axis | G (rad/s² per unit) | τ | T | check |
|---|---|---|---|---|
| hover roll | 56.7 | ≈0 | 4 ms | model.sdf physics 56.9 |
| hover pitch | 44.8 | ≈0 | 4 ms | model.sdf physics 44.8 |
| hover yaw | 5.4 (fit error 45 %) | | | not identifiable, so yaw stays PID |
| aileron, G_ref at scaler 1 | 16.7 | 10 ms | 0 | same within 1 % at 18.4 / 19.9 / 20.8 m/s |
| elevator, G_ref at scaler 1 | 12.4 | 10 ms | 0 | same within 1 % at the three speeds |

The log also showed that **the SITL loop runs every 3 ms (333 Hz), not every
2.5 ms**: JSON SITL steps in whole milliseconds. INDI therefore uses the
measured loop period (commit 3).

## Step 5 — INDI implementation

The law is `ν = K(ω_ref − ω)` and `u = u0_f + (ν − ω̇_f)/G`.
- **ω̇_f:** the gyro rate, differentiated, then a two-pole low-pass at 12.7 Hz
  (80 rad/s, as in Lombaerts 2019).
- **u0_f:** the *applied* command, passed through the same chain: actuator
  model, the half sample of the difference, a copy of the INS gyro filter, and
  the same low-pass. This synchronization is what keeps the loop from limit
  cycling (the old attempt's failure).

Where it runs:
- **Lift motors:** `AC_AttitudeControl_INDI` (a subclass of
  `AC_AttitudeControl_TS`). u0 is recovered from the four motor outputs after
  the mixer. INDI runs on roll and pitch whenever the motors are spooled up;
  yaw stays PID.
- **Surfaces:** INDI branch in `AP_FW_Controller::run_rate_control`, with
  `G = G_ref/scaler²`. It is active only in fixed-wing flight proper
  (transition complete, no assist, not a VTOL mode).
- **Only one INDI loop owns an axis at a time.** In hover and transition the
  motors run INDI and the surfaces keep the stock slaved PID.

## Step 6 — Gains

`analysis/indi_margins.py --fs 333` models the plant, actuator, gyro filter,
INDI filter and synchronization path. It returns the largest K with
GM ≥ 6 dB and PM ≥ 45°:

| loop | largest K | flown |
|---|---|---|
| lift motors | 14 | 13.5 |
| surfaces | 11 | 10.5 |

The loop stays stable if the true G is half or double the value used (GM
2.9–15.6 dB). All values are in `sim/indi.param`.

## Step 7 — Checks before the comparison

- **`Q_INDI_ENABLE 0` on the new binary** reproduces the baseline
  (`runs/reg_indi_branch_pid`): the same events within 0.6 s and the same
  errors. The landing path differed once, which is non-lockstep scatter.
- **INDI calm mission (`runs/indi_calm_test`)** completed.
  - INDI was in control for this share of each phase:

    | phase | INDI in control |
    |---|---|
    | transition | 100 % |
    | fixed-wing (surfaces) | 100 % |
    | landing hover | 100 % |
    | back-transition | 97 % |
    | takeoff | 88 % (ground idle at the start of the phase) |

  - No oscillation: rate content above 5 Hz is 0.2–0.5 deg/s, the same as PID.
  - No motor at its limit.
- **Pseudo-control tracking (ν vs ω̇_f below 5 Hz).** Correlation is 0.96 (FW
  roll) and 0.65 (FW pitch). In calm hover both signals are only about
  0.1 rad/s², at the noise floor, so this is checked on the pulses instead.
- **First look, calm, one run:**
  - Hover pitch attitude error was 0.07° with INDI against 0.53° with PID.
  - FW rate tracking error was 1.9/0.9 against 5.2/2.3 deg/s.
  - FW nav-roll tracking was unchanged (the lag is in the angle loop).

## Step 8 — PID vs INDI campaign

`bash sim/run_campaign.sh 1 2 3`, then `analysis/compare_controllers.py runs`.
Full tables are in `runs/compare/summary.md`; per-run and per-pulse numbers are
in `runs.csv` and `pulses.csv`; overlay plots are `pulses_w6.png` and
`pulses_w9.png`.

The design is {PID, INDI} × {calm, 6 m/s, 9 m/s} × seeds 1–3. Each pair flies
the same binary, plan, wind, turbulence seed and pulses; only the `Q_INDI_*`
lines differ. **All 18 missions landed.** One INDI run (`indi_w9_s3`) had a
startup failure (no heartbeat, SITL connection race) before it ever flew, and
was re-flown. Values below are the mean over 3 seeds. "Clear" means the
difference is larger than the seed spread of both controllers.

**Hover (take-off and landing) — INDI is clearly better in every case.**

| attitude error RMS (deg) | PID | INDI |
|---|---|---|
| calm, take-off pitch | 0.53 | 0.06 |
| 6 m/s, take-off roll / pitch | 0.57 / 1.24 | 0.37 / 0.34 |
| 9 m/s, take-off roll / pitch | 1.05 / 2.08 | 0.48 / 0.32 |
| 9 m/s, landing roll / pitch | 0.59 / 1.69 | 0.27 / 1.12 |

| moment pulse, peak attitude error (deg) / time to settle (s) | PID | INDI |
|---|---|---|
| 6 m/s (1.44 N·m), hover pitch | 4.0 / 2.3 | 0.8 / 0.0 |
| 6 m/s, hover roll | 2.1 / 2.4 | 1.6 / 1.3 |
| 9 m/s (3.24 N·m), hover pitch | 7.7 / 2.3 | 1.8 / 1.3 |
| 9 m/s, hover roll | 4.7 / 2.3 | 3.0 / 1.4 |

INDI took over 81–99 % of the injected moment within about 0.1 s. That share
is Δu₀·G·I divided by the injected M; it is only near 1 if the identified G is
right.

**Transition.**
- **Pitch pulses at 6 m/s:** clearly better with INDI (peak 1.2° vs 4.8°).
- **Pitch pulses at 9 m/s:** about equal (19° vs 21°). At that point the lift
  motors are near minimum thrust, so they have little pitch authority: INDI
  took over only 18 % of the moment, and the surfaces (stock PID in
  transition) do most of the work.
- **Roll pulses:** no clear difference.
- **Whole-phase RMS:** at 6 m/s INDI is slightly worse, by 0.3° in roll and
  pitch.

**Fixed-wing.**
- **9 m/s cruise roll pulse:** PID was upset by **100° peak** and needed the
  lift motors (assist). INDI peaked at 18° and settled in 1.2 s with no
  motors. Fixed-wing roll RMS at 9 m/s was 14.4° (PID) vs 5.2° (INDI).
- **Caveat for 9 m/s:** the fixed-wing leg is 75–80 % assisted flight for
  *both* controllers. The model is thrust-limited, and airspeed averages 16 m/s,
  under `Q_ASSIST_SPEED` 16.7. The surfaces run INDI only about 21 % of that
  segment, so it is mainly a test of the rotors plus assist.
- **6 m/s cruise roll pulse:** INDI peak is slightly larger (11.0° vs 9.3°,
  clear). Note that at 6 m/s these pulses sometimes fired before the transition
  had finished.
- **Calm fixed-wing:** a small cost with INDI. Pitch RMS is +0.3°, TECS height
  error +0.6 m, cross-track +1.0 m. All are clear, but small.

**What this does and doesn't show.** In this simulation the INDI rate loop
rejects roll/pitch disturbances much better than the stock PID in hover, and
recovers far better from a large roll upset in cruise. It does this without
oscillation and without extra actuator saturation. The advantage shrinks or
disappears where the effector it controls has little authority (rotors late in
the transition). It also costs a little in calm fixed-wing tracking.

All of this is SITL with ground-truth attitude and no sensor noise or
vibration (see Caveats). The real aircraft's gyro is much noisier, and ω̇ is
the signal INDI relies on most.

## Follow-up: faster full-physics wind rerun with IMU noise

`sim/run_noisy_wind_campaign.sh` flies PID and INDI on the same unmodified
`sim/square.plan` with 6 and 9 m/s wind, 1.44 and 3.24 N·m roll/pitch pulses,
and seeds 1–3. All **12/12** new missions completed, transitioned and reported
all 16 pulse actions as applied. They are isolated in `runs/noisy_wind/`, so the
original campaign is untouched. The script uses headless `RTF=0` (uncapped),
`SPEEDUP=50`, **the same 1 ms Gazebo physics step**, aerodynamic plugins, and
the same SITL binary; no time step is enlarged or dynamics simplified. This
accelerates wall time only, not simulation time. Gazebo and SITL are not fully
lockstep; seed pairing is not identical motion, hence three repeats per arm.

```bash
bash sim/run_noisy_wind_campaign.sh 1 2 3   # skips existing flights
PY=/home/trananhduy/quadplane-indi/.venv/bin/python
$PY analysis/compare_controllers.py runs/noisy_wind
$PY analysis/extract_dataflash.py runs/noisy_wind/indi_w9_s1/flight.BIN
$PY analysis/imu_noise.py --real /home/trananhduy/quadplane-indi/logs/realflight.bin \
  --sim runs/pid_calm_s1/flight.BIN \
  --sim-noisy runs/noisy_wind/pid_calm_s2/flight.BIN \
  --out runs/noisy_wind/imu_match_final.png
```

The extractor writes one gzip-compressed CSV per DataFlash message (`IMU`,
`ATT`, `RCOU`, `INDI`, `INDF`, etc.) plus a row-count/provenance manifest in
each run's `extracted/`. Each time stamp is the unmodified `TimeUS`; do not
join unlike rates by row. The comparison produces `runs.csv`, `pulses.csv`,
`summary.md`, `metrics_w6.png`, `metrics_w9.png`, and matched pulse histories
in `runs/noisy_wind/compare/`. Plots have **no titles**; the figure captions
are in `paper&thesis/thesis.tex`.

The first noise-calibration log (`noise_cal_pid_calm`) already contained
injected noise and was incorrectly treated as clean. The calibration was redone
with a genuinely clean original PID flight and the known noisy calibration log;
`analysis/imu_noise.py --noise-config ... --suggest-out ...` solves for the
additional per-axis variance in the logged 20–120 Hz hover band. Only the new
campaign uses `sim/imu_noise_matched.json`; earlier profiles/results remain
unchanged. A **separate** calm seed-2 flight checked the final noise setting:

| logged IMU, 20–120 Hz RMS | real hover | new simulation | relative error |
|---|---:|---:|---:|
| roll / pitch / yaw gyro (deg/s) | 0.254 / 0.060 / 0.069 | 0.251 / 0.105 / 0.070 | -1% / +74% / +2% |
| x / y / z accel (m/s²) | 0.175 / 0.169 / 0.205 | 0.177 / 0.166 / 0.240 | +2% / -1% / +17% |

Pitch gyro already exceeds the real level without injected noise in some
simulated hovers; extra Gaussian noise cannot remove this motion. RMS matching
does **not** recreate the real 83/71/53 Hz rotor vibration lines, 42 Hz gyro
filter + notch, 300 Hz scheduler or real airframe/actuator behaviour. The PSD
shape error remains 6–14 dB (pitch 12 dB) on the independent calm check.
The real flight did not fly a wing-borne square, so this is a **hover IMU noise
check**, not a validated sim-to-real controller comparison. Keep the real
flight separate from the PID/INDI paired test; any later filter/plant changes
require new gain and stability checks.

Three-seed results (details and min/max spread in `runs/noisy_wind/compare/summary.md`):

| error RMS or pulse peak (deg) | PID | INDI |
|---|---:|---:|
| 6 m/s take-off pitch RMS | 1.23 | 0.32 |
| 6 m/s transition pitch RMS | 1.76 | 0.96 |
| 6 m/s fixed-wing roll RMS | 8.81 | 13.23 |
| 6 m/s cruise pitch-pulse peak | 12.28 | 18.42 |
| 9 m/s take-off pitch RMS | 2.02 | 0.32 |
| 9 m/s transition roll RMS | 2.71 | 1.70 |
| 9 m/s cruise roll-pulse peak | 100.67 | 25.72 |

The fixed-wing segment is **not necessarily unassisted**: assist averaged
57.8% (PID) versus 0% (INDI) at 6 m/s and 33.4% versus 10.6% at 9 m/s.
Some marker transitions were absent from the 10 Hz `RCOU` stream; pulse
analysis contains 92 of 96 scheduled pulses, so compare the corresponding
per-run CSV counts before interpreting an aggregate. No across-hardware
performance claim follows from this rerun.

## Log 2026-09-26 — zigzag profile, model audit, methodology rewrite

**Second flight profile: zigzag (quick turns).** `sim/Default square zigzag path.plan`
is a lawn-mower survey added to test quick turns. Geometry
(`python3 analysis/auxiliary/plan_geometry.py "sim/Default square zigzag path.plan"`):

- take-off at home to 35 m, speed command 16.1 m/s;
- 5 parallel legs of 500 m, heading 356°/176° (almost N–S), **100 m apart**, 35 m;
- 10 m run-out past each leg end, then a 100 m side step: each of the 4
  turnarounds is two ~90° turns 100 m apart;
- 20 waypoints, 3.0 km path; lands at N 663 m, E 380 m, **765 m from home**
  (not back at home, unlike the square).

Not flown yet. Before it can be flown:
1. `sim/fly_plan_mission.py:load_plan()` accepts only `SimpleItem`; this plan
   is one QGC `Survey` ComplexItem. Flatten it (the waypoints are in
   `TransectStyleComplexItem.Items`, see `plan_geometry.py`) or re-export it
   as plain waypoints. The survey also emits camera-trigger items (206) — harmless.
2. The pulse windows in `class Disturbance` key off "Reached waypoint #4",
   which is a different point of the zigzag. Decide where its pulses go.

Navigation (waypoint radius, L1 period, etc.) is left to ArduPilot and is not
discussed in the thesis; it is identical for PID and INDI.

**Model audit** (`python3 analysis/auxiliary/rank_lifting_surfaces.py
sim/gazebo_models/VTOL_Quadplane/model.sdf --openvsp "../analysis/python/twinbooms_data(1).csv"`):

| surface | S (m²) | CLmax | S·CLmax (m²) | share |
|---|---:|---:|---:|---:|
| main wing (centre) | 0.1832 | 1.25 | 0.230 | 54% |
| aileron sections ×2 | 0.0177 | **4.35** | 0.077 | 18% |
| elevator | 0.0155 | **4.35** | 0.067 | 16% |
| horizontal tail | 0.0425 | 1.25 | 0.053 | 12% |

- Wing and aileron coefficients (a0 0.13/0.15, cla 3.7/6.8, alpha_stall
  0.339/0.639) are **copied from the stock Zephyr model**, not from OpenVSP.
  Areas do match OpenVSP (0.201 vs 0.204 m²). OpenVSP: CLα 5.98 /rad, a0 0.049 rad.
- Aileron and elevator panels: CLmax 4.35 at 36.6° stall — not physical.
- `cma = 0` on every panel: OpenVSP pitching moment is not in the model.
- Total mass is **3.184 kg (31.2 N)**; the old "27.5 N" was base_link only.
  Stall speed: 15.0 m/s centre wing, 13.0 with aileron panels, 11.0 all panels.
  The earlier "0.494 m², 9.6–14.1 m/s" was wrong. AIRSPEED_MIN 14 is below the
  centre-wing bound; it only works because of the aileron panels' CLmax.
- **Rudders do not move**: channel 3 has multiplier 0.0010472, offset −1.5708
  → travel −0.094° … −0.034°. Ailerons/elevator ±25°.
- Rotor thrust comes from blade lift-drag panels, not from bench kT/kQ.

None of this was changed (changing the model invalidates the campaign).

**Thesis (paper&thesis/thesis.tex).** Methodology rewritten: shorter, plain
words, steps in order; code/parameter names moved to the appendix
("Names used in ArduPilot, Gazebo and the project files"). The transition
subsection now describes what branch `indi` does — a *handover* (motors run
INDI in hover/transition/assist, surfaces run INDI only after the transition is
complete and assist is off; `Plane::stabilize()`), not the motor/surface
pseudo-inverse allocation of the old text, which was never in this code. The
disturbance table now matches the flown schedule (4 event windows × roll+pitch,
1.44 / 3.24 N·m); the user had asked for 1.44 N·m at corners and leg midpoints,
which is not what `class Disturbance` does. Appendix parameter table now
matches `sim/indi.param`. Still stale and flagged with `% NOTE` in the .tex:
identification numbers in Results (61.7/55.4, τ 23 ms vs flown 56.7/44.8) and
the "surface INDI active in transition" claim.

## Log 2026-09-26 (later) — motors and surfaces share one INDI correction; two-profile campaign

**Backup first.** Pre-change state committed in `~/Bachelor-thesis` (clone of
github.com/trananhduyusth/Bachelor-thesis, commit "Backup of rework/ before the
joint motor/surface INDI change"): thesis, scripts, plans, params, model, run
summaries, ArduPilot as `patches/ardupilot/*.patch` on upstream `9f648ccabc`.
**Not pushed**: the stored GitHub login `TranAnhDuy123` has no write access
to that repo (403). Push it from `~/Bachelor-thesis` once access is sorted.
Local copy of the old code: ArduPilot branch `indi-handover-backup`.

**Code change (ArduPilot `indi`, uncommitted working tree).** Stock ArduPilot
runs the lift-motor rate loop *and* the surface rate loop together in hover
modes (surfaces slaved to the VTOL rate target), transition and assist. The
old branch put INDI only on the motors there (surfaces stock PID) and on the
surfaces only when wing-borne. Now both run INDI together and split one
correction (minimum-norm): `u_i = u0_i + G_i (nu - wdot_f) / (G_m^2 + G_s^2)`.
Two independent laws would double the correction (e_{k+1} = -e_k in the ideal
discrete loop); `analysis/auxiliary/joint_indi_margins.py` gives GM 9.0 dB /
PM 46° shared vs 3.5 dB / 27° independent at every airspeed.
- `AC_INDI.h/.cpp`: `AC_INDI_Share` (each set reports G, other reads G² if < 10 ms old); `update(..., G_sumsq)`.
- `AC_AttitudeControl_INDI.cpp`: motors use/report the share; log `INDI.RS/PS`.
- `AP_FW_Controller`: `set_indi(params, axis, G)`; G from the vehicle, not the speed scaler (which ArduPilot shrinks toward 0.001 at low speed in VTOL, i.e. G/scaler² → huge).
- `Attitude.cpp` (`Plane::stabilize`): G_s = FW_G·(V_EAS/SCALING_SPEED)², 0 below 0.5·AIRSPEED_MIN (7 m/s); offered in every phase; log `INDF.RS/PS`.
Build: `/usr/bin/python3 ./waf plane`.

**Harness.** `sim/fly_plan_mission.py`: flattens QGC Survey items (zigzag),
drops camera-trigger items, drops waypoints < 60 m before a NAV_LAND (VTOL land
needs ≥ 54 m approach; zigzag loses its last leg end + 10 m run-out, same line).
Pulse windows: roll 1 s, 2 s rest, pitch 1 s at take-off (+4 s), transition
(+3 s), every corner (turn > 45°, on "Reached/Passed waypoint" text *and*
MISSION_ITEM_REACHED — the latter is skipped after WP2 of square.plan because
a CONDITION_YAW follows it), every leg midpoint (legs ≥ 200 m), landing (+5 s);
corner/midpoint only between "Transition done" and back-transition; overlapping
windows are deferred. Pulses logged with vehicle time + position in
`mission_status.json`. Requests ATTITUDE 20 Hz, GLOBAL_POSITION_INT 10 Hz.
`sim/run_square_mission.sh` takes `PLAN=`; `sim/run_joint_campaign.sh` runs
both profiles.

**Campaign** (`bash sim/run_joint_campaign.sh 1`, ~75–100 s per run): 12/12
landed, all pulses applied → `runs/joint/{square,zigzag}/`. The first square w6
pair used the harness before the corner fix; moved to `runs/joint_superseded/`
and re-flown. Analysis:
```bash
PY=/home/trananhduy/quadplane-indi/.venv/bin/python
$PY analysis/compare_controllers.py runs/joint/square   # and runs/joint/zigzag
$PY analysis/path_error.py runs/joint/square runs/joint/zigzag
```
`analysis/path_error.py` (new): nearest-point error to the planned polyline,
x = north, y = east, z = height (transition/FW/back-transition only), signed
cross-track; per phase and straight legs only (> 60 m from a waypoint).

Findings (one seed — "clear" column in compare summaries is meaningless here):
- Split works: in transition surfaces take ~4 % (max 9 %) of each correction; 100 % wing-borne; ~1 % in 9 m/s hover.
- INDI lower hover/transition pitch RMS in all 6 pairs (×1.2–9) and pitch-pulse peaks (×2.5–10).
- Wing-borne roll mixed: better at 6 m/s, worse calm and 9 m/s; zigzag 9 m/s INDI corner upset 208° (PID max 138–140°).
- Zigzag straight-leg cross-track: INDI 1.7/2.6 m vs PID 2.4/4.1 m (6/9 m/s), worse calm (1.6 vs 0.95).
- 9 m/s: PID assisted 87 % (square) / 78 % (zigzag) of FW at 12.7/13.0 m/s; INDI 0 %/41 % at 16.9/14.6 m/s → INDI square overshoots downwind corner by 88 m.
- 9 m/s INDI back-transition starts early ("VTOL pos1 low speed", ~400 m out on zigzag) → its last-leg midpoint pulse is skipped.

## Log 2026-09-26 (evening) — hover torque test; first pulse of every run was lost

**Bug: the first torque of every run never reached Gazebo.** `sim/disturb.py`
waited a fixed 1 s (wall time) for gz-transport discovery; `publish()` returns
True even when no subscriber is connected, so the first message was dropped
silently. In every campaign so far (`runs/noisy_wind`, first `runs/joint`) that
was the **take-off roll pulse** (INDI moment estimate ≈ 0 there). Fixed: wait
until `has_connections()` on both topics. All 8 disturbed `runs/joint` missions
re-flown; old ones in `runs/joint_superseded/first_pulse_lost/`.

**Hover torque test** (`bash sim/run_hover_disturbance.sh` →
`runs/hover_test/{pid,indi}_{1p44,3p24}/`; `sim/fly_hover_disturbance.py`):
QLOITER over the origin via RC-override throttle, climb to 20 m, settle 15 s,
roll torque 5 s, 15 s, pitch torque 5 s, 15 s, QLAND. Calm, IMU noise.
Analysis: `$PY analysis/hover_disturbance.py runs/hover_test` (time-based plot,
pushes shaded; "takeover" = −Δu·G·I/M of the rate loop's own command).
Start the torque helper *before* take-off — started just before a push, the
logged start was stale by the connection wait (first attempt shortened the
roll push to 3.2 s; kept in `runs/hover_test_superseded/`).

| torque, axis | peak dev (°) PID/INDI | held dev (°) | drift (m) |
|---|---|---|---|
| 1.44 roll | 1.64 / 1.24 | 0.37 / 0.53 | 0.44 / 0.15 |
| 1.44 pitch | 3.26 / 0.79 | 0.81 / 0.04 | 0.90 / 0.09 |
| 3.24 roll | 4.02 / 4.10 | 1.53 / 1.50 | 2.05 / 1.08 (front-left motor at min in both → mixer saturation) |
| 3.24 pitch | 7.44 / 1.81 | 2.48 / 0.10 | 2.02 / 0.20 |

INDI flag 100 % during pushes; PID pitch rings 1.7–3.4° RMS for 10 s after
release, INDI 0.1–0.3°. Both commands settle at takeover ≈ 0.8 → the command
sees ≈1.25× the identified G (thrust compensation gain? not checked).

**Why wing-borne INDI is not better** (checked on the logs):
- FW roll-*rate* tracking: INDI 2.0–3.3 °/s RMS vs PID 7–31 °/s; lag 130 vs 280 ms (calm). Rate loop is better.
- FW roll *attitude* error 13–18° RMS for both, ailerons ~5 % travel in calm: set by navigation bank steps (≤45°) + FW angle loop, not the rate loop.
- 9 m/s: full-aileron moment G_s·Ixx = 2.2–3.0 N·m < 3.24 N·m pulse → INDI saturates ailerons in 100 % of FW roll pulses; PID flew assisted (motors 79–95 %) and could use lift motors.
- Model: unphysical aileron/elevator panels, dead rudders, weak G_s identification.

`analysis/compare_controllers.py` now draws `timeline_<case>.png/pdf`
(attitude error vs time since arming; pulses shaded per run/axis; phase ticks)
instead of the phase-axis `metrics_*.png`.

**Common clock + saturation (2026-09-26, later).** `analysis/alignment.py` maps
the INDI run onto the PID run's clock: each pulse window both runs received is
an anchor, true time inside the 4.5 s window, linear stretch between windows
(phase changes as anchors in calm runs). Used by `timeline_<case>`,
`path_<profile>` (cross-track / height panels now shaded where torque is on)
and the new `analysis/saturation.py` → `runs/joint/<profile>/saturation/`
(`summary.md`: % time lift motors on / at min / at max, surfaces and pusher
saturated per phase; `pulses.csv`: did each pulse meet a saturated effector
within 2 s; `saturation_<case>.png/pdf`). `path_error.py` also writes
`path3d_<profile>.png/pdf` (whole flight in 3D, height ×8, pulse starts as dots).

At 9 m/s every hover/transition/landing roll pulse met a lift motor at its
limit for both controllers; wing-borne, PID's pulses met lift motors at a limit
(13/14 square, 24/26 zigzag — it flies assisted), INDI's met saturated surfaces
(10/14, 17/24). At 6 m/s saturation is rare. So the 9 m/s differences are
effector saturation, not the rate law. Lowest wing-borne height: 16.8 m (PID
square w9), 14.4 m (INDI zigzag w9), after upsets.

**Thesis restructure (2026-09-26).** Results rewritten around the new runs:
summary table `tab:summary` first; removed figures from superseded runs (F2 PID
rate tracking, old out-and-back path, F4 effectiveness/handover, F5 stability,
F6 transition, F8 pseudo-control, F9 authority); kept real-log F1/F3 and IMU-noise
figure. Identification table now the flown values (runs/sysid_*/identify*.json,
fit above 3 Hz). Crab angle re-measured on joint runs (within 2° of asin(W/Va)).
New lit-review subsection `sec:pid-vs-indi-disturbance` (integral vs incremental).

## Log 2026-09-26 (night) — saturation-aware INDI; final campaign `runs/joint_final`

**Code (ArduPilot `indi`, uncommitted; full diff in `patches/ardupilot-final-uncommitted.patch`,
previous state in `patches/ardupilot-shared-correction-uncommitted.patch`).**
1. Redistribution: surfaces report saturation direction (|u| >= 0.999) in `AC_INDI_Share`;
   the motor loop drops G_s^2 from the denominator when the needed correction points
   into that limit, so the motors take the whole remainder in the same step.
2. Saturation-triggered assist (vehicle level, PID and INDI): new `Q_ASSIST_SAT_MS`
   (`VTOL_Assist::should_assist`, QTUN assist bit 7); aileron or elevator at >= 98 %
   for 100 ms starts the lift motors, clear 1 s after. Overlay `sim/assist_sat.param`,
   appended to both parameter files via `EXTRA_PARAMS` in `sim/run_campaign.sh`.
3. Surfaces first (INDI): fixed-wing mode and airspeed >= AIRSPEED_MIN → surfaces take
   the whole correction, motors take it only where surfaces are saturated, otherwise
   their roll/pitch command fades (tau 0.5 s). Flag set in `Plane::stabilize()`.

Runs: `OUT_ROOT=$PWD/runs/joint_final EXTRA_PARAMS=$PWD/sim/assist_sat.param bash sim/run_joint_campaign.sh 1`
(also `runs/joint_assist`: steps 1+2 only). New analysis: `analysis/pulse_effectors.py <root>/<profile> w6 w9`.

9 m/s wing-borne, square/zigzag (INDI fixed split → +assist/redistribution → +surfaces first; PID):
roll RMS 17.0/19.3 → 12.2/11.4 → 10.6/10.5 (PID 15.8/15.5); pitch RMS 3.4/5.9 → 6.1/5.2 → 1.7/2.2
(PID 3.4/4.2); upsets 0/4 → 0/0 → 0/0 (PID 3/7). Step 2 alone made pitch worse (motors got
~93 % of each pitch correction via hover G_m); step 3 fixed it.
Remaining: calm wing-borne attitude (navigation-dominated) and 9 m/s landing hover (INDI arrives
faster: 5.7–8.8 vs 3.3–3.9 m/s → up to 30° braking pitch). Motor fade is ~5 s not 0.5 s: the yaw
PID saturates the motors in wing-borne assist (dead rudders), applied roll follows the mixer.

## Hover test video (2026-10-07)

`bash sim/run_hover_recording.sh` re-flies the four hover torque runs into `runs/hover_video/`
with a fixed camera (`sim/worlds/hover_camera.sdf`, inserted via the new `WORLD_INSERT`; rendered
with `HEADLESS_RENDER=1`, i.e. `gz sim --headless-rendering`) at real time (RTF=1, SITL speedup 1).
`sim/record_camera.py` (system python) saves frames by sim time, ground-truth pose and torque
on/off events. `python analysis/hover_video.py runs/hover_video` writes
`hover_side_by_side.mp4` (2x2: rows 1.44 / 3.24 N.m, columns PID / INDI) and `hover_1p44.mp4`,
`hover_3p24.mp4` (PID | INDI), aligned at the roll push, with torque banner, roll/pitch/drift
readouts and a strip chart; encoded with GStreamer openh264enc (no ffmpeg on this machine).
The recorded runs reproduce `runs/hover_test` within 0.3 deg / 0.02 m. At 3.24 N.m both
controllers yaw ~90 deg during the roll push (motor saturation drops yaw; yaw is stock PID).
Gotcha: never put the runner's name in a `pkill -f` pattern on the same command line that
also starts it — the pattern matches the calling shell.

## Fixed-wing video, 6 m/s square (2026-10-07)

`bash sim/run_fw_recording.sh` re-flies case w6 of `runs/joint_final/square` (same plan, wind,
turbulence, seed, 1.44 N.m pulses, firmware and parameter files) for PID and INDI into
`runs/fw_video/{pid,indi}_w6/`, at real time, with a chase camera. The camera is a sensor in
the aircraft's base_link (`sim/worlds/chase_camera_sensor.sdf`, added to the run's model copy
by the new `MODEL_INSERT`; `sim/worlds/render_sensors.sdf` adds the render system): 3.4 m
behind, 1.1 m right, 0.8 m above, square 1200 px. A sensor has no mass, so the dynamics are
unchanged, and a body-fixed camera has no tracking lag or jitter (a world camera moved with
`set_pose` would). `run_square_mission.sh` starts `sim/record_camera.py` when `RECORD_TOPIC` is
set; the runner keeps only frames from the transition pulses to the last wing-borne pulse.
`python analysis/fw_video.py runs/fw_video --anchor mid_2 --before 4 --length 42` writes
`fw_w6.mp4` (PID | INDI): each frame is rotated about the aircraft by the camera's roll
(from the Gazebo attitude) so the horizon is level, then cropped to 960x540; the INDI clock is
aligned to the PID clock window by window as in `analysis/alignment.py` (INDI flies the clip
~5 % faster, so its between-window stretches are slowed; inside a window both are real time).
Overlay: actual and commanded roll / pitch (ATT), distance from the planned track (Gazebo
truth), torque banner with mid-leg / corner, strip chart of the error (commanded - actual).
Sim and log clocks agree within 0.08 s. Peak roll error per pulse follows the campaign
(PID / INDI, deg): mid_2 42 / 3, mid_4 49 / 25, mid_5 68 / 20, mid_6 80 / 4.5; corners are
large for both (commanded turn). Reading the attitude itself: during the 1 s roll pulse the
torque moves PID 25-50 deg (mid_2 2 -> 29, mid_4 -35 -> -5, mid_5 2 -> 51) and INDI 1-3 deg.
INDI's mid_4 / mid_5 errors are lag behind a commanded roll-out after the corner (INDI follows
bank commands ~1-1.5 s late, PID faster but with overshoot) - visible in the video and worth
a check of the INDI fixed-wing command path / gains if roll tracking matters.
The INDI recorder aborted in interpreter teardown (pybind11 GIL check) after saving everything,
which failed the runner before the log copy; fixed with `os._exit(0)` in the recorder and a
tolerant `wait`, and that run's log copy and frame pruning were done by hand.

## Caveats

- `AHRS_EKF_TYPE = 10`: the controllers use **SITL ground-truth attitude and
  rates**, not EKF3. There is also no gyro noise or vibration in SITL; the real
  aircraft's ω̇ is much noisier. INDI results here are therefore best case for
  the ω̇ estimate.
- **SITL is not run in lockstep with Gazebo**, so identical runs scatter a
  little. The campaign flies 3 seeds per case and only calls a difference
  "clear" when it is larger than that scatter.
- `WP_RADIUS = 90` on a 250 m square: waypoints are accepted 66–86 m early and
  corners are cut. This is the same for both controllers.
- The `NAV_VTOL_LAND` item in `square.plan` has altitude 35 m. With no
  rangefinder, ArduPlane measures descent height against that waypoint, so
  "Land final" starts at about 29 m and the landing descends at 0.5 m/s. This
  is the same for both controllers. Setting the item's altitude to 0 fixes it.
- The Gazebo plugin (`sim/ardupilot_gazebo/src/ArduPilotPlugin.cc`) has a
  local, uncommitted patch that sends `velocity_wind`.

## Where an INDI attitude controller plugs into stock ArduPlane

Call chain in VTOL flight, per 400 Hz loop (`SCHED_LOOP_RATE`):

```
Plane FAST_TASK stabilize()      ArduPlane/Attitude.cpp:424   -> mode->run() sets attitude targets
Plane FAST_TASK set_servos()     ArduPlane/servos.cpp:894     -> quadplane.update()
  QuadPlane::motors_output()     ArduPlane/quadplane.cpp:1906
    attitude_control->rate_controller_run()                   <-- the rate loop INDI replaces
      AC_AttitudeControl_Multi::rate_controller_run_dt()      AC_AttitudeControl_Multi.cpp:456
        _motors.set_roll/pitch/yaw(PID.update_all(target, gyro))   normalised [-1,1] torque demands
    motors->output()  -> AP_MotorsMatrix::output_armed_stabilizing()  mixer, saturation, THST_EXPO
```

What matters for the design:

- **Object and replacement point.** `QuadPlane::setup()`
  (`quadplane.cpp:731`) creates `AC_AttitudeControl_TS`, a subclass of
  `AC_AttitudeControl_Multi`. An INDI class would derive from it, override
  `rate_controller_run_dt()`, and keep the angle loop (`input_euler_*` →
  `_ang_vel_body_rads`) unchanged. INDI's virtual input `u` is then the
  normalised roll/pitch/yaw sent to `_motors.set_*()`, with effectiveness `G`
  in rad/s² per unit command.
- **Actuator feedback `u0`.** The mixer rescales roll/pitch/yaw on saturation
  (`rpy_scale`, `AP_MotorsMatrix.cpp:360`) and raises `limit.roll/pitch/yaw`.
  `u0` must be what was *applied*, not what was requested. It must pass through
  the same filter as the angular-acceleration estimate (the synchronization
  result; see references below), or the loop limit-cycles.
- **Gyro path.** `get_gyro_latest()` is already filtered by `INS_GYRO_FILTER`
  (20 Hz here), and the harmonic notch is off. The ω̇ derivative has to account
  for that lag too.
- **Surfaces are slaved to the VTOL rate loop in VTOL modes.**
  `Plane::stabilize_roll_get_roll_out()` (`Attitude.cpp:149`) and the pitch
  equivalent read `attitude_control->get_rate_roll_pid().get_pid_info().target`
  whenever `!quadplane.use_fw_attitude_controllers()`. A replacement loop must
  keep that PID info populated, or the ailerons and elevator silently get a
  zero rate target in hover and transition.
- **Parameters.** `Q_A_` is registered in `ArduPlane/Parameters.cpp:818` with
  the static `AC_AttitudeControl_Multi::var_info`, so a subclass can't add
  parameters to it. New INDI parameters need their own group, for example a
  pointer subgroup in `QuadPlane::var_info2` (`quadplane.cpp:296`; next free
  index 44).
- **Fixed-wing side (later).** `AP_FW_Controller::run_rate_control()`
  (`libraries/APM_Control/AP_FW_Controller.cpp:132`) is the roll/pitch rate loop
  for surfaces. Its effectiveness scales with dynamic pressure, which ArduPlane
  already carries as the speed scaler (`Plane::calc_speed_scaler`).

What the references say that this code needs (Steinert et al. 2025, *Survey on
INDI* Parts I and II; Lombaerts et al. 2019, AIAA 2019-0134):

- Sensor-based INDI: `u = u0 + G⁻¹(ν_des − ω̇0)`. The `f(x)` model is replaced
  by the measured ω̇; only `G` is needed (Part I, §4, and §12.1 for rate loops).
- **Synchronize** the ω̇ path and the `u0` path: the same filter, and the same
  delay. Unequal delays cause oscillation, and the ω̇ path is the more sensitive
  one (Part II, §7 and §9.2).
- **Actuator dynamics** (motor spool time `Q_M_SPOOL_TIME`, ESC lag) are the
  weakest assumption. If they are slow relative to the loop rate, `u0` should
  come from an actuator model or from RPM (Part I §9, E-INDI/ANDI; Part II §7.2).
- **Saturation.** Roll/pitch come before thrust, and thrust before yaw. That is
  the prioritized WLS allocation in Lombaerts §VI, and ArduPilot's mixer already
  approximates it with yaw headroom and `rpy_scale`. Keeping ArduPilot's mixer
  is the simple first step.
- **Effectiveness `G` has to be identified with deliberate excitation.**
  Closed-loop hover data only returns the inverse of the controller (see
  project memory / old repo F-05).
- Beard & McLain ch. 3–4 (rigid-body dynamics, aero and propulsion moments) and
  ch. 14 (multirotors) provide the model that `G` approximates.
- `Analysis_of_VTOL_UAV_Propellant_Technology.pdf` is a general trend review
  with nothing specific to the controller.



<!-- 


Plan: INDI rate control (hover, transition, forward flight) with a PID-vs-INDI comparison under wind and injected disturbances
Context
The stock-PID baseline in rework/ is done and repeatable (runs/pid_calm_01, pid_calm_02; README.md). The next job follows the README's three "Next steps", in order:

Validate the simulator against the real aircraft.
Identify control effectiveness G.
Build INDI and A/B it against PID on the same plan and parameters.
The user added:

INDI must cover hover, transition and forward flight. INDI replaces the attitude/rate loop, not altitude.
Disturbances on roll and pitch at 6 and 9 m/s: steady Gazebo wind plus identical injected roll/pitch moment pulses.
A script that compares stock PID with INDI.
The real flight log is 1st fbwa(real flight).bin. It is byte-identical to /home/trananhduy/quadplane-indi/logs/realflight.bin.

Why the old attempt failed, and what that changes here
The old tree (firmware/ardupilot, mostly uncommitted) had these defects:

The flown G was 10× wrong.
There was a 14.7 Hz limit cycle: the synchronization path left out the gyro filter, giving −13° phase margin.
The fed-back u0 was taken before the mixer.
The PID integrators went stale while INDI ran.
Parameter indices were reused.
The "PID baseline" was not stock.
Wind never reached the airspeed.
There was no matched A/B pair.
So in this plan:

One binary serves both arms.
With Q_INDI_ENABLE=0 the code path is exactly stock, and this is checked by regression against the existing baseline.
G is identified with deliberate excitation, not from closed-loop flight data.
Synchronization covers every filter and delay in the ω̇ path.
u0 is what the mixer actually applied.
What the references prescribe (and where the design uses it)
Steinert et al. 2025, INDI survey Part I

The sensor-based law is u = u0 + G⁻¹(ν − ω̇0). Only G is model-dependent (§4, §12.1).
Actuator dynamics delay Δu. With motors, use an actuator model for u0 (E-INDI, §9).
Quad-plane INDI appears in Zhou et al. and De Ponti et al. (§13.6).
Steinert et al. 2025, Part II

ω̇ comes from differentiating the rate and passing it through a 2nd-order low-pass (§6.2).
Synchronize: apply the identical filter and delay chain to u0 (§7.1). Unequal delays cause oscillation, and ω̇ delay is the more harmful of the two (§9.2).
Over-actuation (rotors + surfaces) is handled by allocating on increments with a weighted pseudo-inverse. Watch for path dependency (§8).
Gains are designed against margins using a model that includes the filters (§10).
Lombaerts et al. 2019 (AIAA 2019-0134)

Hover INDI needs only inertia, geometry and the rotor torque/thrust constant.
Filter H = ωn²/(s² + 2ζωn s + ωn²) with ωn ≈ 80 rad/s and ζ = 1, applied to both ω̇ and u0.
Error controller: ν = K(ref − y) + FF.
Saturation priority is roll/pitch > thrust > yaw (§VI). ArduPilot's mixer already approximates this, so we keep it.
Beard & McLain

Ch. 4.2: the aileron/elevator moment is ∝ q̄ = ½ρV², so G_surf(V) = G_ref·(V/V_ref)² = G_ref / scaler², using ArduPlane's speed scaler.
Ch. 14.4: rotor moments. Model G_rotor = arm · ∂T/∂u / I.
Step 0 — Simulation infrastructure (shared by both controllers)
All ArduPilot changes go on branch indi in rework/ardupilot (a nested repo, ignored by the outer repo). Each commit is also exported with git format-patch into rework/patches/ so the outer repo keeps a record.

SITL wind→airspeed fix. In libraries/SITL/SIM_JSON.cpp:437-459, apply state.velocity_wind to wind_ef before computing velocity_air_ef/bf and update_eas_airspeed(). This is the minimal patch from the exploration and matches the old tree's fix. Separate commit.
Repeatability
Add SEED to sim/run_gazebo_sitl.sh, passed as gz sim --seed $SEED. This fixes the turbulence realisation, so PID and INDI see the same gusts.
Try <lock_step>1</lock_step> in sim/gazebo_models/VTOL_Quadplane/model.sdf. Keep it only if the calm run still completes. Either way, every case is flown with 3 seeds.
Moment-disturbance injection
Add <plugin filename="gz-sim-apply-link-wrench-system" name="gz::sim::systems::ApplyLinkWrench"/> to sim/worlds/quadplane_runway.sdf.in (world zephyr_runway).
New sim/disturb.py drives it: gz topic on /world/zephyr_runway/wrench/persistent to apply, /wrench/clear to remove. The target is My_QuadPlane::base_link.
The body-axis torque is rotated into world ENU using the ATTITUDE received over MAVLink at pulse start. Note that base_link is FLU, so FRD pitch = −y.
Size of the moment: M = ½ρ u_g² · S_w · (b/4), with u_g = 6 or 9 m/s and S_w, b taken from model.sdf. That is roughly 1.4 N·m at 6 m/s and 3.1 N·m at 9 m/s, i.e. a full-q̄ gust on one semi-span. The script prints M/I (rad/s²) and M/(G·I) (the fraction of rotor authority).
Pulse schedule per run, the same for both controllers: roll +M for 1.0 s, then 4 s rest, then pitch +M for 1.0 s. This is repeated in four windows:
hover climb (8 s after arming);
transition (3 s after "Transition started");
cruise (5 s after "Reached waypoint #4");
landing hover (5 s after "Land descend started").
sim/fly_plan_mission.py gains --disturb-nm M and fires the pulses from those STATUSTEXT events.
Marker in the DataFlash log: MAV_CMD_DO_SET_SERVO on channel 9 (SERVO9_FUNCTION=0, nothing connected) is set to 1900 (roll) or 1100 (pitch) during a pulse and 1500 otherwise. RCOU.C9 then shows exactly when each pulse happened.
Wind cases: steady wind from the west (Gazebo WIND_E = 0 / 6 / 9 m/s), with the old campaign's light turbulence (TURB_MAG 0.3, TURB_DIR 0.02, TURB_VERT 0.05). The square then gives crosswind on the N/S legs and head/tailwind on the E/W legs.
Checks:
A calm rerun reproduces the pid_calm_01 metrics.
At 9 m/s, logged airspeed ≈ |ground velocity − wind|, with upwind vs downwind ground speed differing by about 18 m/s.
A hover pulse test: a pure roll torque gives mostly roll response, with cross-axis peak below 15 %.
Step 1 — Sim vs real (README next step 1)
New analysis/sim_vs_real.py.

Real log: /home/trananhduy/quadplane-indi/logs/realflight.bin.
About 40 s airborne in QHOVER (93.4–133.4 s, from QTUN), and 8.4 s of FBWA at about 4 m/s. It was never wing-borne and had no airspeed sensor.
So only hover can be validated. Cruise throttle and forward-flight response stay open and are stated as such in the README.
Controller-parameter diff (real vs sim/stock_pid.param): Q_A_RAT_* (real RLL P 0.16 vs SITL 0.7), INS_GYRO_FILTER (42 vs 20), harmonic notch (on vs off), SCHED_LOOP_RATE (300 vs 400), Q_M_THST_HOVER (0.408 vs 0.298), Q_M_THST_EXPO.
Same controller, same inputs:
Build sim/realctl.param = stock_pid.param with the real aircraft's Q_A_*, INS filter, notch, loop-rate and motor-curve parameters.
New sim/replay_sticks.py replays the real flight's RCIN roll/pitch/yaw/throttle sticks into SITL QHOVER via RC_CHANNELS_OVERRIDE (system ID 255).
Plant differences then show up directly in the comparison.
Metrics, real vs SITL:
airborne hover throttle;
DesRoll→Roll and DesPitch→Pitch lag (cross-correlation peak) and RMS error;
rate tracking RMS (PIQx);
motor-command spread;
gyro and ω̇ noise spectrum (real roll noise is about 47× SITL).
Gate (reported, not silently tuned away):
hover throttle within ±15 %;
attitude-response lag within ±30 ms;
RMS errors within 2×.
If the gate fails, a model fix (motor constant or inertia in model.sdf) is a separate, documented commit, followed by a re-baseline.
Decision recorded: which controller parameter set is "the PID" for the A/B.
Preferred: realctl.param, if it flies the model stably.
Otherwise the SITL-tuned set, with that fact documented.
Optional robustness case: set SIM_GYR1_RND to the real noise level (it works under JSON because it is applied in the INS SITL backend).
Step 2 — Identify effectiveness G and actuator dynamics (README next step 2)
Use stock SystemID: SITL builds have AP_PLANE_SYSTEMID_ENABLED, and it logs to SIDD and SIDS.

New sim/fly_sysid.py:

Hover: VTOL takeoff to 40 m, switch to QLOITER, then for each axis set SID_AXIS 10/11/12 (mixer roll/pitch/yaw). Settings: SID_MAGNITUDE 0.05–0.1, SID_F_START_HZ 0.5, SID_F_STOP_HZ 20, SID_T_REC 40. Start/stop with MAV_CMD_DO_AUX_FUNCTION 184.
Forward flight: SID_AXIS 22/23 (FW mixer roll/pitch) in LOITER at AIRSPEED_CRUISE set to 16, 20 and 24 m/s.
New analysis/identify_G.py (reusing methods from the old analysis/python/thesis_figures.py and indi_validation.py):

Instrumental FRF: use the injected chirp r as the instrument, H(u→ω̇) = S_rω̇ / S_ru. Plain S_uω̇/S_uu is avoided: with the loop closed it returns the inverse of the controller.
Gate: keep only bins with coherence ≥ 0.6 that sit ≥ 12 dB above the controller-inverse floor |2πf / C_PID(jω)|, where C_PID is built from the logged P/I/D/FLT* values. The floor is indi_inverse_asymptote for INDI runs.
Fit G·e^{-sT}/(1+sτ). This gives G (rad/s² per unit normalized command), the actuator time constant τ, and the delay T, for rotors (roll/pitch/yaw) and surfaces (roll/pitch).
Cross-checks:
G_rotor against the physical model computed from model.sdf: inertia Ixx 0.140, Iyy 0.214; motor positions ±0.294 m x, ±0.244 m y; rear motors tilted ±9°; thrust curve from hover. This is the Lombaerts "inertia + geometry + thrust constant" point.
G_surf ∝ V²: the three airspeeds must collapse onto G_ref/scaler² within ±20 %.
Yaw: if it fails the gate (as F-04 did in the old tree), INDI yaw stays disabled and yaw remains PID.
Output: sim/indi.param overlay with the identified Q_INDI_* values and their source run IDs.

Step 3 — INDI implementation (README next step 3)
Library code
libraries/AC_AttitudeControl/AC_INDI_Axis.h/.cpp (new). Ported from the old AP_INDI_Axis.h with its defects fixed. One scalar axis:

ω̇ path: ω_gyro (already passed through the INS LowPassFilter2p(INS_GYRO_FILTER)) → backward difference at nominal dt = 1/loop_rate → 2nd-order filter H(ωn, ζ=1).
u0 path (synchronized): u_applied → actuator model e^{-sT}/(1+sτ) → the same INS LowPassFilter2p → the same H. The same nominal dt is used everywhere, to avoid the jitter problem F-09.
Law: u = u0_f + Σ_j w_j G_j (ν − ω̇_f) / Σ_j w_j G_j². This is the weighted minimum-norm increment over this axis's effectors (rotor and/or surface).
Output clamped to ±1. Optional safety clamp on the increment, default wide.
reset(u, ω) seeds bumplessly on entry.
libraries/AC_AttitudeControl/AC_AttitudeControl_INDI.h/.cpp, subclass of AC_AttitudeControl_TS. It overrides rate_controller_run_dt():

ν = _ang_accel_target (FF) + K_ω·(ω_target + sysid − ω). The angle loop and Q_A_ANG_* are unchanged, so the A/B isolates the rate loop.
u_applied for the rotors is read back from the mixer. For each motor, thrust_i = thr_lin.actuator_to_thrust(get_raw_motor_throttle(i)). Then u_roll = Σ roll_factor_i·thrust_i / Σ roll_factor_i², and the same for pitch and yaw. This picks up rpy_scale saturation, yaw headroom, slew and expo, and fixes the old pre-mixer bug.
Rotor G is scheduled with thrust: G = G_hover · (∂T/∂u at current throttle) / (∂T/∂u at hover), using thr_lin. The old linear-throttle schedule is not used.
PID objects are kept consistent every INDI cycle: set_target_rate, set_actual_rate, reset_I. PID info stays populated, which matters because Plane::stabilize_*_get_*_out reads get_pid_info().target for surfaces slaved in VTOL. Handing back to the PID is then bumpless.
update_throttle_gain_boost() and update_throttle_rpy_mix() still run on every path.
_actuator_sysid is still added, so SystemID can excite INDI runs too.
libraries/APM_Control/AP_FW_Controller.{h,cpp}. run_rate_control() gets an INDI branch:

ν = rate FF + K_fw·(desired − gyro), with G_surf = G_ref/scaler².
u_applied = the last commanded surface / 4500 (Gazebo surfaces are joint position servos, whose τ is identified in step 2).
Output in centidegrees, and _pid_info is filled in.
It is still called exactly once per loop, because of the SITL panic check at :134-146.
The rudder stays stock.
Transition and over-actuation
ArduPlane/Attitude.cpp, where VTOL slaves the surfaces (!use_fw_attitude_controllers(), roughly lines 149–232):

When INDI is on, the multicopter INDI axis owns both effectors for roll and pitch: B = [G_rotor, G_surf(V)] with weights w_rotor, w_surf(V).
The surface share of the single increment goes to the aileron/elevator, instead of running a second, independent loop. Two parallel INDI loops acting on the same ω̇ would each try to cancel the whole error, doubling the loop gain.
Once use_fw_attitude_controllers() is true (fixed-wing and assisted flight), the FW-controller INDI owns the surfaces and the rotors follow stock assist logic.
The mode boundaries use reset() seeding so the handover is bumpless.
Parameters, selection, logging
Parameters go in their own group, Q_INDI_, via AP_SUBGROUPPTR(indi_params, "INDI_", 44, QuadPlane, AC_INDI_Params) in QuadPlane::var_info2 (quadplane.cpp:296). Fresh indices only.

Parameter	Meaning
ENABLE	reboot to take effect
AXES	bitmask: MC roll/pitch/yaw, FW roll/pitch
FILT_HZ	filter ωn/2π, default 12.7 Hz (80 rad/s)
RLL_K, PIT_K, YAW_K	rate error gains
RLL_G, PIT_G, YAW_G	rotor effectiveness at hover
ACT_TC, ACT_DLY	rotor actuator time constant and delay
FW_RLL_G, FW_PIT_G	surface effectiveness at the scaling speed
FW_TC	surface actuator time constant
FW_RLL_K, FW_PIT_K	surface rate error gains
INC_MAX	increment safety clamp
Q_A_ stays as it is.

Selection (QuadPlane::setup(), quadplane.cpp:731):

Q_INDI_ENABLE=1 → allocate AC_AttitudeControl_INDI.
Otherwise → stock AC_AttitudeControl_TS, and every INDI hook is short-circuited. The PID arm of the A/B runs ENABLE=0 from the same binary.
Logging: INDI message via WriteStreaming at loop rate. Fields: TimeUS, Act mask, ν[3], ω̇f[3], u0f[3], du[3], u[3], G[3], lim. Plus INDF for surfaces: ν, ω̇f, u0f, u, G(V), aspd.

Gain design before flight
New analysis/indi_margins.py: a discrete linear model per axis containing:

the rigid body G/s;
the identified actuator e^{-sT}/(1+sτ);
the INS LPF2p;
H;
the synchronization path;
one sample of delay.
Pick K_ω = the largest value giving GM ≥ 6 dB and PM ≥ 45°, with rate bandwidth at least that of the measured PID loop. Rotor and surface are each checked across the airspeed range.

Step 4 — Validate INDI alone, then the A/B campaign
Validation gates (calm, before any comparison)
Replay equivalence: new analysis/indi_replay.py re-runs the Python reference of the axis law on the logged INDI inputs. Outputs must match the logged u to within 1e-4.
Regression: Q_INDI_ENABLE=0 on the indi branch reproduces the pid_calm_01 metrics (hover attitude RMS within 10 %, same event times ±0.5 s).
INDI calm mission:
INDI active ≥ 95 % of time in every phase where it is enabled;
no limit cycle: rate RMS above 5 Hz ≤ 2× the PID value, and no single spectral line carrying > 50 % of the power;
ν vs ω̇_f correlation ≥ 0.8 below 5 Hz;
no actuator saturation beyond PID's level.
Campaign
New sim/run_campaign.sh: {pid, indi} × {calm, w6, w9} × seeds {1, 2, 3} = 18 runs of about 3 min each, via run_square_mission.sh.

WIND_E = 0 / 6 / 9.
--disturb-nm from the 6 or 9 m/s sizing. There are no pulses in calm; calm is the undisturbed reference.
Parameters: PARAMS = the chosen PID set, plus sim/indi.param for the INDI runs.
Output goes to runs/<ctrl>_<case>_s<seed>/.
Comparison script: analysis/compare_controllers.py
It imports the phase and metric functions from analysis/validate_log.py, refactored into importable functions without changing behaviour.

Per run, per phase (hover takeoff, transition, fixed-wing, back-transition, hover land):

attitude and rate tracking RMS and max;
height/TECS error;
cross-track error;
airspeed;
motor and surface usage;
% of samples at saturation;
assist % of time;
the spectral check.
Per disturbance pulse (found from RCOU.C9):

peak roll or pitch deviation;
IAE over the 5 s after the pulse;
time to get back within 1°;
peak rate;
peak actuator command;
whether it saturated.
Also the off-axis coupling peak.
Aggregation: mean ± range over the 3 seeds; Δ = INDI − PID. A difference is called real only if it exceeds the combined seed range. The verdict rules are fixed in the script before the campaign runs.

Outputs:

runs/compare/summary.md and summary.csv;
overlay plots of PID vs INDI for each pulse type and phase;
per-case bar charts;
the README gets a results section linking to them.
Critical files
New

ArduPilot branch indi:
libraries/AC_AttitudeControl/AC_INDI_Axis.*
libraries/AC_AttitudeControl/AC_AttitudeControl_INDI.*
AC_INDI_Params (in the same place)
rework/sim/:
disturb.py
fly_sysid.py
replay_sticks.py
run_campaign.sh
realctl.param
indi.param
rework/analysis/:
sim_vs_real.py
identify_G.py
indi_margins.py
indi_replay.py
compare_controllers.py
rework/patches/
Modified

libraries/SITL/SIM_JSON.cpp
libraries/APM_Control/AP_FW_Controller.*
ArduPlane/quadplane.{h,cpp} (setup, var_info2)
ArduPlane/Attitude.cpp
ArduPlane/Log.cpp, or WriteStreaming in the library
sim/run_gazebo_sitl.sh (SEED)
sim/worlds/quadplane_runway.sdf.in (wrench plugin)
sim/fly_plan_mission.py (--disturb-nm, C9 marker)
analysis/validate_log.py (refactor into importable functions)
README.md
Reused as-is

sim/run_square_mission.sh
sim/square.plan, unchanged. The known quirks (WP_RADIUS 90, VTOL_LAND at 35 m) are kept identical for both arms.
Reference only (logic ported, not copied blindly)

/home/trananhduy/quadplane-indi/firmware/ardupilot/libraries/AC_AttitudeControl/AP_INDI_Axis.h
old scripts/identify_effectiveness.py
old analysis/python/{thesis_figures,indi_validation,square_campaign_summary}.py
Verification (end to end)
Build: /usr/bin/python3 ./waf plane on branch indi.
Regression: PARAMS=sim/stock_pid.param bash sim/run_square_mission.sh reg_pid_calm. Metrics must match pid_calm_01.
Wind: WIND_E=9 SEED=1 bash sim/run_square_mission.sh wind_check. Airspeed ≠ ground speed, and the crab angle is visible.
Disturbance: a hover-only pulse check; the RCOU.C9 markers line up with the roll/pitch response.
Step 1: analysis/sim_vs_real.py. Gate table written to the README.
Step 2: sim/fly_sysid.py then analysis/identify_G.py. G, τ, T with gate dB and model cross-check; this produces sim/indi.param.
Margins: analysis/indi_margins.py. GM/PM table.
INDI calm: analysis/indi_replay.py plus the validation gates.
Campaign and comparison: bash sim/run_campaign.sh, then analysis/compare_controllers.py runs/. This produces runs/compare/summary.md with the PID vs INDI table, per-pulse metrics and plots.
Each step is a gate. If a gate fails, stop, record it in the README, and fix it before moving on. Do not skip ahead to the A/B. -->