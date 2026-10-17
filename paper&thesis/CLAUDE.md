# CLAUDE.md — Quadplane INDI Thesis (revision 2)

Context file for AI-assisted review and editing sessions. Read this first, then `TODO.md`.
Update both at the end of every session. This file replaces the version written for the first draft (2026-09-21); the old line-number references no longer apply.

---

## 1. Project snapshot

| Item | Value |
|---|---|
| Document | Bachelor thesis, USTH, Dept. of Space and Earth Sciences, ~58 pages PDF (`thesis.pdf`) |
| Author | Tran Anh Duy |
| Supervisors | Assoc. Prof. Pham Van Bach Ngoc (VNSC); Dr. Nguyen Duc Hung (USTH) |
| Title | *Development and Implementation of Autonomous Flight Control for a VTOL Quadplane UAV* (overclaims — see W-01) |
| Contribution | INDI rate loop (roll, pitch) inside ArduPilot QuadPlane; one INDI correction shared between lift motors and surfaces by min-norm allocation (Eq. 23); Gazebo/SITL model; 12 paired PID/INDI missions + hover torque test |
| Status (review of 2026-09-29) | Results exist and are honestly caveated. **Major revision needed**: simulated aero model is not the described aircraft, PID baseline possibly untuned and flying a different regime (assist), no actuator dynamics in sim, single seed, several leftover contradictions and citation errors. |
| Review standard | Graduate-thesis standard, submission-ready; every scientific claim cross-referenced to published literature |

## 2. Files and sources

| Path | What it is |
|---|---|
| `thesis.pdf` / `thesis.tex` | Current revision (upload the `.tex` for edits; `.bib` still not provided) |
| `rework/` repo | `sim/indi.param`, `sim/stock_pid.param`, `sim/fly_plan_mission.py`, `sim/disturb.py`, `sim/gazebo_models/VTOL_Quadplane/model.sdf`, `analysis/identify_G.py`, `analysis/indi_margins.py` |
| `FIGURES.md` | Earlier analysis of real log + older SITL log (G = 4.5 limit cycle). Superseded by revision 2 results, but F05 (sync delay) and F06 (vibration) reasoning still useful |
| Project PDF `indi_review_1.pdf` | Steinert et al. 2025, INDI survey **Part I** → thesis ref [10] |
| Project PDF `indi_review2.pdf` | Steinert et al. 2025, INDI survey **Part II** → thesis ref [17]. §7 synchronization, §8 control allocation (incremental CA, path dependency) |
| Project PDF `INDI_evtol.pdf` | Lombaerts et al. 2019, AIAA 2019-0134 → ref [7]. Hover only; WLS prioritised allocation; engine time-constant threshold; η_aero→hover is a *plant-model* blending coefficient |
| Project PDF `uavbook.pdf` | Beard & McLain → ref [6]. §4.4 wind/Dryden, Table 4.1; finite-wing lift slope formula |

## 3. Verified facts (do not re-check unless sources change)

- Quotes "NDI is a model-based approach…", "less model-dependent", "encountered robustness issues…", "one integration level earlier" → **Steinert Part II [17] introduction**, not Part I [10]. Part I says "one derivative level earlier".
- Lombaerts quote "gain scheduling is not needed" and split model-dependent / independent part: accurate.
- Ref [8] correct metadata: **Li Zhou, Jingtao Yang, Tilman Strampe, Uwe Klingauf**, IJRNC 33(17), 2023, **doi:10.1002/rnc.6503** (thesis has wrong names and rnc.6863). Abstract: laws for fixed-wing and hover regimes; does not state the "single law inadequate" motivation.
- Ref [18] Smeur et al., JGCD 39(3) → year **2016**.
- Wingcopter 198: manufacturer page lists 94 km max range, 4.7 kg payload — thesis correct.
- Steinert I references Buffington 1999 AFRL-VA-WP-TR-1999-3057 — thesis ref [12] matches.

## 4. Key numbers (from thesis revision 2)

| Quantity | Value | Note |
|---|---|---|
| Real aircraft loop / gyro filter | 300 Hz / 42 Hz | from real log |
| SITL loop / gyro filter | 400 Hz set, 333 Hz achieved / 20 Hz | Table 17, Table 3 — **not** matched to hardware |
| G lift motors roll / pitch | 56.7 / 44.8 rad/s² per unit | rough calc 56.9 / 44.8 |
| G surfaces roll / pitch @15 m/s | 16.7 / 12.4 | identified at 18.4–20.8 m/s only |
| Motor lag / delay | 0 / 4 ms | simulator commands rotor speed directly |
| INDI K roll/pitch (motors / surfaces) | 13.5 / 10.5 s⁻¹ | ≥6 dB, ≥45° |
| H filter | 2 poles @ 12.7 Hz | |
| Gazebo wing panel | S 0.183 m², CLα 3.70/rad, α0 0.130 rad, Cmα 0, αstall 19.4°, kδ −5/rad | OpenVSP: S 0.204, CLα 5.98 |
| Table 2 wing area / span | 0.220 m² / 1.415 m | AR ≈ 9.1 → CLα ≈ 5.0/rad (Beard & McLain formula) |
| CAD inertia (SolidWorks axes, Y up, Z longitudinal) | roll = SW Izz 0.163, pitch = SW Ixx 0.255, yaw = SW Iyy 0.414 | §5.6 authority numbers imply roll I ≈ 0.14 — unresolved |
| Pulse size | 1.44 / 3.24 N·m | = ½ρu²·0.204·1.296/4, not Table 2 geometry |
| Transition-pitch spread threshold | 0.33° (hover/transition), 0.6° (wing-borne) | borrowed from an earlier campaign |

## 5. Session rules (hard)

1. Never invent data, results, numbers or citations. Missing result → `% TODO: pending`.
2. SITL is not flight. Only PID has flown on hardware (one manual, mostly hover log).
3. Cite precisely (section/equation/page). Paraphrase; verify every direct quote against the PDF.
4. References recalled from memory → mark `% VERIFY-CITE` until checked.
5. Edit surgically; keep the author's voice; show diffs.
6. Refer to TODO IDs in comments and commits. Never reuse an ID.
7. Reply in English.

## 6. Comment tags in `thesis.tex`

```
% REVIEW[P0..P3] <ID>: <issue>
% TODO <ID>: <action>
% CUT <ID>: <reason>
% VERIFY-CITE <key>: <what to check>
```
Find open items: `grep -n "% \(REVIEW\|TODO\|CUT\|VERIFY-CITE\)" thesis.tex`

Priorities: **P0** conclusion-changing · **P1** required for submission · **P2** quality · **P3** polish.

## 7. Conventions

| Topic | Convention |
|---|---|
| Frames | NED / FRD in thesis and ArduPilot; Gazebo ENU world — convert at bridge only |
| Effectiveness units | rad/s² per unit normalised command (full mixer input for motors, full surface travel for surfaces) |
| Inertia | Always report in body FRD axes (Ixx roll, Iyy pitch, Izz yaw), with source (CAD vs SDF) |
| Wind | Must enter aerodynamics through air-relative velocity (panels read world wind); body-force wind plugin disabled |
| Comparisons | Paired, same mission/seed/pulses; claim only differences larger than the measured seed spread of *this* campaign |

## 8. Open questions for the author

1. Were PID gains tuned for this model? Which parameters differ from ArduPilot defaults?
2. Which assist trigger (speed / angle / altitude) keeps PID's lift motors on 72–83 % of wing-borne time?
3. Which aircraft flew the real log — the twin-boom (Fig. 3) or the airframe in Fig. 13?
4. What generates the 0.3 m/s turbulence if WindEffects is disabled?
5. What inertia values are in `model.sdf`? Which I is used in the "−Δu G I/M" plot of Fig. 7?
6. What is the OpenVSP moment reference point for Fig. 5?
7. Is there bench thrust / motor step-response data from the stand mentioned in §4.1?
8. Source for "ArduPilot treats surfaces as ineffective below 7 m/s" and "vertical landing needs 54 m of approach"?
9. USTH formatting rules and submission deadline (title page date 28 Aug 2026 has passed).

## 9. Decisions log

| Date | Decision | Reason |
|---|---|---|
| 2026-09-21 | First review, TODO v1 | Draft 1 |
| 2026-09-29 | Review of revision 2; TODO v2 replaces v1 | Most v1 items fixed; new issues found |
| | *(add here)* | |

## 10. Session end checklist

- [ ] Update TODO statuses with date
- [ ] Add new findings with new IDs
- [ ] Record decisions in §9
- [ ] Update §1 status and §4 numbers if results changed
