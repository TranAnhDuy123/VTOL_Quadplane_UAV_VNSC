# TODO — Quadplane INDI Thesis (revision 2)

Review date: 2026-09-29. Locations are PDF section / table / figure numbers of revision 2.
Each item: **Purpose** (why) · **Done when** (acceptance test).
Priorities: **P0** conclusion-changing · **P1** required for submission · **P2** quality · **P3** polish.

**Order of work:** A (model) → B (baseline + sim hardware match) → C (re-run campaign) → D (analysis) → E (controller/allocation) → F (text contradictions) → G (citations) → H (format, language).
Reason: A–C change the numbers in Tables 9–15. Rewriting text before re-running means rewriting twice.

---

## A. Simulation model (P0)

- [ ] **A-01 [P0] Make the flown aero model the described aircraft — or describe what was flown.**
  §4, §4.2 say OpenVSP → Gazebo; §5.7 says wing/aileron coefficients copied from another aircraft. Table 19: CLα 3.70 vs 5.98/rad; S 0.183 vs 0.204 vs Table 2 0.220 m²; Cmα 0.
  *Purpose:* wing-borne and transition conclusions currently describe a generic model.
  *Done when:* either (a) panels rebuilt from OpenVSP (S, CLα, α0, Cm0/Cmα, stall) and campaign re-run, or (b) §4.2 states exactly which coefficients are borrowed and from where, and wing-borne/transition claims are restricted accordingly.

- [ ] **A-02 [P0] Fix control-surface derivative kδ = −5/rad.** It exceeds the wing's CLα (3.70) → flap effectiveness > 1, impossible for a plain flap. Derive from flap-chord ratio (flap effectiveness τ < 1) or OpenVSP control-surface sweep.
  *Purpose:* Gs, aileron authority (2.2–3.0 N·m) and all surface-INDI results depend on it.
  *Done when:* kδ = τ·CLα with τ sourced; Gs re-identified.

- [ ] **A-03 [P1] Check lift slope and CL,max.** AR ≈ 9.1 → CLα ≈ 5.0/rad (Beard & McLain finite-wing formula). Panel implies CL,max ≈ 3.70 × (0.339 + 0.130) ≈ 1.7, optimistic for NACA 4415 at Re ≈ 1.6×10⁵.
  *Purpose:* stall speed 12 m/s sets AIRSPEED_MIN, Q_ASSIST_SPEED and the 7 m/s Gs cut-off.
  *Done when:* CLα and CL,max justified with a source; stall speed recomputed; Table 17 speeds updated if needed.

- [ ] **A-04 [P1] Add pitching moment (Cm0 / Cmα) to panels; make rudders move** (§5.7).
  *Purpose:* trim elevator and yaw coordination are unrealistic without them.

- [ ] **A-05 [P0] Add lift-motor dynamics.** Motor lag identified as 0, delay 4 ms (Tables 8, 16) because the plugin commands rotor speed.
  *Purpose:* INDI's main practical sensitivity (Lombaerts 2019: engine time-constant threshold; Steinert II §7: A must match the real actuator). Synchronization model A, K = 13.5 and Table 3 margins are all for an instantaneous motor.
  *Done when:* bench step test gives motor τ; first-order lag in Gazebo motor model; G, A, margins re-identified; torque test re-run.

- [ ] **A-06 [P1] Servo rate limit / bandwidth** matched to real servos (FW_TC = 10 ms is sim-only).

- [ ] **A-07 [P1] Report the inertia actually in `model.sdf`, in body axes.** Table 2 uses SolidWorks axes (roll = SW Izz 0.163, pitch = SW Ixx 0.255, yaw = SW Iyy 0.414). §5.6 "Gs·Ixx = 2.2–3.0 N·m" implies roll I ≈ 0.14.
  *Purpose:* readers will read Ixx = 0.255 as roll; authority numbers must match the model.
  *Done when:* Table 2 has a body-axis row (CAD) and an SDF row; §5.6 numbers recomputed.

- [ ] **A-08 [P2] Reconcile geometry.** dy 0.245 m (Table 2) vs 480/2 = 240 mm (Fig. 15); af/ar 0.290/0.300 vs 293.73/296.25 mm (Fig. 14); state which values are in the SDF.

- [ ] **A-09 [P1] State the OpenVSP moment reference point; compute trim.** Fig. 5 Cm crosses zero near 19°, next to stall (19.4°).
  *Purpose:* if the reference is the CG, the design trims near stall with neutral elevator (CG/tail-incidence problem); if not, the stability sentence in §5.2 is unsupported.
  *Done when:* trim α and elevator at 17 m/s reported.

- [ ] **A-10 [P1] Clarify turbulence source.** §4.7 lists turbulence (0.3 m/s, 0.02 rad, 0.05 m/s) but §5.7.1 says the wind plugin was disabled.
  *Done when:* §4.7 states which system produces it, and that it changes air-relative velocity, not body force.

## B. Baseline fairness and hardware match (P0)

- [ ] **B-01 [P0] Tune the PID baseline in the same SITL model.** QAUTOTUNE (hover), AUTOTUNE (fixed-wing), or design both controllers to the same margin spec. List every non-default PID gain in Appendix C.
  *Purpose:* INDI gains come from an identified plant and margins; "stock firmware" PID may be default gains. The 3× rate-tracking advantage may be a tuning gap.

- [ ] **B-02 [P0] Explain and remove the assist-regime confound.** PID wing-borne with assist 72–83 %, INDI 0–9 %; INDI flies faster; PID loses 10 m height (square 6 m/s).
  *Purpose:* wing-borne attitude, path and height differences currently reflect regime/airspeed, not the control law.
  *Done when:* assist trigger logged (speed / angle / altitude); airspeed per phase reported for both; cause fixed or comparisons restricted to matched-regime segments.

- [ ] **B-03 [P0] Match SITL to the hardware facts, or stop claiming they are used.** §5.1 says 300 Hz loop and 42 Hz gyro filter are "used in the simulation"; Table 17 has 400 Hz (333 achieved) and 20 Hz.
  *Purpose:* H, Fg, margins and noise results should be designed at hardware settings.
  *Done when:* SITL runs at 300 Hz loop and 42 Hz gyro filter (or §5.1 is corrected and the mismatch listed as a limitation).

- [ ] **B-04 [P1] Airspeed sensor dependency.** Surface law scales Gs with measured V²; real aircraft has no pitot (synthetic CTUN.As). 20 % airspeed error → 44 % G error.
  *Done when:* limitation stated; "fit a pitot" added to future work before hardware INDI.

## C. Experiment campaign (re-run after A, B)

- [ ] **C-01 [P0] Multiple seeds.** ≥ 5 seeds per case (SITL is cheap); spread from *this* campaign, not the earlier forced-assist one.
  *Purpose:* §4.9 rule; currently 3 of 6 "every case" transition-pitch claims are at or below the 0.33° threshold (zigzag 6 m/s: 0.21°; square calm: 0.33°; zigzag calm: 0.34°).
  *Done when:* mean ± 95 % CI, paired test (e.g. Wilcoxon), abstract wording follows the statistics.

- [ ] **C-02 [P1] Real gust disturbances.** Current disturbances are body torques (no change in α, β, V). Add discrete 1-cosine gusts (lateral and vertical, several amplitudes/lengths) and Dryden turbulence light/moderate (Beard & McLain §4.4, Table 4.1) through the world wind; include vertical gust during transition.
  *Purpose:* tests gust aerodynamics, which torque pulses cannot; matches the thesis goal of wind-gust rejection.

- [ ] **C-03 [P1] Size torque pulses as a fraction of available authority** (e.g. 25 / 50 % per axis and phase), and state that M = ½ρu²S·b/4 is a scaling convention (uses S = 0.204, b = 1.296, not Table 2).
  *Purpose:* at 9 m/s most pulses saturate an effector → tests saturation handling, not control laws.

- [ ] **C-04 [P1] Fly the "independent INDI laws" case in SITL.**
  *Purpose:* turns Table 3 (GM 9.0 → 3.5 dB) from analysis into evidence; safe in simulation.

- [ ] **C-05 [P1] Investigate hover roll held deviation.** INDI 0.53° vs PID 0.37° at 1.44 N·m (Table 9) contradicts §3.4.2 prediction; not mentioned in caption or text.
  *Done when:* cause identified (motor near minimum? position loop?) and discussed.

- [ ] **C-06 [P1] Resolve the 1.25 effectiveness factor** (§5.5). Two independent estimates agree (56.7 vs 56.9), so check the torque-test normalisation first: I used in −Δu G I/M, torque application point/frame, then mixer thrust compensation.
  *Done when:* factor explained or G corrected; ρ reported.

- [ ] **C-07 [P2] Surface G at transition speeds.** Identified at 18–21 m/s only, extrapolated by V² down to 7 m/s. Add a sweep at ~10 and ~14 m/s (with assist).

- [ ] **C-08 [P2] Re-identify G after A-01/A-02/A-05**, update Tables 8, 16 and Table 3.

## D. Analysis and data pipeline

- [ ] **D-01 [P1] State the identification estimator** (§4.4): with test signal r as instrument, G(jω) = S_rω̇ / S_ru. Add frequency-domain ID reference (Tischler & Remple) `% VERIFY-CITE`.
- [ ] **D-02 [P1] Noise comparison on equal footing.** Apply the identical filter to real and simulated gyro in Table 7; use one band (25–90 Hz or 20–120 Hz) in §5.1 and §5.4; tune injected noise per axis (pitch currently 74 % high).
- [ ] **D-03 [P1] One phase-segmentation rule.** §4.8 (transition state + airspeed + motor output) vs §4.9 (transition messages). Pick one, describe it once.
- [ ] **D-04 [P2] Angle wrapping.** Roll errors reach 164° (Table 14); state wrapping in Eq. 24 or use quaternion error angle.
- [ ] **D-05 [P2] Replace the "Better" column in Table 10** with values only, or add a "cause" column (control law / assist / saturation / navigation).
- [ ] **D-06 [P2] Cheap sim-vs-real checks from the existing log:** hover throttle (~1270 µs ≈ 0.27 real vs ~0.30 sim), hover attitude trims. Report in §5.1.
- [ ] **D-07 [P2] Note in §5.1** that the FBWA segment was motor-assisted (lift motors ~1550 µs in Fig. 4), and that real roll noise is ~6× pitch (possible structural mode).
- [ ] **D-08 [P3] Common-clock plots:** state that time between windows is warped, so durations there cannot be read from Figs. 8–11, 17.

## E. Controller and allocation

- [ ] **E-01 [P1] Position Eq. 23 against prior work.** Incremental control allocation (Steinert II §8), Lombaerts' WLS allocation (they list extending it to surfaces as next step), INCA literature `% VERIFY-CITE`.
  *Purpose:* examiners will ask what is new.
- [ ] **E-02 [P1] Discuss limitations of min-norm allocation:** arbitrary weighting between "full mixer input" and "full surface travel"; no redistribution under saturation (dominant at 9 m/s, Table 15); path dependency / null-space drift (Steinert II §8). Either add weighting/redistribution or list as limitation.
- [ ] **E-03 [P1] Reconcile Eq. 21 with Table 3.** Ideal recursion: factor-2 tolerance, c = 2 "edge of stability before any filter". Table 3 with filters: 9 dB (factor 2.8), independent laws still stable at 3.5 dB. Present Eq. 21 as an illustration, Table 3 as the claim.
- [ ] **E-04 [P2] Backward-difference half-sample delay** (dt/2) — say whether A's identified 4 ms absorbs it.
- [ ] **E-05 [P2] Two loops in different tasks:** state their rates and phase; confirm the shared ω̇f is the same signal and that the 10 ms stale-G rule cannot double count within a cycle.

## F. Text contradictions and structure

| ID | Pri | Location | Problem | Fix |
|---|---|---|---|---|
| F-01 | P1 | §1.3, §3.1.4 | Vertical acceleration trim "(§4.3)" vs §4.8 "adds nothing to the vertical loop" | Delete vertical claims |
| F-02 | P1 | §3.1.4 | "Actuator allocation entirely provided by ArduPilot" vs own allocation Eq. 23 | Rewrite |
| F-03 | P1 | §3.1.4 | Whole subsection is methodology inside the literature review; says INDI replaces (p,q,r) incl. yaw | Move to §4.1; roll and pitch only |
| F-04 | P1 | §1.2, §3.4.3 | "Fixed gains / single set of PID gains" vs separate motor/surface PIDs (§4.5) and SCALING_SPEED airspeed scaling (Table 16) | State ArduPilot already schedules FW gains with airspeed |
| F-05 | P1 | §3.5 | "diagonal, hover-only effectiveness matrix" | Outdated; surfaces included |
| F-06 | P1 | §3.3 | l(x) = J⁻¹(−ω×Jω) omits aerodynamic moment, contradicts next sentence ("entire aerodynamic model") | l(x) = J⁻¹(τ_a(x) − ω×Jω) |
| F-07 | P1 | §4.7 | "Three things must hold", only two listed | Add third or change to two |
| F-08 | P1 | §5.6, §5.7 | Refer to §5.2 for unphysical panels, rudders, no pitching moment — not in §5.2 | Move that content into §5.2 (with Table 19) |
| F-09 | P1 | §4.1, Fig. 2 | "Each boom holds … its own tail" vs one H-tail with two fins | Correct text and caption |
| F-10 | P2 | §4.1 | "Tested on a bench stand, Fig. 3" — no stand shown, no bench data | Add data or correct |
| F-11 | P1 | Fig. 13 | Appears to show a different tail configuration; which airframe flew the real log? | State airframe |
| F-12 | P2 | §4.6 | "Desired path is the one flown" vs runner drops waypoints | Say figures use the modified list |
| F-13 | P1 | Objective 3, Fig. 1, title | "Verified simulation framework" / "Final validated UAV" not achieved | Reword objective, mark completed stages in Fig. 1, narrow title (W-01) |
| F-14 | P1 | Abstract | Requires validation "before any comparison can be trusted", then presents the comparison | Frame as simulation-internal comparison |
| F-15 | P1 | §6 | Repeats last paragraph of §5.6 nearly verbatim; objectives not answered | Rewrite: answer each objective/hypothesis |
| F-16 | P1 | §5.7 | Motor dynamics, loop-rate/filter mismatch, single seed, PID tuning, airspeed sensor missing from limitations | Add |
| F-17 | P2 | §3.1.3 | ω×v called "centripetal" | "rotating-frame (transport theorem) term" |
| F-18 | P2 | §3.1.1 | "(… in section 6)" | → §4.1 |
| F-19 | P2 | §5.7 | "comparison between PID should still be meaningful" | "between PID and INDI"; soften given B-01/B-02 |

- [ ] **W-01 [P1] Title:** narrow to the actual scope, e.g. "…Incremental Nonlinear Dynamic Inversion Rate Control for a VTOL Quadplane UAV: Design and Software-in-the-Loop Evaluation".
- [ ] **W-02 [P1] Add explicit research questions / hypotheses** (e.g. H1 disturbance rejection hover/transition; H2 shared correction preserves margins; H3 sim represents real aircraft) and answer each in §6.
- [ ] **W-03 [P2] Future work:** add motor dynamics identification, pitot, PID tuning, gust models, multi-seed.

## G. Citations

- [ ] **G-01 [P1]** §3.3 and §3.4.3 quotes → cite **[17] Steinert Part II**, not [10]. (Verified in project PDFs.)
- [ ] **G-02 [P1]** Ref [8] → Li Zhou, Jingtao Yang, Tilman Strampe, Uwe Klingauf; IJRNC 33(17), 2023; doi:10.1002/rnc.6503. Verify page range.
- [ ] **G-03 [P1]** §3.2.3: Zhou "specifically because a single control law was not judged adequate" — give page reference or soften.
- [ ] **G-04 [P2]** Ref [18] year → 2016.
- [ ] **G-05 [P1]** §3.2.3: Lombaerts' η_aero→hover blends aerodynamic force models in the simulation plant, not control laws — reword "transition corridor" sentence.
- [ ] **G-06 [P1]** Unsourced claims: van 't Veld delay sensitivity (§3.5, cite via [17]); "surfaces ineffective below 7 m/s" (§4.5, cite code); "vertical landing needs 54 m approach" (§4.6, cite log message/docs).
- [ ] **G-07 [P2]** §1.1 VTOL classification omits tailsitters; cite a survey (Ducard & Allenspach 2021; Saeed et al. 2018) `% VERIFY-CITE`.
- [ ] **G-08 [P2]** Upload `thesis.bib` for a full entry check.

## H. Format and language

- [ ] **H-01 [P1] Front matter:** remove stray "Project" (title page); update date; "Assoc. Prof." with full stop; fix acknowledgements ("quite entirely" → "quit entirely"; "fostering both professional growth" → "…professional and personal growth"; "He has been a source of inspiration and for giving me…").
- [ ] **H-02 [P1] Table numbering:** List of Tables starts at 2 and skips 18 (Appendix D table has no caption). Caption or un-number those tables.
- [ ] **H-03 [P2] Abbreviations:** remove unused (UIUC, RPM); add FLU, PSD, CG, RMS, FBWA, QHOVER, ESC; fix page break.
- [ ] **H-04 [P2] Appendix E** contains no SITL/MAVLink configuration: add launch command, physics step, ports, param files — or rename.
- [ ] **H-05 [P2] Figures:** enlarge fonts in Figs. 8–12, 17; Fig. 5 axis units (deg), replace "CMytot"; Fig. 12 → appendix; Fig. 18 higher resolution; Fig. 6 caption ("is trying to match" → "matches the roll-gyro level but not…").
- [ ] **H-06 [P2] Table 2:** one quantity per row; realistic significant figures (e.g. 0.0425 m², not 0.042534).
- [ ] **H-07 [P2] Consistent "section" (article class) — no "chapter".**
- [ ] **H-08 [P1] Proofread.** Known errors: "Figsure", "dyanmic", "efficency", "comparasion", "does not captures", "this is limits", "This is an result", "This derivation matter", "a low-pass filters", "Their fix according to Steinert et al. [17] call synchronization", "coordinates throttle and elevator with throttle primarily changes", "A further issue is specific to the quadplane is that", "Consider a nonlinear system, we have:", "Consider a very simple one rotational axis", Objective 1 unfinished sentence, abstract sentence "Twelve SITL missions, …" has no verb, "It does provide the main risk of the difference…".

---

## Progress log

| Date | Items closed | Notes |
|---|---|---|
| 2026-09-29 | — | TODO v2 created from review of revision 2 |
