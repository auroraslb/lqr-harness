# Friction log: MuJoCo LQR humanoid tutorial (manual run)

Date: 2026-10-06
Tutorial: MuJoCo `python/LQR.ipynb` (balancing the humanoid on its left leg with LQR)
Model: `mujoco-src/model/humanoid/humanoid.xml`

These are my notes from running the tutorial by hand, and the experiments I ran afterwards to understand where it breaks. They are what the harness design is based on; `DESIGN.md` describes what I ended up building.

---

## 1. Environment setup

| Step | What happened | Fix | Note for the harness/README |
|---|---|---|---|
| Python | My system Python was 3.9.6 (Apple's, can't be upgraded) | Installed Python 3.13 with Homebrew, put it first on PATH, venv in `~/lqr-harness/.venv` | Require Python >= 3.10 in the README |
| Viewer | `python -m mujoco.viewer --mjcf=<relative path>` failed with "error opening file" | Used an absolute path | Resolve paths to absolute in tooling |
| Viewer exit | Ctrl-C gave a traceback and a GLFW error | Closed it with Cmd+Q instead | Cosmetic |
| Notebook | Written for Colab: GPU checks, `MUJOCO_GL=egl`, `!pip install`, a clone cell | Skipped those cells and loaded the XML from a local absolute path | The tutorial isn't a portable script |
| Notebook | `NameError: mujoco`, because the imports lived in the skipped Colab setup cell | Added an imports cell at the top | Environment code and process code are mixed together |

---

## 2. Model inventory (checked in code)

- nq = 28, nv = 27, nu = 21, nkey = 4
- Joints: `root` (free joint) plus 21 hinge joints, with one actuator per hinge joint (same names and order). The root has no actuator
- Keyframes: `squat`, `stand_on_left_leg`, `prone`, `supine` (I select them by name, not index)
- The robot is underactuated: 27 degrees of freedom, 21 actuators. The 6 floating-base degrees of freedom can only be held through contact forces at the stance foot
- LQR state size is 2 x nv = 54, control size 21
- The naming isn't fully consistent (`shoulder1/2` vs axis letters, no side suffix on the abdomen), so the harness needs an explicit joint-to-group mapping
- The tutorial's balance group leaves out the `z` joints (`abdomen_z`, `hip_z_left`); the harness does the same

---

## 3. Manual run, step by step

| Step | What I ran or edited | Hard-coded choices | How I knew it worked | What broke or surprised me |
|---|---|---|---|---|
| Viewer | Loaded `stand_on_left_leg` and let it run | Keyframe | Visually | Falls immediately without a controller |
| Load model | `MjModel.from_xml_string(xml)` | Model file | A rendered frame | Needed the import fix above |
| Height offset | `mj_inverse` sweep over the root height, picking the offset with the smallest vertical force | Sweep range +/-1 mm, number of points, keyframe | Looked at the sweep plot | It only zeroes the vertical force, and the range is picked by hand |
| Forces to controls | `ctrl0` via the pseudo-inverse of the actuator moment | Assumes force is linear in ctrl (plain motors) | Printed the setpoint | Would break for position servos or other actuator types |
| Q / R | Centre-of-mass-over-foot balance cost plus joint costs by group | `BALANCE_COST=1000`, `BALANCE_JOINT_COST=3`, `OTHER_JOINT_COST=0.3`, which joints count as balance joints, `R = I` | Nothing explicit | Tuning is manual, and nothing checks the controller before simulating |
| Linearize + LQR | `mjd_transitionFD`, `solve_discrete_are`, K | Finite-difference epsilon, centred differences | Nothing explicit | Solver success is just trusted |
| Simulate | Rollout with `ctrl = ctrl0 - K dx` plus smoothed noise | `DURATION=12`, `CTRL_RATE=0.8`, `BALANCE_STD=0.01`, `OTHER_STD=0.08`, seed 1 | Watched the video | Success is judged by eye |
| Post-process | Rendered a video | Camera, frame rate | Visually | Output only |

### The manual steps I found

1. Finding the height offset is a manual search over a hand-picked range.
2. Mapping forces to actuator commands is manual, and only this simple for plain motors.
3. LQR tuning (cost weights, joint groups, R) is manual.

---

## 4. Variant test: 3 kg payload on the right forearm

I added 3 kg to `lower_arm_right` (`model.body_mass`, then `mj_setConst`) before the setpoint cell, so the equilibrium gets recomputed. Total mass went up by exactly 3 kg (checked).

### The equilibrium looked valid

- Height sweep: zero crossing at -0.54 mm, well inside the +/-1 mm range
- The plateau on the right of the plot (~430 N) matches the total weight with the foot off the ground (~44 kg), so the payload is included
- All `ctrl0` values within +/-1

### Setpoint comparison

| Actuator | 0 kg | 3 kg |
|---|---|---|
| `ankle_y_left` (stance ankle) | 0.667 | 0.726 |
| `shoulder1_right` (loaded arm) | -0.288 | -0.754 |
| Largest overall | 0.667 | 0.754 |

The right leg and left arm values are identical in both, which is what I'd expect.

### Simulation result (seed 1)

| | 0 kg | 3 kg |
|---|---|---|
| Fell | never | step 287 (1.43 s) |
| Saturation rate before the fall | 0% | 58% |
| First saturation | never | step 120 (0.6 s) |
| Top saturating | none | `ankle_y_left` (all saturated steps), `ankle_x_left`, `elbow_right` |
| Max centre of mass to foot distance | 5.3 cm | 31.8 cm |

### Five tuning attempts, all failed

| Experiment | Change | Fell at | Top saturating | Max CoM-foot |
|---|---|---|---|---|
| baseline | none | 1.43 s | ankle_y_left | 31.8 cm |
| arm | right arm joint cost 0.3 -> 3 | 1.47 s | ankle_y_left | 22.9 cm |
| r_uniform | R x3 | 1.36 s | ankle_y_left | 29.1 cm |
| r_ankle | stance ankle R = 5 | 1.39 s | abdomen_z, elbow_right | 54.9 cm |
| arm+r_ankle | both | 1.36 s | ankle_y_left | 45.5 cm |

- The fall time barely moved across very different controllers, which suggests the cause is upstream of the controller
- `r_ankle` did move effort away from the ankle (to the torso and elbow), but balance got worse
- The spectral radius was ~0.99999999997 in every run, passing and failing alike (unpenalized directions such as yaw are never corrected), so the stability check alone doesn't predict falls here

### Root cause: the starting pose isn't balanced

I ran `mj_inverse` at `qpos0` and looked at all 6 root residuals, not only the vertical force.

| | 0 kg | 3 kg |
|---|---|---|
| Vertical force | -0.19 N | -0.01 N |
| Root torque (x, y, z) | -3.45, 0.22, -0.82 | -9.9, -7.61, -5.74 |
| Root torque norm | 3.6 Nm | 13.7 Nm |
| Initial CoM-foot distance | 4.0 cm | 6.4 cm |

- The height sweep only fixes the vertical force. The payload shifts the centre of mass away from the foot, and the pose then needs a torque on the floating base that no motor can supply
- So the robot starts tipping from the first step, whatever the controller does
- The failure shows up in stage 3 (simulation) but is caused in stage 1 (equilibrium), so tuning in stage 2 can't fix it
- Even the tutorial's own pose isn't perfectly balanced (3.6 Nm); the controller just tolerates a small imbalance

---

## 5. Experiments after the manual run

### Payload sweep (seed 1, tutorial weights)

| Payload | Root torque norm | Fell? | Fell at | First sat | Max CoM-foot |
|---|---|---|---|---|---|
| 0 kg | 3.55 | no | | never | 5.3 cm |
| 0.01 kg | 3.58 | no (missed, see below) | | 10.5 s | 37 cm |
| 0.05 kg | 3.72 | yes | 10.0 s | 8.7 s | 47 cm |
| 0.1 kg | 3.81 | yes | 9.2 s | 6.8 s | 47 cm |
| 3 kg | 13.7 | yes | 1.4 s | 0.6 s | 32 cm |

- A 10 g payload already breaks it, with almost no change in root torque. The torque explains 3 kg, but not this
- So there seem to be two failure modes:
  - Gross imbalance (3 kg): tips from the start and falls in ~1.4 s, visible in the stage 1 root torque
  - Late failure (tiny payloads): stands for 7-10 s, then loses balance, which suggests the baseline is only marginally robust
- A bug in the fall detector: at 0.01 kg the centre of mass drifted 37 cm, but `fell` was false because the torso hadn't dropped below 80% of its height by 12 s. Height alone misses falls, so I added a centre-of-mass distance criterion (0.15 m)

### Test A: is the tutorial baseline robust? (0 kg, tutorial weights, 5 noise seeds)

| Seed | Fell? | Fell at | First sat | Sat rate | Max CoM-foot |
|---|---|---|---|---|---|
| 1 | no | | never | 0% | 5.3 cm |
| 2 | yes | 8.5 s | 5.0 s | 13.7% | 14.9 cm |
| 3 | no | | never | 0% | 5.1 cm |
| 4 | no (close) | | 11.1 s | 1.4% | 5.2 cm |
| 5 | yes | 7.4 s | 6.6 s | 10.7% | 14.9 cm |

- The unmodified tutorial fails on 2 of 5 seeds, and seed 4 starts saturating near the end
- So seed 1 (the tutorial's default) passing was partly luck, and a single-seed pass doesn't mean much
- The failing seeds show the same pattern as the small-payload runs: the stance ankle saturates late, then balance is lost

### Test B: noise-driven or drift-driven?

| Run | Noise | Fell? | Fell at | Max CoM-foot |
|---|---|---|---|---|
| 0.1 kg | off | no | | 4.7 cm |
| 3 kg | off | yes | 1.08 s | 15.0 cm |

- 0.1 kg stands perfectly without noise, so the late failures are about robustness to noise, not a slow drift
- 3 kg falls even faster without noise, so the gross imbalance is deterministic and has nothing to do with noise

### Two separate failure modes

| | Gross imbalance | Robustness margin |
|---|---|---|
| Example | 3 kg payload | tutorial baseline on seeds 2 and 5; tiny payloads |
| Cause | Pose not balanced (root torque 13.7 Nm) | Controller only marginally robust to the noise |
| Depends on noise? | No (falls with noise off) | Yes (stands with noise off) |
| Detected in | Stage 1, without simulating | Stage 3, only across several seeds |
| Fixable by LQR tuning? | No (5 retunes failed) | Possibly (tested below) |
| Fix | Re-pose the free joints, or escalate | Retune Q/R for robustness |

### Robustness sweep: can tuning fix the baseline? (0 kg, seeds 1-5)

| Setting | Pass | Failed seeds (time) | Worst CoM-foot | Mean sat | max abs K |
|---|---|---|---|---|---|
| baseline | 3/5 | 2 (8.5 s), 5 (7.4 s) | 14.9 cm | 5.2% | 32.1 |
| arm (arm cost 3) | 3/5 | 1 (11.7 s), 2 (10.1 s) | 15.0 cm | 4.0% | 32.2 |
| R x0.5 | **5/5** | none | 4.7 cm | 0.0% | 38.4 |
| R x2 | 0/5 | all (3.4-6.5 s) | 15.0 cm | 19.7% | 27.9 |
| R x3 | 0/5 | all (3.3-4.2 s) | 15.0 cm | 21.8% | 26.2 |
| ankle R 5 | 0/5 | all (5.6-7.1 s) | 14.9 cm | 12.9% | 27.1 |
| arm + ankle R | 1/5 | 2, 3, 4, 5 | 15.0 cm | 8.9% | 27.2 |
| balance cost x2 | 3/5 | 1 (11.7 s), 2 (10.2 s) | 14.9 cm | 4.5% | 36.9 |
| balance cost x0.5 | 2/5 | 1, 2, 5 | 14.8 cm | 6.5% | 28.8 |
| joint costs x2 | **5/5** | none | 4.7 cm | 0.0% | 33.3 |

- The baseline row reproduces the manual runs exactly
- Two settings fix it: R x0.5 and joint costs x2. Both pass 5/5 with no saturation and the centre of mass within 4.7 cm
- Joint costs x2 does it with lower gains (33.3 vs 38.4), so I'd call it the less aggressive fix
- Raising R (softening the controller) made it much worse (0/5). The saturation turned out to be a symptom: a sluggish controller lets the robot drift, and then the motors saturate trying to recover. The fix was more feedback, not less
- Penalizing the ankle also made it worse (0/5). The ankle is the right actuator for balance, and pushing effort elsewhere hurts
- For the harness, this means a plausible-sounding heuristic can point the wrong way, so any fix has to be checked against the gates rather than trusted

### Held-out validation (0 kg, 20 fresh seeds 6-25)

| Setting | Pass | Worst CoM-foot | Mean sat | max abs K |
|---|---|---|---|---|
| baseline (tutorial) | **6/20** | 15.0 cm | 11.4% | 32.1 |
| R x0.5 | 20/20 | 5.3 cm | 0.1% | 38.4 |
| joint costs x2 | 20/20 | 5.2 cm | 0.1% | 33.3 |
| joint costs x2 + R x0.5 | 20/20 | 4.8 cm | 0.0% | 40.1 |

- The tutorial's settings pass only 6 of 20 fresh seeds; its default seed passing was luck
- Both fixes hold up on seeds they weren't chosen on (20/20)
- I'd pick joint costs x2: same robustness, lowest gains
- Passing runs stay within ~5 cm of the foot and ~0% saturation, which is where the stage 3 thresholds come from

### Stage 1 threshold: how much root torque can the controller tolerate? (tuned controller, 20 seeds)

| Payload | Root torque | Noise off | Pass (20 seeds) |
|---|---|---|---|
| 0 kg | 3.55 Nm | stands | 20/20 |
| 0.1 kg | 3.81 Nm | stands | 20/20 |
| 0.2 kg | 4.08 Nm | stands | 17/20 |
| 0.3 kg | 4.37 Nm | stands | 11/20 |
| 0.4 kg | 4.68 Nm | stands | 1/20 |
| 0.5 kg | 4.99 Nm | stands | 0/20 |
| 1 kg | 6.59 Nm | falls (2.6 s) | 0/20 |
| 2 kg | 10.12 Nm | falls (1.6 s) | 0/20 |
| 3 kg | 13.75 Nm | falls (1.2 s) | 0/20 |

- The margin is tiny: robustness collapses between 3.8 and 4.7 Nm, and above ~6 Nm it falls even without noise
- Root torque below ~3.9 Nm predicts the stage 3 outcome for the tutorial's equilibrium. But it mixes up two things: an equilibrium that was set up wrongly (fixable with an ankle trim) and a pose that is physically impossible. The static feasibility check below separates the two
- The tutorial pose itself sits at 3.55 Nm, close to the edge

### Can the payload be held without changing the posture? A static feasibility check

To stand still, the point where the foot presses on the ground has to sit directly under the centre of mass. That gives a check that needs no controller and no simulation, and doesn't depend on the contact model:
1. the centre of mass, projected onto the ground, must lie inside the stance foot (the polygon spanned by its contact points)
2. with the full weight acting at that point, every motor torque must be within its limit

Original tutorial posture:

| Payload | CoM margin inside foot | Static motor need (max) | Limiting motor |
|---|---|---|---|
| 0 kg | 11.9 mm | 68% | hip_x_left |
| 1 kg | 6.8 mm | 83% | ankle_y_left |
| 2 kg | 2.0 mm | 100% | ankle_y_left |
| 3 kg | **-2.6 mm (outside)** | **116%** | ankle_y_left |

- 0-1 kg: physically holdable in the original posture. The tutorial's root torque at 1 kg (6.6 Nm) comes from its equilibrium step (height only), not from the physics
- 2 kg: at the motor limit, with no headroom for corrections
- 3 kg: impossible in this posture. The centre of mass is outside the foot, and the stance ankle would need 116%. No controller tuning can fix this, and the check says so before any tuning or simulation

### Fix 1: ankle trim (posture unchanged)

Adjust the root height plus the stance ankle (about 1 degree of roll) so the foot pressure sits under the centre of mass. This is the tutorial's 1D height sweep, extended to 3 contact variables.

| Payload | Ankle change | Root torque after | Pass (20 seeds) |
|---|---|---|---|
| 0 kg | 0.6 deg | 0 | 20/20 |
| 1 kg | 1.1 deg | 0 | 20/20 |
| 2 kg | 1.4 deg | 0.7 Nm | 0/20 (motor at 100%) |
| 3 kg | 1.5 deg | 2.7 Nm | 0/20 (CoM outside foot) |

### Fix 2: posture change (torso and arms may move)

The same solve, but also allowing the torso and arm joints to move, with two extra targets: every motor below 80% of its range, and the centre of mass at least 6 mm inside the foot.

| Payload | Main change | Static need | CoM margin | Pass (20 seeds) |
|---|---|---|---|---|
| 1 kg | ~1 deg | 80% | 6.0 mm | 20/20 |
| 2 kg | right shoulder 8 deg | 80% | 6.0 mm | 20/20 |
| 3 kg | right shoulder 20 deg | 80% | 7.6 mm | 20/20 |
| 4 kg | right shoulder 30 deg | 80% | 12.2 mm | 20/20 |
| 6 kg | right shoulder 42 deg | 80% | 19.0 mm | 20/20 |
| 10 kg | shoulder 47, elbow 49 deg, left arm counterbalancing | 80% | 13.2 mm | 20/20 |
| 12 kg | arms near their limits | 80% | 6.0 mm | 14/20 |

- So 3 kg isn't physically infeasible; it's infeasible in the tutorial posture. Bringing the loaded arm in ~20 degrees towards the body brings the centre of mass back over the foot and the ankle back to 80%
- "Infeasible" therefore always means "infeasible within the changes the harness is allowed to make". Whether moving the arm 20 or 47 degrees is acceptable depends on the task (the payload may have to be held in a specific place), which is a judgement call rather than a calculation
- Two things mattered here: without the centre-of-mass margin target, a 2 kg solution balanced on the edge of the foot and fell; and one posture made the Riccati solve fail (an unpenalized yaw direction), which a tiny penalty on every state fixed

### Summary of the cases

| Case | Stage 1 result | Fix | Outcome |
|---|---|---|---|
| Tutorial, 0 kg | feasible (68%, 12 mm margin) | Stage 3 fails 6/20, so retune (joint costs x2) | 20/20 |
| 1 kg | feasible in the original posture, but the tutorial's equilibrium is wrong (6.6 Nm) | Ankle trim (1 deg) | 20/20 |
| 3 kg | infeasible in the original posture (CoM outside the foot, ankle 116%) | Posture change: loaded arm in 20 deg, applied and raised as a warning | 20/20 |
| 12 kg | infeasible in the original posture | Even large posture changes leave little margin | 14/20: report the limit |

The code for all of this is in `harness/` (`stages.py`, `stance.py`, `tuning.py`). It reproduces the manual-run results exactly (seeds 2 and 5 fall at 8.49 s and 7.38 s).

---

### Tuning from scratch: a tolerance-based search

The tutorial's weights translate directly into Bryson's rule (weight = 1 / acceptable deviation squared):

| Weight | Tutorial value | As a tolerance | Tuned fix (joints x2) |
|---|---|---|---|
| balance_cost | 1000 | CoM may wander 3.2 cm | 3.2 cm |
| balance_joint_cost | 3 | balance joints 33 deg | 23 deg |
| other_joint_cost | 0.3 | other joints 105 deg | 74 deg |
| R | identity | motor command up to 1.0 | 1.0 |

So tuning can be done in physical units someone can reason about, instead of opaque weights.

I searched a grid of 81 tolerance combinations (CoM 2/3/5 cm, balance joints 15/30/60 deg, other joints 50/100/200 deg, u_max 0.7/1.0/1.4), screened each one (Riccati solves, stands without noise, 5 seeds), and confirmed the best candidates on 20 held-out seeds. About 40 s per case.

| Case | Pass 5/5 screen | Best on 20 held-out seeds |
|---|---|---|
| 0 kg, tutorial equilibrium | 41/81 | 20/20 (balance 15 deg, other 50 deg, u_max 0.7) |
| 0 kg, ankle trim | 45/81 | 20/20 |
| 3 kg, after the posture fix | 39/81 | 20/20 (balance 15 deg, other 50 deg, u_max 0.7) |

- The 5-seed screen isn't reliable on its own: some configs that scored 5/5 only scored 7-8/20 on fresh seeds, so selection needs a confirmation step on held-out seeds
- Picking the lowest-gain passing config without that confirmation would have picked a bad one
- The same trends hold in all three cases: tighter joint tolerances and more control authority help, and the centre-of-mass tolerance barely matters. So tuning knowledge seems to transfer across variants
- After the posture fix, the 3 kg case is fully tunable. The order matters: feasibility first, then tuning

### From the ground up: no keyframe, just the model

Here I ignored all keyframes, started from the model's default pose (standing straight on both feet) and tried to find a one-leg stance automatically.

I split it into two phases, so the search doesn't get stuck on contacts:
1. Geometry only (smooth): centre of mass over the middle of the stance sole, stance foot flat at floor height, free foot lifted 10 cm, staying close to the default pose
2. With contacts: put the pose on the ground, then run the same balanced-pose solve as the payload fix (root residuals ~0, centre of mass >= 6 mm inside the foot, motors <= 80%)

(A single-phase search from the default pose got stuck immediately, since contact penalties give no useful gradient.)

| Case | Search time | Stage 1 after search | Tuning | Pass (20 held-out seeds) |
|---|---|---|---|---|
| 0 kg | 0.8 s | torque 0, CoM 19.9 mm inside, motors 78% | 81-config search | 20/20 |
| 3 kg | <1 s | torque 0, CoM 6.9 mm inside, motors 80% | 0 kg tolerances reused: 15/20, so re-tuned | 20/20 |
| 6 kg | <1 s | torque 1.4 Nm, motors 92%: gate 1 fails | not run | |

- The pose it finds is different from the tutorial's keyframe: both legs straight, the body leaning over the left foot and the right leg swinging out to the side. Physically valid, but not what a person would design. Preferences like "bend the free knee" would belong in the spec
- Its centre-of-mass margin (19.9 mm) is actually better than the tutorial keyframe's (11.9 mm)
- Tuning results transfer partly between variants (15/20), but not fully, so I'd re-tune per variant
- At 6 kg the search lands in a local optimum, and gate 1 catches it. The keyframe-based posture fix did solve 6 kg earlier, so a better starting point exists. Choosing a different start (another keyframe, a bent-knee guess, the previous variant's pose) is a judgement call I'd leave to the agent
- Riccati failed for 11 of the 81 tolerance combinations on this pose (ill-conditioned); the search records and skips them

### What a new model would need

| Step | Tutorial humanoid | New model | Owner |
|---|---|---|---|
| Find the feet | named `foot_left` | bodies touching the floor when the default pose is lowered | Code (the LLM checks the names make sense) |
| Balance joint group | name filter | joints on the chain from the stance foot to the root, from the model tree | Code |
| Actuator mapping | pseudo-inverse (motors) | measured effect plus a bounded solve; position servos have a bias term | Code |
| Starting pose | keyframe | default pose, two-phase search, multi-start if it fails | Code, with the LLM picking starts |
| Tolerances | tutorial weights | tolerance search | Code, with the LLM setting ranges |
| Test conditions | tutorial noise | from the request | Human |

---

## 6. What this meant for the design

These are the conclusions I drew before building the harness. `DESIGN.md` has what I ended up with.

### Who owns each manual step

| Manual step | Owner | Why |
|---|---|---|
| Height offset | Deterministic code plus a gate | A 1D search; gate on "zero crossing inside the range" and widen it if not |
| Forces to actuator commands | Deterministic code | Can be generalized (bounded least squares over the measured actuator effect), which also detects infeasibility |
| Pose balance (new) | Code (optimizing the free joints to remove the root residual), chosen by the agent | Came out of the payload test; not in the original tutorial |
| LQR tuning | Agent, with the search itself in code | Needs judgement when the design has changed |
| Which stage to fix | Agent | Five failed retunes showed that fixing the stage where the symptom appears can be wrong |

### Gates I considered

- Stage 1 (equilibrium):
  - zero crossing inside the sweep range (not at an edge)
  - plateau force roughly equal to total mass x g (checks the variant was applied)
  - root torque below a threshold (later replaced by the static feasibility check)
  - `ctrl0` within actuator limits
- Stage 2 (controller):
  - Riccati solves
  - spectral radius below 1 + a tolerance (not strictly below 1, because of unpenalized marginal modes; not predictive on its own)
- Stage 3 (simulation):
  - a fall is torso height below a threshold or centre-of-mass distance above one
  - saturation rate, first saturation time, per-actuator saturation counts
  - max centre-of-mass to foot distance (baseline 5.3 cm)
  - several seeds, with the gate on the pass rate

### What the agent needs to see

- Per-actuator saturation counts and the first-saturation time (without them, it can only guess)
- Root residual forces and torques (without them, stage 3 failures get blamed on stage 2)

---

## 7. Takeaways

1. The tutorial judges success by watching a video; I replaced that with numbers.
2. The tutorial's equilibrium step only fixes the vertical force, and a design change exposed that.
3. Five controller retunes failed the same way, and the residual check showed the problem was upstream. The harness has to trace failures back to the right stage.
4. The tutorial baseline is only marginally robust: on 20 fresh seeds it passes 6. Its default seed happens to pass, which is an argument for judging on many seeds.
5. Turning the noise off separates two failure modes: a deterministic imbalance (stage 1, not tunable) and a robustness margin (stage 3, tunable). Each needs a different gate and a different fix.
6. The fix for robustness was more feedback, not a softer controller, which is the opposite of what saturation seemed to suggest. Heuristics need checking against the gates.
7. A static feasibility check (centre of mass inside the foot, static motor torques within limits) tells you before any tuning or simulation whether a controller can possibly work. In the tutorial posture, 3 kg fails it: 2.6 mm outside the foot, ankle at 116%.
8. "Infeasible" is relative to what the harness may change. 1 kg needs a 1 degree ankle trim, 3 kg needs the loaded arm brought in 20 degrees, 10 kg needs large arm changes. The physics is computable; whether a posture change is acceptable is a judgement.
9. The tutorial's weights are physical tolerances in disguise (CoM 3.2 cm, joints 33 and 105 deg). Tuning in tolerance units gives a search space both people and an LLM can reason about.
10. A quick 5-seed screen picked configs that scored 7/20 on fresh seeds. Every tuning result needs held-out confirmation, like a train/test split.
11. With no keyframe at all, a two-phase search found a one-leg stance in under a second, and tuning brought it to 20/20. So the harness doesn't need a hand-made starting pose; it needs a good search and a gate that catches it when the search gets stuck (6 kg).
