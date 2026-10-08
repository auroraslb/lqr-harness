# payload_1kg_3_5x

**Request:** Add a 1 kg tool to the right hand. The robot must survive 3.5 times the usual disturbance.

**Status:** `no_solution_found`

## Changes made to the design
- **INFO** trim: fix_stance(mode=trim) (ankle_x_left -1.1 deg)
- **INFO** posture: fix_stance(mode=posture) (elbow_left +0.4 deg, shoulder1_left +0.1 deg, shoulder1_right +0.6 deg, shoulder2_left -0.7 deg, shoulder2_right -2.0 deg)

## Summary

Request: add 1 kg tool to right hand (lower_arm_right), one-leg (left) stance, survive 3.5x the tutorial's disturbance at 95% pass rate over 20 seeds.

Stage 1 (pose): Default keyframe failed (root torque 6.59 Nm, static ctrl 0.831 at ankle_y_left). Fixed with fix_stance trim (~1.1 deg ankle_x_left adjustment) which zeroed root torque, then fix_stance posture (small torso/arm shifts, largest 2.0 deg on shoulder2_right) which brought static ankle_y_left need to exactly 0.80 and CoM margin to 6.0 mm. Stage 1 passed cleanly (2 of 3 stance-fix budget used).

Stage 2 (controller): Default multipliers (1,1,1,1) fell on all 5 quick-check seeds with heavy ankle_x_left/ankle_y_left saturation (mean_sat 0.38, max_gain 46.5) — the 3.5x disturbance plus 1 kg lateral tool mass overwhelms the default-tolerance LQR. Iterating multipliers: loosening motor cost (motor=2) and tightening balance tolerance (balance=0.25) got to 4/5 quick-check pass with low saturation; tightening balance further to 0.15 with motor=3 reached 5/5 quick-check pass, zero saturation, max_gain ~93. A tune sweep (balance 0.1-0.35, motor 3-4) confirmed several configs reaching 20/20 on the internal choose-set.

Stage 3 (judge, 2/2 evaluations used): 
- com=1, balance=0.2, other=1, motor=3 (tune's top pick): 18/20 (90%) on judged seeds, failed pass_rate gate.
- com=1, balance=0.15, other=1, motor=3 (best quick-check margins, lower CoM drift): 18/20 (90%) again on a fresh seed block, failing both pass_rate and mean_saturation gates (hip_x_right, shoulder1_left/right saturating from ~2.7s).

Both independently-evaluated controllers plateau at 90% pass rate against 3.5x disturbance — consistently 2 falls per 20 seeds, with saturation concentrated on hip_x_right and the ankle actuators. All controller-search budgets are now exhausted (try_controller 8/8, tune 2/2, evaluate 2/2; 1 stance-fix budget remains but stage 1 already passes so it would not help). 

Conclusion: no_solution_found. The 1 kg right-hand payload combined with a required 3.5x disturbance margin (vs. the tutorial baseline) pushes hip_x_right and the stance ankles into saturation for a persistent ~10% of large-disturbance seeds; the LQR multiplier search (com/balance/other/motor) could not close this last 5-percentage-point gap to the 95% pass-rate requirement within the allotted budget.

## Steps

| # | Actor | Action | Gate | Rationale |
|---|---|---|---|---|
| 1 | agent | create_case |  | Starting a new case per request: 1kg tool on right hand, standing on left leg, must survive 3.5x disturbance. |
| 2 | agent | check_feasibility | fail: root_torque_norm_Nm, static_max_ctrl | Check if the one-leg stance with 1kg payload on right hand is statically feasible before controller work. |
| 3 | agent | fix_stance:trim | fail: static_max_ctrl | Root torque and static motor need fail; trim the stance ankle (~1 deg) to bring CoM/pressure into balance before any posture change. |
| 4 | agent | fix_stance:posture | pass | Static motor need at ankle_y_left (0.83) still exceeds the 80% gate after trim; posture fix can shift torso/arms to reduce that ankle's static load while keeping CoM margin. |
| 5 | agent | try_controller | pass | Baseline LQR with default tolerances to see how it handles the 1kg payload and quick-check noise before tuning for the 3.5x disturbance requirement. |
| 6 | agent | try_controller | pass | Baseline saturates heavily on ankle_x_left/ankle_y_left with very high gain (46.5) and falls on all 5 seeds; tightening motor cost (lower multiplier) should reduce gain size and saturation. |
| 7 | agent | try_controller | pass | Tightening motor cost made things worse (falls faster); try loosening motor cost (motor=2) to allow stronger corrective action against the large 3.5x disturbance. |
| 8 | agent | try_controller | pass | motor=2 improved pass rate to 1/5 with reduced saturation; now tighten balance tolerance (0.5) to better constrain stance-leg/torso motion since ankle_x_left remains the dominant saturating actuator. |
| 9 | agent | try_controller | pass | Tightening balance to 0.5 pushed fall times much later (up to 10.6s) and cut saturation; push further to 0.25 to see if it clears the remaining falls. |
| 10 | agent | try_controller | pass | balance=0.25 reached 4/5 pass with low saturation (0.0195); push balance tighter still (0.15) to try to clear the last fall. |
| 11 | agent | tune | pass | balance=0.25,motor=2 gave 4/5 pass; balance=0.15 was worse (3/5). Narrow search around balance 0.2-0.35 and motor 2-3 to find the best combo before committing budget to full evaluation. |
| 12 | agent | evaluate | fail: pass_rate | Tune found com=1, balance=0.2, other=1, motor=3 passing 20/20 on choose-set with zero saturation; run the stage 3 judge on fresh unseen seeds to confirm it meets the 3.5x disturbance acceptance. |
| 13 | agent | try_controller | pass | Stage 3 eval of balance=0.2/motor=3 gave 18/20 (90%), short of 95%. Tighten balance further to 0.15 with motor=3 (earlier balance=0.15 was tried only with motor=2) to see if it reduces the remaining falls. |
| 14 | agent | tune | pass | balance=0.15,motor=3 gave 5/5 quick-check with zero saturation and much lower CoM drift than balance=0.2; use the last tune budget to confirm robustness and find the best candidate on 20-seed choose set before the final evaluation. |
| 15 | agent | try_controller | pass | Tune re-adopted balance=0.2/motor=3 (the config that scored only 18/20=90% on the real judge), but our direct test of balance=0.15/motor=3 showed better margins (worst_com_m 0.0507 vs 0.1498, mean_sat exactly 0). Re-adopt balance=0.15/motor=3 as the controller for the final evaluation, using the last try_controller budget. |
| 16 | agent | evaluate | fail: pass_rate, mean_saturation | Final judged evaluation with adopted controller com=1,balance=0.15,other=1,motor=3, which showed better margins than the previously-evaluated 90%-passing config; this is the last evaluation budget. |
| 17 | agent | finish |  | Both of the two allowed stage-3 evaluations (budget exhausted) returned 18/20 (90%) pass rate, below the required 95%, with no remaining controller-tuning or evaluation budget to close the gap. |

Simulations run: 288
