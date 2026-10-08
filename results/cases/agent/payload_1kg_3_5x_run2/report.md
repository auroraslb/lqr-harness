# payload_1kg_3_5x_run2

**Request:** Add a 1 kg tool to the right hand. The robot must survive 3.5 times the usual disturbance.

**Status:** `passed_with_changes`

## Changes made to the design
- **INFO** trim: fix_stance(mode=trim) (ankle_x_left -1.1 deg)
- **INFO** posture: fix_stance(mode=posture) (elbow_left +0.4 deg, shoulder1_left +0.1 deg, shoulder1_right +0.6 deg, shoulder2_left -0.7 deg, shoulder2_right -2.0 deg)

## Summary

Request: add a 1 kg tool to the right hand (payload on lower_arm_right), surviving 3.5x the usual disturbance.

Stage 1 (pose): Starting from the tutorial one-leg keyframe, the 1kg hand payload broke balance (root torque 6.59 Nm, ankle_y_left static need 83%). Fixed with fix_stance: a ~1.1deg ankle_x_left trim zeroed the root torque, then a small posture adjustment (shoulder/elbow shifts of 0.1-2.0deg) brought ankle_y_left static need to exactly 80% and CoM margin to 6.0mm - gate passes.

Stage 2 (controller): Default multipliers (all 1) failed badly under the 3.5x disturbance (0/5 quick-check, mean saturation 38%, dominated by ankle_x_left/ankle_y_left/hip_x_left). Making control effort more expensive (motor=0.5) made it worse (0/5, 58% sat). Making effort cheaper via the motor multiplier (larger gains, more control authority) steadily helped: motor=2 -> 1/5; motor=3 -> 5/5 quick-check with a hint of late saturation (0.7%, 8.8s); motor=4 -> 5/5 with zero saturation. A tune over motor in {3,4,5} (com/balance/other fixed at 1) confirmed motor=4 and motor=5 both reach 20/20 on the larger comparison set with zero mean saturation; motor=3 only reached 15/20. Adopted motor=4 (smaller gain than 5, same robustness: max_gain 139 vs 167).

Stage 3 (judged): Evaluated on 20 fresh unseen seeds plus the no-noise run at the required 3.5x disturbance: 20/20 pass, no falls, mean saturation 0.03% (only ankle_y_left briefly, starting at 2.12s), worst CoM drift 5.1cm. Gate passed with no failures.

Final design: right-hand payload 1kg, trimmed/posture-adjusted one-leg stance, LQR with com=1, balance=1, other=1, motor=4. Meets the 3.5x disturbance acceptance criterion with full margin.</parameter>
</invoke>


## Steps

| # | Actor | Action | Gate | Rationale |
|---|---|---|---|---|
| 1 | agent | create_case |  | Start the case using the tutorial one-leg keyframe, with 1kg payload on the right forearm/hand and acceptance requiring surviving 3.5x the default disturbance. |
| 2 | agent | check_feasibility | fail: root_torque_norm_Nm, static_max_ctrl | Check if the one-leg keyframe pose is still statically feasible with the added 1kg hand payload before any controller work. |
| 3 | agent | fix_stance:trim | fail: static_max_ctrl | Root torque and static motor need (ankle_y_left at 83%) fail; trim the stance ankle to re-center pressure under the shifted CoM from the hand payload. |
| 4 | agent | fix_stance:posture | pass | Static motor need at ankle_y_left is still 83% (>80%) after trim; adjust torso/arm posture to redistribute load and bring motor need under the limit while keeping CoM margin >=6mm. |
| 5 | agent | try_controller | pass | Baseline LQR check with default multipliers to see how the controller handles the payload and the elevated 3.5x disturbance requirement before tuning. |
| 6 | agent | try_controller | pass | Baseline shows heavy saturation (mean 37.6%) on ankle_x_left/ankle_y_left/hip_x_left across all 5 noisy seeds, causing falls. Tightening the motor tolerance (0.5x, making effort more costly) should yield smaller gains and less saturation, testing if that improves survival under the 3.5x disturbance. |
| 7 | agent | try_controller | pass | Motor=0.5 made things worse (faster falls, higher saturation), so the gains were too weak to counter the 3.5x disturbance. Try the opposite direction, motor=2 (cheaper effort, larger gains), to see if more aggressive early correction reduces CoM drift before saturation. |
| 8 | agent | try_controller | pass | Motor=2 improved results (1/5 pass, saturation dropped to 22%, longer fall times). Push further to motor=4 to see if even larger gains further reduce saturation/CoM drift under the 3.5x disturbance. |
| 9 | agent | try_controller | pass | Motor=4 achieved 5/5 with zero saturation; check motor=3 to see where the pass threshold lies and whether a less aggressive gain still fully succeeds, giving margin information before committing. |
| 10 | agent | tune | pass | Both motor=3 and motor=4 passed 5/5 quick checks (motor=4 with zero saturation). Run a narrow tune across motor=3/4/5 (others fixed at 1) screened on 5 seeds and compared on 20 more, to pick the most robust setting before spending an evaluation. |
| 11 | agent | evaluate | pass | Adopted controller (com=1, balance=1, other=1, motor=4) passed 20/20 on the tune's comparison set with zero saturation; run the formal stage 3 judgment on fresh unseen seeds under the 3.5x disturbance requirement. |
| 12 | agent | finish |  | Stage 3 judged the controller on 20 unseen seeds at the required 3.5x disturbance and it passed cleanly (20/20, no falls, negligible saturation), satisfying the acceptance criterion. |

Simulations run: 132
