# payload_1kg_3_5x_run3

**Request:** Add a 1 kg tool to the right hand. The robot must survive 3.5 times the usual disturbance.

**Status:** `passed_with_changes`

## Changes made to the design
- **INFO** trim: fix_stance(mode=trim) (ankle_x_left -1.1 deg)
- **INFO** posture: fix_stance(mode=posture) (elbow_left +0.4 deg, shoulder1_left +0.1 deg, shoulder1_right +0.6 deg, shoulder2_left -0.7 deg, shoulder2_right -2.0 deg)

## Summary

1 kg tool added to lower_arm_right (right hand). Stage 1: initial keyframe pose failed (root torque 6.59 Nm, static ankle_y_left ctrl 83.1%); fixed with a 1.1° ankle trim (ankle_x_left) to zero root torque, then a posture adjustment (small torso/arm shifts, max 2°, mostly shoulder2_right -2°) to bring static ctrl to exactly 80% and CoM margin to 6.0 mm — gate passed. Stage 2: baseline LQR (all multipliers=1) failed badly under the required 3.5x disturbance (0/5, heavy ankle_x_left/ankle_y_left saturation, falls <3s). Tightening motor cost (motor=0.5) made it worse. Loosening motor cost (cheaper control effort) helped progressively: motor=2 gave 1/5 pass with saturation ankle_y/x_left; motor=4 gave 5/5 pass with zero saturation and CoM drift ~0.049 m (well inside tolerance). A tune sweep (motor=3,4,5; others=1) confirmed motor=4 as the robust choice (20/20 on extended screening vs 15/20 at motor=3), adopted automatically. Stage 3 judge on 20 fresh seeds plus no-noise run: 20/20 survived, no falls, mean saturation 0.03% (only brief ankle_y_left), worst CoM drift 0.051 m. Final design: trimmed/posture-adjusted stance + LQR with com=1, balance=1, other=1, motor=4. Meets the 3.5x disturbance, 95% pass-rate requirement.

## Steps

| # | Actor | Action | Gate | Rationale |
|---|---|---|---|---|
| 1 | agent | create_case |  | Start case for 1kg right-hand payload with 3.5x disturbance requirement, using default keyframe stance on left leg since no locking constraints are stated. |
| 2 | agent | check_feasibility | fail: root_torque_norm_Nm, static_max_ctrl | Check if the keyframe stance pose with 1kg right-hand payload is statically holdable before building a controller. |
| 3 | agent | fix_stance:trim | fail: static_max_ctrl | Root torque and static ctrl fail; trim the stance ankle to rebalance CoM under the foot before trying posture changes. |
| 4 | agent | fix_stance:posture | pass | Static motor need still exceeds 80% at ankle_y_left; adjust torso/arm posture to reduce motor load while keeping CoM margin, following the trim. |
| 5 | agent | try_controller | pass | Baseline LQR controller with all multipliers at default to see how the stance handles noise before tuning for the demanding 3.5x disturbance requirement. |
| 6 | agent | try_controller | pass | Heavy saturation on ankle_x_left/ankle_y_left with default weights suggests control effort is too cheap; tighten motor tolerance (motor=0.5) to penalize large commands and reduce saturation. |
| 7 | agent | try_controller | pass | Tightening motor cost worsened saturation and falls; try the opposite direction (motor=2, cheaper effort) to see if stronger corrective torques better resist the 3.5x disturbance. |
| 8 | agent | try_controller | pass | motor=2 improved pass rate (1/5) and reduced mean saturation; push further to motor=4 to see if even cheaper control effort yields stronger disturbance rejection without excessive saturation. |
| 9 | agent | tune | pass | Confirm motor=4 is a robust choice (not a knife-edge) by comparing nearby motor multipliers on more seeds before committing to evaluation. |
| 10 | agent | evaluate | pass | Controller with motor=4 passed 5/5 quick check and 20/20 in tune screening; run the official stage 3 judge on fresh seeds to confirm it meets the 3.5x disturbance, 95% pass-rate requirement. |
| 11 | agent | finish |  | Stage 3 gate passed cleanly on unseen seeds, satisfying the request's 3.5x disturbance and default 95% pass-rate acceptance criteria. |

Simulations run: 126
