# locked_arm_5kg

**Request:** Add a 5 kg tool to the right hand. The right arm must hold the tool exactly where it is now, so the right shoulder and elbow may not move.

**Status:** `no_solution_found`

## Changes made to the design
- **MAJOR** posture: fix_stance(mode=posture) (abdomen_x +0.6 deg, abdomen_y +1.1 deg, abdomen_z -8.0 deg, ankle_x_left +0.4 deg, ankle_y_left +2.0 deg, elbow_left -26.3 deg, shoulder1_left -40.9 deg, shoulder2_left -34.5 deg)
- **INFO** trim: fix_stance(mode=trim) (ankle_x_left -1.6 deg)
- **INFO** posture: fix_stance(mode=posture) (elbow_left -0.3 deg, shoulder1_left -0.4 deg)

## Summary

Request: one-leg (left) stance, 5 kg tool added to lower_arm_right, with shoulder1_right/shoulder2_right/elbow_right locked so the tool stays exactly where it was.

Stage 1 result: infeasible, and cannot be fixed within the locked-joint constraint.
- Starting from the tutorial keyframe + 5kg payload, the pose failed on root torque (21.1 Nm), static motor need (static_max_ctrl 1.496, i.e. 149.6% of range on ankle_y_left) and CoM margin (-11.2 mm, outside the foot).
- fix_stance(trim) (~1.6° ankle adjustment) reduced root torque to 6.7 Nm but static need and CoM margin were still failing.
- fix_stance(posture) (torso yaw/pitch/roll + large compensating swing of the free left arm: shoulder1_left -40.9°, elbow_left -26.3°, shoulder2_left -34.5°, abdomen_z -8°) successfully fixed root torque (0.02 Nm) and CoM margin (+20.2 mm, well inside the foot).
- However, the locked right shoulder joint (shoulder1_right) itself now needs 106.5% of its static command range just to hold the 5 kg tool + forearm at its fixed (keyframe) angle against gravity — above the 80% static-motor gate. A second posture pass (last of 3 stance-fix budget) converged to the same 106.5% figure with only a 0.4° tweak elsewhere, confirming this is the physical limit, not a search artifact.

Physical reason this can't be fixed: the torque needed at shoulder1_right to hold a 5 kg tool at the hand, with the shoulder/elbow angles frozen at their original (unloaded) keyframe values, exceeds what that actuator can produce statically (106.5% > 80% limit). The only ways to reduce this load are to change the shoulder/elbow angles (shortening the effective lever arm or letting the torso take more of the load through the arm geometry) or to reduce torso/other-joint compensation further — both are blocked because the request explicitly requires the right shoulder and elbow to stay exactly where they are. All 3 stance-fix budget calls (trim + 2x posture) are now used with no further lever to pull.

Conclusion: no controller can rescue this — stage 1 (pose feasibility) never reached the gate, so stage 2/3 were not attempted. The limiting actuator is shoulder1_right at ~106.5% of its static command range (gate requires <=80%), driven directly by the locked-arm constraint.

## Steps

| # | Actor | Action | Gate | Rationale |
|---|---|---|---|---|
| 1 | agent | create_case |  | Start the case using the tutorial's one-leg keyframe, locking the right arm joints as the tool must stay fixed in place. |
| 2 | agent | check_feasibility | fail: root_torque_norm_Nm, static_max_ctrl, com_margin_mm | Check if the one-leg keyframe pose with the added 5kg right-hand payload is statically feasible before touching the controller. |
| 3 | agent | fix_stance:trim | fail: root_torque_norm_Nm, static_max_ctrl, com_margin_mm | Trim the stance ankle to bring foot pressure under the CoM, which is now shifted by the 5kg right-hand payload. |
| 4 | agent | fix_stance:posture | fail: static_max_ctrl | Trim alone didn't fix CoM margin or motor saturation; posture mode can move torso and the free (left) arm to recenter CoM over the foot while respecting the locked right arm. |
| 5 | agent | fix_stance:posture | fail: static_max_ctrl | Last stance-fix budget: see if further torso/left-arm repositioning can reduce the static load on the locked shoulder1_right actuator below 80%, without moving the locked right-arm joints. |
| 6 | agent | finish |  | Stage 1 remains infeasible after exhausting all 3 stance-fix attempts; the locked right shoulder joint physically cannot hold the 5kg tool within its 80% static-motor limit, and no controller tuning can fix a pose-level torque deficit. |

Simulations run: 0
