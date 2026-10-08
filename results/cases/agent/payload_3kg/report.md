# payload_3kg

**Request:** Add a 3 kg tool to the right hand and keep the robot balancing on its left leg.

**Status:** `passed_with_changes`

## Changes made to the design
- **WARNING** posture: fix_stance(mode=posture) (abdomen_x -0.3 deg, abdomen_y -0.9 deg, ankle_x_left +0.3 deg, ankle_y_left +0.8 deg, elbow_left +1.3 deg, elbow_right +4.6 deg, shoulder1_left -4.2 deg, shoulder1_right +20.7 deg, shoulder2_left -5.8 deg, shoulder2_right -2.5 deg)
- **INFO** trim: fix_stance(mode=trim) (ankle_x_left -1.5 deg)

## Summary

Added 3 kg payload to lower_arm_right (right hand/tool). Stage 1 initially failed on all three gates (root torque 13.75 Nm, static ctrl 116% on ankle_y_left, CoM margin -2.6 mm) due to the added mass pulling the CoM outside the left foot. A 1.5 deg ankle trim improved root torque but motor need/CoM margin still failed; a posture fix then redistributed torso and arms (notably shoulder_right +20.7 deg, elbow_right +4.6 deg, torso abdomen_y -0.9 deg, plus smaller left-arm/ankle adjustments) to counterbalance the right-hand payload. This brought the pose to root torque 0, static ctrl exactly at the 80% limit on ankle_y_left, and CoM margin +8.6 mm — all gates pass. Built the default LQR controller (com=balance=other=motor=1); quick check passed 5/5 with zero actuator saturation and worst CoM drift 4.6 cm. Evaluated on 20 unseen seeds (stage 3): 20/20 passed, no falls (including the no-noise run), zero mean saturation, same CoM drift magnitude. No tuning was needed beyond the pose fix. Design is accepted as passed_with_changes due to the posture modification required to carry the payload.

## Steps

| # | Actor | Action | Gate | Rationale |
|---|---|---|---|---|
| 1 | agent | create_case |  | Create the case per the request: 3kg tool on right hand, balance on left leg, starting from the tutorial keyframe since no mention of a new model. |
| 2 | agent | check_feasibility | fail: root_torque_norm_Nm, static_max_ctrl, com_margin_mm | Check stage 1 feasibility of the one-leg stance with the 3kg payload before touching the controller. |
| 3 | agent | fix_stance:trim | fail: root_torque_norm_Nm, static_max_ctrl, com_margin_mm | Root torque residual and CoM margin fail; start with a small ankle trim to re-center foot pressure under CoM before trying posture changes. |
| 4 | agent | fix_stance:posture | pass | Trim alone insufficient; posture mode can shift torso/arms to compensate for the 3kg right-hand payload, reducing ankle motor load and restoring CoM margin. |
| 5 | agent | try_controller | pass | Baseline LQR controller check with all multipliers at default to see how the posture-adjusted pose handles noise. |
| 6 | agent | evaluate | pass | Quick check passed 5/5 with no saturation at default multipliers; evaluate on the 20 unseen seeds to judge against acceptance criteria. |
| 7 | agent | finish |  | Stage 3 evaluation passed 20/20 against default acceptance criteria (disturbance=1, pass_rate=0.95); design is complete and successful. |

Simulations run: 27
