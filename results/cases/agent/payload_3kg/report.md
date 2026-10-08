# payload_3kg

**Request:** Add a 3 kg tool to the right hand and keep the robot balancing on its left leg.

**Status:** `passed_with_changes`

## Changes made to the design
- **WARNING** posture: fix_stance(mode=posture) (abdomen_x -0.3 deg, abdomen_y -0.9 deg, ankle_x_left +0.3 deg, ankle_y_left +0.8 deg, elbow_left +1.3 deg, elbow_right +4.6 deg, shoulder1_left -4.2 deg, shoulder1_right +20.7 deg, shoulder2_left -5.8 deg, shoulder2_right -2.5 deg)
- **INFO** trim: fix_stance(mode=trim) (ankle_x_left -1.5 deg)

## Summary

Added a 3 kg payload to lower_arm_right (right hand) with the robot balancing on the left leg from the tutorial keyframe.

Stage 1: Initial pose failed all three gates (root torque 13.75 Nm, CoM margin -2.6 mm outside foot, ankle_y_left static load 116% of range) due to the asymmetric 3kg load pulling CoM off the support foot. A 1.5° ankle trim reduced root torque to 2.66 Nm but CoM margin and motor need still failed. A posture fix then redistributed load: torso tilted (abdomen_y -0.9°, abdomen_x -0.3°), right arm raised/retracted (shoulder1_right +20.7°, shoulder2_right -2.5°, elbow_right +4.6°) and left arm counter-balanced (shoulder1_left -4.2°, shoulder2_left -5.8°, elbow_left +1.3°), plus a small ankle adjustment. Result: root torque 0 Nm, CoM margin +8.6 mm, static ctrl exactly at the 80% limit on ankle_y_left — all gates passed.

Stage 2: Default LQR multipliers (com=balance=other=motor=1) gave tolerances (CoM 1.94 cm, balance joints 25°, other joints 72.5°) that already handled the payload well: quick check passed 5/5 seeds, no falls, zero actuator saturation, worst CoM drift 4.56 cm, max gain 47.6.

Stage 3: Evaluated on 20 fresh unseen seeds (26-45) plus the no-noise run: 20/20 passed, no falls, zero saturation, worst CoM drift 4.64 cm. Meets the default acceptance (disturbance=1, pass_rate=0.95).

No further tuning was needed; default multipliers were sufficient once the stance posture was corrected for the added hand mass.

## Steps

| # | Actor | Action | Gate | Rationale |
|---|---|---|---|---|
| 1 | agent | create_case |  | Start case with 3kg payload on right hand (default lower_arm_right), balancing on left leg from tutorial keyframe. |
| 2 | agent | check_feasibility | fail: root_torque_norm_Nm, static_max_ctrl, com_margin_mm | Check stage 1 feasibility of the one-leg stance with the 3kg payload before any controller work. |
| 3 | agent | fix_stance:trim | fail: root_torque_norm_Nm, static_max_ctrl, com_margin_mm | Root torque and CoM margin fail; start with the small ankle trim to re-center foot pressure before heavier posture changes. |
| 4 | agent | fix_stance:posture | pass | Trim alone insufficient; the 3kg hand payload needs torso/arm repositioning to bring CoM back over the foot and reduce ankle motor load. |
| 5 | agent | try_controller | pass | Baseline LQR controller with default multipliers to see how the payload-adjusted stance behaves under noise. |
| 6 | agent | evaluate | pass | Quick check passed 5/5 with no saturation and low gains; evaluate on the 20 unseen judged seeds to confirm stage 3. |
| 7 | agent | finish |  | Stage 3 evaluation passed 20/20 on unseen seeds with no saturation, meeting the default acceptance criteria; the case is complete. |

Simulations run: 27
