# locked_arm_5kg

**Request:** Add a 5 kg tool to the right hand. The right arm must hold the tool exactly where it is now, so the right shoulder and elbow may not move.

**Status:** `no_solution_found`

## Changes made to the design
- **MAJOR** posture: fix_stance(mode=posture) (abdomen_x +0.6 deg, abdomen_y +1.1 deg, abdomen_z -8.0 deg, ankle_x_left +0.4 deg, ankle_y_left +2.0 deg, elbow_left -26.3 deg, shoulder1_left -40.9 deg, shoulder2_left -34.5 deg)
- **INFO** trim: fix_stance(mode=trim) (ankle_x_left -1.6 deg)

## Summary

Stage 1 (stance feasibility) cannot be satisfied. With the right arm locked at its current configuration (shoulder1_right, shoulder2_right, elbow_right fixed) and a 5 kg tool added to the right hand, holding the arm in place statically requires 106.5% of the shoulder1_right actuator's torque range (ctrl0_max_frac = 1.065, unchanged before and after posture correction). An ankle trim (1.6°) fixed the root-torque residual, and a posture adjustment (torso yaw ~8°, left shoulder/elbow counter-raise ~27-41°) successfully restored CoM margin to +20.2 mm (from -13 mm) and root torque to ~0.02 Nm, so those two gates now pass. However the shoulder1_right torque requirement is intrinsic to the locked arm geometry plus the 5 kg load at the hand's moment arm - it does not change with torso or other-limb posture, since the lock forbids any change to the loaded joint itself. No remaining stance-fix budget or allowed joint motion can reduce this torque: the only joints that could reduce it (shoulder1_right/shoulder2_right/elbow_right) are exactly the ones the request locks. Conclusion: a 5 kg payload held at the current right-arm pose exceeds the shoulder1_right motor's static torque capacity by about 6.5%; this is a hardware/pose conflict with the "arm must not move" constraint, not a controller-tuning issue, so no LQR controller can compensate for it. No solution exists under the given constraints.

## Steps

| # | Actor | Action | Gate | Rationale |
|---|---|---|---|---|
| 1 | agent | create_case |  | Start a new case for the 5kg right-hand payload with the right arm locked, balancing on the left leg per the tutorial default. |
| 2 | agent | check_feasibility | fail: root_torque_norm_Nm, static_max_ctrl, com_margin_mm | Check if the one-leg stance with 5kg payload on the locked right arm is statically feasible before any controller work. |
| 3 | agent | fix_stance:trim | fail: root_torque_norm_Nm, static_max_ctrl, com_margin_mm | Root torque and CoM margin fail; start with a small ankle trim to re-center pressure under the CoM before trying posture changes. |
| 4 | agent | fix_stance:posture | fail: static_max_ctrl | Trim alone left CoM margin and motor saturation failing; try posture adjustment (torso/other arm) to counterbalance the 5kg right-hand payload while keeping the locked right arm fixed. |
| 5 | agent | finish |  | Stage 1 static motor-need gate fails on the locked shoulder1_right actuator (106.5% of range) and this value is invariant to all allowed posture changes, since the overloaded joint itself cannot be moved; further budget cannot fix a torque limit on a locked joint. |

Simulations run: 0
