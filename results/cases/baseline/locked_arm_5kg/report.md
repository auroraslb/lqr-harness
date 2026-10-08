# locked_arm_5kg

**Request:** Add a 5 kg tool to the right hand. The right arm must hold the tool exactly where it is now, so the right shoulder and elbow may not move.

**Status:** `no_solution_found`

## Changes made to the design
- **MAJOR** posture: fix_stance(mode=posture) (abdomen_x +0.6 deg, abdomen_y +1.1 deg, abdomen_z -8.0 deg, ankle_x_left +0.4 deg, ankle_y_left +2.0 deg, elbow_left -26.3 deg, shoulder1_left -40.9 deg, shoulder2_left -34.5 deg)
- **INFO** trim: fix_stance(mode=trim) (ankle_x_left -1.6 deg)

## Summary

Stage 1 still fails after trim, posture: ['static_max_ctrl']; metrics {'height_offset_mm': 0.0, 'offset_at_sweep_edge': False, 'vertical_residual_N': -0.001, 'root_torque_norm_Nm': 0.02, 'root_force_horiz_N': 0.0, 'init_com_foot_m': 0.0408, 'ctrl0_max_frac': 1.065, 'ctrl0_in_limits': False, 'com_margin_mm': 20.2, 'static_max_ctrl': 1.065, 'static_limiting_actuator': 'shoulder1_right'}

## Steps

| # | Actor | Action | Gate | Rationale |
|---|---|---|---|---|
| 1 | baseline | create_case |  | rule: create the case from the structured spec |
| 2 | baseline | check_feasibility | fail: root_torque_norm_Nm, static_max_ctrl, com_margin_mm | rule: always check stage 1 first |
| 3 | baseline | fix_stance:trim | fail: root_torque_norm_Nm, static_max_ctrl, com_margin_mm | rule: stage 1 failed, next fix is trim |
| 4 | baseline | fix_stance:posture | fail: static_max_ctrl | rule: stage 1 failed, next fix is posture |
| 5 | baseline | finish |  | rule: no stance fix left |

Simulations run: 0
