# payload_3kg

**Request:** Add a 3 kg tool to the right hand and keep the robot balancing on its left leg.

**Status:** `passed_with_changes`

## Changes made to the design
- **WARNING** posture: fix_stance(mode=posture) (abdomen_x -0.3 deg, abdomen_y -0.9 deg, ankle_x_left +0.3 deg, ankle_y_left +0.8 deg, elbow_left +1.3 deg, elbow_right +4.6 deg, shoulder1_left -4.2 deg, shoulder1_right +20.7 deg, shoulder2_left -5.8 deg, shoulder2_right -2.5 deg)
- **INFO** trim: fix_stance(mode=trim) (ankle_x_left -1.5 deg)

## Summary

Tuned multipliers {'com': 2, 'balance': 0.5, 'other': 0.5, 'motor': 0.5}; judged 20/20.

## Steps

| # | Actor | Action | Gate | Rationale |
|---|---|---|---|---|
| 1 | baseline | create_case |  | rule: create the case from the structured spec |
| 2 | baseline | check_feasibility | fail: root_torque_norm_Nm, static_max_ctrl, com_margin_mm | rule: always check stage 1 first |
| 3 | baseline | fix_stance:trim | fail: root_torque_norm_Nm, static_max_ctrl, com_margin_mm | rule: stage 1 failed, next fix is trim |
| 4 | baseline | fix_stance:posture | pass | rule: stage 1 failed, next fix is posture |
| 5 | baseline | tune | pass | rule: tune with the default grid |
| 6 | baseline | evaluate | pass | rule: judge the tuned controller |
| 7 | baseline | finish |  | rule: report the judged result |

Simulations run: 570
