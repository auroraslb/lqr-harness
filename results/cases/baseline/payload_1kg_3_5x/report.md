# payload_1kg_3_5x

**Request:** Add a 1 kg tool to the right hand. The robot must survive 3.5 times the usual disturbance.

**Status:** `no_solution_found`

## Changes made to the design
- **INFO** trim: fix_stance(mode=trim) (ankle_x_left -1.1 deg)
- **INFO** posture: fix_stance(mode=posture) (elbow_left +0.4 deg, shoulder1_left +0.1 deg, shoulder1_right +0.6 deg, shoulder2_left -0.7 deg, shoulder2_right -2.0 deg)

## Summary

Tuned multipliers {'com': 2, 'balance': 1, 'other': 0.5, 'motor': 2}; judged 17/20.

## Steps

| # | Actor | Action | Gate | Rationale |
|---|---|---|---|---|
| 1 | baseline | create_case |  | rule: create the case from the structured spec |
| 2 | baseline | check_feasibility | fail: root_torque_norm_Nm, static_max_ctrl | rule: always check stage 1 first |
| 3 | baseline | fix_stance:trim | fail: static_max_ctrl | rule: stage 1 failed, next fix is trim |
| 4 | baseline | fix_stance:posture | pass | rule: stage 1 failed, next fix is posture |
| 5 | baseline | tune | pass | rule: tune with the default grid |
| 6 | baseline | evaluate | fail: pass_rate, mean_saturation | rule: judge the tuned controller |
| 7 | baseline | finish |  | rule: report the judged result |

Simulations run: 570
