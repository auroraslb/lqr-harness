# payload_1kg_3_5x

**Request:** Add a 1 kg tool to the right hand. The robot must survive 3.5 times the usual disturbance.

**Status:** `passed_with_changes`

## Changes made to the design
- **INFO** trim: fix_stance(mode=trim) (ankle_x_left -1.1 deg)
- **INFO** posture: fix_stance(mode=posture) (elbow_left +0.4 deg, shoulder1_left +0.1 deg, shoulder1_right +0.6 deg, shoulder2_left -0.7 deg, shoulder2_right -2.0 deg)

## Summary

Request: add a 1 kg tool to the right hand (lower_arm_right), survive 3.5x the usual disturbance, left-leg stance from the tutorial keyframe.

Stage 1 (pose): Keyframe pose failed initially (root torque 6.59 Nm, static ankle_y_left at 83% > 80% limit). Applied a ~1.1deg trim on ankle_x_left, which zeroed root torque but left ankle_y_left at 83%. Applied a posture fix (small torso/arm redistribution: elbow_left +0.4deg, shoulder1_left +0.1deg, shoulder1_right +0.6deg, shoulder2_left -0.7deg, shoulder2_right -2.0deg) which brought static ankle_y_left exactly to 80% and CoM margin to 6.0 mm (>= 5 mm required). Gate passed.

Stage 2 (controller): Default multipliers (all=1) failed badly under the 3.5x disturbance - 0/5 quick-check seeds survived, with heavy saturation on ankle_x_left/ankle_y_left/hip_x_left starting at 0.2s. Increasing motor authority and tightening non-balance joint tolerance (motor=2, other=0.5) immediately fixed this: 5/5 quick check, zero saturation. A narrow tune search around that point found a better point, motor=3, other=0.75 (com=1, balance=1), which screened 5/5 and won the 20-seed comparison with zero/near-zero saturation and the highest margin.

Stage 3 (judged evaluation): adopted controller (com=1, balance=1, other=0.75, motor=3) evaluated on 20 fresh unseen seeds plus the no-noise case: 20/20 stayed up, no fall without noise, mean saturation 0.0003 (only brief ankle_y_left saturation starting at 2.1s), worst CoM drift 5.1 cm. This clears the 95% pass-rate acceptance for 3.5x disturbance with full margin.

Result: PASSED (passed_with_changes - pose trim + posture adjustment applied, controller tuned to motor=3/other=0.75 vs. default 1/1/1/1). The extra motor authority was needed because the hand payload plus 3.5x disturbance pushes the stance ankle/hip into saturation at default gains; loosening non-balance joint tolerance let the controller use more of the arm/torso DOFs to absorb disturbance instead of relying solely on the ankle.

## Steps

| # | Actor | Action | Gate | Rationale |
|---|---|---|---|---|
| 1 | agent | create_case |  | Create case for 1kg tool payload on right hand with 3.5x disturbance requirement, starting from tutorial one-leg keyframe. |
| 2 | agent | check_feasibility | fail: root_torque_norm_Nm, static_max_ctrl | Check if the one-leg keyframe pose with the added 1kg hand payload is statically feasible before building a controller. |
| 3 | agent | fix_stance:trim | fail: static_max_ctrl | Root torque and static ctrl fail; trim the stance ankle to center CoM under the foot before trying posture changes. |
| 4 | agent | fix_stance:posture | pass | Ankle_y_left still exceeds 80% motor capacity; adjust torso/arm posture to reduce static ankle torque while keeping CoM margin. |
| 5 | agent | check_feasibility | pass | Confirm the gate passes cleanly after posture fix before proceeding to controller design. |
| 6 | agent | try_controller | pass | Baseline LQR with default multipliers to see how it handles the payload-shifted pose before tuning for the 3.5x disturbance requirement. |
| 7 | agent | try_controller | pass | Heavy saturation on ankle_x_left/ankle_y_left and all seeds fell; per prior experience, increasing motor authority and tightening non-balance joint tolerance should reduce drift and saturation under this large 3.5x disturbance. |
| 8 | agent | tune | pass | Narrow search around the successful (other=0.5, motor=2) point to confirm robustness and check if a less aggressive or more aggressive setting performs better on more seeds before committing an evaluation. |
| 9 | agent | evaluate | pass | Adopted controller (other=0.75, motor=3) passed 20/20 on the tune's choose set with zero saturation; run the formal stage-3 judgment on fresh unseen seeds. |
| 10 | agent | finish |  | Stage 3 evaluation passed 20/20 (100% > 95% required) on unseen seeds with no falls and negligible saturation, satisfying the 3.5x disturbance acceptance criterion. |

Simulations run: 150
