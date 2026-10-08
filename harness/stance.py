"""Find a balanced one-leg stance for a model with no usable keyframe.

Phase 1 (kinematics only, smooth): put the CoM over the centre of the stance
foot's sole, keep that foot flat, lift the free foot, stay close to the default pose.
Phase 2 (with contacts): ground the pose, then the same balanced-pose solve as
the payload fix (root residuals ~0, CoM margin, motor headroom).
"""
import numpy as np
import mujoco
from scipy.optimize import least_squares
from .stages import root_residual, solve_ctrl0, support_margin

def compose(q0, p, jadr):
    q = q0.copy()
    q[2] += p[0]
    v = np.asarray(p[1:4]); ang = np.linalg.norm(v)
    if ang > 1e-12:
        r = np.zeros(4); out = np.zeros(4)
        mujoco.mju_axisAngle2Quat(r, v / ang, ang)
        mujoco.mju_mulQuat(out, r, q0[3:7]); q[3:7] = out
    q[jadr] = q0[jadr] + p[4:]
    return q

def hinge_joints(model, locked=()):
    """Hinge joints the search may move (locked joints are left out)."""
    hinge = [j for j in range(model.njnt) if model.jnt_type[j] == mujoco.mjtJoint.mjJNT_HINGE
             and model.joint(j).name not in locked]
    return hinge, np.array([model.jnt_qposadr[j] for j in hinge])

def sole_reference(model, data, stance):
    """From the default pose standing on the floor: the stance foot's sole centre (in the
    foot's own frame), its height and orientation when flat."""
    q = model.qpos0.copy()
    # lower until the feet touch: bisect on the vertical residual
    lo, hi = -0.5, 0.5
    for _ in range(60):
        mid = 0.5 * (lo + hi); qq = q.copy(); qq[2] += mid
        if root_residual(model, data, qq)[2] > 0: hi = mid
        else: lo = mid
    q[2] += 0.5 * (lo + hi)
    root_residual(model, data, q)
    sid = model.body(stance).id
    pts = np.array([data.contact[i].pos for i in range(data.ncon)
                    if sid in (model.geom_bodyid[data.contact[i].geom1], model.geom_bodyid[data.contact[i].geom2])])
    centre_world = pts.mean(axis=0)
    R = data.xmat[sid].reshape(3, 3).copy()
    centre_local = R.T @ (centre_world - data.xpos[sid])
    return q, centre_local, data.xpos[sid][2].copy(), R

def phase1_kinematic(model, data, stance, free, clearance=0.10, reg=0.5, locked=(), hold=None):
    q0, c_local, z_flat, R_flat = sole_reference(model, data, stance)
    if hold is not None:   # locked joints keep the case's current values, not the default pose's
        for name in locked:
            a = model.joint(name).qposadr[0]
            q0[a] = hold[a]
    hinge, jadr = hinge_joints(model, locked)
    lim = model.jnt_limited[hinge].astype(bool)
    lb = np.concatenate([[-0.5] * 4, np.minimum(np.where(lim, model.jnt_range[hinge, 0] - q0[jadr], -np.inf), 0)])
    ub = np.concatenate([[0.5] * 4, np.maximum(np.where(lim, model.jnt_range[hinge, 1] - q0[jadr], np.inf), 0)])
    sid, fid, torso = model.body(stance).id, model.body(free).id, model.body("torso").id

    def residual(p):
        q = compose(q0, p, jadr)
        data.qpos[:] = q
        mujoco.mj_kinematics(model, data); mujoco.mj_comPos(model, data)
        R = data.xmat[sid].reshape(3, 3)
        sole = data.xpos[sid] + R @ c_local
        com = data.subtree_com[torso]
        lift = data.xpos[fid][2] - data.xpos[sid][2]
        return np.concatenate([
            20.0 * (com[:2] - sole[:2]),                 # CoM over the sole centre
            20.0 * np.array([data.xpos[sid][2] - z_flat]),   # stance foot at floor height
            5.0 * (R - R_flat).ravel(),                  # stance foot flat
            [20.0 * max(clearance - lift, 0.0)],         # free foot lifted
            reg * p[4:] * 0.1,                           # stay near default pose
        ])
    sol = least_squares(residual, np.zeros(len(lb)), bounds=(lb, ub), x_scale="jac", max_nfev=2000)
    return compose(q0, sol.x, jadr), sol

def phase2_contact(model, data, q_start, stance, free, reg=0.3, ctrl_cap=0.8, margin_target=0.006, clearance=0.08, locked=()):
    hinge, jadr = hinge_joints(model, locked)
    lim = model.jnt_limited[hinge].astype(bool)
    lb = np.concatenate([[-0.01, -0.05, -0.05, -0.05], np.minimum(np.where(lim, model.jnt_range[hinge, 0] - q_start[jadr], -np.inf), 0)])
    ub = np.concatenate([[0.01, 0.05, 0.05, 0.05], np.maximum(np.where(lim, model.jnt_range[hinge, 1] - q_start[jadr], np.inf), 0)])
    sid, fid = model.body(stance).id, model.body(free).id

    def residual(p):
        q = compose(q_start, p, jadr)
        f = root_residual(model, data, q)
        lift = data.xpos[fid][2] - data.xpos[sid][2]
        c = solve_ctrl0(model, data, q, f)
        return np.concatenate([f[:6], reg * p[4:],
                               [200.0 * max(clearance - lift, 0.0)],
                               [2000.0 * max(margin_target - support_margin(model, data), 0.0)],
                               50.0 * np.maximum(np.abs(c) - ctrl_cap, 0.0)])
    sol = least_squares(residual, np.zeros(len(lb)), bounds=(lb, ub), diff_step=1e-6, x_scale="jac", max_nfev=1500)
    return compose(q_start, sol.x, jadr), sol

def find_stance(model, data, stance="foot_left", free="foot_right", locked=(), hold=None):
    """hold: the case's current qpos; locked joints are kept at its values."""
    q1, s1 = phase1_kinematic(model, data, stance, free, locked=locked, hold=hold)
    # ground it: bisect root height on the vertical residual
    lo, hi = -0.05, 0.05
    for _ in range(50):
        mid = 0.5 * (lo + hi); qq = q1.copy(); qq[2] += mid
        if root_residual(model, data, qq)[2] > 0: hi = mid
        else: lo = mid
    q1[2] += 0.5 * (lo + hi)
    q2, s2 = phase2_contact(model, data, q1, stance, free, locked=locked)
    return q2, (s1, s2)
