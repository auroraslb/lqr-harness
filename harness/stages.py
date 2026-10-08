"""Deterministic stage code for the LQR humanoid harness.

Reproduces the MuJoCo LQR tutorial as plain functions, one per stage, so a
design variant (payload) can be run end to end in a loop:

  stage 1  build_variant + find_equilibrium   -> qpos0, ctrl0, root residuals
           static_feasibility                 -> can ANY controller hold this pose?
  (fix)    balance_pose                       -> trim the ankle, or change posture
  stage 2  design_controller                  -> K
  stage 3  simulate / evaluate                -> fall, saturation, CoM metrics

Tuning lives in tuning.py, the stance search with no keyframe in stance.py.

Every function takes a MuJoCo model/data pair and returns plain numbers or
arrays. No LLM involved here: this is the deterministic layer the agent's
tools call into.
"""
import numpy as np
import scipy.linalg
from scipy.optimize import least_squares
import mujoco

KEYFRAME = "stand_on_left_leg"
STANCE_FOOT = "foot_left"

# Tutorial cost settings: the starting controller before any tuning
TUTORIAL = dict(name="tutorial", balance_cost=1000, balance_joint_cost=3, other_joint_cost=0.3, r_scale=1.0)

# Tutorial simulation settings
DURATION = 12
CTRL_RATE = 0.8
BALANCE_STD = 0.01
OTHER_STD = 0.08
COM_FALL_DIST = 0.15   # m
UPRIGHT_FRAC = 0.8

# Torso and arm joints a posture fix may move (on top of the stance-ankle trim).
POSE_JOINTS = ["abdomen_z", "abdomen_y", "abdomen_x",
               "shoulder1_right", "shoulder2_right", "elbow_right",
               "shoulder1_left", "shoulder2_left", "elbow_left"]


# ---------------------------------------------------------------- stage 1
def build_variant(xml_path, payload_kg=0.0, payload_body="lower_arm_right"):
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)
    if payload_kg:
        model.body_mass[model.body(payload_body).id] += payload_kg
        mujoco.mj_setConst(model, data)
    return model, data


def joint_groups(model):
    """Same grouping as the tutorial (z joints excluded from the balance group)."""
    nv = model.nv
    names = [model.joint(i).name for i in range(model.njnt)]
    abdomen = [model.joint(n).dofadr[0] for n in names if "abdomen" in n and "z" not in n]
    left_leg = [model.joint(n).dofadr[0] for n in names
                if "left" in n and ("hip" in n or "knee" in n or "ankle" in n) and "z" not in n]
    balance = abdomen + left_leg
    other = np.setdiff1d(range(6, nv), balance)
    return list(range(6)), balance, other


def root_residual(model, data, qpos):
    """Forces/torques the unactuated floating base would need to hold this pose still."""
    mujoco.mj_resetData(model, data)
    data.qpos = qpos
    mujoco.mj_forward(model, data)
    data.qacc = 0
    mujoco.mj_inverse(model, data)
    return data.qfrc_inverse.copy()


def height_sweep(model, data, qpos_start, lo=-0.001, hi=0.001, n=2001):
    offsets = np.linspace(lo, hi, n)
    vertical = []
    for off in offsets:
        q = qpos_start.copy()
        q[2] += off
        vertical.append(root_residual(model, data, q)[2])
    vertical = np.array(vertical)
    idx = int(np.argmin(np.abs(vertical)))
    return offsets[idx], idx in (0, n - 1), vertical


def solve_ctrl0(model, data, qpos0, qfrc0):
    """Tutorial method: pseudo-inverse of the actuator moment matrix."""
    mujoco.mj_resetData(model, data)
    data.qpos = qpos0
    mujoco.mj_forward(model, data)
    moment = np.zeros((model.nu, model.nv))
    mujoco.mju_sparse2dense(moment, data.actuator_moment.reshape(-1), data.moment_rownnz,
                            data.moment_rowadr, data.moment_colind.reshape(-1))
    return (np.atleast_2d(qfrc0) @ np.linalg.pinv(moment)).flatten()


def find_equilibrium(model, data, qpos_start=None):
    if qpos_start is None:
        mujoco.mj_resetDataKeyframe(model, data, model.key(KEYFRAME).id)
        qpos_start = data.qpos.copy()
    best, at_edge, _ = height_sweep(model, data, qpos_start)
    qpos0 = qpos_start.copy()
    qpos0[2] += best
    qfrc0 = root_residual(model, data, qpos0)
    ctrl0 = solve_ctrl0(model, data, qpos0, qfrc0)

    mujoco.mj_resetData(model, data)
    data.qpos = qpos0
    mujoco.mj_forward(model, data)
    com = data.subtree_com[model.body("torso").id][:2]
    foot = data.xpos[model.body(STANCE_FOOT).id][:2]
    lo, hi = model.actuator_ctrlrange.T
    metrics = {
        "height_offset_mm": round(best * 1000, 4),
        "offset_at_sweep_edge": bool(at_edge),
        "vertical_residual_N": round(float(qfrc0[2]), 3),
        "root_torque_norm_Nm": round(float(np.linalg.norm(qfrc0[3:6])), 2),
        "root_force_horiz_N": round(float(np.linalg.norm(qfrc0[0:2])), 3),
        "init_com_foot_m": round(float(np.linalg.norm(com - foot)), 4),
        "ctrl0_max_frac": round(float(np.max(np.abs(ctrl0) / hi)), 3),
        "ctrl0_in_limits": bool(np.all((ctrl0 >= lo) & (ctrl0 <= hi))),
    }
    return qpos0, ctrl0, metrics


# ---------------------------------------------------------------- feasibility
def static_feasibility(model, data, qpos):
    """Physics-only check, no controller and no simulation.

    Standing still needs the centre of pressure directly under the CoM, so:
      1. the CoM ground projection must be inside the stance foot's support polygon
      2. with the full weight acting at that point, every motor torque must be in limits
    Independent of the contact model and of the controller.
    """
    nv = model.nv
    mujoco.mj_resetData(model, data)
    data.qpos = qpos
    mujoco.mj_forward(model, data)
    torso, fid = model.body("torso").id, model.body(STANCE_FOOT).id
    com = data.subtree_com[torso].copy()
    margin = support_margin(model, data)

    data.qacc = 0
    saved = model.opt.disableflags
    model.opt.disableflags |= mujoco.mjtDisableBit.mjDSBL_CONTACT
    mujoco.mj_inverse(model, data)
    model.opt.disableflags = saved
    weight = model.body_subtreemass[torso] * -model.opt.gravity[2]
    point = np.array([com[0], com[1], data.xpos[fid][2] - 0.03])
    jacp = np.zeros((3, nv))
    mujoco.mj_jac(model, data, jacp, None, point, fid)
    need = data.qfrc_inverse - jacp.T @ np.array([0.0, 0.0, weight])
    moment = np.zeros((model.nu, nv))
    mujoco.mju_sparse2dense(moment, data.actuator_moment.reshape(-1), data.moment_rownnz,
                            data.moment_rowadr, data.moment_colind.reshape(-1))
    ctrl = (np.atleast_2d(need) @ np.linalg.pinv(moment)).flatten()
    i = int(np.argmax(np.abs(ctrl)))
    return {"com_margin_mm": round(float(margin) * 1000, 1),
            "static_max_ctrl": round(float(abs(ctrl[i])), 3),
            "static_limiting_actuator": model.actuator(i).name}


def support_margin(model, data):
    """Distance (m) from the CoM ground projection to the edge of the stance foot's
    support polygon, using the contacts currently in `data`. Positive = inside."""
    from scipy.spatial import ConvexHull
    fid = model.body(STANCE_FOOT).id
    pts = np.array([data.contact[i].pos[:2] for i in range(data.ncon)
                    if fid in (model.geom_bodyid[data.contact[i].geom1],
                               model.geom_bodyid[data.contact[i].geom2])])
    if len(pts) < 3:
        return -1.0
    com = data.subtree_com[model.body("torso").id][:2]
    return float(min(-(e[:2] @ com + e[2]) for e in ConvexHull(pts).equations))


TRIM_JOINTS = ["ankle_y_left", "ankle_x_left"]           # keeps the posture, shifts foot pressure
POSTURE_JOINTS = TRIM_JOINTS + POSE_JOINTS               # also allows torso and arms to move


def balance_pose(model, data, qpos_start, joints=TRIM_JOINTS, reg=0.3,
                 ctrl_cap=None, margin_target=None, w_ctrl=50.0, w_margin=2000.0):
    """Adjust root height and the given joints so the pose is truly balanced.

    Minimises the 6 root residuals + reg * joint change. Optional penalties:
      ctrl_cap       any motor above this fraction of its range (keeps headroom)
      margin_target  CoM closer than this (m) to the edge of the foot (keeps it centred)
    Joint limits are respected.
    """
    adr = np.array([model.joint(n).qposadr[0] for n in joints])
    jid = np.array([model.joint(n).id for n in joints])
    q_ref = qpos_start[adr]
    limited = model.jnt_limited[jid].astype(bool)
    lb = np.minimum(np.where(limited, model.jnt_range[jid, 0] - q_ref, -np.inf), 0.0)
    ub = np.maximum(np.where(limited, model.jnt_range[jid, 1] - q_ref, np.inf), 0.0)
    lb, ub = np.concatenate([[-0.003], lb]), np.concatenate([[0.003], ub])

    def pose(p):
        q = qpos_start.copy()
        q[2] += p[0]
        q[adr] += p[1:]
        return q

    def residual(p):
        q = pose(p)
        f = root_residual(model, data, q)
        r = [f[:6], reg * p[1:]]
        if margin_target is not None:
            r.append([w_margin * max(margin_target - support_margin(model, data), 0.0)])
        if ctrl_cap is not None:
            c = solve_ctrl0(model, data, q, f)
            r.append(w_ctrl * np.maximum(np.abs(c) - ctrl_cap, 0.0))
        return np.concatenate(r)

    sol = least_squares(residual, np.zeros(len(joints) + 1), bounds=(lb, ub),
                        diff_step=1e-6, x_scale="jac", max_nfev=600)
    changes = {n: round(float(np.degrees(d)), 1) for n, d in zip(joints, sol.x[1:]) if abs(d) > 1e-3}
    return pose(sol.x), changes


# ---------------------------------------------------------------- stage 2
def balance_jacobian(model, data, qpos0):
    """Jacobian of (CoM - stance foot) position: how each DOF moves the CoM relative to the foot."""
    nv = model.nv
    mujoco.mj_resetData(model, data)
    data.qpos = qpos0
    mujoco.mj_forward(model, data)
    jac_com = np.zeros((3, nv))
    mujoco.mj_jacSubtreeCom(model, data, jac_com, model.body("torso").id)
    jac_foot = np.zeros((3, nv))
    mujoco.mj_jacBodyCom(model, data, jac_foot, None, model.body(STANCE_FOOT).id)
    return jac_com - jac_foot


def linearize(model, data, qpos0, ctrl0):
    """A, B of the discrete-time dynamics around the setpoint (finite differences)."""
    nv, nu = model.nv, model.nu
    mujoco.mj_resetData(model, data)
    data.ctrl = ctrl0
    data.qpos = qpos0
    A = np.zeros((2 * nv, 2 * nv))
    B = np.zeros((2 * nv, nu))
    mujoco.mjd_transitionFD(model, data, 1e-6, True, A, B, None, None)
    return A, B


def lqr_gain(A, B, Q, R):
    try:
        P = scipy.linalg.solve_discrete_are(A, B, Q, R)
    except (np.linalg.LinAlgError, ValueError):
        # Riccati can fail when some state directions are not penalised at all
        # (e.g. rotation about the vertical axis). A tiny penalty on every state
        # fixes the numerics without changing the controller meaningfully.
        Q = Q + 1e-6 * np.eye(Q.shape[0])
        P = scipy.linalg.solve_discrete_are(A, B, Q, R)
    return np.linalg.inv(R + B.T @ P @ B) @ B.T @ P @ A


def design_controller(model, data, qpos0, ctrl0, cfg):
    """The tutorial's controller (group weights). Kept only as a reference to compare
    against; the harness builds its controllers in tuning.py."""
    nv, nu = model.nv, model.nu
    root_dofs, balance_dofs, other_dofs = joint_groups(model)
    jac_diff = balance_jacobian(model, data, qpos0)
    Qjoint = np.eye(nv)
    Qjoint[root_dofs, root_dofs] *= 0
    Qjoint[balance_dofs, balance_dofs] *= cfg["balance_joint_cost"]
    Qjoint[other_dofs, other_dofs] *= cfg["other_joint_cost"]
    Qpos = cfg["balance_cost"] * jac_diff.T @ jac_diff + Qjoint
    Q = np.block([[Qpos, np.zeros((nv, nv))],
                  [np.zeros((nv, 2 * nv))]])
    R = np.eye(nu) * cfg.get("r_scale", 1.0)
    A, B = linearize(model, data, qpos0, ctrl0)
    return lqr_gain(A, B, Q, R)


# ---------------------------------------------------------------- stage 3
def make_noise(model, seed, scale=1.0):
    nu = model.nu
    _, balance_dofs, _ = joint_groups(model)
    nsteps = int(np.ceil(DURATION / model.opt.timestep))
    ctrl_std = np.empty(nu)
    for i in range(nu):
        dof = model.joint(model.actuator(i).trnid[0]).dofadr[0]
        ctrl_std[i] = BALANCE_STD if dof in balance_dofs else OTHER_STD
    np.random.seed(seed)
    perturb = np.random.randn(nsteps, nu)
    width = int(nsteps * CTRL_RATE / DURATION)
    kernel = np.exp(-0.5 * np.linspace(-3, 3, width) ** 2)
    kernel /= np.linalg.norm(kernel)
    for i in range(nu):
        perturb[:, i] = np.convolve(perturb[:, i], kernel, mode="same")
    return perturb * ctrl_std * scale


def simulate(model, data, qpos0, ctrl0, K, noise=None):
    """One rollout. noise=None means no noise. Stops at the first fall.

    Besides fall and saturation rate, reports which actuators hit their limits
    and when saturation started: the clues needed to diagnose a failure.
    """
    nv, nu = model.nv, model.nu
    dt = model.opt.timestep
    nsteps = int(np.ceil(DURATION / dt))
    torso, foot = model.body("torso").id, model.body(STANCE_FOOT).id
    upright = UPRIGHT_FRAC * qpos0[2]
    lo, hi = model.actuator_ctrlrange.T
    mujoco.mj_resetData(model, data)
    data.qpos = qpos0
    dq = np.zeros(nv)
    sat_steps, max_com, fell_step, first_sat = 0, 0.0, None, None
    sat_per_act = np.zeros(nu, dtype=int)
    step = 0
    while data.time < DURATION and step < nsteps:
        mujoco.mj_differentiatePos(model, dq, 1, qpos0, data.qpos)
        data.ctrl = ctrl0 - K @ np.hstack((dq, data.qvel))
        if noise is not None:
            data.ctrl += noise[step]
        step += 1
        com_dist = float(np.linalg.norm(data.subtree_com[torso][:2] - data.xpos[foot][:2]))
        if data.qpos[2] < upright or com_dist > COM_FALL_DIST:
            fell_step = step
            break
        at_limit = (data.ctrl <= lo) | (data.ctrl >= hi)
        if at_limit.any():
            sat_steps += 1
            sat_per_act += at_limit
            if first_sat is None:
                first_sat = step
        max_com = max(max_com, com_dist)
        mujoco.mj_step(model, data)
    counted = fell_step or step
    return {
        "fell": fell_step is not None,
        "fell_time": None if fell_step is None else round(fell_step * dt, 3),
        "sat_rate": round(sat_steps / counted, 4),
        "first_sat_time": None if first_sat is None else round(first_sat * dt, 3),
        "sat_steps_per_actuator": {model.actuator(i).name: int(n) for i, n in enumerate(sat_per_act) if n},
        "max_com_foot_m": round(max_com, 4),
    }


def evaluate(model, data, qpos0, ctrl0, K, seeds, noise_scale=1.0):
    """Noise-off run plus a pass rate over noise seeds, with failure clues.
    noise_scale: disturbance size relative to the tutorial's (a requirement, not a knob)."""
    no_noise = simulate(model, data, qpos0, ctrl0, K, None)
    runs = [simulate(model, data, qpos0, ctrl0, K, make_noise(model, s, noise_scale)) for s in seeds]
    sat_totals = {}
    for r in runs:
        for name, n in r["sat_steps_per_actuator"].items():
            sat_totals[name] = sat_totals.get(name, 0) + n
    top = dict(sorted(sat_totals.items(), key=lambda kv: -kv[1])[:3])
    fall_times = [r["fell_time"] for r in runs if r["fell"]]
    first_sats = [r["first_sat_time"] for r in runs if r["first_sat_time"] is not None]
    return {
        "no_noise_fell": no_noise["fell"],
        "no_noise_fell_time": no_noise["fell_time"],
        "pass": sum(not r["fell"] for r in runs),
        "n_seeds": len(seeds),
        "fall_times_s": fall_times,
        "earliest_saturation_s": min(first_sats) if first_sats else None,
        "top_saturating_actuators": top,
        "worst_com_m": max(r["max_com_foot_m"] for r in runs),
        "mean_sat": round(float(np.mean([r["sat_rate"] for r in runs])), 4),
        "max_gain": round(float(np.abs(K).max()), 1),
    }
