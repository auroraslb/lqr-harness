"""Controller tuning, starting from the model itself.

The controller is LQR. Its weights come from Bryson's rule:
weight = 1 / (largest acceptable deviation)^2. The acceptable deviations
(tolerances) are derived from the model, with no hand-tuned numbers:

  CoM drift       COM_FRAC     x half the stance foot's half-width
  balance joints  BALANCE_FRAC x each joint's range of motion
  other joints    OTHER_FRAC   x each joint's range of motion
  motor commands  each actuator's command range

Tuning then works in four multipliers on that starting point
(com, balance, other, motor): 0.5 means half the tolerance (tighter),
2 means twice (looser). The agent reasons in "tighter or looser than the
physical default", never in raw weights.

Seeds are split like train / validation / test:
  TRY_SEEDS     quick checks while searching
  CHOOSE_SEEDS  choosing between the best candidates
  (judge_seeds in case.py: the final stage 3 check, a fresh block per evaluation)
"""
import itertools

import numpy as np
import mujoco

from .stages import (STANCE_FOOT, joint_groups, root_residual, balance_jacobian,
                     linearize, lqr_gain, simulate, evaluate, make_noise)

COM_FRAC = 0.5
BALANCE_FRAC = 0.25
OTHER_FRAC = 0.5

TRY_SEEDS = list(range(1, 6))
CHOOSE_SEEDS = list(range(6, 26))

DEFAULT_GRID = dict(com=[0.5, 1, 2], balance=[0.5, 1, 2], other=[0.5, 1, 2], motor=[0.5, 1, 2])


def initial_tolerances(model, data, qpos0):
    """Starting tolerances derived from the model's geometry and limits."""
    # foot size: spread of the stance foot's contact points
    root_residual(model, data, qpos0)
    fid = model.body(STANCE_FOOT).id
    pts = np.array([data.contact[i].pos[:2] for i in range(data.ncon)
                    if fid in (model.geom_bodyid[data.contact[i].geom1],
                               model.geom_bodyid[data.contact[i].geom2])])
    centred = pts - pts.mean(axis=0)
    smallest_spread = np.linalg.eigvalsh(centred.T @ centred / len(pts)).min()
    half_width = np.sqrt(3 * smallest_spread)        # half-width of a uniform spread
    com_m = COM_FRAC * max(half_width, 0.005)

    _, balance_dofs, _ = joint_groups(model)
    joint_rad = np.full(model.nv, np.inf)            # inf = not penalised (the floating base)
    for j in range(model.njnt):
        if model.jnt_type[j] != mujoco.mjtJoint.mjJNT_HINGE:
            continue
        dof = model.jnt_dofadr[j]
        span = (model.jnt_range[j, 1] - model.jnt_range[j, 0]) if model.jnt_limited[j] else np.pi
        joint_rad[dof] = (BALANCE_FRAC if dof in balance_dofs else OTHER_FRAC) * span

    lo, hi = model.actuator_ctrlrange.T
    motor = (hi - lo) / 2
    return {"com_m": float(com_m), "joint_rad": joint_rad, "motor": motor}


def scaled(model, tol, com=1.0, balance=1.0, other=1.0, motor=1.0):
    """Apply the four multipliers to a set of tolerances."""
    _, balance_dofs, _ = joint_groups(model)
    joint = tol["joint_rad"].copy()
    for dof in range(model.nv):
        if np.isfinite(joint[dof]):
            joint[dof] *= balance if dof in balance_dofs else other
    return {"com_m": tol["com_m"] * com, "joint_rad": joint, "motor": tol["motor"] * motor}


def controller_from(model, data, qpos0, tol, AB):
    """Bryson's rule per joint and per actuator, then LQR."""
    nv = model.nv
    A, B = AB
    jd = balance_jacobian(model, data, qpos0)
    joint_w = np.where(np.isfinite(tol["joint_rad"]), 1.0 / tol["joint_rad"] ** 2, 0.0)
    Qpos = jd.T @ jd / tol["com_m"] ** 2 + np.diag(joint_w)
    Q = np.block([[Qpos, np.zeros((nv, nv))],
                  [np.zeros((nv, 2 * nv))]])
    R = np.diag(1.0 / tol["motor"] ** 2)
    return lqr_gain(A, B, Q, R)


def describe(model, tol):
    """Human-readable summary of a set of tolerances, for logs and the agent."""
    _, balance_dofs, other_dofs = joint_groups(model)
    deg = np.degrees(tol["joint_rad"])
    return {"com_cm": round(tol["com_m"] * 100, 2),
            "balance_joints_deg_median": round(float(np.median(deg[balance_dofs])), 1),
            "other_joints_deg_median": round(float(np.median(deg[other_dofs])), 1),
            "motor_command_median": round(float(np.median(tol["motor"])), 2)}


def screen(model, data, qpos0, ctrl0, K, seeds=TRY_SEEDS, noise_scale=1.0):
    """Quick check with diagnostics: one run without noise, then a few noise seeds."""
    return evaluate(model, data, qpos0, ctrl0, K, seeds, noise_scale)


def tune(model, data, qpos0, ctrl0, tol0, grid=DEFAULT_GRID, top_k=3, noise_scale=1.0):
    """Two-step search over multipliers.

    1. Every grid point: Riccati must solve, must stand without noise, then a
       pass count on TRY_SEEDS.
    2. The best few are compared on CHOOSE_SEEDS (a 5-seed screen alone is not
       reliable: configs scoring 5/5 have scored 7/20 on fresh seeds).
    Returns the ranking, the best multipliers, which multipliers mattered, and
    how many simulations were used.
    """
    AB = linearize(model, data, qpos0, ctrl0)
    noises = {s: make_noise(model, s, noise_scale) for s in TRY_SEEDS}
    keys = ["com", "balance", "other", "motor"]
    screened, sims = [], 0
    for values in itertools.product(*(grid[k] for k in keys)):
        mult = dict(zip(keys, values))
        try:
            K = controller_from(model, data, qpos0, scaled(model, tol0, **mult), AB)
        except (np.linalg.LinAlgError, ValueError):
            screened.append((mult, "riccati_failed", 0, None))
            continue
        sims += 1
        if simulate(model, data, qpos0, ctrl0, K, None)["fell"]:
            screened.append((mult, "falls_without_noise", 0, None))
            continue
        n = sum(not simulate(model, data, qpos0, ctrl0, K, noises[s])["fell"] for s in TRY_SEEDS)
        sims += len(TRY_SEEDS)
        screened.append((mult, "ok", n, float(np.abs(K).max())))

    ok = [s for s in screened if s[1] == "ok"]
    best_screen = max((s[2] for s in ok), default=0)
    candidates = sorted([s for s in ok if s[2] == best_screen], key=lambda s: s[3])[:top_k]
    ranking = []
    for mult, _, n, gain in candidates:
        K = controller_from(model, data, qpos0, scaled(model, tol0, **mult), AB)
        ev = evaluate(model, data, qpos0, ctrl0, K, CHOOSE_SEEDS, noise_scale)
        sims += len(CHOOSE_SEEDS) + 1
        ranking.append({"multipliers": mult, "screen": f"{n}/{len(TRY_SEEDS)}",
                        "choose_pass": f"{ev['pass']}/{ev['n_seeds']}", "mean_sat": ev["mean_sat"],
                        "max_gain": round(gain, 1), "_pass": ev["pass"]})
    ranking.sort(key=lambda r: (-r["_pass"], r["mean_sat"], r["max_gain"]))
    for r in ranking:
        r.pop("_pass")

    marginals = {k: {str(v): sum(1 for s in ok if s[0][k] == v and s[2] == best_screen) for v in grid[k]}
                 for k in keys}
    counts = {st: sum(1 for s in screened if s[1] == st) for st in ("ok", "falls_without_noise", "riccati_failed")}
    return {"configs_tried": len(screened), "status_counts": counts, "best_screen": f"{best_screen}/{len(TRY_SEEDS)}",
            "ranking": ranking, "best": ranking[0]["multipliers"] if ranking else None,
            "multipliers_that_reached_best_screen": marginals, "simulations": sims}
