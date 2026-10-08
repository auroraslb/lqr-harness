"""The Case: one design variant moving through the pipeline.

Holds the run state (pose, controls, controller), checks the gates, tracks
every change made to the design, and writes the manifest and report.
Both the LLM agent and the rule-based baseline drive a Case through the same
tools, so they are judged on exactly the same terms.
"""
from dataclasses import dataclass, field, asdict
from datetime import datetime, timezone
import json

import numpy as np
import mujoco

from . import stages, tuning
from .paths import MODEL_XML, RESULTS


# ---------------------------------------------------------------- gates
# Locked thresholds. The agent can read these but never change them.
# Each number comes from the manual experiments (see tutorial/friction_log.md).
GATES = {
    "stage1": {
        "max_root_torque_Nm": 0.5,     # trimmed poses reach ~0; tutorial equilibrium was 3.55
        "max_static_ctrl": 0.80,       # 78-80% passed 20/20; 97-100% gave 0-15/20
        "min_com_margin_mm": 5.0,      # a 2.6 mm pose fell; 6+ mm poses passed
    },
    "stage3": {
        "max_mean_saturation": 0.01,   # passing runs sat ~0%
        # the required pass rate is part of the spec's acceptance block
    },
}

# What the harness supports today. The stance leg is an input, but only the left
# leg is implemented: the stance foot, ankle joints and balance chain are still
# named in stages.py rather than derived from the model tree.
SUPPORTED_STANCES = ("left",)

DEFAULT_ACCEPTANCE = {
    "disturbance": 1.0,   # disturbance to survive, relative to the tutorial's noise level
    "pass_rate": 0.95,    # fraction of judged seeds that must stay up (tutorial settings: 6/20)
}

# How a change to the design is raised, by the largest joint change (degrees)
SEVERITY_DEG = {"warning": 5.0, "major": 25.0}

# Test conditions: requirements, not tuning knobs. Locked.
# Final stage 3 check only; never used while searching or choosing (see tuning.py).
# Every evaluation gets a fresh block of seeds (26-45, then 46-65, ...), so a judgement
# is never made on seeds whose results have already been seen and acted on.
JUDGE_BLOCK = 20
JUDGE_START = 26


def judge_seeds(n):
    """Seeds for the n-th evaluation of a case (n starts at 1)."""
    start = JUDGE_START + JUDGE_BLOCK * (n - 1)
    return list(range(start, start + JUDGE_BLOCK))


@dataclass
class Spec:
    """What the run is asked to do. Written by the planner (or by hand).

    Three parts: the design (payload), the task (stance, starting pose, what may
    not move) and the acceptance criteria (what counts as success).
    """
    request: str                       # the original natural-language request
    # design
    payload_kg: float = 0.0
    payload_body: str = "lower_arm_right"
    # task
    stance: str = "left"               # which leg to balance on (only "left" implemented)
    use_keyframe: bool = True          # False = find a stance from the default pose
    locked_joints: list = field(default_factory=list)   # joints the harness may not move
    # acceptance
    acceptance: dict = field(default_factory=lambda: dict(DEFAULT_ACCEPTANCE))


def check_stage1(m):
    failed = []
    g = GATES["stage1"]
    if m["offset_at_sweep_edge"]:
        failed.append("offset_at_sweep_edge")
    if m["root_torque_norm_Nm"] > g["max_root_torque_Nm"]:
        failed.append("root_torque_norm_Nm")
    if m["static_max_ctrl"] > g["max_static_ctrl"]:
        failed.append("static_max_ctrl")
    if m["com_margin_mm"] < g["min_com_margin_mm"]:
        failed.append("com_margin_mm")
    return {"pass": not failed, "failed": failed}


def check_stage3(m, acceptance):
    failed = []
    g = GATES["stage3"]
    if m["no_noise_fell"]:
        failed.append("no_noise_fell")
    if m["pass"] / m["n_seeds"] < acceptance["pass_rate"]:
        failed.append("pass_rate")
    if m["mean_sat"] > g["max_mean_saturation"]:
        failed.append("mean_saturation")
    return {"pass": not failed, "failed": failed}


class Case:
    def __init__(self, case_id, spec: Spec):
        self.case_id = case_id
        self.spec = spec
        self.model, self.data = stages.build_variant(str(MODEL_XML), spec.payload_kg, spec.payload_body)

        # pose and controls (stage 1), controller (stage 2)
        if spec.use_keyframe:
            mujoco.mj_resetDataKeyframe(self.model, self.data, self.model.key(stages.KEYFRAME).id)
            self.qpos = self.data.qpos.copy()
        else:
            self.qpos = self.model.qpos0.copy()   # default pose, no keyframe
        self.reference_qpos = self.qpos.copy()   # what changes are measured against
        self.ctrl0 = None
        # Controller: tolerances derived from the model once stage 1 has a pose,
        # then four multipliers on top (1.0 = the physical default). No tutorial weights.
        self.tol0 = None
        self.multipliers = None
        self.K = None

        self.stage_passed = {"stage1": False, "stage2": False, "stage3": False}
        self.counters = {"stance_fixes": 0, "controller_tries": 0, "tunes": 0, "evaluations": 0, "simulations": 0}
        self.changes = []     # every design change, with severity
        self.log = []         # every action: who, what, why, metrics, gate
        self.status = "running"
        self.summary = ""

    # ------------------------------------------------------------ state helpers
    def stage1_metrics(self):
        """Equilibrium on the current pose + static feasibility. Updates qpos/ctrl0."""
        qpos0, ctrl0, m = stages.find_equilibrium(self.model, self.data, self.qpos)
        m.update(stages.static_feasibility(self.model, self.data, qpos0))
        self.qpos, self.ctrl0 = qpos0, ctrl0
        return {k: (float(v) if isinstance(v, (np.floating, float)) else v) for k, v in m.items()}

    def reset_controller(self):
        """A new pose invalidates the derived tolerances and any controller built on them."""
        self.tol0, self.multipliers, self.K = None, None, None
        self.stage_passed["stage2"] = self.stage_passed["stage3"] = False

    def joint_changes_deg(self):
        """Per-joint change from the reference pose, for hinge joints, in degrees."""
        out = {}
        for j in range(self.model.njnt):
            if self.model.jnt_type[j] != mujoco.mjtJoint.mjJNT_HINGE:
                continue
            a = self.model.jnt_qposadr[j]
            d = float(np.degrees(self.qpos[a] - self.reference_qpos[a]))
            if abs(d) >= 0.1:
                out[self.model.joint(j).name] = round(d, 1)
        return out

    def raise_change(self, kind, detail, joint_changes):
        biggest = max((abs(v) for v in joint_changes.values()), default=0.0)
        severity = ("major" if biggest > SEVERITY_DEG["major"]
                    else "warning" if biggest > SEVERITY_DEG["warning"] else "info")
        change = {"kind": kind, "severity": severity, "max_change_deg": round(biggest, 1),
                  "detail": detail, "joint_changes_deg": joint_changes}
        self.changes.append(change)
        return change

    def record(self, actor, action, rationale, params=None, metrics=None, gate=None):
        self.log.append({
            "step": len(self.log) + 1, "actor": actor, "action": action,
            "rationale": rationale, "params": params or {},
            "metrics": metrics or {}, "gate": gate,
        })

    # ------------------------------------------------------------ outputs
    def manifest(self):
        return {
            "case_id": self.case_id,
            "finished_at": datetime.now(timezone.utc).isoformat(timespec="seconds"),
            "spec": asdict(self.spec),
            "gates": GATES,
            "status": self.status,
            "summary": self.summary,
            "changes": self.changes,
            "final_controller": {"multipliers": self.multipliers,
                                 "tolerances": (tuning.describe(self.model, tuning.scaled(self.model, self.tol0, **self.multipliers))
                                                if self.multipliers else None)},
            "counters": self.counters,
            "steps": self.log,
        }

    def report(self):
        lines = [f"# {self.case_id}", "", f"**Request:** {self.spec.request}", "",
                 f"**Status:** `{self.status}`", ""]
        lines.append("## Changes made to the design")
        if self.changes:
            order = {"major": 0, "warning": 1, "info": 2}
            for c in sorted(self.changes, key=lambda c: order[c["severity"]]):
                joints = ", ".join(f"{k} {v:+.1f} deg" for k, v in c["joint_changes_deg"].items())
                lines.append(f"- **{c['severity'].upper()}** {c['kind']}: {c['detail']} ({joints})")
        else:
            lines.append("- None")
        lines += ["", "## Summary", "", self.summary or "(none)", "",
                  "## Steps", "", "| # | Actor | Action | Gate | Rationale |", "|---|---|---|---|---|"]
        for s in self.log:
            gate = "" if s["gate"] is None else ("pass" if s["gate"]["pass"] else "fail: " + ", ".join(s["gate"]["failed"]))
            lines.append(f"| {s['step']} | {s['actor']} | {s['action']} | {gate} | {s['rationale']} |")
        lines += ["", f"Simulations run: {self.counters['simulations']}"]
        return "\n".join(lines) + "\n"

    def save(self, folder=None):
        folder = folder or RESULTS / "cases" / self.case_id
        folder.mkdir(parents=True, exist_ok=True)
        (folder / "manifest.json").write_text(json.dumps(self.manifest(), indent=2))
        (folder / "report.md").write_text(self.report())
        np.savez(folder / "final_state.npz", qpos=self.qpos,
                 ctrl0=self.ctrl0 if self.ctrl0 is not None else [],
                 K=self.K if self.K is not None else [])
        return folder
