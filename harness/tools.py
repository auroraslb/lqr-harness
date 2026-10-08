"""The tools: the only way the agent (or the baseline) can act on a case.

Each tool wraps deterministic code from stages.py / stance.py / tuning.py and
returns compact JSON: metrics plus the gate result. The rules that must hold
no matter what the LLM decides are enforced here, in code:

  - no controller work before stage 1 has passed
  - locked joints are never moved
  - per-case budgets on stance fixes, controller tries, tuning runs and evaluations
  - every design change is raised with a severity
  - stage 3 is only judged on seeds never used for searching or choosing
  - the final status is checked against the gates (no "passed" without stage 3)
"""
import numpy as np

from . import stages, stance, tuning
from .paths import RESULTS
from .case import (Case, Spec, GATES, DEFAULT_ACCEPTANCE, SUPPORTED_STANCES,
                   check_stage1, check_stage3, judge_seeds)

BUDGETS = {"stance_fixes": 3, "controller_tries": 8, "tunes": 2, "evaluations": 2}

# Tool definitions in the format the LLM API expects. Every acting tool takes a
# required `rationale`: the agent has to say why before it acts, and the
# reason goes straight into the manifest.
_RATIONALE = {"type": "string", "description": "Why you are taking this action, in one or two sentences."}
_MULT = {"type": "number", "description": "Multiplier on the physical default tolerance: <1 tighter, >1 looser. 1 = default."}

TOOL_SCHEMAS = [
    {
        "name": "create_case",
        "description": "Start a new case from a design request. Call once, first.",
        "input_schema": {
            "type": "object",
            "properties": {
                "case_id": {"type": "string", "description": "Short id, e.g. payload_3kg_keyframe."},
                "request": {"type": "string", "description": "The original request text."},
                "payload_kg": {"type": "number", "description": "Extra mass added to the model, 0 for none."},
                "payload_body": {"type": "string", "description": "Body the payload is attached to. Default lower_arm_right."},
                "stance": {"type": "string", "enum": ["left", "right"],
                           "description": "Which leg to balance on. Only 'left' is implemented; 'right' returns an error."},
                "use_keyframe": {"type": "boolean", "description": "true: start from the tutorial's one-leg keyframe. false: no keyframe, search a stance from the default pose."},
                "locked_joints": {"type": "array", "items": {"type": "string"},
                                  "description": "Joints the harness may not move, from the request (e.g. an arm holding a tool in place)."},
                "acceptance": {"type": "object", "description": (
                    "What counts as success, from the request. disturbance: disturbance the robot must survive, "
                    "relative to the tutorial's noise (default 1; 'twice the disturbance' = 2). pass_rate: fraction "
                    "of judged runs that must stay up (default 0.95). Fixed for the whole run."),
                    "properties": {"disturbance": {"type": "number"}, "pass_rate": {"type": "number"}}},
                "rationale": _RATIONALE,
            },
            "required": ["case_id", "request", "payload_kg", "use_keyframe", "locked_joints", "rationale"],
        },
    },
    {
        "name": "check_feasibility",
        "description": ("Stage 1: compute the equilibrium for the current pose and check static feasibility "
                        "(CoM inside the foot, motor need, root residual torque). Returns metrics and the gate result."),
        "input_schema": {"type": "object", "properties": {"case_id": {"type": "string"}, "rationale": _RATIONALE},
                         "required": ["case_id", "rationale"]},
    },
    {
        "name": "fix_stance",
        "description": ("Change the pose to pass stage 1. mode='trim': adjust only the stance ankle (~1 deg) so foot "
                        "pressure sits under the CoM; keeps the posture. mode='posture': also move torso and arms, keeping "
                        "motors <= 80% and the CoM >= 6 mm inside the foot; works best after a trim. mode='search': find a "
                        "one-leg stance from the model's default pose (for models with no usable keyframe). Locked joints "
                        "are never moved. Every change is raised with a severity. A new pose resets the controller. "
                        "Budget: 3 per case."),
        "input_schema": {"type": "object", "properties": {
            "case_id": {"type": "string"},
            "mode": {"type": "string", "enum": ["trim", "posture", "search"]},
            "rationale": _RATIONALE},
            "required": ["case_id", "mode", "rationale"]},
    },
    {
        "name": "try_controller",
        "description": ("Stage 2: build an LQR controller from tolerances derived from the model (CoM drift from foot "
                        "size, joint motion from joint ranges, motor effort from command ranges), scaled by your four "
                        "multipliers, and adopt it. Then a quick check: one run without noise and 5 noise seeds. Returns "
                        "pass count, fall times, which actuators saturate and from when, CoM drift and gain size. Use it "
                        "to learn what this setup needs; start with all multipliers at 1. Requires stage 1 passed. "
                        "Budget: 8 per case."),
        "input_schema": {"type": "object", "properties": {
            "case_id": {"type": "string"},
            "com": _MULT, "balance": _MULT, "other": _MULT, "motor": _MULT,
            "rationale": _RATIONALE},
            "required": ["case_id", "com", "balance", "other", "motor", "rationale"]},
    },
    {
        "name": "tune",
        "description": ("Stage 2: systematic search over multiplier combinations: screen each on 5 seeds, compare the "
                        "best on 20 further seeds, adopt the winner. Default grid is 0.5/1/2 for each multiplier (81 "
                        "combinations); pass your own grid to search around what your tries suggest. Returns the ranking "
                        "and which multipliers mattered. Costs many simulations. Requires stage 1 passed. Budget: 2 per case."),
        "input_schema": {"type": "object", "properties": {
            "case_id": {"type": "string"},
            "grid": {"type": "object", "description": ("Optional. Keys com, balance, other, motor; each a list of 1-3 "
                                                      "multipliers. Omit for the default grid.")},
            "rationale": _RATIONALE},
            "required": ["case_id", "rationale"]},
    },
    {
        "name": "evaluate",
        "description": ("Stage 3: judge the adopted controller on 20 seeds never used for searching or choosing, plus "
                        "one run without noise. Each evaluation uses a fresh block of seeds, so a second evaluation is "
                        "judged on seeds you have not seen. The only way to pass stage 3. Requires a controller (from try_controller "
                        "or tune). Budget: 2 per case."),
        "input_schema": {"type": "object", "properties": {"case_id": {"type": "string"}, "rationale": _RATIONALE},
                         "required": ["case_id", "rationale"]},
    },
    {
        "name": "get_history",
        "description": "All steps taken on this case so far, with metrics, gate results, changes and budgets used.",
        "input_schema": {"type": "object", "properties": {"case_id": {"type": "string"}}, "required": ["case_id"]},
    },
    {
        "name": "finish",
        "description": ("End the run. status='passed' requires stage 3 to have passed (it becomes "
                        "'passed_with_changes' automatically if the design was changed). status='no_solution_found' when no "
                        "allowed change or budget left makes it work; name the stage and the limiting reason in the summary. "
                        "status='unsupported' when the request asks for something the harness cannot do (create_case "
                        "says so); no case_id needed in that situation, pass the one you tried."),
        "input_schema": {"type": "object", "properties": {
            "case_id": {"type": "string"},
            "status": {"type": "string", "enum": ["passed", "no_solution_found", "unsupported"]},
            "summary": {"type": "string", "description": "What happened, what changed and why, for an engineer."},
            "rationale": _RATIONALE},
            "required": ["case_id", "status", "summary", "rationale"]},
    },
]


def _clean(x):
    """Make numpy values JSON-friendly and round floats for a compact LLM view."""
    if isinstance(x, dict):
        return {k: _clean(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [_clean(v) for v in x]
    if isinstance(x, (np.floating, float)):
        return round(float(x), 4)
    if isinstance(x, np.integer):
        return int(x)
    if isinstance(x, np.bool_):
        return bool(x)
    return x


class Toolbox:
    """Executes tool calls for one actor ('agent' or 'baseline')."""

    def __init__(self, actor):
        self.actor = actor
        self.cases = {}
        self.rejected = {}   # requests create_case could not accept (e.g. unsupported stance)

    def call(self, name, args):
        if name not in {t["name"] for t in TOOL_SCHEMAS}:
            return {"error": f"unknown tool {name!r}"}
        cid = args.get("case_id")
        if name == "finish" and cid in self.rejected:
            pass
        elif name != "create_case" and cid not in self.cases:
            return {"error": f"unknown case_id {args.get('case_id')!r}; call create_case first"}
        try:
            return _clean(getattr(self, name)(**args))
        except TypeError as e:
            return {"error": f"bad arguments for {name}: {e}"}

    # ------------------------------------------------------------ helpers
    def _budget(self, c, key):
        if c.counters.get(key, 0) >= BUDGETS[key]:
            return {"error": f"{key} budget used up ({BUDGETS[key]}); finish the case or use another tool"}
        c.counters[key] = c.counters.get(key, 0) + 1
        return None

    def _ensure_tolerances(self, c):
        if c.tol0 is None:
            c.tol0 = tuning.initial_tolerances(c.model, c.data, c.qpos)

    # ------------------------------------------------------------ tools
    def create_case(self, case_id, request, payload_kg, use_keyframe, locked_joints,
                    rationale, payload_body="lower_arm_right", stance="left", acceptance=None):
        if stance not in SUPPORTED_STANCES:
            reason = (f"stance {stance!r} is not implemented: the harness currently balances on the left leg only")
            self.rejected[case_id] = {"request": request, "stance": stance, "reason": reason, "rationale": rationale}
            return {"error": reason + ". Finish this case with status='unsupported'."}
        acc = dict(DEFAULT_ACCEPTANCE, **(acceptance or {}))
        if not (0.5 <= acc["disturbance"] <= 5 and 0.5 <= acc["pass_rate"] <= 1):
            return {"error": "acceptance: disturbance must be 0.5-5 and pass_rate 0.5-1"}
        spec = Spec(request=request, payload_kg=payload_kg, payload_body=payload_body, stance=stance,
                    use_keyframe=use_keyframe, locked_joints=list(locked_joints), acceptance=acc)
        c = Case(case_id, spec)
        names = {c.model.joint(i).name for i in range(c.model.njnt)}
        unknown = [j for j in spec.locked_joints if j not in names]
        if unknown:
            return {"error": f"unknown joints in locked_joints: {unknown}"}
        self.cases[case_id] = c
        c.record(self.actor, "create_case", rationale, params={"spec": vars(spec)})
        return {"case_id": case_id, "spec": vars(spec), "gates": GATES, "budgets": BUDGETS}

    def check_feasibility(self, case_id, rationale):
        c = self.cases[case_id]
        m = c.stage1_metrics()
        gate = check_stage1(m)
        c.stage_passed["stage1"] = gate["pass"]
        c.record(self.actor, "check_feasibility", rationale, metrics=m, gate=gate)
        return {"stage": 1, "metrics": m, "gate": gate}

    def fix_stance(self, case_id, mode, rationale):
        c = self.cases[case_id]
        if mode not in ("trim", "posture", "search"):
            return {"error": f"unknown mode {mode!r}"}
        locked = set(c.spec.locked_joints)
        if mode == "trim" and all(j in locked for j in stages.TRIM_JOINTS):
            return {"error": "both stance ankle joints are locked; trim is not possible"}
        if (err := self._budget(c, "stance_fixes")):
            return err
        before = c.joint_changes_deg()

        if mode == "trim":
            joints = [j for j in stages.TRIM_JOINTS if j not in locked]
            c.qpos, _ = stages.balance_pose(c.model, c.data, c.qpos, joints)
        elif mode == "posture":
            joints = [j for j in stages.POSTURE_JOINTS if j not in locked]
            c.qpos, _ = stages.balance_pose(c.model, c.data, c.qpos, joints, ctrl_cap=0.8, margin_target=0.006)
        else:
            c.qpos, _ = stance.find_stance(c.model, c.data, locked=locked, hold=c.qpos)

        m = c.stage1_metrics()
        gate = check_stage1(m)
        c.stage_passed["stage1"] = gate["pass"]
        c.reset_controller()
        after = c.joint_changes_deg()
        delta = {k: round(after.get(k, 0.0) - before.get(k, 0.0), 1) for k in set(before) | set(after)}
        delta = {k: v for k, v in sorted(delta.items()) if abs(v) >= 0.1}
        change = c.raise_change(mode, f"fix_stance(mode={mode})", delta) if delta else None
        c.record(self.actor, f"fix_stance:{mode}", rationale, params={"mode": mode}, metrics=m, gate=gate)
        return {"stage": 1, "metrics": m, "gate": gate, "change_raised": change,
                "budget_left": BUDGETS["stance_fixes"] - c.counters["stance_fixes"]}

    def try_controller(self, case_id, com, balance, other, motor, rationale):
        c = self.cases[case_id]
        if not c.stage_passed["stage1"]:
            return {"error": "stage 1 has not passed: no controller can fix an unbalanced or infeasible pose"}
        mult = {"com": com, "balance": balance, "other": other, "motor": motor}
        if any(not (0.05 <= v <= 20) for v in mult.values()):
            return {"error": "multipliers must be between 0.05 and 20"}
        if (err := self._budget(c, "controller_tries")):
            return err
        self._ensure_tolerances(c)
        tol = tuning.scaled(c.model, c.tol0, **mult)
        try:
            AB = stages.linearize(c.model, c.data, c.qpos, c.ctrl0)
            K = tuning.controller_from(c.model, c.data, c.qpos, tol, AB)
        except (np.linalg.LinAlgError, ValueError) as e:
            gate2 = {"pass": False, "failed": ["riccati_failed"]}
            c.record(self.actor, "try_controller", rationale, params=mult, gate=gate2)
            return {"stage": 2, "gate": gate2, "detail": str(e)[:200]}
        c.multipliers, c.K = mult, K
        c.stage_passed["stage2"], c.stage_passed["stage3"] = True, False
        diag = tuning.screen(c.model, c.data, c.qpos, c.ctrl0, K, noise_scale=c.spec.acceptance["disturbance"])
        c.counters["simulations"] += len(tuning.TRY_SEEDS) + 1
        out = {"stage": 2, "multipliers": mult, "tolerances": tuning.describe(c.model, tol),
               "quick_check": diag, "budget_left": BUDGETS["controller_tries"] - c.counters["controller_tries"]}
        c.record(self.actor, "try_controller", rationale, params=mult, metrics=diag,
                 gate={"pass": True, "failed": []})
        return out

    def tune(self, case_id, rationale, grid=None):
        c = self.cases[case_id]
        if not c.stage_passed["stage1"]:
            return {"error": "stage 1 has not passed: no controller can fix an unbalanced or infeasible pose"}
        g = dict(tuning.DEFAULT_GRID)
        if grid:
            bad = [k for k in grid if k not in g or not isinstance(grid[k], list) or not 1 <= len(grid[k]) <= 3]
            if bad:
                return {"error": f"grid keys must be com/balance/other/motor with 1-3 values each; bad: {bad}"}
            g.update(grid)
        if (err := self._budget(c, "tunes")):
            return err
        self._ensure_tolerances(c)
        res = tuning.tune(c.model, c.data, c.qpos, c.ctrl0, c.tol0, grid=g, noise_scale=c.spec.acceptance["disturbance"])
        c.counters["simulations"] += res["simulations"]
        if res["best"]:
            c.multipliers = res["best"]
            AB = stages.linearize(c.model, c.data, c.qpos, c.ctrl0)
            c.K = tuning.controller_from(c.model, c.data, c.qpos, tuning.scaled(c.model, c.tol0, **c.multipliers), AB)
            c.stage_passed["stage2"], c.stage_passed["stage3"] = True, False
        res["adopted"] = c.multipliers if res["best"] else None
        res["budget_left"] = BUDGETS["tunes"] - c.counters["tunes"]
        c.record(self.actor, "tune", rationale, params={"grid": g},
                 metrics={"adopted": res["adopted"], "ranking": res["ranking"][:2],
                          "simulations": res["simulations"]},
                 gate={"pass": bool(res["best"]), "failed": [] if res["best"] else ["no_config_passed_screen"]})
        return res

    def evaluate(self, case_id, rationale):
        c = self.cases[case_id]
        if not c.stage_passed["stage1"]:
            return {"error": "stage 1 has not passed"}
        if c.K is None:
            return {"error": "no controller adopted yet; call try_controller or tune first"}
        if (err := self._budget(c, "evaluations")):
            return err
        seeds = judge_seeds(c.counters["evaluations"])   # fresh seeds for every judgement
        m = stages.evaluate(c.model, c.data, c.qpos, c.ctrl0, c.K, seeds, c.spec.acceptance["disturbance"])
        m["seeds"] = f"{seeds[0]}-{seeds[-1]}"
        c.counters["simulations"] += len(seeds) + 1
        gate = check_stage3(m, c.spec.acceptance)
        c.stage_passed["stage3"] = gate["pass"]
        c.record(self.actor, "evaluate", rationale, params={"multipliers": c.multipliers}, metrics=m, gate=gate)
        return {"stage": 3, "multipliers": c.multipliers, "metrics": m, "gate": gate,
                "budget_left": BUDGETS["evaluations"] - c.counters["evaluations"]}

    def get_history(self, case_id):
        c = self.cases[case_id]
        return {"steps": c.log, "changes": c.changes, "stage_passed": c.stage_passed,
                "counters": c.counters, "budgets": BUDGETS}

    def finish(self, case_id, status, summary, rationale):
        if case_id in self.rejected:
            if status != "unsupported":
                return {"error": "this request was rejected at create_case; finish it with status='unsupported'"}
            import json
            folder = RESULTS / "cases" / self.actor / case_id
            folder.mkdir(parents=True, exist_ok=True)
            r = self.rejected[case_id]
            (folder / "manifest.json").write_text(json.dumps(
                {"case_id": case_id, "status": "unsupported", "summary": summary, "rejected": r,
                 "steps": [{"actor": self.actor, "action": "finish", "rationale": rationale}]}, indent=2))
            (folder / "report.md").write_text(f"# {case_id}\n\n**Request:** {r['request']}\n\n"
                                              f"**Status:** `unsupported`\n\n{r['reason']}.\n\n{summary}\n")
            return {"status": "unsupported", "saved_to": str(folder)}
        c = self.cases[case_id]
        if status == "unsupported":
            return {"error": "the case was created, so it is supported; finish with 'passed' or 'no_solution_found'"}
        if status == "passed":
            if not c.stage_passed["stage3"]:
                return {"error": "cannot finish as passed: stage 3 has not passed"}
            status = "passed_with_changes" if c.changes else "passed"
        elif status != "no_solution_found":
            return {"error": "status must be 'passed', 'no_solution_found' or 'unsupported'"}
        c.status, c.summary = status, summary
        c.record(self.actor, "finish", rationale, params={"status": status})
        folder = c.save(RESULTS / "cases" / self.actor / case_id)
        return {"status": status, "saved_to": str(folder)}
