"""Rule-based baseline: the same tools, a fixed order, no LLM.

It gets the request already turned into structured fields (it cannot read
text), always fixes the stance in the same order, and always tunes with the
default grid instead of informed tries. It is what a reasonable script would
do, and the yardstick for what the agent adds.
"""
from .tools import Toolbox


def run_baseline(case_id, spec, verbose=True):
    """spec: dict with payload_kg, use_keyframe, and optionally stance, locked_joints, acceptance."""
    tb = Toolbox("baseline")

    def call(name, **args):
        out = tb.call(name, dict(args, case_id=case_id))
        if verbose:
            gate = out.get("gate") or out.get("stage3_gate")
            print(f"[baseline] {name}: {gate if gate else str(out)[:160]}")
        return out

    created = call("create_case", request=spec.get("request", ""), payload_kg=spec.get("payload_kg", 0.0),
                   use_keyframe=spec.get("use_keyframe", True), locked_joints=spec.get("locked_joints", []),
                   stance=spec.get("stance", "left"), acceptance=spec.get("acceptance"),
                   rationale="rule: create the case from the structured spec")
    if "error" in created:
        out = call("finish", status="unsupported", summary=created["error"], rationale="rule: create_case rejected it")
        return _stats(tb, case_id, out)

    # Stage 1: fixed order of fixes
    s1 = call("check_feasibility", rationale="rule: always check stage 1 first")
    order = ["trim", "posture"] if spec.get("use_keyframe", True) else ["search", "trim", "posture"]
    for mode in order:
        if s1["gate"]["pass"]:
            break
        s1 = call("fix_stance", mode=mode, rationale=f"rule: stage 1 failed, next fix is {mode}")
    if not s1["gate"]["pass"]:
        out = call("finish", status="no_solution_found",
                   summary=f"Stage 1 still fails after {', '.join(order)}: {s1['gate']['failed']}; metrics {s1['metrics']}",
                   rationale="rule: no stance fix left")
        return _stats(tb, case_id, out)

    # Stages 2-3: blind grid, then judge
    tuned = call("tune", rationale="rule: tune with the default grid")
    if not tuned.get("best"):
        out = call("finish", status="no_solution_found", summary="No controller in the default grid passed the screen.",
                   rationale="rule: tuning found nothing")
        return _stats(tb, case_id, out)
    judged = call("evaluate", rationale="rule: judge the tuned controller")
    status = "passed" if judged["gate"]["pass"] else "no_solution_found"
    out = call("finish", status=status,
               summary=f"Tuned multipliers {tuned['best']}; judged {judged['metrics']['pass']}/{judged['metrics']['n_seeds']}.",
               rationale="rule: report the judged result")
    return _stats(tb, case_id, out)


def _stats(tb, case_id, out):
    c = tb.cases.get(case_id)
    return {"status": out.get("status", "error"), "case_id": case_id,
            "simulations": c.counters["simulations"] if c else 0, "llm_calls": 0}
