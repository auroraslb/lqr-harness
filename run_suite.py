"""Run the test cases through the agent and/or the rule-based baseline.

    python run_suite.py --actor both                 # agent via the API + baseline
    python run_suite.py --actor agent --runtime claude-code
    python run_suite.py --actor baseline --cases payload_3kg nominal

Each run writes results/cases/<actor>/<case_id>/ (manifest, report, transcript)
and the suite writes results/summary.md.
"""
import argparse
import json
import time

from harness.paths import RESULTS


# request: what the agent reads. spec: the same request already structured, for the baseline.
CASES = {
    # Nominal: the unchanged tutorial. Only a small ankle trim is needed; a script is enough.
    "nominal": dict(
        request="Balance the robot on its left leg as in the tutorial, with no changes to the design.",
        spec=dict(payload_kg=0.0, use_keyframe=True)),
    # Easy: the fixed pipeline fits the problem, so the script and the agent reach the same answer.
    "payload_3kg": dict(
        request="Add a 3 kg tool to the right hand and keep the robot balancing on its left leg.",
        spec=dict(payload_kg=3.0, use_keyframe=True)),
    # Realistic: the answer lies outside the script's fixed search space.
    "payload_1kg_3_5x": dict(
        request="Add a 1 kg tool to the right hand. The robot must survive 3.5 times the usual disturbance.",
        spec=dict(payload_kg=1.0, use_keyframe=True, acceptance={"disturbance": 3.5})),
    # Stop: the arm must hold the tool where it is, and no allowed fix makes the pose holdable.
    # The right answer is no_solution_found, with the limiting motor.
    "locked_arm_5kg": dict(
        request=("Add a 5 kg tool to the right hand. The right arm must hold the tool exactly where it is now, "
                 "so the right shoulder and elbow may not move."),
        spec=dict(payload_kg=5.0, use_keyframe=True,
                  locked_joints=["shoulder1_right", "shoulder2_right", "elbow_right"])),
}


def run(actor, runtime, names, model, suffix=""):
    rows = []
    for name in names:
        case = CASES[name]
        cid = name + suffix if actor == "agent" else name
        print(f"\n===== {actor}: {cid} =====")
        t0 = time.time()
        if actor == "baseline":
            from harness.baseline import run_baseline
            stats = run_baseline(name, dict(case["spec"], request=case["request"]))
        elif runtime == "claude-code":
            from harness.agent_cc import run_agent_cc
            stats = run_agent_cc(case["request"], case_id=cid, model=model or "sonnet")
        else:
            from harness.agent import run_agent, DEFAULT_MODEL
            stats = run_agent(case["request"], case_id=cid, model=model or DEFAULT_MODEL)
        stats["seconds"] = round(time.time() - t0, 1)
        manifest = RESULTS / "cases" / actor / cid / "manifest.json"
        changes = []
        if manifest.exists():
            m = json.loads(manifest.read_text())
            changes = [f"{c['severity']} {c['kind']} ({c['max_change_deg']} deg)" for c in m.get("changes", [])]
            stats.setdefault("simulations", m.get("counters", {}).get("simulations"))
        rows.append({"case": cid, "actor": actor, **stats, "changes": changes})
    return rows


def write_summary(rows):
    RESULTS.mkdir(parents=True, exist_ok=True)
    path = RESULTS / "summary.json"
    old = json.loads(path.read_text()) if path.exists() else []
    keep = [r for r in old if (r["case"], r["actor"]) not in {(x["case"], x["actor"]) for x in rows}]
    allrows = sorted(keep + rows, key=lambda r: (r["case"], r["actor"]))
    path.write_text(json.dumps(allrows, indent=2))
    lines = ["| Case | Actor | Status | Simulations | LLM calls | Seconds | Changes |", "|---|---|---|---|---|---|---|"]
    for r in allrows:
        lines.append(f"| {r['case']} | {r['actor']} | {r['status']} | {r.get('simulations', '')} | "
                     f"{r.get('llm_calls', '')} | {r.get('seconds', '')} | {'; '.join(r['changes']) or 'none'} |")
    (RESULTS / "summary.md").write_text("\n".join(lines) + "\n")
    print("\n" + "\n".join(lines))


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--actor", choices=["agent", "baseline", "both"], default="both")
    p.add_argument("--runtime", choices=["api", "claude-code"], default="api",
                   help="How the agent's LLM is called: Anthropic API, or Claude Code (subscription).")
    p.add_argument("--cases", nargs="*", default=list(CASES), choices=list(CASES))
    p.add_argument("--model", default=None)
    p.add_argument("--suffix", default="", help="Appended to agent case ids, to keep repeated runs, e.g. _run2.")
    a = p.parse_args()
    actors = ["agent", "baseline"] if a.actor == "both" else [a.actor]
    rows = []
    for actor in actors:
        rows += run(actor, a.runtime, a.cases, a.model, a.suffix)
    write_summary(rows)


if __name__ == "__main__":
    main()
