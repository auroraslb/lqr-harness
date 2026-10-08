"""The LLM agent: reads a design request, drives a case through the tools, finishes it.

This is the only file that talks to an LLM. Everything it can do goes through
tools.Toolbox, so the rules in tools.py (gates, budgets, locked joints,
change-raising) hold whatever the model decides.

Run:  python -m harness.agent "Add a 3 kg tool to the right hand"
Needs ANTHROPIC_API_KEY in the environment.
"""
import argparse
import json
import time

from .tools import TOOL_SCHEMAS, Toolbox
from .paths import RESULTS

DEFAULT_MODEL = "claude-sonnet-5-5"
MAX_TURNS = 30

SYSTEM_PROMPT = """\
You run a simulation harness for a MuJoCo humanoid that must balance on one leg using an LQR controller.
An engineer gives you a design request. Your job: get the design to balance, or explain precisely why it can't.
You act only through the tools. Code computes everything; you decide what to do next and why.

## Pipeline and gates
1. Stage 1, stance: is the pose physically holdable? Gate checks root residual torque (pose truly balanced),
   static motor need (<= 80% of range) and CoM margin (CoM inside the foot by >= 5 mm).
2. Stage 2, controller: build an LQR controller. Its tolerances start from the model (foot size, joint ranges,
   motor ranges); you adjust four multipliers: com, balance, other, motor (<1 tighter, >1 looser).
3. Stage 3, judge: evaluate on 20 unseen disturbance seeds. Must meet the request's acceptance criteria.

## Turning the request into a case (create_case)
- payload_kg / payload_body: extra mass and where (default body lower_arm_right = right forearm/hand).
- stance: which leg. use_keyframe: false if the request says there is no starting pose or the model is new.
- locked_joints: joints that must not move because of the task, e.g. a tool that must stay where it is held
  locks the right arm. Lock nothing the request doesn't constrain. Joint names:
  torso: abdomen_z, abdomen_y, abdomen_x
  right arm: shoulder1_right, shoulder2_right, elbow_right; left arm: shoulder1_left, shoulder2_left, elbow_left
  stance (left) leg: hip_x_left, hip_z_left, hip_y_left, knee_left, ankle_y_left, ankle_x_left
- acceptance: only if the request asks for more than default, e.g. "twice the disturbance" -> {"disturbance": 2}.
- If create_case says the request is unsupported, finish with status "unsupported".

## How to work
- Always check feasibility before any controller work. No controller can fix a pose that fails stage 1.
- Stage 1 fixes: if only root torque fails, try fix_stance mode "trim" (about 1 degree of ankle). If motor need
  or CoM margin fails, trim first, then "posture" (works best after a trim). With no keyframe, use "search".
- If stage 1 still fails and the remaining changes are blocked by locked joints or the budget, no solution was
  found: finish "no_solution_found" and name the limiting actuator or constraint and the numbers.
- Controller: start with try_controller at all multipliers 1. Read the quick check:
  * falls even without noise -> not a tuning problem; re-check stage 1.
  * fails only with noise -> read which actuators saturate, when, and the CoM drift; change one or two
    multipliers at a time with a stated hypothesis, and compare with the previous try.
  * Notes from earlier runs on this robot (verify, don't assume): under heavy disturbance, more motor authority
    (motor > 1) and tighter non-balance joints (other < 1) helped; restricting the motors never did.
- If a few tries don't converge, use tune, ideally with a narrow grid around what your tries suggest.
- Only evaluate once a quick check passes 5/5. Evaluation seeds are never used for choosing.
- Design changes (pose fixes) are applied and raised automatically by severity; mention the important ones
  in your summary.
- Simulations cost time; don't repeat a check whose result you already have.

## Finishing
finish with "passed" (it becomes passed_with_changes automatically if the design changed), "no_solution_found", or
"unsupported". The summary is for an engineer: what was done, what changed and by how much, the final result,
and for failures the stage and the physical reason.
Every tool call needs a short rationale: why this action, now.
"""


def first_message(request, case_id=None):
    msg = f"Design request:\n{request}"
    if case_id:
        msg += f"\n\nUse case_id: {case_id}"
    return msg


def run_agent(request, case_id=None, model=DEFAULT_MODEL, client=None, max_turns=MAX_TURNS, verbose=True):
    """Run one request to completion. Returns the final status, case id and run statistics."""
    if client is None:
        import anthropic
        client = anthropic.Anthropic()
    tools = Toolbox("agent")
    messages = [{"role": "user", "content": first_message(request, case_id)}]
    transcript, usage = [], {"input_tokens": 0, "output_tokens": 0, "llm_calls": 0}
    final, case_id, nudged = None, None, False
    t0 = time.time()

    for _ in range(max_turns):
        resp = client.messages.create(model=model, max_tokens=2000, system=SYSTEM_PROMPT,
                                      tools=TOOL_SCHEMAS, messages=messages)
        usage["llm_calls"] += 1
        usage["input_tokens"] += resp.usage.input_tokens
        usage["output_tokens"] += resp.usage.output_tokens
        messages.append({"role": "assistant", "content": resp.content})

        tool_uses = [b for b in resp.content if b.type == "tool_use"]
        for b in resp.content:
            if b.type == "text" and b.text.strip():
                transcript.append({"role": "assistant", "text": b.text})
                if verbose:
                    print(f"\n[agent] {b.text.strip()}")

        if not tool_uses:
            if final or nudged:
                break
            nudged = True   # the model stopped without finishing: remind it once
            messages.append({"role": "user", "content": "Use the tools to continue, and call finish when done."})
            continue

        results = []
        for b in tool_uses:
            out = tools.call(b.name, dict(b.input))
            transcript.append({"role": "tool", "name": b.name, "input": b.input, "output": out})
            if verbose:
                why = b.input.get("rationale", "")
                print(f"\n[{b.name}] {why}\n  -> {json.dumps(out)[:300]}")
            if b.name in ("create_case", "finish") and "case_id" in b.input:
                case_id = b.input["case_id"]
            if b.name == "finish" and "status" in out:
                final = out["status"]
            results.append({"type": "tool_result", "tool_use_id": b.id, "content": json.dumps(out)})
        messages.append({"role": "user", "content": results})
        if final:
            break

    stats = {"status": final or "did_not_finish", "case_id": case_id, "model": model,
             "seconds": round(time.time() - t0, 1), **usage}
    if case_id in tools.cases:
        stats["simulations"] = tools.cases[case_id].counters["simulations"]
    if case_id:
        folder = RESULTS / "cases" / "agent" / case_id
        folder.mkdir(parents=True, exist_ok=True)
        (folder / "transcript.json").write_text(json.dumps(
            {"request": request, "stats": stats, "transcript": transcript}, indent=2, default=str))
    return stats


def main():
    p = argparse.ArgumentParser(description="Run the LLM agent on one design request.")
    p.add_argument("request", help="The design request, in plain language.")
    p.add_argument("--case-id", default=None)
    p.add_argument("--model", default=DEFAULT_MODEL)
    args = p.parse_args()
    stats = run_agent(args.request, case_id=args.case_id, model=args.model)
    print("\n" + json.dumps(stats, indent=2))


if __name__ == "__main__":
    main()
