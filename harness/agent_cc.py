"""Run the agent through Claude Code instead of the API (uses a Claude subscription).

Claude Code runs headless (`claude -p`) with:
  - our system prompt replacing its own (it is not a coding assistant here)
  - only the harness tools: built-in tools (shell, files, web) are switched off
  - the harness tools served by mcp_server.py

The transcript is parsed from Claude Code's stream output and saved next to
the case's manifest, like agent.py does.

Run:  python -m harness.agent_cc "Add a 3 kg tool to the right hand"
Needs the `claude` CLI, signed in.
"""
import argparse
import json
import os
import subprocess
import sys
import tempfile
import time
from pathlib import Path

from .agent import SYSTEM_PROMPT, MAX_TURNS, first_message
from .paths import REPO, RESULTS

PREFIX = "mcp__lqr__"


def run_agent_cc(request, case_id=None, model="sonnet", max_turns=MAX_TURNS, verbose=True):
    with tempfile.TemporaryDirectory() as tmp:
        cfg = Path(tmp) / "mcp.json"
        cfg.write_text(json.dumps({"mcpServers": {"lqr": {
            "command": sys.executable, "args": ["-m", "harness.mcp_server"], "cwd": str(REPO),
            # tune can take a minute or two; give tool calls room
            "env": {"MCP_TOOL_TIMEOUT": "600000"}}}}))
        prompt_file = Path(tmp) / "system.txt"
        prompt_file.write_text(SYSTEM_PROMPT)
        cmd = ["claude", "-p", first_message(request, case_id),
               "--system-prompt-file", str(prompt_file),
               "--mcp-config", str(cfg), "--strict-mcp-config",
               "--tools", "",                         # no built-in tools at all
               "--allowedTools", f"{PREFIX}*",        # harness tools run without prompts
               "--output-format", "stream-json", "--verbose",
               "--max-turns", str(max_turns), "--model", model]
        t0 = time.time()
        proc = subprocess.Popen(cmd, cwd=REPO, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
                                env={**os.environ, "MCP_TOOL_TIMEOUT": "600000"})
        transcript, final, cid, llm_turns, result_msg = [], None, case_id, 0, {}
        pending = {}
        for line in proc.stdout:
            try:
                ev = json.loads(line)
            except json.JSONDecodeError:
                continue
            if ev.get("type") == "assistant":
                llm_turns += 1
                for b in ev["message"].get("content", []):
                    if b.get("type") == "text" and b["text"].strip():
                        transcript.append({"role": "assistant", "text": b["text"]})
                        if verbose:
                            print(f"\n[agent] {b['text'].strip()}")
                    elif b.get("type") == "tool_use":
                        name = b["name"].removeprefix(PREFIX)
                        pending[b["id"]] = (name, b.get("input", {}))
                        if name in ("create_case", "finish") and "case_id" in b.get("input", {}):
                            cid = b["input"]["case_id"]
                        if verbose:
                            print(f"\n[{name}] {b.get('input', {}).get('rationale', '')}")
            elif ev.get("type") == "user":
                for b in ev["message"].get("content", []):
                    if isinstance(b, dict) and b.get("type") == "tool_result":
                        name, inp = pending.pop(b.get("tool_use_id"), ("?", {}))
                        content = b.get("content")
                        text = content[0].get("text", "") if isinstance(content, list) and content else str(content)
                        try:
                            out = json.loads(text)
                        except (json.JSONDecodeError, TypeError):
                            out = {"raw": text}
                        transcript.append({"role": "tool", "name": name, "input": inp, "output": out})
                        if name == "finish" and isinstance(out, dict) and "status" in out:
                            final = out["status"]
                        if verbose:
                            print(f"  -> {json.dumps(out)[:300]}")
            elif ev.get("type") == "result":
                result_msg = ev
        proc.wait()
        err = proc.stderr.read()

    stats = {"status": final or "did_not_finish", "case_id": cid, "model": model, "runtime": "claude-code",
             "seconds": round(time.time() - t0, 1), "llm_calls": llm_turns,
             "input_tokens": (result_msg.get("usage") or {}).get("input_tokens"),
             "output_tokens": (result_msg.get("usage") or {}).get("output_tokens")}
    if not final and err:
        stats["stderr"] = err[-500:]
    if cid:
        folder = RESULTS / "cases" / "agent" / cid
        folder.mkdir(parents=True, exist_ok=True)
        manifest = folder / "manifest.json"
        if manifest.exists():
            stats["simulations"] = json.loads(manifest.read_text()).get("counters", {}).get("simulations")
        (folder / "transcript.json").write_text(json.dumps(
            {"request": request, "stats": stats, "transcript": transcript}, indent=2, default=str))
    return stats


def main():
    p = argparse.ArgumentParser(description="Run the agent through Claude Code (subscription).")
    p.add_argument("request")
    p.add_argument("--case-id", default=None)
    p.add_argument("--model", default="sonnet", help="Claude Code model alias or full name")
    args = p.parse_args()
    print("\n" + json.dumps(run_agent_cc(args.request, args.case_id, args.model), indent=2))


if __name__ == "__main__":
    main()
