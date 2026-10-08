# LQR humanoid harness

An agent-driven harness around the [MuJoCo LQR tutorial](https://github.com/google-deepmind/mujoco/blob/main/python/LQR.ipynb): a humanoid balancing on one leg. Given a design request in plain language ("add a 3 kg tool to the right hand"), it returns a controller validated on unseen disturbances, or explains why it couldn't find one.

**Start here:** [DESIGN.md](DESIGN.md), the design document (also sent as a PDF). The manual run and the experiments behind its numbers are in [tutorial/friction_log.md](tutorial/friction_log.md).

The code is a spike that tests one assumption: an LLM working only from gate metrics can route failures to the right stage and find fixes a fixed script can't, while guardrails in code keep every result valid. It runs four cases through the agent and a rule-based baseline: the unchanged tutorial, a 3 kg payload, a 1 kg payload at 3.5 times the disturbance (run three times, as `payload_1kg_3_5x`, `_run2` and `_run3`), and a 5 kg payload with the arm locked (results in `results/summary.md`).

## Layout

```
DESIGN.md          the design document
docs/              pipeline and data-model diagrams (make_diagrams.py redraws them)
tutorial/          friction log from the manual run
harness/
  stages.py        deterministic stages: equilibrium, feasibility, pose fixes, LQR, simulation
  stance.py        stance search for a model with no starting pose
  tuning.py        controller from model-derived tolerances (Bryson's rule), tuning search
  case.py          run state, gates, change tracking, manifest and report
  tools.py         the tools the agent and the baseline act through; guardrails enforced here
  agent.py         the agent via the Anthropic API
  agent_cc.py      the same agent via Claude Code, tools served over MCP by mcp_server.py
  baseline.py      rule-based baseline on the same tools, no LLM
  paths.py         repository paths
run_suite.py       runs the cases for the agent and/or the baseline
render.py          videos and images of saved cases
results/
  cases/           per case and actor: manifest, report, transcript, final state
  media/           baseline-vs-agent GIF and the images in DESIGN.md
  summary.md       one row per case and actor
```

## Setup

```bash
git clone https://github.com/google-deepmind/mujoco.git mujoco-src --depth 1   # the humanoid model
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
```

Python 3.10 or newer. For the agent, either set `ANTHROPIC_API_KEY` or sign in to Claude Code (`claude auth login`).

## Run

```bash
python run_suite.py --actor baseline                        # all cases, baseline only, no LLM needed
python run_suite.py --actor both --runtime claude-code      # agent and baseline
python -m harness.agent_cc "Add a 3 kg tool to the right hand"   # one request
python render.py compare payload_1kg_3_5x --agent-case payload_1kg_3_5x_run3   # baseline-vs-agent video
```
