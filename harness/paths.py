from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
MODEL_XML = REPO / "mujoco-src" / "model" / "humanoid" / "humanoid.xml"
RESULTS = REPO / "results"
