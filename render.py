"""Render saved cases as videos and images, for the README and the debrief.

    python render.py compare payload_1kg_3_5x      # baseline vs agent, side by side, same seed
    python render.py compare payload_1kg_3_5x --agent-case payload_1kg_3_5x_run3   # a specific agent run
    python render.py pose payload_3kg              # tutorial pose vs fixed pose (still image)
    python render.py run payload_3kg --actor agent # one controller, one seed

Rebuilds a finished case from results/cases/<actor>/<case>/ (manifest + final_state.npz)
and re-runs one seed with the same control law and disturbance as stage 3.
Writes MP4 (for presenting), GIF (for GitHub) and PNG stills (for the PDF) to results/media/.
"""
import argparse
import json

import numpy as np
import mujoco
from PIL import Image, ImageDraw

from harness import stages
from harness.case import JUDGE_START, JUDGE_BLOCK
from harness.paths import MODEL_XML, RESULTS

W, H = 400, 320          # size of one panel
FPS = 15
LOADED = (0.15, 0.55, 1.0, 1.0)  # blue: the body carrying the payload (the model itself is orange)
MEDIA = RESULTS / "media"


def load(case, actor):
    folder = RESULTS / "cases" / actor / case
    manifest = json.loads((folder / "manifest.json").read_text())
    state = np.load(folder / "final_state.npz")
    spec = manifest["spec"]
    model, data = stages.build_variant(str(MODEL_XML), spec["payload_kg"], spec["payload_body"])
    if spec["payload_kg"]:
        body = model.body(spec["payload_body"]).id
        for g in range(model.ngeom):
            if model.geom_bodyid[g] == body:
                model.geom_rgba[g] = LOADED
    return {"model": model, "data": data, "manifest": manifest, "qpos": state["qpos"],
            "ctrl0": state["ctrl0"], "K": state["K"],
            "disturbance": spec["acceptance"]["disturbance"], "actor": actor}


def make_camera(model, data, azimuth=135, distance=2.6, elevation=-12):
    cam = mujoco.MjvCamera()
    mujoco.mjv_defaultFreeCamera(model, cam)
    cam.azimuth, cam.distance, cam.elevation = azimuth, distance, elevation
    cam.lookat[:] = data.subtree_com[model.body("torso").id]
    return cam


def rollout(c, seed, render=True, after_fall=1.5):
    """Re-run one seed. Returns frames (if render) and the fall time (None = stayed up)."""
    m, d = c["model"], c["data"]
    noise = stages.make_noise(m, seed, c["disturbance"])
    dt = m.opt.timestep
    every = max(1, round(1 / (FPS * dt)))
    torso, foot = m.body("torso").id, m.body(stages.STANCE_FOOT).id
    upright = stages.UPRIGHT_FRAC * c["qpos"][2]
    mujoco.mj_resetData(m, d)
    d.qpos = c["qpos"]
    mujoco.mj_forward(m, d)
    renderer = mujoco.Renderer(m, H, W) if render else None
    cam = make_camera(m, d)
    dq = np.zeros(m.nv)
    frames, fell = [], None
    for step in range(len(noise)):
        mujoco.mj_differentiatePos(m, dq, 1, c["qpos"], d.qpos)
        d.ctrl = c["ctrl0"] - c["K"] @ np.hstack((dq, d.qvel)) + noise[step]
        com_dist = np.linalg.norm(d.subtree_com[torso][:2] - d.xpos[foot][:2])
        if fell is None and (d.qpos[2] < upright or com_dist > stages.COM_FALL_DIST):
            fell = step * dt
        if fell is not None and step * dt > fell + after_fall:
            break
        mujoco.mj_step(m, d)
        if render and step % every == 0:
            cam.lookat[:] = 0.9 * cam.lookat + 0.1 * d.subtree_com[torso]   # follow smoothly
            renderer.update_scene(d, cam)
            frames.append(renderer.render().copy())
    return frames, fell


def _font(size=15):
    from PIL import ImageFont
    for name in ("DejaVuSans.ttf", "Arial.ttf", "/System/Library/Fonts/Supplemental/Arial.ttf", "Helvetica.ttc"):
        try:
            return ImageFont.truetype(name, size)
        except OSError:
            pass
    return ImageFont.load_default()


def label(frame, lines, size=15):
    img = Image.fromarray(frame)
    draw = ImageDraw.Draw(img)
    font = _font(size)
    for i, text in enumerate(lines):
        draw.text((12, 10 + (size + 5) * i), text, font=font, fill=(255, 255, 255), stroke_width=2, stroke_fill=(0, 0, 0))
    return np.asarray(img)


def save(frames, name):
    MEDIA.mkdir(parents=True, exist_ok=True)
    import mediapy
    mediapy.write_video(MEDIA / f"{name}.mp4", frames, fps=FPS)
    gif = [Image.fromarray(f).convert("P", palette=Image.ADAPTIVE, colors=128) for f in frames[::2]]
    gif[0].save(MEDIA / f"{name}.gif", save_all=True, append_images=gif[1:],
                duration=int(2000 / FPS), loop=0, optimize=True)
    print(f"wrote {MEDIA / name}.mp4 and .gif ({len(frames)} frames)")


def describe(c):
    mult = c["manifest"]["final_controller"]["multipliers"] or {}
    return ", ".join(f"{k} {v}x" for k, v in mult.items())


def cmd_run(case, actor, seed):
    c = load(case, actor)
    frames, fell = rollout(c, seed)
    status = "stays up" if fell is None else f"falls at {fell:.1f} s"
    frames = [label(f, [f"{actor}: {describe(c)}", f"seed {seed}, {c['disturbance']}x disturbance: {status}"])
              for f in frames]
    save(frames, f"{case}_{actor}_seed{seed}")


def cmd_compare(case, seed, agent_case=None):
    a, b = load(case, "baseline"), load(agent_case or case, "agent")
    if seed is None:   # first judge seed where the baseline falls and the agent stays up
        seeds = range(JUDGE_START, JUDGE_START + JUDGE_BLOCK)
        seed = next((s for s in seeds
                     if rollout(a, s, render=False)[1] is not None and rollout(b, s, render=False)[1] is None),
                    JUDGE_START)
        print("using seed", seed)
    panels = []
    for c, who in ((a, "baseline (fixed grid)"), (b, "agent")):
        frames, fell = rollout(c, seed)
        status = "stays up" if fell is None else f"falls at {fell:.1f} s"
        panels.append([label(f, [who, describe(c), status]) for f in frames])
    n = max(len(p) for p in panels)
    panels = [p + [p[-1]] * (n - len(p)) for p in panels]          # hold the last frame
    frames = [np.concatenate([p[i] for p in panels], axis=1) for i in range(n)]
    save(frames, f"{case}_compare_seed{seed}")
    # a still of the last frame, for documents that can't show animation (e.g. PDF)
    still = MEDIA / f"{case}_compare_seed{seed}_end.png"
    Image.fromarray(frames[-1]).save(still)
    print("wrote", still)


def cmd_pose(case, actor):
    c = load(case, actor)
    m, d = c["model"], c["data"]
    mujoco.mj_resetDataKeyframe(m, d, m.key(stages.KEYFRAME).id)
    before = d.qpos.copy()
    renderer = mujoco.Renderer(m, H, W)
    images = []
    changes = [ch for ch in c["manifest"]["changes"] if ch["severity"] != "info"] or c["manifest"]["changes"]
    biggest = max(changes, key=lambda ch: ch["max_change_deg"]) if changes else None
    after = ["After the harness's fixes"]
    if biggest:
        joint, deg = max(biggest["joint_changes_deg"].items(), key=lambda kv: abs(kv[1]))
        after.append(f"loaded arm brought in {abs(deg):.0f} degrees" if "right" in joint and "shoulder" in joint
                     else f"largest change: {joint.replace('_', ' ')}, {abs(deg):.0f} degrees")
    kg = c["manifest"]["spec"]["payload_kg"]
    for q, title in ((before, [f"Tutorial pose with a {kg:g} kg tool"]), (c["qpos"], after)):
        mujoco.mj_resetData(m, d)
        d.qpos = q
        mujoco.mj_forward(m, d)
        renderer.update_scene(d, make_camera(m, d, azimuth=160, distance=2.4, elevation=-8))
        images.append(label(renderer.render().copy(), title))
    MEDIA.mkdir(parents=True, exist_ok=True)
    path = MEDIA / f"{case}_pose.png"
    Image.fromarray(np.concatenate(images, axis=1)).save(path)
    print("wrote", path)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("compare"); s.add_argument("case"); s.add_argument("--seed", type=int)
    s.add_argument("--agent-case", default=None, help="agent run to show, if not the same id as the case")
    s = sub.add_parser("pose"); s.add_argument("case"); s.add_argument("--actor", default="agent")
    s = sub.add_parser("run"); s.add_argument("case"); s.add_argument("--actor", default="agent")
    s.add_argument("--seed", type=int, default=JUDGE_START)
    a = p.parse_args()
    if a.cmd == "compare":
        cmd_compare(a.case, a.seed, a.agent_case)
    elif a.cmd == "pose":
        cmd_pose(a.case, a.actor)
    else:
        cmd_run(a.case, a.actor, a.seed)


if __name__ == "__main__":
    main()
