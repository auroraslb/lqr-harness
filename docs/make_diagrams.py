"""Draws the two diagrams in DESIGN.md as SVG and PNG.

    python docs/make_diagrams.py      (needs: pip install cairosvg)
Edit the text or positions below and rerun.
"""
from pathlib import Path

OUT = Path(__file__).parent
FONT = "Inter, Helvetica, Arial, sans-serif"

C = {
    "ink": "#1f2430", "muted": "#5b6170", "line": "#7a8090",
    "agent_bg": "#efe9fb", "agent_box": "#ffffff", "agent_edge": "#7a5cc7",
    "code_bg": "#eaf1f8", "code_box": "#ffffff", "code_edge": "#3f6fa3",
    "guard_bg": "#fdf3e3", "guard_box": "#fffaf1", "guard_edge": "#c58a1b",
    "pass": "#2e8b57", "fail": "#c0392b",
}


class Svg:
    def __init__(self, w, h):
        self.w, self.h, self.parts = w, h, []

    def add(self, s):
        self.parts.append(s)

    def rect(self, x, y, w, h, fill, stroke, rx=10, sw=1.5, dash=None):
        d = f' stroke-dasharray="{dash}"' if dash else ""
        self.add(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="{rx}" fill="{fill}" stroke="{stroke}" stroke-width="{sw}"{d}/>')

    def text(self, x, y, lines, size=14, weight=400, color=None, anchor="middle", lh=1.3):
        color = color or C["ink"]
        if isinstance(lines, str):
            lines = [lines]
        out = []
        for i, ln in enumerate(lines):
            w, s = weight, ln
            if s.startswith("**"):
                w, s = 650, s.strip("*")
            s = s.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")
            out.append(f'<text x="{x}" y="{y + i * size * lh}" font-family="{FONT}" font-size="{size}" '
                       f'font-weight="{w}" fill="{color}" text-anchor="{anchor}">{s}</text>')
        self.add("".join(out))

    def box(self, x, y, w, h, lines, kind, size=14):
        fill, edge = C[f"{kind}_box"], C[f"{kind}_edge"]
        self.rect(x, y, w, h, fill, edge)
        n = len(lines)
        top = y + h / 2 - (n - 1) * size * 1.3 / 2 + size * 0.35
        self.text(x + w / 2, top, lines, size=size)

    def diamond(self, cx, cy, r, lines, size=13):
        pts = f"{cx},{cy - r} {cx + r * 1.25},{cy} {cx},{cy + r} {cx - r * 1.25},{cy}"
        self.add(f'<polygon points="{pts}" fill="#ffffff" stroke="{C["code_edge"]}" stroke-width="1.8"/>')
        n = len(lines)
        self.text(cx, cy - (n - 1) * size * 1.25 / 2 + size * 0.35, lines, size=size, lh=1.25)

    def arrow(self, pts, color=None, dash=None, label=None, lpos=None, lanchor="middle", lcolor=None, sw=1.6):
        color = color or C["line"]
        d = " ".join(f"{'M' if i == 0 else 'L'}{x},{y}" for i, (x, y) in enumerate(pts))
        da = f' stroke-dasharray="{dash}"' if dash else ""
        mid = {"#7a8090": "a", C["pass"]: "p", C["fail"]: "f", C["agent_edge"]: "g", C["guard_edge"]: "o", C["code_edge"]: "c"}.get(color, "a")
        self.add(f'<path d="{d}" fill="none" stroke="{color}" stroke-width="{sw}"{da} marker-end="url(#m{mid})"/>')
        if label:
            lx, ly = lpos
            self.text(lx, ly, label, size=12, color=lcolor or color, anchor=lanchor)

    def save(self, name):
        markers = "".join(
            f'<marker id="m{k}" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto-start-reverse">'
            f'<path d="M0,0 L10,5 L0,10 z" fill="{v}"/></marker>'
            for k, v in {"a": C["line"], "p": C["pass"], "f": C["fail"], "g": C["agent_edge"], "o": C["guard_edge"], "c": C["code_edge"]}.items())
        svg = (f'<svg xmlns="http://www.w3.org/2000/svg" width="{self.w}" height="{self.h}" viewBox="0 0 {self.w} {self.h}">'
               f'<defs>{markers}</defs><rect width="100%" height="100%" fill="#ffffff"/>' + "".join(self.parts) + "</svg>")
        (OUT / f"{name}.svg").write_text(svg)
        try:
            import cairosvg
            cairosvg.svg2png(bytestring=svg.encode(), write_to=str(OUT / f"{name}.png"), scale=2)
        except ImportError:
            print("cairosvg not installed: wrote SVG only")
        print("wrote", OUT / name)


def pipeline():
    s = Svg(1240, 700)
    # two lanes: what the agent decides, what the harness computes
    for y, h, kind, title, sub in ((60, 222, "agent", "AGENT (LLM)", "decides, guided by its prompt"),
                                   (300, 222, "code", "DETERMINISTIC HARNESS", "computes and checks")):
        s.rect(20, y, 1200, h, C[f"{kind}_bg"], "none", rx=14)
        cy = y + h / 2
        s.add(f'<text transform="translate(38,{cy}) rotate(-90)" font-family="{FONT}" font-size="13" font-weight="700" '
              f'fill="{C[kind + "_edge"]}" text-anchor="middle">{title}</text>')
        s.add(f'<text transform="translate(54,{cy}) rotate(-90)" font-family="{FONT}" font-size="11.5" '
              f'fill="{C["muted"]}" text-anchor="middle">{sub}</text>')

    # request
    s.rect(70, 8, 300, 40, "#ffffff", C["line"], rx=20)
    s.text(220, 33, 'Design revision: "Add a 3 kg tool to the right hand"', size=12)
    s.arrow([(142, 48), (142, 103)])

    # agent: decisions, with the rules it is given
    ya, ha = 105, 150
    s.box(75, ya, 135, ha, ["**Plan**", "request → spec:", "payload, locked", "joints, required", "pass rate"], "agent", size=13)
    s.box(235, ya, 225, ha, ["**Choose a pose fix**", "trim, posture or search,", "picked from the check", "that failed"], "agent", size=12.5)
    s.box(550, ya, 320, ha, ["**Tune the controller**", "sets 4 multipliers or a search range,", "one hypothesis at a time", "falls without noise → back to the pose", "5/5 on the quick check → evaluate"], "agent", size=12.5)
    s.box(1075, ya, 140, ha, ["**Finish**", "status and", "the reason"], "agent", size=13)

    # harness: stages and the two gates
    yc, hc = 340, 110
    mid = yc + hc / 2
    s.box(75, yc, 135, hc, ["**Case**", "spec + state,", "budgets used"], "code", size=13)
    s.box(235, yc, 150, hc, ["**Stage 1: pose**", "equilibrium +", "static check", "(no simulation)"], "code", size=13)
    s.box(545, yc, 140, hc, ["**Stage 2:**", "**build + tune**", "LQR + quick check,", "5 tuning seeds"], "code", size=13)
    s.box(705, yc, 140, hc, ["**Stage 3:**", "**held-out judge**", "20 held-out", "seeds"], "code", size=13)
    s.box(1075, yc, 140, hc, ["**Status check**", "against the gates;", "manifest written"], "code", size=13)
    for x, w, lines, crit in ((405, 120, ["**Pose gate**", "can it be held", "still?"],
                               ["passes if: base balanced,", "CoM ≥ 5 mm inside foot,", "motors ≤ 80%"]),
                              (865, 130, ["**Robustness**", "**gate**", "does it stay up?"],
                               ["passes if: ≥ 95% stay up,", "saturation ≤ 1%, stands", "without noise"])):
        s.rect(x, yc + 10, w, hc - 20, "#ffffff", C["code_edge"], rx=6, sw=2.2)
        s.text(x + w / 2, yc + 34, lines, size=12.5, lh=1.25)
        s.text(x + w / 2, yc + hc + 18, crit, size=10.5, color=C["code_edge"], lh=1.2)

    # main flow
    s.arrow([(210, mid), (233, mid)])
    s.arrow([(385, mid), (403, mid)])
    s.arrow([(525, mid), (543, mid)], color=C["pass"])
    s.arrow([(845, mid), (863, mid)])

    # agent -> harness (tool calls) and harness -> agent (results)
    ly = 322
    s.arrow([(142, 255), (142, 338)], color=C["agent_edge"], label="create_case", lpos=(150, ly), lanchor="start")
    s.arrow([(300, 255), (300, 338)], color=C["agent_edge"], label="fix_stance", lpos=(308, ly), lanchor="start")
    s.arrow([(440, yc + 10), (440, 257)], color=C["fail"], label="fail", lpos=(448, ly), lanchor="start")
    s.arrow([(580, 255), (580, 338)], color=C["agent_edge"], label="try / tune", lpos=(572, ly), lanchor="end")
    s.arrow([(650, 338), (650, 257)], color=C["code_edge"])
    s.text(658, ly, "results", size=12, color=C["code_edge"], anchor="start")
    s.arrow([(780, 255), (780, 338)], color=C["agent_edge"], label="evaluate", lpos=(788, ly), lanchor="start")
    s.arrow([(895, yc + 10), (895, 272), (850, 272), (850, 257)], color=C["fail"])
    s.text(903, ly, "fail", size=12, color=C["fail"], anchor="start")
    s.arrow([(965, yc + 10), (965, 290), (1110, 290), (1110, 257)], color=C["pass"])
    s.text(1003, 286, "pass → finish", size=12, color=C["pass"], anchor="start")
    s.arrow([(1180, 255), (1180, 338)], color=C["agent_edge"], label="finish", lpos=(1172, ly), lanchor="end")

    # the agent's own routes: back to the pose, and stop
    s.arrow([(550, 205), (462, 205)], color=C["fail"])
    s.text(506, 172, ["falls without", "noise → pose"], size=11, color=C["fail"], lh=1.15)
    s.add(f'<path d="M362,105 L362,84 L1145,84" fill="none" stroke="{C["fail"]}" stroke-width="1.6" stroke-dasharray="6 4"/>')
    s.add(f'<line x1="760" y1="105" x2="760" y2="84" stroke="{C["fail"]}" stroke-width="1.6" stroke-dasharray="6 4"/>')
    s.arrow([(1145, 84), (1145, 103)], color=C["fail"], dash="6 4")
    s.text(845, 78, "options or budget exhausted → no solution found", size=12, color=C["fail"], anchor="start")

    # bottom band: what is enforced in code, and how a run ends
    by = 545
    s.rect(20, by, 820, 92, C["guard_bg"], "none", rx=14)
    s.text(40, by + 24, "ENFORCED IN CODE", size=12.5, weight=700, color=C["guard_edge"], anchor="start")
    rules = ["requirements and thresholds fixed once set", "locked joints never move",
             "no controller work until the pose gate passes", "held-out seeds never used for tuning",
             "budgets end every run", "invalid actions refused with an error, nothing changes"]
    for k, r in enumerate(rules):
        s.text(40 + (k % 2) * 400, by + 48 + (k // 2) * 18, "· " + r, size=12, anchor="start")
    s.rect(860, by, 360, 92, "#f3f4f6", "none", rx=14)
    s.text(880, by + 24, "RUN ENDS AS", size=12.5, weight=700, color=C["ink"], anchor="start")
    for k, (txt, col) in enumerate((("passed / passed with changes", C["pass"]), ("no solution found", C["fail"]),
                                    ("unsupported (rejected at the plan)", C["muted"]))):
        s.text(880, by + 48 + k * 18, "● " + txt, size=12, color=col, anchor="start")
    s.arrow([(1145, yc + hc), (1145, by - 2)])

    # legend
    lx, ly2 = 75, 668
    for color, label, dash in ((C["agent_edge"], "tool call", None), (C["code_edge"], "diagnostics", None), (C["pass"], "pass", None),
                               (C["fail"], "fail", None), (C["fail"], "stop", "6 4")):
        d = f' stroke-dasharray="{dash}"' if dash else ""
        s.add(f'<line x1="{lx}" y1="{ly2 - 4}" x2="{lx + 26}" y2="{ly2 - 4}" stroke="{color}" stroke-width="2"{d}/>')
        s.text(lx + 32, ly2, label, size=12, color=C["muted"], anchor="start")
        lx += 46 + len(label) * 6.1
    s.text(1210, 668, "Upstream, out of scope: CAD → simulation-ready model", size=12, color=C["muted"], anchor="end")
    s.save("pipeline")


def data_model():
    s = Svg(1240, 320)
    cols = [
        (20, "Spec", "what is asked: set once by the agent's plan", "agent",
         ["**request**   the original text",
          "**design**   payload_kg, payload_body",
          "**task**   stance, use_keyframe, locked_joints",
          "**acceptance**   disturbance, pass_rate"]),
        (450, "Case", "live state: changed only through the tools", "code",
         ["**stage 1**   pose, setpoint, gate result",
          "**stage 2**   model tolerances, 4 multipliers, gain K",
          "**bookkeeping**   stages passed, budgets used",
          "**history**   changes, steps so far"]),
        (880, "Manifest", "audit trail: written by the harness at finish", "guard",
         ["**inputs**   request, spec, gate thresholds",
          "**result**   status, summary, final controller",
          "**changes[]**   kind, severity, joint deltas",
          "**steps[]**   actor, action, reason, metrics, gate",
          "+ report.md, transcript.json"]),
    ]
    for x, title, sub, kind, lines in cols:
        s.rect(x, 20, 340, 285, C[f"{kind}_bg"], C[f"{kind}_edge"], rx=14)
        s.text(x + 20, 54, title, size=20, weight=700, color=C[f"{kind}_edge"], anchor="start")
        s.text(x + 20, 76, sub, size=12.5, color=C["muted"], anchor="start")
        y = 112
        for ln in lines:
            if "   " in ln:
                head, rest = ln.split("   ", 1)
                s.text(x + 22, y, head, size=13.5, anchor="start")
                s.text(x + 22, y + 20, rest, size=13.5, anchor="start")
                y += 52
            else:
                s.text(x + 22, y, ln, size=13.5, anchor="start", color=C["muted"])
                y += 34
    s.arrow([(362, 190), (448, 190)], label="create_case", lpos=(405, 180))
    s.arrow([(792, 190), (878, 190)], label="finish", lpos=(835, 180))
    s.save("data_model")


if __name__ == "__main__":
    pipeline()
    data_model()
