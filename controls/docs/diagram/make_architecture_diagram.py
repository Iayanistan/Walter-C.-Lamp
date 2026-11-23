"""Draws controls/docs/architecture_fr3.svg (and .png if Playwright is installed).

    python3 make_architecture_diagram.py

Edit the box(...) / arrow(...) calls below and rerun. Coordinates are pixels on a 1600 x 1150 canvas.
If Python Playwright is missing, make the PNG with Node instead:
    node render_png.js ../architecture_fr3.svg ../architecture_fr3.png 2
"""
from pathlib import Path
from html import escape

OUT = Path(__file__).resolve().parent.parent
W, H = 1600, 1150

# fill, stroke per subsystem colour
C = {
    "you":    ("#EDE7F6", "#5E35B1"),
    "opt":    ("#FAFAFA", "#9E9E9E"),
    "ros":    ("#E3F2FD", "#1E88E5"),
    "safe":   ("#FFEBEE", "#E53935"),
    "ai":     ("#E8F5E9", "#43A047"),
    "vision": ("#E0F7FA", "#00897B"),
    "fr":     ("#FFF3E0", "#EF6C00"),
    "planb":  ("#FCE4EC", "#D81B60"),
}
INK, MUTED = "#263238", "#546E7A"

out = []
def zone(x, y, w, h, label, color="#B0BEC5", label_right=None):
    out.append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="16" fill="#F7F9FA" stroke="{color}" stroke-width="1.5"/>')
    lx = label_right if label_right is not None else x + w - 14
    out.append(f'<text x="{lx}" y="{y+22}" text-anchor="end" class="zone">{escape(label)}</text>')

def box(x, y, w, h, kind, title, lines=(), dashed=False, tag=None):
    fill, stroke = C[kind]
    dash = ' stroke-dasharray="8 5"' if dashed else ""
    out.append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="10" fill="{fill}" stroke="{stroke}" stroke-width="2"{dash}/>')
    out.append(f'<text x="{x+12}" y="{y+22}" class="title">{escape(title)}</text>')
    if tag:
        tw = 8 * len(tag) + 14
        out.append(f'<rect x="{x+w-tw-8}" y="{y+8}" width="{tw}" height="20" rx="10" fill="{stroke}"/>')
        out.append(f'<text x="{x+w-8-tw/2}" y="{y+22}" text-anchor="middle" class="tag">{escape(tag)}</text>')
    for i, line in enumerate(lines):
        out.append(f'<text x="{x+12}" y="{y+42+i*17}" class="body">{escape(line)}</text>')

def chip(x, y, w, h, kind, text):
    fill, stroke = C[kind]
    out.append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="8" fill="#FFFFFF" stroke="{stroke}" stroke-width="1.5"/>')
    out.append(f'<text x="{x+w/2}" y="{y+h/2+4}" text-anchor="middle" class="chip">{escape(text)}</text>')

def arrow(pts, color=INK, dashed=False, both=False, label=None, at=None, width=2):
    d = "M" + " L".join(f"{x},{y}" for x, y in pts)
    dash = ' stroke-dasharray="7 5"' if dashed else ""
    mid = color.strip("#")
    start = f' marker-start="url(#s{mid})"' if both else ""
    out.append(f'<path d="{d}" fill="none" stroke="{color}" stroke-width="{width}"{dash} marker-end="url(#e{mid})"{start}/>')
    if label:
        lx, ly = at
        tw = 6.4 * len(label) + 10
        out.append(f'<rect x="{lx-tw/2}" y="{ly-9}" width="{tw}" height="17" rx="4" fill="#FFFFFF" opacity="0.95"/>')
        out.append(f'<text x="{lx}" y="{ly+4}" text-anchor="middle" class="lbl" fill="{color}">{escape(label)}</text>')

# ---------- zones ----------
zone(30, 250, 1100, 520, "CONTROL PC · Ubuntu 24.04 · ROS 2 (walter_ws)", C["ros"][1])
zone(1160, 250, 410, 520, "GPU MACHINE", C["ai"][1])
zone(30, 800, 1100, 220, "FAIRINO FR3 CELL (hardware)", C["fr"][1], label_right=990)
zone(1160, 800, 410, 220, "FAIRINO'S OWN SOFTWARE", C["fr"][1])

# ---------- top row: how you give orders ----------
box(50, 115, 170, 95, "you", "You (operator)", ["load glassware,", "confirm manual steps,", "press Halt"])
box(250, 115, 330, 70, "opt", "Procedure Builder app", ["walter_app · browser :8080", ], dashed=True, tag="OPTIONAL")
out.append(f'<text x="262" y="174" class="body">drag skill blocks, export YAML</text>')
box(610, 115, 500, 95, "you", "Procedure file (YAML)", [
    "d4607_flask.yaml: a list of skills (pick, place, shake, pour)",
    "at named stations from stations_fr3.yaml",
    "the default path: works with or without the app"])
box(1160, 115, 410, 95, "vision", "Spill-detection PoC (Windows laptop)", [
    "YOLO vessel boxes + colour masks on a camera",
    "spill seen for N frames → UDP :50555 → HALT"])

# ---------- control PC ----------
box(50, 295, 250, 95, "ros", "Procedure compiler", ["walter_skills: checks skills", "+ stations, writes a task", "file for the runner →"])
box(320, 295, 330, 95, "ros", "D4607 task runner", ["runs the steps in order, keeps the clock", "boil 30 ± 2 s · shake 30 ± 1 s",
                                                  "recovery ladder when a check fails"])
box(670, 295, 440, 95, "safe", "Supervisor (software stop)", ["one mode at a time: IDLE · SCRIPTED · VLA · HALTED",
                                                            "spill event or /walter/halt latches HALTED",
                                                            "only a deliberate reset un-latches it"])
box(50, 425, 250, 95, "vision", "AprilTag placement check", ["is each piece of glassware", "on its marks? (error in mm)"])
box(320, 425, 330, 95, "ros", "MoveIt 2", ["plans the scripted, repeatable moves", "with collision checking"])
box(670, 425, 400, 95, "ai", "GR00T bridge + safety filter", ["sends images + joints to GR00T, then filters",
                                                             "every chunk: joint limits, bench box,",
                                                             "speed ≤ 25 %; anything odd → HOLD"])
fill, stroke = C["ros"]
out.append(f'<rect x="50" y="560" width="1060" height="110" rx="10" fill="{fill}" stroke="{stroke}" stroke-width="2"/>')
out.append('<text x="62" y="582" class="title">ros2_control</text>')
out.append('<text x="170" y="582" class="body">controller_manager runs the arm trajectory controller; one launch switch picks what is on the other end:</text>')
chip(65, 600, 340, 46, "fr", "fairino_hardware ★  real FR3 (Fairino's ROS 2 driver)")
chip(420, 600, 280, 46, "ai", "topic_based  →  Isaac Sim twin")
chip(715, 600, 200, 46, "ros", "mock  →  laptop tests")
out.append('<text x="930" y="620" class="body">hardware:=fairino</text>')
out.append('<text x="930" y="637" class="body">| isaac | mock</text>')

box(140, 690, 280, 66, "fr", "fr_io_gripper", ["gripper open/close → FR3 tool output"])
box(440, 690, 330, 66, "planb", "Plan B: fr_sdk_bridge", ["only if fairino_hardware won't run: stream", ], dashed=True, tag="NOT BUILT")
out.append('<text x="452" y="749" class="body">ServoJ through Fairino\'s Python SDK</text>')
box(790, 690, 320, 66, "ros", "Episode recorder", ["demos → LeRobot datasets for training"])

# ---------- GPU ----------
box(1180, 295, 370, 95, "ai", "GR00T N1.7 policy server", ["fine-tuned on FR3 demos only",
                                                           "in: images + joints + instruction",
                                                           "out: next 16 steps (action chunk)"])
box(1180, 425, 370, 95, "ai", "Fine-tuning (finetune.sh)", ["sim + real demos → new checkpoint",
                                                            "needs a 40 GB+ GPU or a cloud node"])
box(1180, 560, 370, 110, "ai", "Isaac Sim 5.x digital twin", ["FR3 + bench model with virtual cameras",
                                                              "practice runs and cheap demo data",
                                                              "stands in for the real arm"])

# ---------- FR3 cell ----------
box(50, 850, 420, 140, "fr", "FR3 control box (Fairino firmware)", [
    "real servo loops, brakes, e-stop,",
    "collision detection, joint + speed limits",
    "takes joint targets from ROS over Ethernet",
    "the PC only suggests; this box has the final say"])
box(500, 850, 190, 140, "fr", "FR3 arm", ["6 axes", "3 kg payload", "622 mm reach"])
box(720, 850, 180, 140, "fr", "Gripper", ["on the flange", "tool I/O or Modbus", "Robotiq 2F-140", "is the lead pick"])
box(930, 850, 180, 140, "vision", "Cameras + tags", ["scene + wrist (USB)", "AprilTags on stations", "feed bridge, tag", "check and recorder"])
box(1180, 850, 370, 140, "fr", "Fairino WebApp + FRCap", [
    "browser / teach pendant UI on the box:",
    "enable, jog, teach points, tool frame,",
    "safety + collision settings, firmware",
    "FRCap plug-ins = extra pages only;",
    "they can't host ROS 2 or GR00T"])

# ---------- bottom strip ----------
fill, stroke = C["opt"]
out.append(f'<rect x="30" y="1045" width="560" height="85" rx="10" fill="{fill}" stroke="{stroke}" stroke-width="1.5" stroke-dasharray="8 5"/>')
out.append('<text x="44" y="1069" class="title" fill="#616161">Side quest (not wired in): ABB IRB 120 + RobotStudio</text>')
out.append('<text x="44" y="1090" class="body">basic RAPID + one ROS 2 EGM move to show control skills</text>')
out.append('<text x="44" y="1107" class="body">(robot:=irb120). No stations, GR00T or D4607 on it.</text>')

lx, ly = 610, 1045
out.append(f'<rect x="{lx}" y="{ly}" width="960" height="85" rx="10" fill="#FFFFFF" stroke="#CFD8DC"/>')
out.append(f'<text x="{lx+14}" y="{ly+22}" class="title">Legend</text>')
arrow([(lx+90, ly+18), (lx+150, ly+18)])
out.append(f'<text x="{lx+158}" y="{ly+22}" class="body">always used</text>')
arrow([(lx+260, ly+18), (lx+320, ly+18)], color=C["opt"][1], dashed=True)
out.append(f'<text x="{lx+328}" y="{ly+22}" class="body">optional</text>')
arrow([(lx+410, ly+18), (lx+470, ly+18)], color=C["planb"][1], dashed=True)
out.append(f'<text x="{lx+478}" y="{ly+22}" class="body">Plan B (backup control path)</text>')
arrow([(lx+690, ly+18), (lx+750, ly+18)], color=C["ai"][1], dashed=True)
out.append(f'<text x="{lx+758}" y="{ly+22}" class="body">simulation only</text>')
swatches = [("you", "you + your orders"), ("ros", "ROS 2 brains"), ("safe", "safety"),
            ("ai", "AI / GPU"), ("vision", "vision"), ("fr", "Fairino FR3")]
for i, (k, t) in enumerate(swatches):
    sx = lx + 14 + i * 155
    out.append(f'<rect x="{sx}" y="{ly+46}" width="22" height="22" rx="5" fill="{C[k][0]}" stroke="{C[k][1]}" stroke-width="2"/>')
    out.append(f'<text x="{sx+30}" y="{ly+62}" class="body">{t}</text>')

# ---------- arrows ----------
G, R, O, M, T, GR = C["opt"][1], C["safe"][1], C["fr"][1], C["planb"][1], C["vision"][1], C["ai"][1]
arrow([(220, 150), (248, 150)], G, dashed=True)
arrow([(580, 150), (608, 150)], G, dashed=True)
arrow([(220, 195), (608, 195)], label="or write the YAML yourself", at=(415, 195))
arrow([(700, 210), (700, 238), (175, 238), (175, 293)], label="ros2 run walter_skills run", at=(440, 238))
arrow([(300, 342), (318, 342)])
arrow([(650, 342), (668, 342)])
arrow([(340, 390), (340, 407), (175, 407), (175, 423)], label="check", at=(258, 407))
arrow([(485, 390), (485, 423)], label="scripted move", at=(485, 407))
arrow([(620, 390), (620, 407), (760, 407), (760, 423)], label="recover", at=(690, 407))
arrow([(1000, 390), (1000, 423)], R, label="mode gate", at=(1000, 407))
arrow([(485, 520), (485, 558)], label="trajectories", at=(485, 540))
arrow([(870, 520), (870, 558)], label="safe chunks", at=(870, 540))
arrow([(1070, 470), (1148, 470), (1148, 365), (1178, 365)], GR, both=True, label="ZMQ", at=(1148, 418))
arrow([(1160, 170), (1140, 170), (1140, 320), (1112, 320)], R, label="UDP spill_halt", at=(1140, 222))
arrow([(1110, 634), (1178, 634)], GR, dashed=True, both=True, label="sim", at=(1144, 634))
arrow([(1110, 723), (1562, 723), (1562, 472), (1552, 472)], label="datasets", at=(1340, 723))
arrow([(1365, 425), (1365, 392)], GR, label="checkpoint", at=(1365, 407))
arrow([(95, 646), (95, 848)], O, both=True, width=3, label="Ethernet · ServoJ every 8 ms", at=(183, 778))
arrow([(280, 756), (280, 848)], O, label="tool output", at=(280, 818))
arrow([(605, 670), (605, 688)], M, dashed=True)
arrow([(605, 756), (605, 830), (440, 830), (440, 848)], M, dashed=True, label="Plan B", at=(525, 830))
arrow([(470, 920), (498, 920)], O)
arrow([(690, 920), (718, 920)], O)
arrow([(1020, 850), (1020, 758)], T, label="camera images", at=(1020, 790))
arrow([(1180, 975), (1150, 975), (1150, 1005), (260, 1005), (260, 992)], O, label="browser over Ethernet", at=(700, 1005))

markers = []
for col in {INK, G, R, O, M, T, GR}:
    m = col.strip("#")
    markers.append(f'<marker id="e{m}" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto"><path d="M0,0 L10,5 L0,10 z" fill="{col}"/></marker>')
    markers.append(f'<marker id="s{m}" viewBox="0 0 10 10" refX="1" refY="5" markerWidth="7" markerHeight="7" orient="auto"><path d="M10,0 L0,5 L10,10 z" fill="{col}"/></marker>')

svg = f'''<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}">
<defs>{"".join(markers)}
<style>
 text {{ font-family: Inter, "DejaVu Sans", Arial, sans-serif; fill: {INK}; }}
 .h1 {{ font-size: 26px; font-weight: 700; }}
 .sub {{ font-size: 14px; fill: {MUTED}; }}
 .zone {{ font-size: 12px; font-weight: 700; letter-spacing: 1px; fill: {MUTED}; }}
 .title {{ font-size: 15px; font-weight: 700; }}
 .body {{ font-size: 12.5px; fill: #37474F; }}
 .chip {{ font-size: 12.5px; font-weight: 600; }}
 .tag {{ font-size: 11px; font-weight: 700; fill: #FFFFFF; letter-spacing: .5px; }}
 .lbl {{ font-size: 11.5px; font-weight: 600; }}
</style></defs>
<rect width="{W}" height="{H}" fill="#FFFFFF"/>
<text x="30" y="48" class="h1">Walter C. Lamp: system architecture (FAIRINO FR3)</text>
<text x="30" y="74" class="sub">How each subsystem talks to the others. Read top to bottom: your orders → ROS 2 decides → the FR3 control box moves the arm. Updated 2026-11-04.</text>
{chr(10).join(out)}
</svg>'''
(OUT / "architecture_fr3.svg").write_text(svg)
print("wrote", OUT / "architecture_fr3.svg")

try:
    from playwright.sync_api import sync_playwright
    with sync_playwright() as p:
        b = p.chromium.launch()
        pg = b.new_page(viewport={"width": W, "height": H}, device_scale_factor=2)
        pg.set_content(f"<html><body style='margin:0'>{svg}</body></html>")
        pg.screenshot(path=str(OUT / "architecture_fr3.png"), full_page=True)
        b.close()
    print("wrote", OUT / "architecture_fr3.png")
except Exception as e:
    print("PNG skipped:", e)
