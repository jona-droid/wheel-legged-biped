"""Generates ../wiring.svg for the wheel-legged biped (python make_wiring.py). No dependencies."""
import os
W, H = 1240, 820
out = []
add = out.append
def box(x, y, w, h, title, sub="", fill="#f4f6fa", dash=False):
    d = ' stroke-dasharray="6 4"' if dash else ""
    add(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="8" fill="{fill}" stroke="#334"{d} stroke-width="2"/>')
    add(f'<text x="{x+w/2}" y="{y+22}" class="t">{title}</text>')
    if sub: add(f'<text x="{x+w/2}" y="{y+h-12}" class="s">{sub}</text>')
def pin(x, y, label, side):
    add(f'<circle cx="{x}" cy="{y}" r="4" fill="#334"/>')
    a = "start" if side == "L" else "end"
    add(f'<text x="{x+8 if side=="L" else x-8}" y="{y+4}" class="p" text-anchor="{a}">{label}</text>')
def wire(pts, color, dash=False, width=3):
    d = ' stroke-dasharray="7 5"' if dash else ""
    add(f'<polyline points="{" ".join(f"{x},{y}" for x, y in pts)}" fill="none" stroke="{color}" stroke-width="{width}" stroke-linejoin="round"{d}/>')
def note(x, y, text, cls="n", anchor="start"):
    add(f'<text x="{x}" y="{y}" class="{cls}" text-anchor="{anchor}">{text}</text>')
def res(x, y1, y2, label):
    add(f'<rect x="{x-7}" y="{y1}" width="14" height="{y2-y1}" fill="#fff" stroke="#334" stroke-width="2"/>')
    note(x + 12, (y1 + y2) / 2 + 4, label, "p")

RED, BLK, ORG, BLU, GRN, PUR = "#d62828", "#111", "#f08c00", "#1971c2", "#2b8a3e", "#7048e8"

box(30, 60, 190, 120, "2S 18650 pack", "7.4 V (6.6-8.4), with switch")
box(300, 40, 190, 120, "MP1584EN buck", "SET TO 5.0 V FIRST")
box(300, 200, 190, 120, "UBEC 6 V / 5 A+", "servo power only")
box(600, 40, 240, 560, "ESP32-S3", "DevKitC-1", fill="#eef4ff")
box(960, 40, 230, 175, "GY-521 (MPU6050)", "flat on the deck, X arrow along the axle")
box(960, 250, 230, 330, "4x MG996R", "")
box(300, 440, 190, 200, "MX1508", "wheel motor driver")

# servo connectors
for i, (y, n) in enumerate(((300, "right FRONT (RF)"), (370, "right REAR (RR)"), (440, "left FRONT (LF)"), (510, "left REAR (LR)"))):
    add(f'<rect x="1045" y="{y-18}" width="135" height="44" rx="5" fill="#fff" stroke="#334"/>')
    note(1112, y - 3, n, "p", "middle")
    note(1112, y + 15, "orange / red / brown", "s2", "middle")
    pin(1045, y - 8, "", "L"); pin(1045, y + 2, "", "L"); pin(1045, y + 12, "", "L")

for cy, name in ((560, "LEFT wheel"), (625, "RIGHT wheel")):
    add(f'<circle cx="120" cy="{cy}" r="24" fill="#ddd" stroke="#334" stroke-width="2"/>')
    note(120, cy + 5, "M", "p", "middle")
    note(30, cy - 30, name + " (N20)", "p")

# pins
pin(220, 95, "+", "R"); pin(220, 145, "−", "R")
pin(300, 75, "IN+", "L"); pin(300, 125, "IN−", "L"); pin(490, 75, "OUT+", "R"); pin(490, 125, "OUT−", "R")
pin(300, 235, "IN+", "L"); pin(300, 285, "IN−", "L"); pin(490, 235, "+6V", "R"); pin(490, 285, "GND", "R")
for y, l in ((75, "5V"), (125, "GND"), (262, "GPIO1"), (470, "GPIO4"), (500, "GPIO5"), (530, "GPIO6"), (560, "GPIO7")):
    pin(600, y, l, "L")
for y, l in ((75, "3V3"), (125, "GND"), (160, "GPIO8"), (190, "GPIO9"), (292, "GPIO10"), (362, "GPIO11"), (432, "GPIO12"), (502, "GPIO13")):
    pin(840, y, l, "R")
for y, l in ((75, "VCC"), (125, "GND"), (160, "SDA"), (190, "SCL")):
    pin(960, y, l, "L")
for y, l in ((470, "IN1"), (500, "IN2"), (530, "IN3"), (560, "IN4")):
    pin(490, y, l, "R")
pin(300, 470, "+", "L"); pin(300, 500, "−", "L")
for y, l in ((550, "A1"), (575, "A2"), (610, "B1"), (635, "B2")):
    pin(300, y, l, "L")

# battery distribution
wire([(220, 95), (260, 95), (260, 75), (300, 75)], RED)
wire([(260, 95), (260, 235), (300, 235)], RED)
wire([(260, 235), (260, 470), (300, 470)], RED)
wire([(220, 145), (245, 145), (245, 125), (300, 125)], BLK)
wire([(245, 145), (245, 285), (300, 285)], BLK)
wire([(245, 285), (245, 500), (300, 500)], BLK)
# buck -> ESP32 (power jumper)
wire([(490, 75), (600, 75)], RED, dash=True, width=4)
note(545, 66, "POWER JUMPER", "w", "middle")
wire([(490, 125), (600, 125)], BLK)
# battery sense divider
wire([(260, 95), (270, 95)], RED)
wire([(260, 180), (560, 180), (560, 195)], RED, width=2)
res(560, 195, 240, "22k")
wire([(560, 240), (560, 300)], PUR, width=2)
wire([(560, 262), (600, 262)], PUR, width=2)
res(560, 300, 345, "10k")
wire([(560, 345), (560, 395), (245, 395)], BLK, width=2)
note(505, 380, "battery sense", "s2")
# servo rail
wire([(490, 235), (1000, 235), (1000, 513)], RED)
for y in (302, 372, 442, 512):
    wire([(1000, y - 10), (1045, y - 10 + 10)], RED, width=2)
wire([(490, 285), (530, 285), (530, 610), (1020, 610), (1020, 314)], BLK)
for y in (314, 384, 454, 524):
    wire([(1020, y), (1045, y)], BLK, width=2)
add('<rect x="1007" y="590" width="40" height="14" fill="#fff" stroke="#334"/>')
note(985, 640, "1000 µF 10 V across the servo rail", "s2")
# servo signals
for gy, sy in ((292, 292), (362, 362), (432, 432), (502, 502)):
    wire([(840, gy), (960, gy), (960, sy), (1045, sy)], ORG, width=2)
# IMU
wire([(840, 75), (960, 75)], RED, width=2)
wire([(840, 125), (960, 125)], BLK, width=2)
wire([(840, 160), (960, 160)], BLU, width=2)
wire([(840, 190), (960, 190)], GRN, width=2)
# MX1508 inputs
for y in (470, 500, 530, 560):
    wire([(490, y), (600, y)], ORG, width=2)
# motors
wire([(300, 550), (160, 550), (144, 555)], "#555", width=2); wire([(300, 575), (160, 575), (144, 568)], "#555", width=2)
wire([(300, 610), (160, 610), (144, 618)], "#555", width=2); wire([(300, 635), (160, 635), (144, 632)], "#555", width=2)

# rules
y0 = 700
for i, t in enumerate([
    "1. Set the MP1584 to 5.0 V and check the UBEC's output (switch/jumper on 6 V) with the multimeter BEFORE connecting anything.",
    "2. Servo red wires go ONLY to the UBEC, never to the ESP32 5V or 3V3. Only the orange signal wires go to the ESP32.",
    "3. Pull the power jumper whenever USB is plugged in. All grounds (battery, buck, UBEC, MX1508, ESP32) are joined.",
    "4. The 2S pack is 8.4 V when full: the 22k/10k divider keeps GPIO1 below 3.3 V. Don't use 10k/10k here."]):
    note(30, y0 + i * 26, t, "r")

svg = f'''<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}" font-family="Helvetica, Arial, sans-serif">
<style>.t{{font-size:17px;font-weight:bold;text-anchor:middle;fill:#223}} .s{{font-size:12px;text-anchor:middle;fill:#556}}
.s2{{font-size:11px;fill:#556}} .p{{font-size:13px;fill:#223}} .n{{font-size:13px;fill:#334}} .w{{font-size:11px;font-weight:bold;fill:#d62828}}
.r{{font-size:14px;fill:#223}}</style>
<rect width="100%" height="100%" fill="#fff"/>
<text x="30" y="28" font-size="20" font-weight="bold" fill="#223">Wheel-legged biped wiring</text>
{chr(10).join(out)}
</svg>'''
here = os.path.dirname(os.path.abspath(__file__))
open(os.path.join(here, "..", "wiring.svg"), "w").write(svg)
print("wrote wiring.svg")
