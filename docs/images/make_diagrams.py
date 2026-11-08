"""make_diagrams.py - draws the pictures in docs/images as plain SVG files, which GitHub shows directly.

    python docs/images/make_diagrams.py
"""
import pathlib

HERE = pathlib.Path(__file__).resolve().parent
FONT = "Segoe UI, Helvetica, Arial, sans-serif"
INK, MUTED, LINE = "#1f2a37", "#5b6776", "#c4cad3"
BOARD, BOARD_DARK = "#00797e", "#005f63"
RED, BLACK, YELLOW, BLUE = "#d32f2f", "#111111", "#c98a00", "#1565c0"
BUTTONS = {"BACK": "#7b3fc4", "SELECT": "#00897b", "DOWN": "#e65100", "UP": "#c2185b"}


class Svg:
    def __init__(self, w, h):
        self.w, self.h = w, h
        self.items = [f'<rect width="{w}" height="{h}" rx="14" fill="#ffffff"/>']

    def rect(self, x, y, w, h, fill="none", stroke=None, rx=6, sw=1.5, dash=None):
        s = f' stroke="{stroke}" stroke-width="{sw}"' if stroke else ""
        d = f' stroke-dasharray="{dash}"' if dash else ""
        self.items.append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="{rx}" fill="{fill}"{s}{d}/>')

    def text(self, x, y, t, size=14, fill=INK, bold=False, anchor="middle", rotate=None):
        b = ' font-weight="600"' if bold else ""
        r = f' transform="rotate({rotate} {x} {y})"' if rotate is not None else ""
        t = t.replace("&", "&amp;").replace("<", "&lt;")
        self.items.append(f'<text x="{x}" y="{y}" font-family="{FONT}" font-size="{size}" fill="{fill}" text-anchor="{anchor}"{b}{r}>{t}</text>')

    def wire(self, points, color, sw=3, dash=None):
        d = f' stroke-dasharray="{dash}"' if dash else ""
        path = "M" + " L".join(f"{x} {y}" for x, y in points)
        self.items.append(f'<path d="{path}" fill="none" stroke="{color}" stroke-width="{sw}" stroke-linejoin="round" stroke-linecap="round"{d}/>')

    def path(self, d, color, sw=3):
        self.items.append(f'<path d="{d}" fill="none" stroke="{color}" stroke-width="{sw}" stroke-linejoin="round" stroke-linecap="round"/>')

    def dot(self, x, y, r=4, fill=INK):
        self.items.append(f'<circle cx="{x}" cy="{y}" r="{r}" fill="{fill}"/>')

    def arrow(self, x1, y1, x2, y2, color=MUTED, both=False, sw=2):
        ends = ' marker-end="url(#a)"' + (' marker-start="url(#b)"' if both else "")
        self.items.append(f'<line x1="{x1}" y1="{y1}" x2="{x2}" y2="{y2}" stroke="{color}" stroke-width="{sw}"{ends}/>')

    def poly_arrow(self, points, color=MUTED, sw=2):
        path = "M" + " L".join(f"{x} {y}" for x, y in points)
        self.items.append(f'<path d="{path}" fill="none" stroke="{color}" stroke-width="{sw}" marker-end="url(#a)"/>')

    def box(self, x, y, w, h, title, lines=(), fill="#f4f6f9", stroke=LINE, title_fill=INK, line_fill=MUTED):
        self.rect(x, y, w, h, fill, stroke, rx=10)
        n = len(lines)
        ty = y + h / 2 - (n * 18) / 2 + 5
        self.text(x + w / 2, ty, title, 16, title_fill, True)
        for i, l in enumerate(lines):
            self.text(x + w / 2, ty + 20 * (i + 1), l, 13, line_fill)

    def save(self, name, label):
        defs = ('<defs><marker id="a" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto">'
                f'<path d="M0 0 L10 5 L0 10 z" fill="{MUTED}"/></marker>'
                '<marker id="b" viewBox="0 0 10 10" refX="1" refY="5" markerWidth="7" markerHeight="7" orient="auto">'
                f'<path d="M10 0 L0 5 L10 10 z" fill="{MUTED}"/></marker></defs>')
        body = "\n".join(self.items)
        (HERE / name).write_text(f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {self.w} {self.h}" role="img" aria-label="{label}">\n{defs}\n{body}\n</svg>\n', encoding="utf-8")


def wiring():
    s = Svg(1180, 780)
    s.text(590, 34, "Wiring: the OLED and four buttons go on the board, and everything else goes over USB-C", 19, INK, True)

    # the board, drawn with its USB port on the left, as in Arduino's own pictures
    bx, by, bw, bh = 300, 250, 580, 260
    s.rect(bx, by, bw, bh, BOARD, BOARD_DARK, rx=12, sw=2)
    s.text(bx + 300, by + 122, "Arduino UNO R4 Minima", 24, "#ffffff", True)
    s.text(bx + 300, by + 148, "RA4M1: 48 MHz Cortex-M4 with FPU, 256 KB flash, 32 KB RAM", 13, "#d7f0f0")
    s.rect(bx - 28, by + 50, 44, 58, "#9aa3ad", "#6c757d", rx=5)
    s.text(bx + 60, by + 84, "USB-C", 13, "#ffffff", True)
    s.rect(bx - 22, by + 170, 40, 62, "#2b2f33", None, rx=4)
    s.text(bx + 62, by + 206, "not used", 12, "#d7f0f0")

    top = ["SCL", "SDA", "AREF", "GND", "13", "12", "11", "10", "9", "8", "7", "6", "5", "4", "3", "2", "1", "0"]
    bottom = ["", "IOREF", "RST", "3V3", "5V", "GND", "GND", "VIN", "A0", "A1", "A2", "A3", "A4", "A5"]
    used = {"GND", "5", "4", "3", "2", "5V", "A4", "A5"}
    tx = {}
    bxs = {}
    s.rect(420, by + 5, 416, 16, "#14181c", None, rx=2)
    s.rect(460, by + bh - 21, 328, 16, "#14181c", None, rx=2)
    for i, name in enumerate(top):
        x = 430 + 22 * i + (14 if i >= 10 else 0)
        tx.setdefault(name, x)
        s.dot(x, by + 13, 4, "#d9b64a")
        on = name in used and not (name == "GND" and i != 3)
        s.text(x + 4, by + 30, ("D" + name) if name.isdigit() else name, 11, "#ffffff" if on else "#8fd0d2", on, "end", -90)
    for i, name in enumerate(bottom):
        x = 470 + 22 * i + (14 if i >= 8 else 0)
        if name and name not in bxs:
            bxs[name] = x
        s.dot(x, by + bh - 13, 4, "#d9b64a")
        on = name in used and i != 6
        if name:
            s.text(x + 4, by + bh - 28, name, 11, "#ffffff" if on else "#8fd0d2", on, "start", -90)
    bxs_gnd = 470 + 22 * 5

    # the laptop
    s.box(30, 268, 190, 120, "Laptop or PC", ["runs the simulator", "and its bridge"], "#eef3f9", "#a9bbd0")
    s.wire([(220, 329), (272, 329)], "#39424e", 7)
    s.text(246, 312, "USB-C", 12, MUTED, True)
    s.text(125, 412, "Data and power: 5 V", 13, MUTED)

    # the buttons: one leg to its pin, the other to GND
    rail = 66
    s.wire([(tx["GND"], by + 13), (tx["GND"], rail), (1058, rail)], BLACK)
    buttons = [("BACK", "5", 520, 238), ("SELECT", "4", 630, 222), ("DOWN", "3", 740, 206), ("UP", "2", 850, 222)]
    for name, pin, x, level in buttons:
        c = BUTTONS[name]
        s.wire([(x, rail), (x, 96)], BLACK)
        s.dot(x, rail, 5, BLACK)
        s.wire([(x, 140), (x, level), (tx[pin], level), (tx[pin], by + 13)], c)
        s.rect(x - 22, 96, 44, 44, "#39424e", "#20262d", rx=5)
        s.dot(x, 118, 13, "#c9ced6")
        s.text(x + 30, 112, name, 14, c, True, "start")
        s.text(x + 30, 130, "D" + pin, 13, MUTED, False, "start")
    s.text(tx["GND"] + 12, 56, "GND: the other leg of every button", 13, BLACK, False, "start")

    # the optional adapter
    s.wire([(tx["1"], by + 13), (tx["1"], 240), (965, 240)], "#6c757d", 2.5, "7 6")
    s.wire([(1058, rail), (1058, 190)], BLACK, 2.5, "7 6")
    s.box(965, 190, 190, 104, "USB-TTL adapter", ["optional telemetry,", "460800 baud"], "#f4f6f9", LINE)
    s.text(1068, 150, "GND", 12, BLACK, True, "start")
    s.text(880, 232, "D1 (TX) to RX", 12, "#6c757d", True, "middle")

    # the OLED
    ox, oy, ow, oh = 470, 590, 300, 150
    s.rect(ox, oy, ow, oh, "#1c3f94", "#142c68", rx=8, sw=2)
    pins = [("GND", 530), ("VCC", 590), ("SCL", 650), ("SDA", 710)]
    for name, x in pins:
        s.dot(x, oy + 12, 5, "#d9b64a")
        s.text(x, oy + 34, name, 13, "#ffffff", True)
    s.rect(ox + 40, oy + 44, ow - 80, 92, "#06080c", "#2b2f33", rx=3)
    s.text(ox + ow / 2, oy + 72, "PC:RUNNING    NRM", 13, "#7fd4ff")
    s.text(ox + ow / 2, oy + 92, "FLYING ALTHOLD", 13, "#ffffff")
    s.text(ox + ow / 2, oy + 112, "Alt 1.50   POS HOLD", 13, "#ffffff")
    s.text(ox + ow + 16, oy + 60, "0.96 inch OLED", 15, INK, True, "start")
    s.text(ox + ow + 16, oy + 82, "SSD1306, 128 x 64, I2C", 13, MUTED, False, "start")
    s.text(ox + ow + 16, oy + 102, "address 0x3C or 0x3D", 13, MUTED, False, "start")

    y0 = by + bh - 13
    gnd, v5, a4, a5 = bxs_gnd, bxs["5V"], bxs["A4"], bxs["A5"]
    # each pair crosses once: a small hop shows the wires don't touch
    s.wire([(530, oy + 12), (530, 556), (gnd, 556), (gnd, y0)], BLACK)
    s.path(f"M590 {oy + 12} L590 540 L{gnd + 8} 540 A8 8 0 0 0 {gnd - 8} 540 L{v5} 540 L{v5} {y0}", RED)
    s.wire([(710, oy + 12), (710, 556), (a4, 556), (a4, y0)], BLUE)
    s.path(f"M650 {oy + 12} L650 540 L{a4 - 8} 540 A8 8 0 0 1 {a4 + 8} 540 L{a5} 540 L{a5} {y0}", YELLOW)

    # the table
    rows = [(RED, "OLED VCC to 5V"), (BLACK, "OLED GND to GND"), (YELLOW, "OLED SCL to A5"), (BLUE, "OLED SDA to A4")]
    s.text(36, 560, "OLED", 15, INK, True, "start")
    for i, (c, t) in enumerate(rows):
        s.wire([(36, 580 + 22 * i), (66, 580 + 22 * i)], c, 4)
        s.text(76, 585 + 22 * i, t, 13, INK, False, "start")
    s.text(36, 688, "Read the labels on your OLED: the pin", 13, MUTED, False, "start")
    s.text(36, 706, "order differs between sellers.", 13, MUTED, False, "start")
    s.text(36, 734, "No resistors: the buttons use the board's", 13, MUTED, False, "start")
    s.text(36, 752, "internal pull-ups.", 13, MUTED, False, "start")
    s.text(790, 740, "It also flies with no OLED or buttons.", 13, MUTED, False, "start")
    s.text(790, 758, "The L LED then shows the status.", 13, MUTED, False, "start")
    s.save("wiring.svg", "Wiring of the OLED and four buttons to the Arduino UNO R4 Minima")


def system():
    s = Svg(1180, 470)
    s.text(590, 34, "Hardware in the loop: real board and real code, with a simulated drone", 19, INK, True)
    s.rect(24, 58, 620, 388, "#f7f9fb", LINE, rx=12)
    s.text(40, 84, "Laptop: the simulator", 14, MUTED, True, "start")
    s.box(48, 104, 240, 120, "Gazebo", ["simulates the drone, its motors", "and its IMU, barometer,", "compass and GPS"], "#e8f1fb", "#a9c6e8")
    s.box(372, 104, 250, 120, "Organisers' bridge", ["hil_bridge_v1.py", "sensors out, motors in"], "#e8f1fb", "#a9c6e8")
    s.arrow(288, 148, 372, 148)
    s.arrow(372, 180, 288, 180)
    s.text(330, 136, "sensors", 12, MUTED)
    s.text(330, 198, "motors", 12, MUTED)
    s.box(48, 300, 240, 110, "Their keyboard tool", ["control_input.py", "or a typed command"], "#f4f6f9", LINE)
    s.box(372, 300, 250, 110, "Our ground station", ["optional: keys, gamepad, map,", "plots, cameras, fault tests"], "#eef8f1", "#a8d5b5")
    s.arrow(288, 340, 390, 224)
    s.arrow(497, 300, 497, 224, both=True)

    s.rect(790, 92, 366, 300, BOARD, BOARD_DARK, rx=12, sw=2)
    s.text(973, 124, "Arduino UNO R4 Minima", 19, "#ffffff", True)
    y = 152
    for t in ("Flight code, about 50 times a second:", "protocol and CRC32", "attitude EKF, height and position filters",
              "attitude, height and position control", "return to home, go to a point, missions", "failsafes and sensor fault checks", "motor mixer"):
        s.text(973, y, t, 14, "#ffffff" if y > 152 else "#d7f0f0", y == 152)
        y += 25
    s.rect(814, 322, 150, 52, "#06080c", "#2b2f33", rx=5)
    s.text(889, 353, "OLED display", 14, "#ffffff", True)
    s.rect(982, 322, 150, 52, "#39424e", "#20262d", rx=5)
    s.text(1057, 353, "4 buttons", 14, "#ffffff", True)

    s.arrow(622, 150, 790, 150)
    s.arrow(790, 188, 622, 188)
    s.text(706, 138, "sensor packets", 13, INK, True)
    s.text(706, 208, "motor commands", 13, INK, True)
    s.text(706, 256, "USB-C serial", 14, MUTED, True)
    s.text(706, 276, "same protocol as", 12, MUTED)
    s.text(706, 292, "the organisers define", 12, MUTED)
    s.save("system.svg", "How the simulator on the laptop and the R4 Minima connect")


def states():
    s = Svg(1180, 650)
    s.text(590, 34, "Flight states: every path out of flight ends with the motors stopped", 19, INK, True)
    w, h = 210, 84
    pos = {"DISARMED": (40, 90), "IDLE": (330, 90), "TAKEOFF": (620, 90), "FLYING": (910, 90),
           "LANDING": (620, 270), "AUTO": (910, 270), "FAILSAFE": (620, 430)}
    sub = {"DISARMED": "motors off", "IDLE": "motors at idle", "TAKEOFF": "climbs to 1.5 m", "FLYING": "height hold or manual",
           "AUTO": "return home, trips, missions", "FAILSAFE": "lands on the spot", "LANDING": "comes down gently"}
    for name, (x, y) in pos.items():
        fill, stroke = ("#e8f1fb", "#1f6fd6") if name == "DISARMED" else (("#eef8f1", "#3a9d5d") if name == "AUTO" else ("#ffffff", LINE))
        s.rect(x, y, w, h, fill, stroke, rx=12, sw=2)
        s.text(x + w / 2, y + 36, name, 17, INK, True)
        s.text(x + w / 2, y + 60, sub[name], 13, MUTED)
    s.arrow(250, 132, 330, 132)
    s.text(290, 120, "arm", 13, MUTED)
    s.arrow(540, 132, 620, 132)
    s.text(580, 120, "take off", 13, MUTED)
    s.arrow(830, 132, 910, 132)
    s.text(870, 120, "at height", 13, MUTED)
    s.poly_arrow([(435, 174), (435, 214), (215, 214), (215, 174)])
    s.text(325, 206, "15 s with no take-off", 13, MUTED)
    # flying and flying by itself
    s.arrow(1000, 174, 1000, 270)
    s.text(992, 218, "H, go-to, mission,", 13, MUTED, False, "end")
    s.text(992, 236, "lost RC link", 13, MUTED, False, "end")
    s.arrow(1060, 270, 1060, 174)
    s.text(1068, 218, "stick moved,", 13, MUTED, False, "start")
    s.text(1068, 236, "or arrived", 13, MUTED, False, "start")
    s.poly_arrow([(930, 174), (930, 196), (725, 196), (725, 270)])
    s.text(830, 188, "land command", 13, MUTED)
    s.arrow(910, 312, 830, 312)
    s.text(870, 300, "land, or low", 13, MUTED)
    s.text(870, 330, "over a marker", 13, MUTED)
    s.poly_arrow([(960, 354), (960, 472), (830, 472)])
    s.text(950, 396, "GPS or compass gone,", 13, MUTED, False, "end")
    s.text(950, 414, "or not getting closer", 13, MUTED, False, "end")
    # both come down, and the motors stop on touch-down
    s.poly_arrow([(620, 312), (180, 312), (180, 174)])
    s.wire([(620, 472), (180, 472), (180, 312)], MUTED, 2)
    s.dot(180, 312, 4, MUTED)
    s.text(400, 304, "touch-down: motors stop", 13, MUTED)
    s.text(400, 464, "touch-down: motors stop", 13, MUTED)
    s.poly_arrow([(1090, 354), (1090, 548), (120, 548), (120, 174)])
    s.text(620, 540, "someone holds it at home: motors stop (simulation only)", 13, MUTED)
    s.rect(40, 590, 1100, 44, "#eceef0", LINE, rx=10)
    s.text(590, 618, "From any state: KILL, crash, sensors lost or STOP go straight to DISARMED", 15, INK, True)
    s.save("states.svg", "The flight states and what moves between them")


wiring()
system()
states()
print("drawn")
