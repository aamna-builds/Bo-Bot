from reportlab.lib.pagesizes import A4
from reportlab.lib import colors
from reportlab.lib.units import mm
from reportlab.lib.styles import ParagraphStyle
from reportlab.platypus import (SimpleDocTemplate, Paragraph, Spacer, Table,
                                TableStyle, KeepTogether)
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont

FD = "/usr/share/fonts/truetype/dejavu/"
pdfmetrics.registerFont(TTFont("DV", FD + "DejaVuSans.ttf"))
pdfmetrics.registerFont(TTFont("DVB", FD + "DejaVuSans-Bold.ttf"))
pdfmetrics.registerFont(TTFont("DVI", FD + "DejaVuSans-Oblique.ttf"))
from reportlab.pdfbase.pdfmetrics import registerFontFamily
registerFontFamily("DV", normal="DV", bold="DVB", italic="DVI", boldItalic="DVB")

INK = colors.HexColor("#1f2933")
MUTED = colors.HexColor("#5f6b7a")
ACCENT = colors.HexColor("#0f6e6e")
HEAD_BG = colors.HexColor("#0f6e6e")
ROW_ALT = colors.HexColor("#f2f6f6")
RULE = colors.HexColor("#cfd8dc")
WARN_BG = colors.HexColor("#fff6e5")
LINK = colors.HexColor("#0b5cad")

title = ParagraphStyle("t", fontName="DVB", fontSize=20, leading=24, textColor=INK)
sub = ParagraphStyle("s", fontName="DV", fontSize=9.5, leading=13, textColor=MUTED)
h2 = ParagraphStyle("h2", fontName="DVB", fontSize=12.5, leading=16, textColor=ACCENT,
                    spaceBefore=10, spaceAfter=4)
body = ParagraphStyle("b", fontName="DV", fontSize=9, leading=12, textColor=INK)
small = ParagraphStyle("sm", fontName="DV", fontSize=8, leading=10.5, textColor=INK)
smallm = ParagraphStyle("smm", parent=small, textColor=MUTED)
cellh = ParagraphStyle("ch", fontName="DVB", fontSize=8, leading=10, textColor=colors.white)
cellr = ParagraphStyle("cr", parent=small, alignment=2)
cellhr = ParagraphStyle("chr", parent=cellh, alignment=2)


def P(t, s=small):
    return Paragraph(t, s)


def link(url, label):
    return f'<link href="{url}" color="#0b5cad"><u>{label}</u></link>'


def table(rows, widths, right_cols=(), total_rows=(), header=True):
    data = []
    for i, r in enumerate(rows):
        out = []
        for j, c in enumerate(r):
            if isinstance(c, str):
                if header and i == 0:
                    st = cellhr if j in right_cols else cellh
                else:
                    st = cellr if j in right_cols else small
                c = Paragraph(c, st)
            out.append(c)
        data.append(out)
    t = Table(data, colWidths=widths, repeatRows=1 if header else 0)
    st = [
        ("VALIGN", (0, 0), (-1, -1), "TOP"),
        ("TOPPADDING", (0, 0), (-1, -1), 3),
        ("BOTTOMPADDING", (0, 0), (-1, -1), 3),
        ("LEFTPADDING", (0, 0), (-1, -1), 5),
        ("RIGHTPADDING", (0, 0), (-1, -1), 5),
        ("LINEBELOW", (0, 0), (-1, -1), 0.4, RULE),
    ]
    if header:
        st.append(("BACKGROUND", (0, 0), (-1, 0), HEAD_BG))
    start = 1 if header else 0
    for i in range(start, len(rows)):
        if (i - start) % 2 == 1:
            st.append(("BACKGROUND", (0, i), (-1, i), ROW_ALT))
    for i in total_rows:
        st.append(("BACKGROUND", (0, i), (-1, i), colors.HexColor("#e3eeee")))
        st.append(("LINEABOVE", (0, i), (-1, i), 0.8, ACCENT))
    t.setStyle(TableStyle(st))
    return t


W = A4[0] - 30 * mm  # usable width

story = []
story.append(Paragraph("ESP32 Self-Balancing Robot", title))
story.append(Paragraph("Bill of materials with usage", ParagraphStyle(
    "t2", parent=title, fontName="DV", fontSize=13, leading=17, textColor=ACCENT)))
story.append(Spacer(1, 3))
story.append(Paragraph(
    "Prepared 8 Nov 2026 · Target: robot complete before 15 Nov 2026 · "
    "2S LiPo (7.4 V) · two-wheel inverted pendulum with encoder motors and OLED face",
    sub))
story.append(Spacer(1, 8))

# ---------- Summary box
summary = Table([[P(
    "<b>Grand total: about ₹3,850–4,360</b> (two orders + local parts). "
    "Robokits: <b>₹1,882</b> incl. 18% GST and ₹300 delivery. "
    "Robu: <b>~₹1,895–2,080</b> incl. GST, + ₹0–300 delivery (often free above ₹499). "
    "Robu prices for the OLED, charger and wheels are estimates; the cart shows the exact figure.",
    body)]], colWidths=[W])
summary.setStyle(TableStyle([
    ("BACKGROUND", (0, 0), (-1, -1), colors.HexColor("#e6f2f2")),
    ("BOX", (0, 0), (-1, -1), 0.8, ACCENT),
    ("LEFTPADDING", (0, 0), (-1, -1), 8), ("RIGHTPADDING", (0, 0), (-1, -1), 8),
    ("TOPPADDING", (0, 0), (-1, -1), 6), ("BOTTOMPADDING", (0, 0), (-1, -1), 6),
]))
story.append(summary)

# ---------- Robu
RB = "https://robu.in/product/"
story.append(Paragraph("Order 1 — Robu.in (prices include GST)", h2))
robu = [
    ["#", "Component", "Qty", "Usage in the robot", "Price", "Link"],
    ["1", "<b>GA12-N20 6V 300 RPM micro gear motor with encoder</b><br/>"
          "6 wires, 3 mm D-shaft, Hall encoder",
     "2", "Drive both wheels. On 2S (7.4 V) gives ~370 RPM and ~0.8 kg·cm. "
          "Encoders measure wheel speed so the robot holds position instead of drifting, "
          "and make remote driving smooth. PWM capped at ~80% in code.",
     "₹1,238<br/>(₹619 ea.)",
     link(RB + "ga12-n20-6v-300rpm-micro-dc-reducer-motor-encoder-with-wire/", "robu.in")],
    ["2", "<b>White 3PI miniQ wheel, 34 mm</b><br/>3 mm D-hole, 6.5 mm tyre, 3.2 g",
     "2", "Wheels. Press straight onto the N20 D-shaft. Small diameter keeps torque at "
          "the ground high, which helps recovery from pushes.",
     "~₹80<br/>(est.)",
     link(RB + "white-3pi-miniq-car-wheel-tyre-34mm-for-n20-motor/", "robu.in")],
    ["3", "<b>1.3\" I2C OLED, 4-pin, blue (SH1106, 128×64)</b>",
     "1", "Robot face: eyes and mouth that change with tilt, movement and battery state. "
          "Wired to I2C bus 2 (GPIO 23/4), powered from 3.3 V. Use the Adafruit SH110X or U8g2 library.",
     "~₹275–310<br/>(est.)",
     link(RB + "1-3-inch-i2c-iic-4-pin-oled-display-module-with-vcc-gnd-blue/", "robu.in")],
    ["4", "<b>IMAX B3AC compact balance charger, 2S–3S</b>",
     "1", "Charges the LiPo through its white balance plug (2S port). Stops each cell at "
          "4.2 V (8.4 V total) and keeps cells equal. Not connected to the robot.",
     "~₹300–450<br/>(est.)",
     link(RB + "b3ac-compact-balance-charger-2s-3s-lipo/", "robu.in")],
    ["", "<b>Robu subtotal</b>", "", "", "<b>~₹1,895–2,080</b>", ""],
    ["", "Delivery", "", "Often free above ₹499", "₹0–300", ""],
]
story.append(table(robu, [7*mm, 43*mm, 10*mm, 71*mm, 32*mm, 17*mm],
                   right_cols=(2, 4), total_rows=(5,)))

# ---------- Robokits
RK = "https://robokits.co.in/"
story.append(Paragraph("Order 2 — Robokits.co.in (listed price + 18% GST)", h2))
rk = [
    ["#", "Component", "Qty", "Usage in the robot", "Price", "+ GST", "Link"],
    ["5", "<b>GenX 7.4V 2S 1000mAh 35C/70C LiPo</b><br/>68×37×12 mm, 54 g, XT30 plug",
     "1", "Main power for everything. Feeds the DRV8833 directly and the buck converter "
          "for the ESP32. Mounted low between the wheels. Usable range 8.4 V full to 6.6 V cutoff.",
     "₹1,005", "₹1,186",
     link(RK + "batteries-chargers/drone-batteries/genx-power-premium-lipo-battery/"
               "genxpower-7.4v-lipo-batteries/genx-7.4v-2s-1000mah-35c-70c-premium-lipo-lithium-polymer-battery",
          "robokits")],
    ["6", "<b>DC-DC buck module 6–24V → 5V 3A (USB)</b>",
     "1", "Steps the battery down to 5 V for the ESP32 VIN pin. Solder its 5 V output pads "
          "to VIN/GND instead of using a USB cable inside the robot.",
     "₹42", "₹50",
     link(RK + "batteries-chargers/step-up-step-down-module/"
               "dc-dc-buck-module-6-24v-12v-24v-to-5v-3a-usb-step-down-power-supply-charger-efficiency-97.5",
          "robokits")],
    ["7", "<b>DRV8833 dual motor driver</b><br/>2.7–10.8 V, 1.2 A cont. / 2 A peak per channel",
     "1", "Switches battery power to both motors from four ESP32 PWM pins. MOSFET driver, so "
          "almost no voltage loss (unlike an L298N). SLP/EEP tied to 3.3 V.",
     "₹80", "₹94",
     link(RK + "motor-drives-drivers/dc-motor-driver/two-way-motor-drive-module-drv8833",
          "robokits")],
    ["8", "<b>MPU-6050 gyro + accelerometer breakout</b><br/>I2C address 0x68, 3.3 V only",
     "1", "Measures tilt angle and tilt rate ~200 times a second. A complementary filter plus PID "
          "turns this into motor commands. Mount flat, near the wheel axle, firmly.",
     "₹173", "₹204",
     link(RK + "sensors/accelerometer-and-magnetometer/triple-axis-accelerometer-gyro-mpu-6050-breakout",
          "robokits")],
    ["9", "<b>Amass XT30 male + female pair</b>",
     "1", "Battery connector. Solder the <b>male</b> to the robot's power leads (the battery "
          "has the female). Unplugging it is the on/off switch.",
     "₹41", "₹48",
     link(RK + "batteries-chargers/accessories/plugs-and-connectors/"
               "amass-xt30-male-and-female-connector-original-xt30-f-m", "robokits")],
    ["", "<b>Robokits subtotal</b>", "", "", "₹1,341", "<b>₹1,582</b>", ""],
    ["", "Delivery", "", "", "", "₹300", ""],
    ["", "<b>Robokits total</b>", "", "", "", "<b>₹1,882</b>", ""],
]
story.append(table(rk, [7*mm, 42*mm, 10*mm, 70*mm, 16*mm, 18*mm, 17*mm],
                   right_cols=(2, 4, 5), total_rows=(6, 8)))

# ---------- Already have / print / local
story.append(Paragraph("Already have", h2))
have = [
    ["Component", "Qty", "Usage in the robot"],
    ["<b>ESP32 DevKit (30-pin)</b>", "1",
     "The brain. Core 1 runs balancing (~200 Hz); core 0 runs Bluetooth/Wi-Fi control and the OLED face."],
    ["<b>Jumper wires (F-F, M-F)</b>", "~30",
     "Signal wiring: I2C, motor PWM, encoder lines, 3.3 V and GND to sensors."],
    ["<b>USB-A to micro-USB cable</b>", "1",
     "Uploading code and Serial Monitor. Unplug the battery before connecting USB."],
]
story.append(table(have, [45*mm, 12*mm, W - 57*mm], right_cols=(1,)))

story.append(Paragraph("3D print", h2))
printed = [
    ["Part", "Qty", "Usage in the robot"],
    ["<b>N20 motor brackets</b>", "2",
     "Hold the motors rigidly (bracket packs were out of stock). Thingiverse thing:5944227 or "
     "thing:4593489, or design the mounts into the chassis."],
    ["<b>Chassis base + shelves</b>", "1 set",
     "Battery low between the wheels (~78 mm track); electronics shelf ~25 mm above; "
     "keep the total width about 86–90 mm."],
    ["<b>Shell, ears, bow tie</b>", "1 set",
     "Cosmetic body with a vertical window for the OLED. Print after the robot balances, "
     "then re-tune PID for the extra weight."],
]
story.append(table(printed, [45*mm, 12*mm, W - 57*mm], right_cols=(1,)))

story.append(Paragraph("Buy locally", h2))
local = [
    ["Component", "Need", "Usage in the robot"],
    ["<b>10 kΩ + 4.7 kΩ resistors</b>", "Recommended",
     "Voltage divider from the battery to GPIO 34 (8.4 V → ~2.7 V). Code stops the motors and "
     "shows a low-battery face at ~6.6 V so the LiPo is never over-drained."],
    ["<b>Mini breadboard (170-pt) or perfboard</b>", "Needed",
     "Shared 3.3 V and GND rails for the MPU-6050, OLED, encoders and DRV8833."],
    ["<b>20–22 AWG red/black wire, ~1 m</b>", "Needed",
     "Battery power runs: XT30 → DRV8833 VM and buck input. Jumpers are too thin for motor current."],
    ["<b>M3 screws and nuts</b>", "Needed", "Fixing the printed frame, brackets and boards."],
    ["<b>470 µF 16 V capacitor</b>", "Optional",
     "Across DRV8833 VM–GND to absorb motor spikes. Add only if the ESP32 resets when motors kick."],
    ["<b>Small slide switch</b>", "Optional", "Convenience on/off; the XT30 plug already does this."],
]
story.append(table(local, [55*mm, 25*mm, W - 80*mm]))

# ---------- Cost summary
cost = [
    ["Order", "Items", "Delivery", "Total"],
    ["Robokits (incl. 18% GST)", "₹1,582", "₹300", "<b>₹1,882</b>"],
    ["Robu (GST included)", "~₹1,895–2,080", "₹0–300", "<b>~₹1,895–2,380</b>"],
    ["Local parts", "~₹70–100", "—", "<b>~₹70–100</b>"],
    ["<b>Grand total</b>", "", "", "<b>~₹3,850–4,360</b>"],
]
story.append(KeepTogether([Paragraph("Cost summary", h2),
                           table(cost, [70*mm, 40*mm, 30*mm, W - 140*mm],
                                 right_cols=(1, 2, 3), total_rows=(4,))]))

# ---------- Wiring reference
wiring=[Paragraph("Wiring reference", h2)]
wiring.append(P("<b>Power path.</b> LiPo (XT30) <b>+</b> → DRV8833 VM, buck IN+, and the top of the "
               "10k/4.7k divider. LiPo <b>−</b> → common GND rail. Buck 5 V out → ESP32 VIN. "
               "ESP32 3V3 → 3.3 V rail. All grounds joined.", body))
wiring.append(Spacer(1, 5))
pins = [
    ["ESP32 pin", "Connects to", "ESP32 pin", "Connects to"],
    ["VIN", "Buck 5 V output", "GPIO 25 / 26", "DRV8833 AIN1 / AIN2 (left motor)"],
    ["3V3", "3.3 V rail (MPU, OLED, encoders, DRV SLP)", "GPIO 32 / 33", "DRV8833 BIN1 / BIN2 (right motor)"],
    ["GND", "Common GND rail", "GPIO 18 / 19", "Left encoder A / B"],
    ["GPIO 21 / 22", "MPU-6050 SDA / SCL", "GPIO 16 / 17", "Right encoder A / B"],
    ["GPIO 23 / 4", "OLED SDA / SCL", "GPIO 34", "Battery divider midpoint"],
]
wiring.append(table(pins, [24*mm, W/2 - 24*mm, 24*mm, W/2 - 24*mm]))
story.append(KeepTogether(wiring))
story.append(Spacer(1, 6))
mot = [
    ["Motor wire", "Label", "Function", "Left motor →", "Right motor →"],
    ["Red", "M2", "Motor +", "DRV8833 AOUT1", "DRV8833 BOUT1"],
    ["White", "M1", "Motor −", "DRV8833 AOUT2", "DRV8833 BOUT2"],
    ["Black", "VCC", "Encoder power", "3.3 V", "3.3 V"],
    ["Blue", "GND", "Encoder ground", "GND", "GND"],
    ["Green", "C1", "Encoder A", "GPIO 18", "GPIO 16"],
    ["Yellow", "C2", "Encoder B", "GPIO 19", "GPIO 17"],
]
story.append(KeepTogether([table(mot, [24*mm, 18*mm, 34*mm, (W-76*mm)/2, (W-76*mm)/2]), Spacer(1, 3), P("Trust the labels printed on the encoder board over wire colours. Never swap encoder "
               "VCC and GND. If a motor spins the wrong way, swap red/white or flip it in code.", smallm)]))

# ---------- Checklist & safety
story.append(Paragraph("Check before paying", h2))
checks = [
    "DRV8833 shows <b>In Stock</b> at Robokits (its stock status was conflicting).",
    "1.3\" OLED is in stock. Backups: " + link(RB + "1-3-inch-i2c-iic-oled-4-pin-lcd-module-4pin-with-vcc-gnd-blue/",
                                                 "alternative 1.3\" blue") + " or " +
    link("https://robu.in/?p=617730", "white version") + ".",
    "B3AC charger is the <b>new</b> listing, not refurbished.",
    "Wheels quantity <b>2</b> (stock is low). Motors quantity <b>2</b> at ₹619 each.",
    "MPU-6050 quantity <b>1</b>.",
]
for c in checks:
    story.append(P("☐&nbsp;&nbsp;" + c, body))
    story.append(Spacer(1, 2))

story.append(Paragraph("Safety and code limits", h2))
safety = Table([[P(
    "• Charge the LiPo only on the B3AC, through the balance plug, never unattended.<br/>"
    "• Stop using the battery at 6.6 V (3.3 V per cell); the divider on GPIO 34 lets the code enforce this.<br/>"
    "• Cap motor PWM at ~80%: the 6 V motors see up to 8.4 V on a full pack.<br/>"
    "• Cut the motors when tilt passes 45° so a fallen robot doesn't spin its wheels.<br/>"
    "• Don't use a 3S pack: 12.6 V exceeds the DRV8833's 10.8 V limit.<br/>"
    "• Unplug the XT30 before connecting USB to upload code.", body)]], colWidths=[W])
safety.setStyle(TableStyle([
    ("BACKGROUND", (0, 0), (-1, -1), WARN_BG),
    ("BOX", (0, 0), (-1, -1), 0.6, colors.HexColor("#e0b25c")),
    ("LEFTPADDING", (0, 0), (-1, -1), 8), ("RIGHTPADDING", (0, 0), (-1, -1), 8),
    ("TOPPADDING", (0, 0), (-1, -1), 6), ("BOTTOMPADDING", (0, 0), (-1, -1), 6),
]))
story.append(safety)


def footer(canvas, doc):
    canvas.saveState()
    canvas.setFont("DV", 7.5)
    canvas.setFillColor(MUTED)
    canvas.drawString(15*mm, 9*mm, "ESP32 self-balancing robot — BOM with usage")
    canvas.drawRightString(A4[0] - 15*mm, 9*mm, f"Page {doc.page}")
    canvas.restoreState()


doc = SimpleDocTemplate("robot_BOM.pdf", pagesize=A4,
                        leftMargin=15*mm, rightMargin=15*mm, topMargin=14*mm, bottomMargin=16*mm,
                        title="ESP32 Self-Balancing Robot — BOM", author="aamna-builds")
doc.build(story, onFirstPage=footer, onLaterPages=footer)
print("ok")
