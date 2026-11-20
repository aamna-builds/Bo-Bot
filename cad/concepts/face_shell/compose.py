from PIL import Image, ImageDraw, ImageFont, ImageChops
def crop(p, pad=25):
    im = Image.open(p).convert("RGB")
    bg = Image.new("RGB", im.size, im.getpixel((2,2)))
    bb = ImageChops.difference(im, bg).point(lambda v: 255 if v > 12 else 0).getbbox()
    return im.crop((max(bb[0]-pad,0), max(bb[1]-pad,0), min(bb[2]+pad,im.width), min(bb[3]+pad,im.height)))
def fit(im, w, h):
    im = im.copy(); im.thumbnail((w, h), Image.LANCZOS); return im
import os
os.chdir(os.path.dirname(os.path.abspath(__file__)))
F = next((f for f in ("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", "C:/Windows/Fonts/segoeuib.ttf",
                      "C:/Windows/Fonts/arialbd.ttf") if os.path.exists(f)), "DejaVuSans-Bold.ttf")
ft, fp = ImageFont.truetype(F, 34), ImageFont.truetype(F, 22)
PW, PH = 720, 560
panels = [("1  On the robot (front-left)", "iso_A.png", "Option A - cat-head front shell"),
          ("2  Front view", "front_A.png", ""),
          ("3  Side view - how high it sits", "side_A.png", ""),
          ("4  From behind - OLED pocket + open back", "back_A.png", "wires go out the back to A4/A5/5V/GND"),
          ("5  Tool-free clip foot (x2)", "clip.png", "no screws - uses your 2 front holes"),
          ("6  Option B - floating face on 2 legs", "iso_B.png", "lighter, leaves the middle plate open")]
sheet = Image.new("RGB", (3*PW + 40, 2*(PH+60) + 110), (248,248,248))
d = ImageDraw.Draw(sheet)
d.text((20, 22), "BO-BOT - OLED face shell  |  CONCEPT (not final sizes)", font=ft, fill=(60,40,120))
for i, (t, f, sub) in enumerate(panels):
    cx, cy = 20 + (i % 3) * PW, 90 + (i // 3) * (PH + 60)
    d.rounded_rectangle([cx+4, cy, cx+PW-8, cy+PH+50], radius=14, outline=(200,195,225), width=3, fill=(255,255,255))
    d.text((cx+18, cy+10), t, font=fp, fill=(60,40,120))
    im = fit(crop(f), PW-40, PH-50)
    sheet.paste(im, (cx + (PW - im.width)//2, cy + 42 + (PH-40 - im.height)//2))
    if sub: d.text((cx+18, cy+PH+16), sub, font=fp, fill=(150,30,30))
sheet.save("face_shell_concept.png")
