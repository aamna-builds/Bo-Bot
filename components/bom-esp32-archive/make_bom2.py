from reportlab.lib.pagesizes import A4
from reportlab.lib import colors
from reportlab.lib.units import mm
from reportlab.lib.styles import ParagraphStyle
from reportlab.platypus import SimpleDocTemplate, Paragraph, Spacer, Table, TableStyle
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
FD="/usr/share/fonts/truetype/dejavu/"
pdfmetrics.registerFont(TTFont("DV",FD+"DejaVuSans.ttf")); pdfmetrics.registerFont(TTFont("DVB",FD+"DejaVuSans-Bold.ttf"))
from reportlab.pdfbase.pdfmetrics import registerFontFamily
registerFontFamily("DV",normal="DV",bold="DVB",italic="DV",boldItalic="DVB")
INK=colors.HexColor("#1f2933"); MUTED=colors.HexColor("#5f6b7a"); ACC=colors.HexColor("#0f6e6e")
title=ParagraphStyle("t",fontName="DVB",fontSize=18,leading=22,textColor=INK)
sub=ParagraphStyle("s",fontName="DV",fontSize=9.5,leading=13,textColor=MUTED)
c=ParagraphStyle("c",fontName="DV",fontSize=9,leading=11.5,textColor=INK)
cr=ParagraphStyle("cr",parent=c,alignment=2)
ch=ParagraphStyle("ch",fontName="DVB",fontSize=9,leading=11.5,textColor=colors.white)
chr_=ParagraphStyle("chr",parent=ch,alignment=2)
sec=ParagraphStyle("sec",fontName="DVB",fontSize=9.5,leading=12,textColor=ACC)
note=ParagraphStyle("n",fontName="DV",fontSize=9,leading=12.5,textColor=INK)
W=A4[0]-30*mm
rows=[]; style=[]
def hdr(): rows.append([Paragraph(x,chr_ if i in (2,4) else ch) for i,x in enumerate(["#","Item","Qty","Use","Cost (₹)"])])
def section(name):
    rows.append([Paragraph(name,sec),"","","",""]); i=len(rows)-1
    style.extend([("SPAN",(0,i),(-1,i)),("BACKGROUND",(0,i),(-1,i),colors.HexColor("#e3eeee")),("TOPPADDING",(0,i),(-1,i),5)])
def item(n,name,q,use,cost,bold=False):
    b=lambda s:f"<b>{s}</b>" if bold else s
    rows.append([Paragraph(n,c),Paragraph(b(name),c),Paragraph(q,cr),Paragraph(use,c),Paragraph(b(cost),cr)])
    if bold: style.append(("LINEABOVE",(0,len(rows)-1),(-1,len(rows)-1),0.8,ACC))
hdr()
section("Robu.in (prices include GST)")
item("1","GA12-N20 6V 300 RPM micro DC reducer motor with encoder","2","Drive wheels + speed feedback","1,238.00")
item("2","White 3PI miniQ wheel tyre, 34 mm, for N20","2","Wheels","78.00")
item("3","1.3\" I2C 4-pin OLED, blue (SH1106)","1","Robot face","309.00")
item("4","IMAX B3 AC Pro compact balance charger, 2S–3S","1","Charging the LiPo","349.00")
item("","Shipping","","","149.00")
item("","Robu total","","","2,123.00",True)
section("Robokits order #1141482 (DTDC Priority air, 1–4 working days)")
item("5","Two-way motor drive module DRV8833","1","Drives both motors","80.00")
item("6","MPU-6050 gyro + accelerometer breakout","1","Tilt sensing","173.00")
item("7","DC-DC buck module 6–24V to 5V 3A","1","5 V supply for the ESP32","42.00")
item("8","Amass XT30 male + female connector","1","Battery connector / on-off","41.00")
item("","Shipping","","","130.00")
item("","GST 18% (items + shipping)","","","83.88")
item("","Order total","","","549.88",True)
section("Robokits order #1141481 (free shipping, 3–7 working days)")
item("9","GenX 7.4V 2S 1000mAh 35C/70C LiPo","1","Main power","1,005.00")
item("","Shipping","","","0.00")
item("","GST 18%","","","180.90")
item("","Order total","","","1,185.90",True)
section("Local purchase")
item("10","Jumper wires (budget)","–","Signal wiring","200.00")
section("Already have")
item("–","ESP32 DevKit, USB cable, other parts","–","Controller, programming","0.00")
t=Table(rows,colWidths=[8*mm,72*mm,12*mm,W-8*mm-72*mm-12*mm-26*mm,26*mm],repeatRows=1)
base=[("BACKGROUND",(0,0),(-1,0),ACC),("VALIGN",(0,0),(-1,-1),"MIDDLE"),("LINEBELOW",(0,0),(-1,-1),0.4,colors.HexColor("#cfd8dc")),
      ("TOPPADDING",(0,0),(-1,-1),4),("BOTTOMPADDING",(0,0),(-1,-1),4),("LEFTPADDING",(0,0),(-1,-1),5),("RIGHTPADDING",(0,0),(-1,-1),5)]
t.setStyle(TableStyle(base+style))
tot=Table([[Paragraph("<b>GRAND TOTAL</b>",ParagraphStyle("g",parent=c,fontSize=11,leading=14,textColor=colors.white)),
            Paragraph("<b>₹4,058.78</b>",ParagraphStyle("g2",parent=cr,fontSize=11,leading=14,textColor=colors.white))]],colWidths=[W-40*mm,40*mm])
tot.setStyle(TableStyle([("BACKGROUND",(0,0),(-1,-1),INK),("TOPPADDING",(0,0),(-1,-1),7),("BOTTOMPADDING",(0,0),(-1,-1),7),("LEFTPADDING",(0,0),(-1,-1),8),("RIGHTPADDING",(0,0),(-1,-1),8)]))
summ=Table([[Paragraph(a,c),Paragraph(b,cr)] for a,b in [("Robu.in","2,123.00"),("Robokits #1141482","549.88"),("Robokits #1141481","1,185.90"),("Jumper wires (local)","200.00")]],colWidths=[W-40*mm,40*mm])
summ.setStyle(TableStyle([("LINEBELOW",(0,0),(-1,-1),0.4,colors.HexColor("#cfd8dc")),("TOPPADDING",(0,0),(-1,-1),3),("BOTTOMPADDING",(0,0),(-1,-1),3),("LEFTPADDING",(0,0),(-1,-1),8),("RIGHTPADDING",(0,0),(-1,-1),8)]))
story=[Paragraph("ESP32 Self-Balancing Robot — Bill of Materials",title),Spacer(1,3),
       Paragraph("9 Nov 2026 · Orders placed with Robu.in and Robokits India · Target completion: before 15 Nov 2026",sub),Spacer(1,10),
       t,Spacer(1,12),Paragraph("Summary",sec),Spacer(1,4),summ,Spacer(1,4),tot,Spacer(1,10),
       Paragraph("<b>Note:</b> the battery (order #1141481) ships free, 3–7 working days, so it is the order most likely to arrive after 15 Nov. Until it arrives, the ESP32, MPU-6050 and OLED can be tested from laptop USB power.",note)]
doc=SimpleDocTemplate("robot_BOM_final.pdf",pagesize=A4,leftMargin=15*mm,rightMargin=15*mm,topMargin=15*mm,bottomMargin=15*mm,title="ESP32 Self-Balancing Robot — BOM")
doc.build(story); print("ok")
