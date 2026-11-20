from PIL import Image, ImageDraw, ImageFont
def isq(v):
    r=0
    while (r+1)*(r+1)<=v: r+=1
    return r
def span(a,b,p):
    y0=p*8;y1=y0+7
    if b<y0 or a>y1 or b<a: return 0
    a=max(a,y0); b=min(b,y1)
    return ((0xFF>>(7-(b-a)))<<(a-y0))&0xFF
F_SLEEP,F_WAIT,F_HAPPY,F_FELL,F_SETUP=range(5)
def faceByte(x,p,fs):
    b=0; f=fs['face']
    for e in (0,1):
        dx=x-(88 if e else 40); adx=abs(dx); cy=24+fs['dy']
        if f==F_HAPPY:
            if adx<=12: o=isq(156-dx*dx); i=isq(72-dx*dx) if adx<8 else 0; b|=span(cy+4-o,cy+4-i,p)
        elif f==F_FELL:
            if adx<=8: b|=span(cy+dx-1,cy+dx+1,p)|span(cy-dx-1,cy-dx+1,p)
        elif f==F_SLEEP:
            if adx<=11: b|=span(cy+1,cy+3,p)
        else:
            if adx<=11:
                h=2 if fs['blink'] else 11; cut=0
                if adx>6: k=adx-6; cut=5-isq(30-k*k)
                hh=max(h-cut,1); b|=span(cy-hh,cy+hh,p)
    dx=x-64; adx=abs(dx)
    if f==F_HAPPY:
        if adx<=12:
            if fs['mouth']>2: o=isq(156-dx*dx)*fs['mouth']//12; b|=span(42,42+o,p)
            else: o=isq(156-dx*dx); i=isq(90-dx*dx) if adx<9 else 0; b|=span(40+i,40+o,p)
    elif f==F_FELL:
        if adx<=12: o=isq(156-dx*dx); i=isq(90-dx*dx) if adx<9 else 0; b|=span(58-o,58-i,p)
    elif f==F_WAIT:
        if adx<=5: o=isq(30-dx*dx); i=isq(12-dx*dx) if adx<3 else 0; b|=span(48-o,48-i,p)|span(48+i,48+o,p)
    elif f==F_SLEEP:
        if adx<=5: b|=span(48,49,p)
    else:
        if adx<=8: b|=span(47,48,p)
    return b
def render(fs,scale=4):
    im=Image.new('RGB',(128*scale,64*scale),(10,12,20)); d=ImageDraw.Draw(im)
    for p in range(1,8):
        for x in range(16,112):
            v=faceByte(x,p,fs)
            for bit in range(8):
                if v>>bit&1:
                    y=p*8+bit; d.rectangle([x*scale,y*scale,x*scale+scale-1,y*scale+scale-1],fill=(90,220,255))
    return im
faces=[("Balancing (happy)",dict(face=F_HAPPY,dy=0,blink=0,mouth=0)),
       ("Balancing + moving fast",dict(face=F_HAPPY,dy=2,blink=0,mouth=11)),
       ("Waiting to be stood up",dict(face=F_WAIT,dy=0,blink=0,mouth=0)),
       ("Fell over",dict(face=F_FELL,dy=0,blink=0,mouth=0)),
       ("Stopped (x)",dict(face=F_SLEEP,dy=0,blink=0,mouth=0)),
       ("Setup / calibrating",dict(face=F_SETUP,dy=0,blink=0,mouth=0))]
W,H=128*4,64*4
sheet=Image.new('RGB',(3*(W+30)+30,2*(H+60)+30),(245,245,248)); d=ImageDraw.Draw(sheet)
ft=ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",22)
for k,(t,fs) in enumerate(faces):
    cx=30+(k%3)*(W+30); cy=20+(k//3)*(H+60)
    d.text((cx,cy),t,font=ft,fill=(50,40,110)); sheet.paste(render(fs),(cx,cy+34))
sheet.save('faces.png')
