import numpy as np, PIL.Image as I, PIL.ImageDraw as D
z=np.load('scanstats.npz'); med,pres=z['med'],z['pres']; a=z['amin']+z['ainc']*np.arange(len(med))
m=(pres>=0.5)&np.isfinite(med); x,y=med*np.cos(a),med*np.sin(a)
S=50; W=1000; c=W//2; im=I.new('RGB',(W,W),'white'); d=D.Draw(im)
for R in range(1,10): d.ellipse([c-R*S,c-R*S,c+R*S,c+R*S],outline=(220,220,220))
for k in np.flatnonzero(m):
    col=(255,0,0) if pres[k]>=0.9 else (0,0,255)
    px,py=c+x[k]*S,c-y[k]*S; d.rectangle([px-1,py-1,px+1,py+1],fill=col)
d.line([c,c,c+40,c],fill=(0,160,0),width=3); im.save('scan_xy.png')
print('no-return angles (pres<0.1):',int((pres<0.1).sum()))
