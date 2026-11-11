import sys,glob,os,shutil,cv2,numpy as np
sys.path.insert(0,'work');import label
OUT=r"D:\light_ai\data\imagesbiaozhu"
for sub in ('images','labels','preview'): os.makedirs(os.path.join(OUT,sub),exist_ok=True)
tot=0;rows=[]
for vd in sorted(glob.glob('data/images/*/')):
    nm=os.path.basename(os.path.normpath(vd))
    res,info=label.run3(vd,label.FIXED if nm.endswith('105040') else None)
    for f in sorted(res):
        b=res[f]
        if b is None: continue
        img=cv2.imread(f);H,W=img.shape[:2]
        x1,y1,x2,y2=[min(max(v,0),lim) for v,lim in zip(b,(W,H,W,H))]
        stem=f"{nm[-6:]}_{os.path.splitext(os.path.basename(f))[0]}"   # 105040_frame_0000
        shutil.copyfile(f,os.path.join(OUT,'images',stem+'.jpg'))
        cx=(x1+x2)/2/W;cy=(y1+y2)/2/H;w=(x2-x1)/W;h=(y2-y1)/H
        open(os.path.join(OUT,'labels',stem+'.txt'),'w').write(f"0 {cx:.6f} {cy:.6f} {w:.6f} {h:.6f}\n")
        tot+=1
        if tot%6==0:
            vis=img.copy();cv2.rectangle(vis,(x1,y1),(x2,y2),(0,255,0),2)
            cv2.imwrite(os.path.join(OUT,'preview',stem+'.jpg'),vis)
open(os.path.join(OUT,'classes.txt'),'w').write("light\n")
open(os.path.join(OUT,'data.yaml'),'w').write(f"path: {OUT.replace(chr(92),'/')}\ntrain: images\nval: images\nnc: 1\nnames:\n  0: light\n")
print('labeled',tot)
