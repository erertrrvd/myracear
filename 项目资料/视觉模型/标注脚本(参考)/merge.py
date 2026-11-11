import os,glob,shutil
A=r"D:\light_ai\data\imagesbiaozhu";B=r"D:\light_ai\data\imagesbiaozhu_lamp";OUT=r"D:\light_ai\data\imagesbiaozhu_merged"
for s in ('images','labels','preview'): os.makedirs(os.path.join(OUT,s),exist_ok=True)
n=nb=0;miss=[]
for img in sorted(glob.glob(os.path.join(A,'images','*.jpg'))):
    stem=os.path.basename(img)[:-4]
    la=os.path.join(A,'labels',stem+'.txt');lb=os.path.join(B,'labels',stem+'.txt')
    lines=[]
    for l in open(la).read().split('\n'):
        p=l.split()
        if len(p)==5: lines.append("0 "+" ".join(p[1:]))     # light_box
    if os.path.exists(lb):
        for l in open(lb).read().split('\n'):
            p=l.split()
            if len(p)==5: lines.append("1 "+" ".join(p[1:])); nb+=1   # red_on
    else: miss.append(stem)
    shutil.copyfile(img,os.path.join(OUT,'images',stem+'.jpg'))
    open(os.path.join(OUT,'labels',stem+'.txt'),'w').write("\n".join(lines)+("\n" if lines else ""))
    n+=1
open(os.path.join(OUT,'classes.txt'),'w').write("light_box\nred_on\n")
open(os.path.join(OUT,'data.yaml'),'w').write(f"path: {OUT.replace(chr(92),'/')}\ntrain: images\nval: images\nnc: 2\nnames:\n  0: light_box\n  1: red_on\n")
# 预览：画两类框（每 6 张取 1）
import cv2
for i,img in enumerate(sorted(glob.glob(os.path.join(OUT,'images','*.jpg')))):
    if i%6: continue
    im=cv2.imread(img);H,W=im.shape[:2]
    for l in open(os.path.join(OUT,'labels',os.path.basename(img)[:-4]+'.txt')):
        c,cx,cy,w,h=l.split();c=int(c);cx,cy,w,h=float(cx)*W,float(cy)*H,float(w)*W,float(h)*H
        col=(0,255,0) if c==0 else (0,0,255)
        cv2.rectangle(im,(int(cx-w/2),int(cy-h/2)),(int(cx+w/2),int(cy+h/2)),col,2 if c else 1)
    cv2.imwrite(os.path.join(OUT,'preview',os.path.basename(img)),im)
print('images',n,'red_on boxes',nb,'missing lamp label file',len(miss))
# 校验：红灯框必须在灯箱框内
bad=0;both=0
for f in glob.glob(os.path.join(OUT,'labels','*.txt')):
    box=None;red=None
    for l in open(f):
        c,cx,cy,w,h=l.split();v=(float(cx),float(cy),float(w),float(h))
        if c=='0': box=v
        else: red=v
    if red:
        both+=1
        if not box or not(box[0]-box[2]/2-0.005<=red[0]-red[2]/2 and red[0]+red[2]/2<=box[0]+box[2]/2+0.005 and box[1]-box[3]/2-0.005<=red[1]-red[3]/2 and red[1]+red[3]/2<=box[1]+box[3]/2+0.005): bad+=1
print('red_on inside box check: total',both,'not-inside',bad)
