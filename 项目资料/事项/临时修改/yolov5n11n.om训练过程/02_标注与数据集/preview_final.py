import glob,os,cv2,numpy as np,random
OUT=r"D:\light_ai\data\imagesbiaozhu_merged"
S=r"C:\Users\lzc2\AppData\Local\Temp\claude\C--Users-lzc2-Desktop-my-racecar----\ed0c2ab4-24e0-449c-86fd-50b744ec48f1\scratchpad"
COL={0:(0,255,0),1:(0,0,255),2:(255,128,0)}
def draw(stem):
    im=cv2.imread(os.path.join(OUT,'images',stem+'.jpg'));H,W=im.shape[:2]
    st=[]
    for l in open(os.path.join(OUT,'labels',stem+'.txt')):
        p=l.split()
        if len(p)!=5: continue
        c=int(p[0]);cx,cy,w,h=[float(v) for v in p[1:]];cx*=W;cy*=H;w*=W;h*=H
        cv2.rectangle(im,(int(cx-w/2),int(cy-h/2)),(int(cx+w/2),int(cy+h/2)),COL[c],2 if c else 1);st.append(c)
    return im,st
new=sorted(os.path.basename(f)[:-4] for f in glob.glob(os.path.join(OUT,'images','11*.jpg')))
for i,s in enumerate(new):
    if i%6==0: cv2.imwrite(os.path.join(OUT,'preview',s+'.jpg'),draw(s)[0])
random.seed(7)
def sheet(stems,out,cols=5):
    t=[]
    for s in stems:
        im,_=draw(s);cv2.putText(im,s[:6]+'#'+s[-4:],(4,20),0,0.6,(0,255,255),2);t.append(cv2.resize(im,(256,192)))
    while len(t)%cols: t.append(np.zeros((192,256,3),np.uint8))
    cv2.imwrite(os.path.join(S,out),np.vstack([np.hstack(t[r*cols:r*cols+cols]) for r in range(len(t)//cols)]))
sheet(random.sample(new,30),'fin_rand.jpg')
offs=[s for s in new if open(os.path.join(OUT,'labels',s+'.txt')).read().count('\n')==1]
nones=[s for s in new if open(os.path.join(OUT,'labels',s+'.txt')).read().strip()=='']
print('off(box only)',len(offs),'none(empty)',len(nones))
sheet((offs+nones)[:30],'fin_offnone.jpg')
