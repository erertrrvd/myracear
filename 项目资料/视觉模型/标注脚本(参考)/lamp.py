import sys,glob,os,shutil,cv2,numpy as np
sys.path.insert(0,'work');import label
OUT=r"D:\light_ai\data\imagesbiaozhu_lamp"
def red_lamp(img,hbox,b):
    """在灯箱框内找亮着的红灯：返回 (x1,y1,x2,y2) 或 None。b=(ox,oy,d) 橙灯"""
    ox,oy,d=b;H,W=img.shape[:2]
    x1,y1,x2,y2=hbox
    g=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY);hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    # 红灯预期位置：橙灯左边约 1.9d；搜索范围只取这一带
    ex=ox-1.9*d
    sx1=int(max(x1,ex-1.1*d));sx2=int(min(ox-0.9*d,ex+1.1*d))
    sy1=int(max(y1,oy-1.0*d));sy2=int(min(y2,oy+1.0*d))
    if sx2<=sx1 or sy2<=sy1: return None
    roi_g=g[sy1:sy2,sx1:sx2];roi_h=hsv[sy1:sy2,sx1:sx2]
    core=(roi_g>235)
    # 红晕：红色且较亮
    halo=((roi_h[...,0]<12)|(roi_h[...,0]>165))&(roi_h[...,1]>100)&(roi_h[...,2]>150)
    m=(core|halo).astype(np.uint8)*255
    m=cv2.morphologyEx(m,cv2.MORPH_OPEN,np.ones((3,3),np.uint8))
    n,lab,st,cen=cv2.connectedComponentsWithStats(m)
    best=None;ba=0
    for i in range(1,n):
        if st[i,4]>ba: ba=st[i,4];best=i
    if best is None: return None
    # 必须含足够大的白色核心才算“亮”
    coremask=(lab==best)&core
    if coremask.sum()<0.12*np.pi*(d/2)**2: return None
    ys,xs=np.nonzero(coremask)
    cx=sx1+xs.mean();cy=sy1+ys.mean()
    half=0.55*d
    return (int(max(0,cx-half)),int(max(0,cy-half)),int(min(W-1,cx+half)),int(min(H-1,cy+half)))
if __name__=='__main__':
    for s in ('images','labels','preview'): os.makedirs(os.path.join(OUT,s),exist_ok=True)
    tot=lit=0
    for vd in sorted(glob.glob('data/images/*/')):
        nm=os.path.basename(os.path.normpath(vd))
        res,_=label.run3(vd,label.FIXED if nm.endswith('105040') else None)
        states=[]
        for f in sorted(res):
            hb=res[f]
            if hb is None: continue
            img=cv2.imread(f);H,W=img.shape[:2]
            ob=label.orange_blob(img)
            lb=red_lamp(img,hb,ob) if ob else None
            stem=f"{nm[-6:]}_{os.path.splitext(os.path.basename(f))[0]}"
            shutil.copyfile(f,os.path.join(OUT,'images',stem+'.jpg'))
            txt=""
            if lb:
                x1,y1,x2,y2=lb;txt=f"0 {(x1+x2)/2/W:.6f} {(y1+y2)/2/H:.6f} {(x2-x1)/W:.6f} {(y2-y1)/H:.6f}\n";lit+=1
            open(os.path.join(OUT,'labels',stem+'.txt'),'w').write(txt)
            tot+=1;states.append(1 if lb else 0)
            if tot%5==0:
                v=img.copy();cv2.rectangle(v,hb[:2],hb[2:],(0,255,0),1)
                if lb: cv2.rectangle(v,lb[:2],lb[2:],(0,0,255),2)
                cv2.imwrite(os.path.join(OUT,'preview',stem+'.jpg'),v)
        # 状态变化位置
        ch=[i for i in range(1,len(states)) if states[i]!=states[i-1]]
        print(nm,'frames',len(states),'lit',sum(states),'changes at',ch[:20])
    open(os.path.join(OUT,'classes.txt'),'w').write("red_on\n")
    open(os.path.join(OUT,'data.yaml'),'w').write(f"path: {OUT.replace(chr(92),'/')}\ntrain: images\nval: images\nnc: 1\nnames:\n  0: red_on\n")
    print('total',tot,'lit labeled',lit,'unlit(empty label)',tot-lit)
