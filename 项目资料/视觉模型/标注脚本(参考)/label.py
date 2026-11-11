import cv2,glob,os,numpy as np,json
ROOT=r"D:\light_ai\data"
OUT=os.path.join(ROOT,"imagesbiaozhu")
FIXED=(200,262,467,340)   # 视频1：摄像头固定，灯箱不动
def orange_blob(img):
    hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    m=((hsv[...,0]<=13)&(hsv[...,1]>140)&(hsv[...,2]>120)).astype(np.uint8)*255
    m=cv2.morphologyEx(m,cv2.MORPH_OPEN,np.ones((3,3),np.uint8))
    n,lab,st,cen=cv2.connectedComponentsWithStats(m)
    best=None;bs=0
    H,W=img.shape[:2]
    for i in range(1,n):
        x,y,w,h,a=st[i]
        if a<120 or a>6000: continue
        if not(0.7<w/h<1.4): continue
        if a/(w*h)<0.55: continue
        if y>H*0.8 or y<H*0.2: continue
        # need a dark region to its right (green lamp / housing)
        rx=min(W-1,int(x+w+w*0.9)); ry=int(y+h/2)
        if img[ry,rx].mean()>110: continue
        sc=a
        if sc>bs: bs=sc;best=(x+w/2,y+h/2,(w+h)/2)
    return best
def box_from(b,W,H):
    ox,oy,d=b
    return (int(max(0,ox-3.1*d)),int(max(0,oy-0.95*d)),int(min(W-1,ox+3.1*d)),int(min(H-1,oy+1.0*d)))
def run(vdir):
    fs=sorted(glob.glob(os.path.join(vdir,'*.jpg')))
    res={}
    for f in fs:
        img=cv2.imread(f);H,W=img.shape[:2]
        b=orange_blob(img)
        res[f]=box_from(b,W,H) if b else None
    return res

def refine(img,box,b):
    """用灯箱深色区域修正框；结果与预测差太多就保留预测"""
    H,W=img.shape[:2]
    x1,y1,x2,y2=box; w=x2-x1;h=y2-y1
    sx1=int(max(0,x1-0.3*w));sx2=int(min(W,x2+0.3*w));sy1=int(max(0,y1-0.45*h));sy2=int(min(H,y2+0.45*h))
    roi=img[sy1:sy2,sx1:sx2]
    g=cv2.cvtColor(roi,cv2.COLOR_BGR2GRAY)
    hsv=cv2.cvtColor(roi,cv2.COLOR_BGR2HSV)
    dark=(g<85)
    lamp=((hsv[...,1]>120)&(hsv[...,2]>120))|(g>235)   # 灯本身（橙/红/强光白）
    m=(dark|lamp).astype(np.uint8)*255
    m=cv2.morphologyEx(m,cv2.MORPH_CLOSE,np.ones((9,9),np.uint8))
    m=cv2.morphologyEx(m,cv2.MORPH_OPEN,np.ones((5,5),np.uint8))
    n,lab,st,_=cv2.connectedComponentsWithStats(m)
    cx=int(b[0]-sx1);cy=int(b[1]-sy1)
    cx=min(max(cx,0),m.shape[1]-1);cy=min(max(cy,0),m.shape[0]-1)
    l=lab[cy,cx]
    if l==0: return box
    X,Y,Wd,Ht,_=st[l]
    rb=(sx1+X,sy1+Y,sx1+X+Wd,sy1+Y+Ht)
    rw=rb[2]-rb[0];rh=rb[3]-rb[1]
    if not(0.8*w<=rw<=1.35*w and 0.8*h<=rh<=1.5*h and 2.0<=rw/rh<=5.0): return box
    return rb
def run2(vdir,fixed=None):
    fs=sorted(glob.glob(os.path.join(vdir,'*.jpg')));res={}
    last=None
    for f in fs:
        img=cv2.imread(f);H,W=img.shape[:2]
        if fixed: res[f]=fixed;continue
        b=orange_blob(img)
        if b is None: res[f]=last;continue
        bx=box_from(b,W,H);bx=refine(img,bx,b);res[f]=bx;last=bx
    return res

def profile_box(img,b):
    """从橙灯中心向四周扩展到灯箱（深色/灯色）边缘"""
    H,W=img.shape[:2]
    ox,oy,d=b
    g=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY);hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    hous=((g<90)|((hsv[...,1]>110)&(hsv[...,2]>110))|(g>240)).astype(np.uint8)
    # 横向：取过橙灯中心的细带
    ya=int(max(0,oy-0.35*d));yb=int(min(H,oy+0.35*d))
    band=hous[ya:yb].mean(0)
    def ext(start,step,limit,thr=0.6,gap=6):
        x=int(start);bad=0;last=x
        while 0<=x<limit:
            if band[x]>=thr: bad=0;last=x
            else:
                bad+=1
                if bad>=gap: break
            x+=step
        return last
    xl=ext(ox,-1,W);xr=ext(ox,1,W)
    # 纵向：取灯箱左右范围里靠中间的列带（避开斜边）
    c1=int(max(0,ox-1.0*d));c2=int(min(W,ox+1.0*d))
    col=hous[:,c1:c2].mean(1)
    def exty(start,step,thr=0.55,gap=4):
        y=int(start);bad=0;last=y
        while 0<=y<H:
            if col[y]>=thr: bad=0;last=y
            else:
                bad+=1
                if bad>=gap: break
            y+=step
        return last
    yt=exty(oy,-1);yb2=exty(oy,1)
    return (xl,yt,xr,yb2)

def run3(vdir,fixed=None):
    fs=sorted(glob.glob(os.path.join(vdir,'*.jpg')));res={};last=None;info={}
    for f in fs:
        img=cv2.imread(f);H,W=img.shape[:2]
        if fixed: res[f]=fixed;continue
        b=orange_blob(img)
        if b is None: res[f]=last;continue
        pred=box_from(b,W,H);pb=profile_box(img,b)
        w=pb[2]-pb[0];h=pb[3]-pb[1];pw=pred[2]-pred[0];ph=pred[3]-pred[1]
        ok=(0.9*pw<=w<=1.5*pw) and (0.8*ph<=h<=1.6*ph) and 2.0<=w/max(h,1)<=5.0
        res[f]=pb if ok else pred
        info[f]=ok;last=res[f]
    return res,info
