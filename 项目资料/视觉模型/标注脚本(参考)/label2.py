import sys,glob,os,cv2,numpy as np,shutil
sys.path.insert(0,'work');import label
from ultralytics import YOLO
shutil.copyfile(r"D:\light_ai\runs\light_2cls\weights\best.pt",r"D:\light_ai\work\_prop.pt")
MODEL=YOLO(r"D:\light_ai\work\_prop.pt")
def iou(a,b):
    ix=max(0,min(a[2],b[2])-max(a[0],b[0]));iy=max(0,min(a[3],b[3])-max(a[1],b[1]));i=ix*iy
    u=(a[2]-a[0])*(a[3]-a[1])+(b[2]-b[0])*(b[3]-b[1])-i;return i/u if u>0 else 0
def housing(img):
    """返回 (box, d_lamp, source)；box 可能为 None"""
    H,W=img.shape[:2]
    ob=label.orange_blob(img)
    A=None
    if ob:
        pred=label.box_from(ob,W,H);pb=label.profile_box(img,ob)
        w=pb[2]-pb[0];h=pb[3]-pb[1];pw=pred[2]-pred[0];ph=pred[3]-pred[1]
        ok=(0.9*pw<=w<=1.5*pw) and (0.8*ph<=h<=1.6*ph) and 2.0<=w/max(h,1)<=5.0
        A=pb if ok else pred
    r=MODEL.predict(img,conf=0.3,imgsz=640,verbose=False)[0]
    B=None;bc=0
    for b in r.boxes:
        if int(b.cls)!=0: continue
        x1,y1,x2,y2=b.xyxy[0].tolist();ar=(x2-x1)/max(y2-y1,1);cy=(y1+y2)/2/H
        if 2.3<=ar<=4.8 and 0.1<=cy<=0.9 and float(b.conf)>bc: bc=float(b.conf);B=(int(x1),int(y1),int(x2),int(y2))
    if A and B:
        return (A,ob[2],'orange' if iou(A,B)>=0.5 else 'DISAGREE')
    if A: return (A,ob[2],'orange')
    if B: return (B,None,'model')
    return (None,None,'none')
def lamps(img,box,d):
    """返回 [('red_on'|'green_on',(x1,y1,x2,y2))]"""
    H,W=img.shape[:2];x1,y1,x2,y2=box;w=x2-x1;h=y2-y1
    if d is None: d=0.16*w
    g=img.max(axis=2)   # V 通道：绿灯亮时是亮绿色，灰度不够亮
    ya=int(y1+0.15*h);yb=int(y2-0.10*h)
    roi=g[ya:yb,x1:x2]
    core=(roi>225).astype(np.uint8)
    core=cv2.morphologyEx(core,cv2.MORPH_OPEN,np.ones((3,3),np.uint8))
    n,lab,st,cen=cv2.connectedComponentsWithStats(core)
    out=[]
    for i in range(1,n):
        a=st[i,4]
        if a<0.10*np.pi*(d/2)**2 or a>12.0*np.pi*(d/2)**2: continue   # 太小=噪点，太大=不是灯
        cx=x1+cen[i][0];cy=ya+cen[i][1]
        rel=(cx-x1)/w
        if rel<0.36: name='red_on'
        elif rel>0.64: name='green_on'
        else: continue   # 中间=橙灯，这次不标
        hsv=cv2.cvtColor(img[int(max(0,cy-d*0.9)):int(cy+d*0.9),int(max(0,cx-d*0.9)):int(cx+d*0.9)],cv2.COLOR_BGR2HSV)
        hue=hsv[...,0][(hsv[...,1]>90)&(hsv[...,2]>120)]
        if hue.size<10: continue
        red_like=((hue<14)|(hue>160)).mean();green_like=((hue>60)&(hue<100)).mean()
        if name=='red_on' and red_like<0.3: continue       # 光晕颜色要对得上
        if name=='green_on' and green_like<0.3: continue
        hl=0.55*d
        out.append((name,(int(max(0,cx-hl)),int(max(0,cy-hl)),int(min(W-1,cx+hl)),int(min(H-1,cy+hl)))))
    return out


def housing2(img):
    """优先用模型灯箱框；返回 (box, d, src, ambiguous)"""
    H,W=img.shape[:2]
    r=MODEL.predict(img,conf=0.10,imgsz=640,verbose=False)[0]
    cands=[]
    for b in r.boxes:
        if int(b.cls)!=0: continue
        x1,y1,x2,y2=[int(v) for v in b.xyxy[0].tolist()];c=float(b.conf)
        ar=(x2-x1)/max(y2-y1,1);cy=(y1+y2)/2/H
        cands.append((c,(x1,y1,x2,y2),2.3<=ar<=4.8 and 0.1<=cy<=0.9))
    good=[c for c in cands if c[0]>=0.5 and c[2]]
    if good:
        c,bx,_=max(good);return bx,None,'model',False
    ob=label.orange_blob(img)
    if ob:
        pred=label.box_from(ob,W,H);pb=label.profile_box(img,ob)
        w=pb[2]-pb[0];h=pb[3]-pb[1];pw=pred[2]-pred[0];ph=pred[3]-pred[1]
        ok=(0.9*pw<=w<=1.5*pw) and (0.8*ph<=h<=1.6*ph) and 2.0<=w/max(h,1)<=5.0
        bx=pb if ok else pred
        touch=bx[0]<=3 or bx[1]<=3 or bx[2]>=W-4 or bx[3]>=H-4
        ar=(bx[2]-bx[0])/max(bx[3]-bx[1],1)
        if ok and not touch and 2.5<=ar<=4.6 and (bx[2]-bx[0])>=60:
            return bx,ob[2],'orange',False
        return None,None,'weak',True
    if any(c[0]>=0.10 for c in cands): return None,None,'lowconf',True   # 模型隐约看到灯箱：拿不准，剔除
    return None,None,'none',False


def lit_lamp_present(img):
    """画面里是否有亮着的红/绿灯（白色亮核 + 红/绿光晕）"""
    V=img.max(axis=2);hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    core=(V>232).astype(np.uint8);n,lab,st,cen=cv2.connectedComponentsWithStats(core)
    H,W=V.shape
    for i in range(1,n):
        a=st[i,4]
        if a<25 or a>6000: continue
        r=max(6,int((a/np.pi)**0.5*1.8));cx,cy=int(cen[i][0]),int(cen[i][1])
        x1,x2,y1,y2=max(0,cx-r),min(W,cx+r),max(0,cy-r),min(H,cy+r)
        sub=hsv[y1:y2,x1:x2];hue=sub[...,0][(sub[...,1]>90)&(sub[...,2]>120)]
        if hue.size<15: continue
        if ((hue<14)|(hue>160)).mean()>0.35 or ((hue>60)&(hue<100)).mean()>0.35: return True
    return False
