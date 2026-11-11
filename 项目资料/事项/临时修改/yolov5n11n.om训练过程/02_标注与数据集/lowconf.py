import sys,glob,os,cv2,numpy as np
sys.path.insert(0,'work');import label2
S=r"C:\Users\lzc2\AppData\Local\Temp\claude\C--Users-lzc2-Desktop-my-racecar----\ed0c2ab4-24e0-449c-86fd-50b744ec48f1\scratchpad"
rows=[]
for vd in sorted(glob.glob(r"D:\light_ai\data\images\red_near_20261028_*")):
    k=os.path.basename(vd)[-6:]
    for f in sorted(glob.glob(os.path.join(vd,'*.jpg'))):
        img=cv2.imread(f);box,d,src,amb=label2.housing2(img)
        if src=='lowconf':
            r=label2.MODEL.predict(img,conf=0.10,imgsz=640,verbose=False)[0]
            bs=[(float(b.conf),[int(v) for v in b.xyxy[0].tolist()]) for b in r.boxes if int(b.cls)==0]
            rows.append((f,k,bs,label2.lit_lamp_present(img)))
print('lowconf frames',len(rows),'with lit lamp',sum(1 for r in rows if r[3]))
import pickle;pickle.dump(rows,open(r"D:\light_ai\work\lowconf.pkl",'wb'))
t=[]
for f,k,bs,lit in rows:
    im=cv2.imread(f)
    for c,b in bs: cv2.rectangle(im,tuple(b[:2]),tuple(b[2:]),(0,255,255),2);cv2.putText(im,f'{c:.2f}',(b[0],max(12,b[1]-4)),0,0.6,(0,255,255),2)
    cv2.putText(im,k+'#'+os.path.basename(f)[6:10]+(' LIT' if lit else ''),(4,20),0,0.6,(0,0,255),2);t.append(cv2.resize(im,(256,192)))
cols=6
while len(t)%cols: t.append(np.zeros((192,256,3),np.uint8))
cv2.imwrite(os.path.join(S,'lowconf.jpg'),np.vstack([np.hstack(t[r*cols:r*cols+cols]) for r in range(len(t)//cols)]))
