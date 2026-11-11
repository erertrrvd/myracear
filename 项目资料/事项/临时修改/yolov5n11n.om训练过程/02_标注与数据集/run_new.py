import sys,glob,os,cv2,numpy as np,json
sys.path.insert(0,'work');import label2
from collections import Counter
S=r"C:\Users\lzc2\AppData\Local\Temp\claude\C--Users-lzc2-Desktop-my-racecar----\ed0c2ab4-24e0-449c-86fd-50b744ec48f1\scratchpad"
res={}
for vd in sorted(glob.glob(r"D:\light_ai\data\images\red_near_20261028_*")):
    k=os.path.basename(vd)[-6:];fs=sorted(glob.glob(os.path.join(vd,'*.jpg')));cnt=Counter();seq=[]
    for f in fs:
        img=cv2.imread(f);box,d,src=label2.housing(img)
        ls=label2.lamps(img,box,d) if box else []
        st='none' if not box else ('+'.join(sorted(set(n for n,_ in ls))) or 'off')
        cnt[st]+=1;cnt['src_'+src]+=1;seq.append(st)
        res[f]=dict(box=box,d=d,src=src,lamps=ls)
    print(k,dict(cnt))
    # 状态变化序列
    ch=[(i,seq[i]) for i in range(len(seq)) if i==0 or seq[i]!=seq[i-1]]
    print('   changes:',ch[:14])
import pickle;pickle.dump(res,open(r"D:\light_ai\work\new_labels.pkl",'wb'))
