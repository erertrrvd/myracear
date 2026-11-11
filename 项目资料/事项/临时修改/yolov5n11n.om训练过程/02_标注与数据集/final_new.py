import sys,glob,os,cv2,numpy as np,shutil
sys.path.insert(0,'work');import label2
from collections import Counter
OUT=r"D:\light_ai\data\imagesbiaozhu_merged"
for old in glob.glob(os.path.join(OUT,'images','11*.jpg'))+glob.glob(os.path.join(OUT,'labels','11*.txt'))+glob.glob(os.path.join(OUT,'preview','11*.jpg')): os.remove(old)
cnt=Counter();per={};kept={}
for vd in sorted(glob.glob(r"D:\light_ai\data\images\red_near_20261028_*")):
    k=os.path.basename(vd)[-6:];c=Counter()
    for f in sorted(glob.glob(os.path.join(vd,'*.jpg'))):
        img=cv2.imread(f);H,W=img.shape[:2]
        box,d,src,amb=label2.housing2(img)
        if amb: c['skipped_ambiguous']+=1;continue
        if box and (box[0]<=2 or box[1]<=2 or box[2]>=W-3 or box[3]>=H-3):
            c['skipped_touch_border']+=1;continue            # 灯箱被切在画面边缘：灯可能只露出一半
        if (box is None) and label2.lit_lamp_present(img):
            c['skipped_lit_lamp_no_box']+=1;continue         # 画面里有亮着的灯但没框到灯箱：不能当负样本
        lines=[];ls=[]
        if box:
            x1,y1,x2,y2=[min(max(v,0),l) for v,l in zip(box,(W,H,W,H))]
            lines.append(f"0 {(x1+x2)/2/W:.6f} {(y1+y2)/2/H:.6f} {(x2-x1)/W:.6f} {(y2-y1)/H:.6f}")
            ls=label2.lamps(img,(x1,y1,x2,y2),d)
            for nm,(a,b,cc,dd) in ls:
                lines.append(f"{1 if nm=='red_on' else 2} {(a+cc)/2/W:.6f} {(b+dd)/2/H:.6f} {(cc-a)/W:.6f} {(dd-b)/H:.6f}")
            st='+'.join(sorted(set(n for n,_ in ls))) or 'off'
        else: st='none'
        stem=f"{k}_{os.path.splitext(os.path.basename(f))[0]}"
        shutil.copyfile(f,os.path.join(OUT,'images',stem+'.jpg'))
        open(os.path.join(OUT,'labels',stem+'.txt'),'w').write("\n".join(lines)+("\n" if lines else ""))
        c[st]+=1;c['src_'+src]+=1;kept[stem]=(box,ls)
    per[k]=c;print(k,dict(c))
open(os.path.join(OUT,'classes.txt'),'w').write("light_box\nred_on\ngreen_on\n")
open(os.path.join(OUT,'data.yaml'),'w').write(f"path: {OUT.replace(chr(92),'/')}\ntrain: images\nval: images\nnc: 3\nnames:\n  0: light_box\n  1: red_on\n  2: green_on\n")
import pickle;pickle.dump(kept,open(r"D:\light_ai\work\final_kept.pkl",'wb'))
tot=Counter()
for lab in glob.glob(os.path.join(OUT,'labels','*.txt')):
    for l in open(lab):
        if l.strip(): tot[l.split()[0]]+=1
print('ALL images',len(glob.glob(os.path.join(OUT,'images','*.jpg'))),'boxes by class',dict(tot))
