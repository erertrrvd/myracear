#!/usr/bin/env python3
"""读取 .om 文件头里嵌着的转换信息，确认它是按我们要求转的（不需要昇腾环境，普通 Python 就行）。
用法：python check_om.py yolov5n_bs1.om
"""
import sys

b = open(sys.argv[1], "rb").read()
s = b.decode("latin1")
print("文件大小", len(b), "字节；文件头魔数", b[:4])


def clean(t):
    out = []
    for ch in t:
        if ord(ch) < 32 or ord(ch) > 126:
            break
        out.append(ch)
    return "".join(out)


i = s.find("--model=")
cmd = clean(s[max(0, s.rfind("atc", 0, i)): i + 600]) if i >= 0 else "(没找到)"
print("转换命令:", cmd)
for key in ("Ascend310B4", "images", "1,3,640,640", "7.0.RC1", "7.0.0"):
    print(f"  含 {key!r}:", key in s)
want = ("--soc_version=Ascend310B4", "--input_fp16_nodes=images", "images:1,3,640,640", "--output_type=FP16")
print("参数检查:", "全部符合" if all(w in cmd for w in want) else "有不符合项，缺：" + str([w for w in want if w not in cmd]))
