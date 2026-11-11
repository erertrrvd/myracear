#!/usr/bin/env python3
"""检查导出的 YOLOv5 ONNX 是否符合昇腾转换要求：输入名 images、形状 [1,3,640,640]、输出 [1,25200,5+类别数]、opset 12。
用法：D:/light_ai/venv/Scripts/python.exe check_onnx.py best.onnx
需要 onnx 和 onnxruntime。
"""
import hashlib
import sys

import numpy as np
import onnx
import onnxruntime as ort

p = sys.argv[1]
m = onnx.load(p)
onnx.checker.check_model(m)
print("文件", p, "SHA256", hashlib.sha256(open(p, "rb").read()).hexdigest()[:16])
print("opset", [(o.domain or "ai.onnx", o.version) for o in m.opset_import], "ir_version", m.ir_version)
for kind, ts in (("输入", m.graph.input), ("输出", m.graph.output)):
    for x in ts:
        print(kind, x.name, [d.dim_value or d.dim_param for d in x.type.tensor_type.shape.dim])
print("算子", sorted({n.op_type for n in m.graph.node}))
s = ort.InferenceSession(p, providers=["CPUExecutionProvider"])
out = s.run(None, {s.get_inputs()[0].name: np.zeros((1, 3, 640, 640), np.float32)})
print("onnxruntime 输出形状", [o.shape for o in out])
ok = (s.get_inputs()[0].name == "images" and list(s.get_inputs()[0].shape) == [1, 3, 640, 640]
      and out[0].ndim == 3 and out[0].shape[1] == 25200)
print("符合要求" if ok else "不符合要求：检查导出时是否用了 --dynamic、输入尺寸是否 640")
