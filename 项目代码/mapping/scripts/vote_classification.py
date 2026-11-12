import numpy as np


def classify_votes(hit, passes, threshold=.20, min_pass=3, strict_ratio=False):
    """优先保留至少两次命中的格子；单次命中按经过次数和比例判断。"""
    if strict_ratio:
        return np.where(passes > 0, np.where((hit > 0) & (hit >= threshold * passes), 100, 0), -1).astype(np.int8)
    occupied = ((hit >= threshold * passes) & (hit > 0)) | (hit >= 2)
    return np.where(occupied, 100, np.where(passes >= min_pass, 0, -1)).astype(np.int8)
