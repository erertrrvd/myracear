#!/usr/bin/env python3
"""远程视觉的通信协议（小车和电脑共用）。

小车 → 电脑：二进制帧  [4 字节 JPEG 长度][4 字节帧号][8 字节小车拍摄时间] + JPEG 数据（大端）
电脑 → 小车：一行一个 JSON，以换行结尾：
  {"id":帧号,"state":"red","raw":"red","box":[x1,y1,x2,y2,conf]或null,"lamp_conf":0.9,"pc_ms":36,"w":640,"h":480}

只依赖标准库。小车上用 /usr/bin/python3，电脑上用 D:/light_ai/venv 的 Python 都能 import。
"""
import json
import socket
import struct

STATES = ("red", "green", "off", "none", "conflict")
HDR = struct.Struct(">IId")          # jpeg 长度, 帧号, 小车拍摄时间（小车单调时钟，只在小车上使用）
MAX_JPEG = 4_000_000
MAX_LINE = 65536


def recv_exact(sock, n):
    """读满 n 字节；对方关闭则抛 ConnectionError"""
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("对方已关闭连接")
        buf += chunk
    return bytes(buf)


def send_frame(sock, fid, t_cap, jpeg):
    sock.sendall(HDR.pack(len(jpeg), fid, t_cap) + jpeg)


def recv_frame(sock):
    n, fid, t_cap = HDR.unpack(recv_exact(sock, HDR.size))
    if n == 0 or n > MAX_JPEG:
        raise ValueError(f"帧长度异常：{n}")
    return fid, t_cap, recv_exact(sock, n)


def send_result(sock, d):
    sock.sendall(json.dumps(d, separators=(",", ":")).encode("utf-8") + b"\n")


class LineReader:
    """从套接字读 JSON 行；一次 read() 返回一个 dict"""

    def __init__(self, sock):
        self.sock, self.buf = sock, b""

    def read(self):
        while b"\n" not in self.buf:
            chunk = self.sock.recv(4096)
            if not chunk:
                raise ConnectionError("对方已关闭连接")
            self.buf += chunk
            if len(self.buf) > MAX_LINE:
                raise ValueError("结果行过长")
        line, self.buf = self.buf.split(b"\n", 1)
        return json.loads(line.decode("utf-8"))


def valid_result(d):
    """检查电脑回传的结果是否可用；不可用的直接丢弃，不能让它影响状态"""
    try:
        if d.get("state") not in STATES or not isinstance(d.get("id"), int):
            return False
        box = d.get("box")
        if box is not None and (len(box) != 5 or not all(isinstance(v, (int, float)) for v in box)):
            return False
        return True
    except AttributeError:
        return False


def enable_keepalive(sock, idle=3, interval=1, count=3):
    """尽快发现 WiFi 断开后的死连接（Linux 有细粒度参数，其他系统只开基本保活）"""
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
    for name, val in (("TCP_KEEPIDLE", idle), ("TCP_KEEPINTVL", interval), ("TCP_KEEPCNT", count)):
        if hasattr(socket, name):
            sock.setsockopt(socket.IPPROTO_TCP, getattr(socket, name), val)
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
