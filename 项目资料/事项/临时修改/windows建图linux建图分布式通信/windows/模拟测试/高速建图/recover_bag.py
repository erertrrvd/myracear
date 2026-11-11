"""抢救没收尾的 rosbag2 .db3：不读 sqlite_master，按页扫描所有表叶子页，
找出 topics 行（id, name, type, ...）和 messages 行（id, topic_id, timestamp, data），写成新的 db3。
用法：python recover_bag.py <坏.db3> <新.db3>
"""
import sqlite3, struct, sys

src, dst = sys.argv[1], sys.argv[2]
buf = open(src, 'rb').read()
ps = struct.unpack_from('>H', buf, 16)[0]
ps = 65536 if ps == 1 else ps
U = ps - buf[20]
npages = len(buf) // ps


def varint(b, o):
    v = 0
    for i in range(9):
        c = b[o + i]
        if i == 8:
            return (v << 8) | c, o + 9
        v = (v << 7) | (c & 0x7f)
        if not c & 0x80:
            return v, o + i + 1
    return v, o + 9


def page(n):
    return buf[(n - 1) * ps: n * ps]


def payload(pg, o, P):
    X = U - 35
    if P <= X:
        return pg[o:o + P]
    M = ((U - 12) * 32 // 255) - 23
    K = M + ((P - M) % (U - 4))
    local = K if K <= X else M
    out = bytearray(pg[o:o + local])
    nxt = struct.unpack_from('>I', pg, o + local)[0]
    seen = 0
    while len(out) < P and 0 < nxt <= npages and seen < 10000:
        op = page(nxt)
        nxt = struct.unpack_from('>I', op, 0)[0]
        out += op[4:4 + min(U - 4, P - len(out))]
        seen += 1
    return bytes(out) if len(out) == P else None


def record(p):
    hs, o = varint(p, 0)
    types = []
    while o < hs:
        t, o = varint(p, o)
        types.append(t)
    vals, o = [], hs
    for t in types:
        if t == 0:
            vals.append(None)
        elif 1 <= t <= 6:
            n = (1, 2, 3, 4, 6, 8)[t - 1]
            vals.append(int.from_bytes(p[o:o + n], 'big', signed=True)); o += n
        elif t == 7:
            vals.append(struct.unpack_from('>d', p, o)[0]); o += 8
        elif t in (8, 9):
            vals.append(t - 8)
        elif t >= 12:
            n = (t - 12) // 2 if t % 2 == 0 else (t - 13) // 2
            v = p[o:o + n]; o += n
            vals.append(v if t % 2 == 0 else v.decode('utf-8', 'replace'))
        else:
            return None
    return vals


topics, msgs = {}, {}
for n in range(1, npages + 1):
    pg = page(n)
    h = 100 if n == 1 else 0
    if len(pg) < ps or pg[h] != 0x0D:
        continue
    nc = struct.unpack_from('>H', pg, h + 3)[0]
    for i in range(nc):
        try:
            co = struct.unpack_from('>H', pg, h + 8 + 2 * i)[0]
            if not 0 < co < ps:
                continue
            P, o = varint(pg, co)
            rowid, o = varint(pg, o)
            p = payload(pg, o, P)
            if p is None:
                continue
            r = record(p)
        except Exception:
            continue
        if not r:
            continue
        if len(r) == 4 and isinstance(r[1], int) and isinstance(r[2], int) and isinstance(r[3], bytes):
            msgs[rowid] = (rowid, r[1], r[2], r[3])
        elif len(r) >= 4 and isinstance(r[1], str) and r[1].startswith('/') and isinstance(r[2], str):
            topics[rowid] = r

con = sqlite3.connect(dst)
con.execute('CREATE TABLE topics(id INTEGER PRIMARY KEY, name TEXT NOT NULL, type TEXT NOT NULL, '
            'serialization_format TEXT NOT NULL, offered_qos_profiles TEXT NOT NULL)')
con.execute('CREATE TABLE messages(id INTEGER PRIMARY KEY, topic_id INTEGER NOT NULL, timestamp INTEGER NOT NULL, data BLOB NOT NULL)')
for rid, r in topics.items():
    con.execute('insert or replace into topics values (?,?,?,?,?)',
                (rid, r[1], r[2], r[3] if len(r) > 3 else 'cdr', r[4] if len(r) > 4 else ''))
con.executemany('insert or replace into messages values (?,?,?,?)', msgs.values())
con.commit()
cnt = {}
for _, tid, _, _ in msgs.values():
    cnt[tid] = cnt.get(tid, 0) + 1
names = {rid: r[1] for rid, r in topics.items()}
print('pages', npages, 'topics', {rid: r[1] for rid, r in topics.items()},
      'messages', {names.get(t, t): c for t, c in cnt.items()})
