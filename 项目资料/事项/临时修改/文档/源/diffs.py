# -*- coding: utf-8 -*-
r"""生成《代码改动记录.html》：大模块 → 功能 → 文件，每个文件可展开看代码改动。
由 build.py 在生成四份 PDF 后自动调用；也可单独运行：python 文档\源\diffs.py
规则说明见 C:\Users\lzc2\Desktop\AI约束文件\代码修改记录网页生成\代码修改记录网页生成规范.md

内容：
  features.py   模块、功能、要求、实现、状态、每个文件的作用、来源（提交 / 记录点）
  git           已提交的改动（features 里写的提交号）
  改动快照\     未提交和仓库外文件的记录点：每次运行时，和上次记录不同的文件存一份快照
每个功能显示"开始做之前 → 功能最终完成"的净改动；状态为 verify / wip 且 follow=True 的功能，
以后新记录点改到它的文件会自动并进来（跟进）。新增 / 删除的文件也显示完整代码。
时间一律用北京时间（电脑系统时区是美东）。
"""
import datetime
import difflib
import html
import importlib
import json
import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(os.path.dirname(HERE), "代码改动记录.html")
STORE = os.path.join(HERE, "改动快照")
REPO = "C:\\Users\\lzc2\\Desktop\\my_racecar（复件）"
BASE_COMMIT = "641ae59"     # 未提交改动的起点；之后有新提交时，在 features.py 里写进对应功能
BJ = datetime.timezone(datetime.timedelta(hours=8))

# 仓库外、但属于本项目的文件：显示名 -> 实际路径
OUTSIDE = {
    "电脑 D:\\ubuntu\\slam\\run_slam_windows.ps1": "D:\\ubuntu\\slam\\run_slam_windows.ps1",
    "电脑 D:\\ubuntu\\slam\\view_csv.ps1": "D:\\ubuntu\\slam\\view_csv.ps1",
    "电脑 PowerShell 配置 Microsoft.PowerShell_profile.ps1": "C:\\Users\\lzc2\\Documents\\WindowsPowerShell\\Microsoft.PowerShell_profile.ps1",
}
CONTEXT = 3
STATUS = {
    "done": ("已完成并验证", "s-done"),
    "verify": ("代码完成 · 待验证（跟进中）", "s-verify"),
    "wip": ("进行中（跟进中）", "s-wip"),
}


# ------------------------------------------------------------------ 读文件 / git
def git(*args):
    r = subprocess.run(["git", "-C", REPO, "-c", "core.quotepath=false", *args], capture_output=True)
    return r.stdout if r.returncode == 0 else None


def decode(b):
    """None -> None；二进制 -> False；否则文本（utf-8，失败用 gbk，如 pure_track_test.cpp）。"""
    if b is None:
        return None
    if b"\0" in b[:8000]:
        return False
    for enc in ("utf-8-sig", "gbk"):
        try:
            return b.decode(enc).replace("\r\n", "\n")
        except UnicodeDecodeError:
            pass
    return b.decode("utf-8", "replace").replace("\r\n", "\n")


def git_text(rev, path):
    return decode(git("show", f"{rev}:{path}"))


def read_file(path):
    try:
        with open(path, "rb") as f:
            return decode(f.read())
    except OSError:
        return None


def nlines(t):
    return 0 if not t else t.count("\n") + (0 if t.endswith("\n") else 1)


# ------------------------------------------------------------------ 差异
def hunks(old, new):
    a, b = old.split("\n"), new.split("\n")
    sm = difflib.SequenceMatcher(None, a, b, autojunk=False)
    out = []
    for group in sm.get_grouped_opcodes(CONTEXT):
        rows = []
        for tag, i1, i2, j1, j2 in group:
            if tag == "equal":
                rows += [("ctx", i + 1, j1 + (i - i1) + 1, a[i]) for i in range(i1, i2)]
            else:
                rows += [("del", i + 1, None, a[i]) for i in range(i1, i2)]
                rows += [("add", None, j + 1, b[j]) for j in range(j1, j2)]
        o0, n0 = group[0][1], group[0][3]
        out.append((f"@@ -{o0 + 1},{group[-1][2] - o0} +{n0 + 1},{group[-1][4] - n0} @@", rows))
    return out


def whole(text, kind):
    lines = text.split("\n")
    if lines and lines[-1] == "":
        lines.pop()
    if kind == "add":
        return [("全部内容（新增）", [("add", None, i + 1, l) for i, l in enumerate(lines)])]
    return [("全部内容（删除前）", [("del", i + 1, None, l) for i, l in enumerate(lines)])]


def change_of(old, new):
    """返回 (类型, 段落, +行数, -行数)；类型 add / del / mod / move / bin / same。"""
    if old is False or new is False:
        return "bin", [], 0, 0
    if old is None and isinstance(new, str):
        return "add", whole(new, "add"), nlines(new), 0
    if isinstance(old, str) and new is None:
        return "del", whole(old, "del"), 0, nlines(old)
    if old == new:
        return "same", [], 0, 0
    hs = hunks(old, new)
    p = sum(r[0] == "add" for _, rows in hs for r in rows)
    m = sum(r[0] == "del" for _, rows in hs for r in rows)
    return "mod", hs, p, m


# ------------------------------------------------------------------ 来源：提交 / 记录点
def commit_time(c):
    iso = decode(git("show", "-s", "--format=%aI", c) or b"")
    return datetime.datetime.fromisoformat(iso.strip()).astimezone(BJ) if iso else None


def commit_changes(c):
    """[(旧路径, 新路径, 旧内容, 新内容)]"""
    raw = decode(git("diff-tree", "--no-commit-id", "-r", "-M", "--name-status", f"{c}^", c) or b"")
    out = []
    for line in raw.splitlines():
        cols = line.split("\t")
        if len(cols) < 2:
            continue
        st = cols[0][0]
        if st == "R":
            out.append((cols[1], cols[2], git_text(f"{c}^", cols[1]), git_text(c, cols[2])))
        elif st == "A":
            out.append((None, cols[1], None, git_text(c, cols[1])))
        elif st == "D":
            out.append((cols[1], None, git_text(f"{c}^", cols[1]), None))
        else:
            out.append((cols[1], cols[1], git_text(f"{c}^", cols[1]), git_text(c, cols[1])))
    return out


def key_of(name):
    return re.sub(r'[\\/:*?"<>| ]', "_", name)


def load_index():
    try:
        with open(os.path.join(STORE, "index.json"), encoding="utf-8") as f:
            return json.load(f)
    except OSError:
        return []


def save_index(index):
    os.makedirs(STORE, exist_ok=True)
    with open(os.path.join(STORE, "index.json"), "w", encoding="utf-8") as f:
        json.dump(index, f, ensure_ascii=False, indent=1)


def save_snap(cp, name, which, text):
    if text is None or text is False:
        return None
    rel = os.path.join(cp, key_of(name) + "." + which)
    os.makedirs(os.path.join(STORE, cp), exist_ok=True)
    with open(os.path.join(STORE, rel), "w", encoding="utf-8", newline="\n") as f:
        f.write(text)
    return rel


def load_snap(rel):
    if rel is None:
        return None
    with open(os.path.join(STORE, rel), encoding="utf-8") as f:
        return f.read()


def cp_time(cp):
    return datetime.datetime.strptime(cp["time"], "%Y-%m-%d %H:%M").replace(tzinfo=BJ)


def cp_changes(cp):
    return [(name, name, load_snap(e["old"]), load_snap(e["new"])) for name, e in cp["files"].items()]


def current_state():
    """{显示名: (当前内容, 基准内容)}；基准 = BASE 提交里的版本（仓库外文件为 None）。"""
    state = {}
    changed = decode(git("diff", "HEAD", "--name-only") or b"").splitlines()
    untracked = decode(git("ls-files", "--others", "--exclude-standard") or b"").splitlines()
    for p in changed + untracked:
        p = p.strip()
        if not p or p.startswith(".claude/"):     # Claude 的本地配置，不是项目代码
            continue
        full = os.path.join(REPO, p.replace("/", os.sep))
        if os.path.exists(full) and os.path.getsize(full) > 2_000_000:
            continue
        state[p] = (read_file(full), git_text("HEAD", p))
    for name, path in OUTSIDE.items():
        state[name] = (read_file(path), None)
    return state


def add_checkpoint(index, when, label, files_old_new, note=""):
    cp = base_id = when.strftime("%Y%m%d_%H%M%S")
    n = 1
    while os.path.exists(os.path.join(STORE, cp)):    # 同一秒两次时不覆盖
        n += 1
        cp = f"{base_id}_{n}"
    files = {}
    for name, (prev, now) in files_old_new.items():
        files[name] = {"old": save_snap(cp, name, "old", prev), "new": save_snap(cp, name, "new", now),
                       "outside": name in OUTSIDE}
    index.append({"id": cp, "time": when.strftime("%Y-%m-%d %H:%M"), "label": label, "note": note, "files": files})
    return cp


def latest_label():
    try:
        sys.path.insert(0, HERE)
        import changes
        importlib.reload(changes)
        return changes.LOG["blocks"][0][2][-1][2]
    except Exception:
        return "自动记录"


def checkpoint(label=None, when=None):
    """有文件和上次记录不同就新建一个记录点；返回 id 或 None。"""
    index = load_index()
    last = {}
    for cp in sorted(index, key=lambda c: c["time"]):
        for name, ent in cp["files"].items():
            last[name] = ent["new"]
    entries = {}
    for name, (now, base) in current_state().items():
        prev = load_snap(last[name]) if name in last else base
        if now == prev or now is False or (name not in last and now is None):
            continue
        entries[name] = (prev, now)
    if not entries:
        return None
    cp = add_checkpoint(index, when or datetime.datetime.now(BJ), label or latest_label(), entries)
    save_index(index)
    return cp


# ------------------------------------------------------------------ 功能 = 来源的净改动
def load_features():
    sys.path.insert(0, HERE)
    import features
    importlib.reload(features)
    return features.MODULES


def collect():
    """返回 (modules, 未归类列表)。每个功能补上 _files: [(路径, 说明, 旧, 新, 首次记录?)]、_time、_sources。"""
    modules = load_features()
    index = {c["id"]: c for c in load_index()}
    src_cache = {}

    def source(s):
        if s not in src_cache:
            if s.startswith("cp:"):
                cp = index.get(s[3:])
                src_cache[s] = (cp_time(cp), cp_changes(cp), cp) if cp else (None, [], None)
            else:
                src_cache[s] = (commit_time(s), commit_changes(s), None)
        return src_cache[s]

    used = set()       # (来源, 路径)
    follow = []        # (功能, 最后来源时间)
    for mod in modules:
        for ft in mod["features"]:
            srcs = sorted(ft["sources"], key=lambda s: source(s)[0] or datetime.datetime.max.replace(tzinfo=BJ))
            ft["_srcs"] = list(srcs)
            # 待验证 / 进行中的功能默认跟进；CLAUDE.md 这类一直在改的文件可在已完成的功能上写 follow=True
            if ft.get("follow", ft["status"] in ("verify", "wip")):
                follow.append(ft)
    # 跟进：比功能最后一个来源晚的记录点，改到它的文件就并进来（多个功能都跟进同一文件时给最晚开始的）
    all_cps = sorted(index.values(), key=lambda c: c["time"])
    named = {s for mod in modules for ft in mod["features"] for s in ft["sources"]}
    for cp in all_cps:
        sid = "cp:" + cp["id"]
        if sid in named:
            continue
        for name in cp["files"]:
            cands = [ft for ft in follow if name in ft["files"] and source(ft["_srcs"][-1])[0] <= cp_time(cp)]
            if cands:
                ft = max(cands, key=lambda f: source(f["_srcs"][0])[0])
                if sid not in ft["_srcs"]:
                    ft["_srcs"].append(sid)
    for mod in modules:
        for ft in mod["features"]:
            chain = {}     # 当前路径 -> [最早路径, 旧内容, 新内容, 首次记录?]
            for s in ft["_srcs"]:
                t, changes, cp = source(s)
                for old_p, new_p, old, new in changes:
                    path = new_p or old_p
                    want = path in ft["files"] or (old_p in ft["files"])
                    if not want:
                        continue
                    used.add((s, path))
                    if old_p and old_p in chain:
                        ent = chain.pop(old_p)
                        ent[2] = new
                    else:
                        first = cp is not None and cp["files"].get(path, {}).get("outside") and old is None
                        ent = [old_p, old, new, first]
                    chain[path] = ent
            files = []
            for path, desc in ft["files"].items():
                hit = chain.get(path) or next((v for k, v in chain.items() if v[0] == path), None)
                if hit is None:
                    continue
                key = path if path in chain else next(k for k, v in chain.items() if v[0] == path)
                label = key if not hit[0] or hit[0] == key else f"{hit[0]} → {key}"
                files.append((label, desc, hit[1], hit[2], hit[3]))
            ft["_files"] = files
            times = [source(s)[0] for s in ft["_srcs"] if source(s)[0]]
            ft["_time"] = min(times) if times else None
            ft["_last"] = max(times) if times else None
    # 未归类：记录点、BASE 之后的新提交里，没写进任何功能的文件
    loose = []
    for cp in all_cps:
        sid = "cp:" + cp["id"]
        for name, e in cp["files"].items():
            if (sid, name) not in used:
                loose.append((f"{cp['time']}  {cp['label']}", name, load_snap(e["old"]), load_snap(e["new"])))
    new_commits = decode(git("rev-list", "--reverse", f"{BASE_COMMIT}..HEAD") or b"").split()
    for c in new_commits:
        c7 = c[:7]
        if c7 in named:
            continue
        for old_p, new_p, old, new in commit_changes(c):
            loose.append((f"提交 {c7}", new_p or old_p, old, new))
    return modules, loose


# ------------------------------------------------------------------ 渲染
esc = html.escape
TAG = {"add": ("增", "t-add"), "del": ("删", "t-del"), "mod": ("改", "t-mod"), "move": ("移", "t-move"),
       "bin": ("二进制", "t-move"), "first": ("首", "t-first")}


def render_rows(hs):
    out = []
    for head, rows in hs:
        out.append(f'<tr class="hunk"><td class="ln"></td><td class="ln"></td><td class="code">{esc(head)}</td></tr>')
        for kind, o, n, text in rows:
            sign = {"add": "+", "del": "−", "ctx": " "}[kind]
            out.append(f'<tr class="{kind}"><td class="ln">{o or ""}</td><td class="ln">{n or ""}</td>'
                       f'<td class="code"><span class="sign">{sign}</span>{esc(text) or " "}</td></tr>')
    return "".join(out)


def render_file(label, desc, old, new, first=False):
    kind, hs, p, m = change_of(old, new)
    if kind == "same":
        if "→" not in label:
            return "", 0, 0
        kind = "move"
    if first and kind == "add":
        kind = "first"
    tag, cls = TAG[kind]
    note = {"add": f"新增 {p} 行", "del": f"删除 {m} 行", "mod": "", "move": "只移动位置，内容没变",
            "bin": "二进制文件，不显示", "first": f"第一次存版本（{p} 行），之前的改动没有备份"}[kind]
    stat = (f'<span class="fstat"><b class="p">+{p}</b><b class="m">−{m}</b></span>' if kind in ("add", "del", "mod", "first") else "")
    head = (f'<span class="tag {cls}">{tag}</span><span class="fpath"><code>{esc(label)}</code>'
            f'<span class="fdesc">{esc(desc)}</span></span>{stat}')
    if not hs:
        return f'<div class="file plain"><div class="fhead">{head}<span class="fnote">{esc(note)}</span></div></div>', p, m
    note_html = f'<div class="fnote2">{esc(note)}</div>' if note else ""
    return (f'<details class="file"><summary class="fhead">{head}</summary>{note_html}'
            f'<div class="diff"><table>{render_rows(hs)}</table></div></details>'), p, m


def render_feature(ft, idx):
    name, cls = STATUS[ft["status"]]
    parts, P, M = [], 0, 0
    for f in ft["_files"]:
        h, p, m = render_file(*f)
        if h:
            parts.append(h)
            P += p
            M += m
    chips = []
    for s in ft["_srcs"]:
        if s.startswith("cp:"):
            i = s[3:]     # 20261017_201900
            chips.append(f'<span class="chip chip-cp">未提交 · {i[4:6]}-{i[6:8]} {i[9:11]}:{i[11:13]}</span>')
        else:
            chips.append(f'<span class="chip chip-git">提交 {esc(s)}</span>')
    return f'''<details class="feat" id="{idx}">
<summary><div class="ftop"><span class="status {cls}">{esc(name)}</span><span class="fdate">{esc(ft["date"])}</span></div>
<div class="ftitle">{esc(ft["title"])}</div>
<div class="fmeta">{len(parts)} 个文件 <b class="p">+{P}</b> <b class="m">−{M}</b></div></summary>
<div class="fbody">
<div class="nr"><div class="box need"><h4>要求</h4><p>{esc(ft["need"])}</p></div><div class="box impl"><h4>实现</h4><p>{esc(ft["impl"])}</p></div></div>
<div class="snote"><span class="status {cls}">{esc(name)}</span>{esc(ft.get("status_note", ""))}</div>
<div class="chips">来源：{"".join(chips)}</div>
<div class="files">{"".join(parts) or '<p class="muted">（这些来源里没有内容变化）</p>'}</div>
</div></details>''', len(parts), P, M


CSS = r"""
:root{--bg:#f4f6f9;--fg:#16202c;--mut:#667085;--line:#e2e6ec;--card:#fff;--soft:#f8fafc;--acc:#2f5d8a;--acc2:#e8f0f8;
--addbg:#e7f6ec;--addfg:#17663a;--delbg:#fdecec;--delfg:#a32020;--hunk:#eef2f7;--ln:#98a2b3;
--done:#1f7a4d;--donebg:#e3f4ea;--ver:#9a5b00;--verbg:#fdf0d9;--wip:#2f5d8a;--wipbg:#e3edf8;--shadow:0 1px 2px rgba(16,24,40,.06),0 1px 3px rgba(16,24,40,.08)}
@media (prefers-color-scheme:dark){:root:not([data-theme="light"]){--bg:#11151a;--fg:#e6e9ee;--mut:#98a2b3;--line:#2a313a;--card:#181d23;--soft:#1d232a;
--acc:#8fb7e3;--acc2:#1f2b38;--addbg:#14301f;--addfg:#8fd8a8;--delbg:#3a1b1d;--delfg:#f3a5a5;--hunk:#212932;--ln:#667085;
--done:#7fd3a5;--donebg:#173326;--ver:#f0c072;--verbg:#3a2c12;--wip:#8fb7e3;--wipbg:#1c2a3a;--shadow:none}}
:root[data-theme="dark"]{--bg:#11151a;--fg:#e6e9ee;--mut:#98a2b3;--line:#2a313a;--card:#181d23;--soft:#1d232a;
--acc:#8fb7e3;--acc2:#1f2b38;--addbg:#14301f;--addfg:#8fd8a8;--delbg:#3a1b1d;--delfg:#f3a5a5;--hunk:#212932;--ln:#667085;
--done:#7fd3a5;--donebg:#173326;--ver:#f0c072;--verbg:#3a2c12;--wip:#8fb7e3;--wipbg:#1c2a3a;--shadow:none}
*{box-sizing:border-box}html{scroll-behavior:smooth;scroll-padding-top:16px}
body{margin:0;background:var(--bg);color:var(--fg);font:15px/1.65 "Microsoft YaHei UI","Microsoft YaHei","PingFang SC",system-ui,sans-serif}
code,.code,table{font-family:"Cascadia Mono",Consolas,"Microsoft YaHei UI",monospace}
.wrap{display:grid;grid-template-columns:260px minmax(0,1fr);gap:28px;max-width:1360px;margin:0 auto;padding:24px 20px 80px}
nav.side{position:sticky;top:16px;align-self:start;max-height:calc(100vh - 32px);overflow:auto;padding-right:4px}
nav.side .brand{font-weight:700;font-size:16px;margin-bottom:12px}
nav.side a{display:block;color:var(--fg);text-decoration:none;border-radius:6px;padding:4px 8px;font-size:13.5px}
nav.side a:hover{background:var(--acc2)}
nav.side a.m{font-weight:600;margin-top:10px}
nav.side a.f{color:var(--mut);padding-left:18px;display:flex;gap:6px;align-items:center}
nav.side a.f i{width:7px;height:7px;border-radius:50%;flex:none}
i.s-done{background:var(--done)}i.s-verify{background:var(--ver)}i.s-wip{background:var(--wip)}
header.hero{background:linear-gradient(135deg,var(--acc) 0%,#4b7fb3 100%);color:#fff;border-radius:14px;padding:24px 26px;box-shadow:var(--shadow)}
header.hero h1{margin:0 0 6px;font-size:26px}header.hero p{margin:0;opacity:.9;font-size:14px}
.stats{display:flex;flex-wrap:wrap;gap:10px;margin-top:16px}
.stat{background:rgba(255,255,255,.14);border-radius:10px;padding:8px 14px;font-size:13px}.stat b{display:block;font-size:20px}
.legend{display:flex;flex-wrap:wrap;gap:8px 18px;color:var(--mut);font-size:13px;margin:16px 2px}
.bar{display:flex;gap:8px;flex-wrap:wrap;margin:6px 0 4px}
button{font:inherit;font-size:13px;padding:5px 12px;border:1px solid var(--line);background:var(--card);color:var(--fg);border-radius:8px;cursor:pointer}
button:hover{border-color:var(--acc);color:var(--acc)}
details.mod{margin-top:22px}
.mhead{background:var(--card);border:1px solid var(--line);border-radius:12px;padding:16px 20px;box-shadow:var(--shadow);border-top:4px solid var(--acc);
cursor:pointer;list-style:none;position:relative}
.mhead::-webkit-details-marker{display:none}
.mhead::after{content:"展开功能 ▾";position:absolute;right:20px;top:18px;font-size:12.5px;color:var(--acc);border:1px solid var(--line);border-radius:999px;padding:1px 10px;background:var(--soft)}
details.mod[open]>.mhead::after{content:"收起功能 ▴"}
.mhead:hover{border-color:var(--acc)}
.mtop{display:flex;align-items:center;gap:10px;flex-wrap:wrap;margin:0 110px 8px 0}
.mtop h2{margin:0;font-size:21px}.mcount{color:var(--mut);font-size:13px}
.mdate{font-size:13px;color:var(--acc);background:var(--acc2);border-radius:6px;padding:1px 8px;font-weight:600}
.mbody{padding-bottom:4px}
.mhead .kv{display:grid;grid-template-columns:auto 1fr;gap:4px 12px;font-size:14px}
.mhead .kv b{color:var(--mut);font-weight:600;white-space:nowrap}
details.feat{background:var(--card);border:1px solid var(--line);border-radius:12px;margin:12px 0 0 18px;box-shadow:var(--shadow);position:relative}
details.feat::before{content:"";position:absolute;left:-12px;top:22px;width:8px;height:8px;border-radius:50%;background:var(--line)}
details.feat>summary{list-style:none;cursor:pointer;padding:14px 18px;display:grid;grid-template-columns:minmax(0,1fr) auto;gap:2px 14px;align-items:center}
details.feat>summary::-webkit-details-marker{display:none}
details.feat>summary::after{content:"展开 ▾";grid-column:2;grid-row:1;font-size:12px;color:var(--mut);justify-self:end}
details.feat[open]>summary::after{content:"收起 ▴"}
.ftop{display:flex;gap:10px;align-items:center;flex-wrap:wrap}
.ftitle{font-weight:650;font-size:16.5px;grid-column:1;overflow-wrap:anywhere}.fmeta{grid-column:2;grid-row:2;font-size:13px;color:var(--mut);white-space:nowrap}
.fdate{color:var(--mut);font-size:13px}
.status{font-size:12px;font-weight:600;padding:2px 9px;border-radius:999px;white-space:nowrap}
.status.s-done{color:var(--done);background:var(--donebg)}.status.s-verify{color:var(--ver);background:var(--verbg)}.status.s-wip{color:var(--wip);background:var(--wipbg)}
.fbody{padding:0 18px 16px;border-top:1px solid var(--line)}
.nr{display:grid;grid-template-columns:1fr 1fr;gap:12px;margin-top:14px}
.box{background:var(--soft);border:1px solid var(--line);border-radius:10px;padding:10px 14px}
.box h4{margin:0 0 4px;font-size:13px;color:var(--acc)}.box p{margin:0;font-size:14px}
.snote{margin:12px 0 6px;font-size:13.5px;display:flex;gap:10px;align-items:baseline;flex-wrap:wrap}
.chips{font-size:12.5px;color:var(--mut);display:flex;gap:6px;flex-wrap:wrap;align-items:center;margin-bottom:8px}
.chip{padding:1px 8px;border-radius:6px;border:1px solid var(--line);background:var(--soft)}
.chip-git{color:var(--acc)}.chip-cp{color:var(--ver)}
.file{border:1px solid var(--line);border-radius:9px;margin:8px 0;background:var(--card);overflow:hidden}
.fhead{display:flex;gap:10px;align-items:flex-start;padding:8px 12px;cursor:pointer;list-style:none}
.file.plain .fhead{cursor:default}
.fhead::-webkit-details-marker{display:none}
details.file>summary.fhead::before{content:"▸";color:var(--mut);margin-top:2px;transition:transform .15s}
details.file[open]>summary.fhead::before{transform:rotate(90deg)}
details.file[open]>summary.fhead{background:var(--soft);border-bottom:1px solid var(--line)}
.fpath{flex:1;min-width:0;display:flex;flex-direction:column}
.fpath code{font-size:13px;word-break:break-all}
.fdesc{font-size:13px;color:var(--mut);margin-top:1px}
.fnote{font-size:12.5px;color:var(--mut);margin-left:auto;white-space:nowrap}
.fnote2{font-size:12.5px;color:var(--mut);padding:6px 12px 0}
.fstat{font-size:12.5px;white-space:nowrap;display:flex;gap:6px}
.tag{font-size:12px;font-weight:700;padding:1px 7px;border-radius:5px;color:#fff;flex:none;margin-top:1px}
.t-add{background:#2f8f4e}.t-mod{background:#3d6a99}.t-del{background:#b23b3b}.t-move{background:#7a8290}.t-first{background:#9a6a1f}
b.p{color:var(--addfg)}b.m{color:var(--delfg)}
.diff{overflow:auto;max-height:640px}
table{border-collapse:collapse;width:100%;font-size:12.5px;line-height:1.55}
td{padding:0 10px;vertical-align:top}
td.ln{width:1%;min-width:46px;text-align:right;color:var(--ln);user-select:none;border-right:1px solid var(--line);background:var(--soft)}
td.code{white-space:pre}
.sign{display:inline-block;width:1.3em;color:var(--mut);user-select:none}
tr.add{background:var(--addbg)}tr.add td.code{color:var(--addfg)}
tr.del{background:var(--delbg)}tr.del td.code{color:var(--delfg)}
tr.hunk td{background:var(--hunk);color:var(--mut);padding-top:2px;padding-bottom:2px}
.loose{border:2px dashed var(--ver);border-radius:12px;padding:14px 18px;margin-top:34px;background:var(--card)}
.muted{color:var(--mut)}
footer{color:var(--mut);font-size:13px;margin-top:40px}
p,li,.fdesc,.kv span,.snote{overflow-wrap:anywhere}
@media (max-width:900px){.wrap{grid-template-columns:minmax(0,1fr)}nav.side{position:static;max-height:none}.nr{grid-template-columns:1fr}
details.feat{margin-left:0}details.feat::before{display:none}.fhead{flex-wrap:wrap}.fnote{margin-left:0;white-space:normal}}
"""

JS = r"""
function setAll(sel, open){document.querySelectorAll(sel).forEach(d=>d.open=open)}
document.addEventListener('click',e=>{const a=e.target.closest('nav.side a');if(!a)return;const d=document.querySelector(a.getAttribute('href'));
if(!d)return;d.open=true;const m=d.closest('details.mod');if(m)m.open=true});
"""


def build():
    cp = checkpoint()
    modules, loose = collect()
    nav, secs = [], []
    tot_f = tot_files = tot_p = tot_m = 0
    for mod in modules:
        feats = sorted(mod["features"], key=lambda f: f["_time"] or datetime.datetime.max.replace(tzinfo=BJ))
        cards = []
        nav.append(f'<a class="m" href="#m-{mod["key"]}">{esc(mod["title"])}</a>')
        for i, ft in enumerate(feats):
            fid = f'f-{mod["key"]}-{i}'
            h, nf, p, m = render_feature(ft, fid)
            cards.append(h)
            tot_f += 1
            tot_files += nf
            tot_p += p
            tot_m += m
            nav.append(f'<a class="f" href="#{fid}"><i class="{STATUS[ft["status"]][1]}"></i>{esc(ft["title"].split("：")[0])}</a>')
        counts = {}
        for ft in feats:
            counts[ft["status"]] = counts.get(ft["status"], 0) + 1
        # 模块时间：最早开始的功能 ~ 最后一次改动（02a512a 的小车时钟不对，按功能里写的日期为准）
        firsts = [f["_time"] for f in feats if f["_time"]]
        lasts = [f["_last"] for f in feats if f["_last"]]
        if mod.get("date"):
            span = esc(mod["date"])
        elif firsts:
            a, b = min(firsts).strftime("%m-%d"), max(lasts).strftime("%m-%d")
            span = f"{a} ~ {b}" if a != b else a
        else:
            span = ""
        pills = "".join(f'<span class="status {STATUS[k][1]}">{STATUS[k][0].split("（")[0].split(" · ")[-1]} {n}</span>'
                        for k, n in counts.items())
        secs.append(f'''<details class="mod" id="m-{mod["key"]}"><summary class="mhead">
<div class="mtop"><h2>{esc(mod["title"])}</h2><span class="mdate">{span}</span><span class="mcount">{len(feats)} 个功能</span>{pills}</div>
<div class="kv"><b>要求</b><span>{esc(mod["goal"])}</span><b>实现</b><span>{esc(mod["does"])}</span></div></summary>
<div class="mbody">{"".join(cards)}</div></details>''')
    loose_html = ""
    if loose:
        rows = []
        for where, name, old, new in loose:
            h, _, _ = render_file(name, f"来自 {where}；还没归进任何功能，请在 features.py 里补上", old, new)
            rows.append(h)
        loose_html = f'<div class="loose"><h2 style="margin-top:0">未归类的改动（{len(loose)} 个文件）</h2>{"".join(rows)}</div>'
    now = datetime.datetime.now(BJ).strftime("%Y-%m-%d %H:%M")
    page = f"""<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>代码改动记录</title><style>{CSS}</style><script>{JS}</script></head><body>
<div class="wrap"><nav class="side"><div class="brand">代码改动记录</div>{"".join(nav)}</nav><main>
<header class="hero"><h1>智能车 · 代码改动记录</h1>
<p>按大模块 → 功能列出：每个功能写明要求、实现、验证状态，以及每个文件的作用和具体代码改动（功能最终完成时相对开始前的净改动）。生成于 {now}（北京时间）。</p>
<div class="stats"><div class="stat"><b>{len(modules)}</b>大模块</div><div class="stat"><b>{tot_f}</b>功能</div><div class="stat"><b>{tot_files}</b>文件改动</div>
<div class="stat"><b>+{tot_p} / −{tot_m}</b>行</div></div></header>
<div class="legend"><span><span class="status s-done">已完成并验证</span></span><span><span class="status s-verify">待验证（跟进中）</span></span><span><span class="status s-wip">进行中（跟进中）</span></span>
<span><span class="tag t-add">增</span> 新增文件</span><span><span class="tag t-del">删</span> 删除文件</span><span><span class="tag t-mod">改</span> 修改</span><span><span class="tag t-move">移</span> 只移动</span><span><span class="tag t-first">首</span> 仓库外文件第一次存版本</span>
<span>绿底 + 新加的行，红底 − 删掉的行；左两列是改动前 / 后的行号</span></div>
<div class="bar"><button onclick="setAll('details.mod',true)">展开全部模块</button><button onclick="setAll('details.mod',false)">收起全部模块</button>
<button onclick="setAll('details.mod',true);setAll('details.feat',true)">展开全部功能</button><button onclick="setAll('details.feat',false)">收起全部功能</button>
<button onclick="setAll('details.mod',true);setAll('details.feat',true);setAll('details.file',true)">展开全部代码</button><button onclick="setAll('details.file',false)">收起全部代码</button></div>
{"".join(secs)}{loose_html}
<footer>数据来源：git 提交（已提交）；未提交和仓库外文件来自 文档\\源\\改动快照\\（每次运行 build.py 有变化就记一次；10-21 之前的几次从 Claude 会话的文件备份里还原）。
功能、要求、文件作用写在 文档\\源\\features.py。</footer>
</main></div></body></html>"""
    with open(OUT, "w", encoding="utf-8", newline="\n") as f:
        f.write(page)
    print("OK", OUT, ("（新记录点 " + cp + "）") if cp else "（没有新改动，未新增记录点）",
          f"；未归类 {len(loose)} 个文件" if loose else "")


if __name__ == "__main__":
    build()
