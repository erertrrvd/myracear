# -*- coding: utf-8 -*-
r"""把 manual.py / issues.py / progress.py 的内容生成可折叠网页（和 PDF 同一份内容，放在 PDF 旁边）。
由 build.py 在每个 PDF 生成后调用；单个 HTML，不引用任何外部资源，没网也能打开。
结构：章（可折叠）→ 小节（按 h2 分，可折叠）→ 正文 / 命令（带"复制"按钮）/ 表格 / 提示 / 问题卡片（可折叠）。
顶部有搜索框：输入关键词只显示包含它的小节并自动展开。规则见 AI约束文件\代码修改记录网页生成\ 的规范。
"""
import datetime
import html
import os
import re

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.dirname(HERE)
BJ = datetime.timezone(datetime.timedelta(hours=8))
esc = html.escape

CSS = r"""
:root{--bg:#f4f6f9;--fg:#16202c;--mut:#667085;--line:#e2e6ec;--card:#fff;--soft:#f8fafc;--acc:#2f5d8a;--acc2:#e8f0f8;
--code:#f6f8fa;--codebar:#c3ccd6;--note:#eef3f8;--warn:#fff4e5;--warnbar:#c77a1a;--good:#1f7a4d;--goodbg:#e3f4ea;--mark:#fff1a8;
--shadow:0 1px 2px rgba(16,24,40,.06),0 1px 3px rgba(16,24,40,.08)}
@media (prefers-color-scheme:dark){:root:not([data-theme="light"]){--bg:#11151a;--fg:#e6e9ee;--mut:#98a2b3;--line:#2a313a;--card:#181d23;--soft:#1d232a;
--acc:#8fb7e3;--acc2:#1f2b38;--code:#1d232a;--codebar:#3a4552;--note:#1c2a3a;--warn:#3a2c12;--warnbar:#f0c072;--good:#7fd3a5;--goodbg:#173326;--mark:#5a4b10;--shadow:none}}
:root[data-theme="dark"]{--bg:#11151a;--fg:#e6e9ee;--mut:#98a2b3;--line:#2a313a;--card:#181d23;--soft:#1d232a;
--acc:#8fb7e3;--acc2:#1f2b38;--code:#1d232a;--codebar:#3a4552;--note:#1c2a3a;--warn:#3a2c12;--warnbar:#f0c072;--good:#7fd3a5;--goodbg:#173326;--mark:#5a4b10;--shadow:none}
*{box-sizing:border-box}html{scroll-behavior:smooth;scroll-padding-top:16px}
body{margin:0;background:var(--bg);color:var(--fg);font:15px/1.7 "Microsoft YaHei UI","Microsoft YaHei","PingFang SC",system-ui,sans-serif}
code,pre{font-family:"Cascadia Mono",Consolas,"Microsoft YaHei UI",monospace}
.wrap{display:grid;grid-template-columns:270px minmax(0,1fr);gap:28px;max-width:1360px;margin:0 auto;padding:24px 20px 80px}
nav.side{position:sticky;top:16px;align-self:start;max-height:calc(100vh - 32px);overflow:auto;padding-right:4px}
nav.side .brand{font-weight:700;font-size:16px;margin-bottom:12px}
nav.side a{display:block;color:var(--fg);text-decoration:none;border-radius:6px;padding:3px 8px;font-size:13.5px}
nav.side a:hover{background:var(--acc2)}
nav.side a.c{font-weight:600;margin-top:10px}nav.side a.s{color:var(--mut);padding-left:18px;font-size:13px}
header.hero{background:linear-gradient(135deg,var(--acc) 0%,#4b7fb3 100%);color:#fff;border-radius:14px;padding:24px 26px;box-shadow:var(--shadow)}
header.hero h1{margin:0 0 6px;font-size:26px}header.hero p{margin:0;opacity:.92;font-size:14px}
.tools{display:flex;gap:8px;flex-wrap:wrap;align-items:center;margin:16px 0 4px}
.tools input{flex:1;min-width:200px;font:inherit;font-size:14px;padding:7px 12px;border:1px solid var(--line);border-radius:8px;background:var(--card);color:var(--fg)}
.tools input:focus{outline:2px solid var(--acc2);border-color:var(--acc)}
button{font:inherit;font-size:13px;padding:5px 12px;border:1px solid var(--line);background:var(--card);color:var(--fg);border-radius:8px;cursor:pointer}
button:hover{border-color:var(--acc);color:var(--acc)}
.hits{font-size:13px;color:var(--mut)}
details.ch{margin-top:20px}
details.ch>summary{list-style:none;cursor:pointer;background:var(--card);border:1px solid var(--line);border-top:4px solid var(--acc);border-radius:12px;padding:16px 20px;box-shadow:var(--shadow);position:relative}
details.ch>summary::-webkit-details-marker{display:none}
details.ch>summary::after{content:"展开 ▾";position:absolute;right:20px;top:18px;font-size:12.5px;color:var(--acc);border:1px solid var(--line);border-radius:999px;padding:1px 10px;background:var(--soft)}
details.ch[open]>summary::after{content:"收起 ▴"}
details.ch>summary:hover{border-color:var(--acc)}
.clabel{font-size:13px;color:var(--acc);font-weight:600}
.ctitle{font-size:21px;font-weight:700;margin:2px 110px 4px 0}
.cintro{color:var(--mut);font-size:14px}.ccount{color:var(--mut);font-size:12.5px;margin-top:4px}
.chbody{padding:2px 0 4px 18px}
.lead{background:var(--card);border:1px solid var(--line);border-radius:12px;padding:12px 18px;margin-top:12px;box-shadow:var(--shadow)}
details.sec{background:var(--card);border:1px solid var(--line);border-radius:12px;margin-top:12px;box-shadow:var(--shadow)}
details.sec>summary{list-style:none;cursor:pointer;padding:12px 18px;font-weight:650;font-size:16px;display:flex;gap:10px;align-items:center}
details.sec>summary::-webkit-details-marker{display:none}
details.sec>summary::before{content:"▸";color:var(--mut);transition:transform .15s}
details.sec[open]>summary::before{transform:rotate(90deg)}
details.sec[open]>summary{border-bottom:1px solid var(--line)}
.secbody{padding:6px 18px 14px}
.secbody p{margin:8px 0}
ul{margin:8px 0;padding-left:22px}li{margin:3px 0}
.code{position:relative;background:var(--code);border-left:3px solid var(--codebar);border-radius:8px;margin:10px 0}
.code pre{margin:0;padding:10px 14px;overflow:auto;font-size:13px;line-height:1.55;white-space:pre}
.code .cp{position:absolute;top:6px;right:8px;font-size:12px;padding:2px 10px;opacity:.85}
.code .cp.ok{color:var(--good);border-color:var(--good)}
.cmt{color:var(--mut)}
.call{border-radius:8px;padding:9px 14px;margin:10px 0;font-size:14px}
.call.note{background:var(--note);border-left:3px solid var(--acc)}
.call.warn{background:var(--warn);border-left:3px solid var(--warnbar)}
.call b{margin-right:6px}
.tbl{overflow:auto;margin:10px 0;border:1px solid var(--line);border-radius:8px}
table{border-collapse:collapse;width:100%;font-size:13.5px}
th{text-align:left;color:var(--acc);background:var(--soft);border-bottom:2px solid var(--acc);padding:6px 10px;white-space:nowrap}
td{border-bottom:1px solid var(--line);padding:6px 10px;vertical-align:top}
tr:last-child td{border-bottom:0}
td code{font-size:12.5px;word-break:break-all}
details.issue{border:1px solid var(--line);border-left:4px solid var(--acc);border-radius:10px;margin:10px 0;background:var(--card)}
details.issue.solved{border-left-color:var(--good)}
details.issue>summary{list-style:none;cursor:pointer;padding:9px 14px;font-weight:650;display:flex;gap:8px;align-items:flex-start}
details.issue>summary::-webkit-details-marker{display:none}
details.issue>summary::before{content:"▸";color:var(--mut);transition:transform .15s}
details.issue[open]>summary::before{transform:rotate(90deg)}
.ist{font-size:11.5px;font-weight:600;padding:1px 8px;border-radius:999px;white-space:nowrap;margin-left:auto}
.ist.ok{color:var(--good);background:var(--goodbg)}.ist.no{color:var(--acc);background:var(--acc2)}
.ibody{padding:0 14px 10px;display:grid;grid-template-columns:auto minmax(0,1fr);gap:6px 14px}
.ibody>b{color:var(--acc);font-size:13.5px;padding-top:2px;white-space:nowrap}
.ibody>div p{margin:2px 0}
mark{background:var(--mark);color:inherit;border-radius:2px}
.hide{display:none!important}
footer{color:var(--mut);font-size:13px;margin-top:40px}
p,li,td,.cintro,.ctitle,summary{overflow-wrap:anywhere}
@media (max-width:900px){.wrap{grid-template-columns:minmax(0,1fr)}nav.side{position:static;max-height:none}.chbody{padding-left:0}}
"""

JS = r"""
function setAll(sel,open){document.querySelectorAll(sel).forEach(d=>d.open=open)}
function copyCode(btn){
  const t=btn.parentElement.querySelector('pre').innerText;
  const done=()=>{btn.textContent='已复制';btn.classList.add('ok');setTimeout(()=>{btn.textContent='复制';btn.classList.remove('ok')},1500)};
  const fall=()=>{const a=document.createElement('textarea');a.value=t;document.body.appendChild(a);a.select();try{document.execCommand('copy');done()}catch(e){}a.remove()};
  if(navigator.clipboard&&window.isSecureContext){navigator.clipboard.writeText(t).then(done,fall)}else fall();
}
document.addEventListener('click',e=>{const a=e.target.closest('nav.side a');if(!a)return;const d=document.querySelector(a.getAttribute('href'));
  if(!d)return;d.open=true;const c=d.closest('details.ch');if(c)c.open=true});
function search(q){
  q=q.trim().toLowerCase();let n=0;
  document.querySelectorAll('details.ch').forEach(ch=>{
    let any=false;
    ch.querySelectorAll('.unit').forEach(u=>{const hit=!q||u.textContent.toLowerCase().includes(q);u.classList.toggle('hide',!hit);
      if(hit&&q){any=true;n++;if(u.tagName==='DETAILS')u.open=true;u.querySelectorAll('details.issue').forEach(i=>{if(i.textContent.toLowerCase().includes(q))i.open=true})}});
    const own=!q||ch.querySelector('summary').textContent.toLowerCase().includes(q);
    ch.classList.toggle('hide',!!q&&!any&&!own);if(q&&any)ch.open=true;
  });
  document.getElementById('hits').textContent=q?('找到 '+n+' 处'):'';
}
"""


def text(t):
    return esc(str(t)).replace("\n", "<br>")


def code_html(t):
    lines = t.strip("\n").split("\n")
    out = []
    for l in lines:
        e = esc(l)
        if l.lstrip().startswith("#"):
            e = f'<span class="cmt">{e}</span>'
        out.append(e)
    return (f'<div class="code"><button class="cp" onclick="copyCode(this)">复制</button>'
            f'<pre>{chr(10).join(out)}</pre></div>')


def table_html(head, rows):
    h = "".join(f"<th>{esc(str(c))}</th>" for c in head)
    body = "".join("<tr>" + "".join(f"<td>{text(c)}</td>" for c in r) + "</tr>" for r in rows)
    return f'<div class="tbl"><table><thead><tr>{h}</tr></thead><tbody>{body}</tbody></table></div>'


def issue_html(title, d):
    solved = "结果" in d
    rows = []
    for k in ("现象", "排查", "原因", "解决", "结果"):
        if k not in d:
            continue
        v = d[k]
        if isinstance(v, (str, tuple)):
            v = [v]
        cell = []
        for item in v:
            if isinstance(item, tuple) and item[0] == "code":
                cell.append(code_html(item[1]))
            else:
                cell.append(f"<p>{text(item)}</p>")
        rows.append(f"<b>{k}</b><div>{''.join(cell)}</div>")
    st = '<span class="ist ok">有结果</span>' if solved else '<span class="ist no">记录</span>'
    return (f'<details class="issue{" solved" if solved else ""}"><summary><span>{esc(title)}</span>{st}</summary>'
            f'<div class="ibody">{"".join(rows)}</div></details>')


def block_html(b, pkg_path):
    k = b[0]
    if k == "p":
        return f"<p>{text(b[1])}</p>"
    if k == "ul":
        return "<ul>" + "".join(f"<li>{text(i)}</li>" for i in b[1]) + "</ul>"
    if k == "code":
        return code_html(b[1])
    if k == "note":
        return f'<div class="call note"><b>提示</b>{text(b[1])}</div>'
    if k == "warn":
        return f'<div class="call warn"><b>注意</b>{text(b[1])}</div>'
    if k == "table":
        return table_html(b[1], b[2])
    if k == "pkg":
        return table_html(["位置", "文件（相对位置）", "作用"], [[r[0], r[1], r[2]] for r in b[1]])
    if k == "paths":
        return table_html(["位置", "实际路径"], [[n, p] for n, p in pkg_path.items()])
    if k == "issue":
        return issue_html(b[1], b[2])
    return ""


def sections(blocks):
    """按 h2 切成小节：[(标题或 None, [块])]；pkg 单独成"本章配套文件"小节。"""
    out = [(None, [])]
    for b in blocks:
        if b[0] == "h2":
            out.append((b[1], []))
        elif b[0] == "pkg":
            out.append(("本章配套文件", [b]))
        else:
            out[-1][1].append(b)
    return [s for s in out if s[0] or s[1]]


def build_html(fname, title, subtitle, chapters, updated, pkg_path=None):
    pkg_path = pkg_path or {}
    nav, body = [], []
    for ci, ch in enumerate(chapters):
        m = re.match(r"^(第\s*\d+\s*章|附录|[一二三四五六七八九十]+、)\s*(.*)$", ch["title"])
        label, name = (m.group(1).rstrip("、"), m.group(2)) if m else ("", ch["title"])
        cid = f"c{ci}"
        secs = sections(ch["blocks"])
        nav.append(f'<a class="c" href="#{cid}">{esc(ch["title"])}</a>')
        parts = []
        for si, (st, blocks) in enumerate(secs):
            inner = "".join(block_html(b, pkg_path) for b in blocks)
            if st is None:
                parts.append(f'<div class="lead unit">{inner}</div>')
                continue
            sid = f"{cid}s{si}"
            nav.append(f'<a class="s" href="#{sid}">{esc(st)}</a>')
            parts.append(f'<details class="sec unit" id="{sid}"><summary>{esc(st)}</summary><div class="secbody">{inner}</div></details>')
        nsec = sum(1 for s in secs if s[0])
        intro = f'<div class="cintro">{esc(ch["intro"])}</div>' if ch.get("intro") else ""
        body.append(f'''<details class="ch" id="{cid}"><summary>
<div class="clabel">{esc(label)}</div><div class="ctitle">{esc(name or label)}</div>{intro}
<div class="ccount">{nsec} 个小节</div></summary><div class="chbody">{"".join(parts)}</div></details>''')
    now = datetime.datetime.now(BJ).strftime("%Y-%m-%d %H:%M")
    page = f"""<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>{esc(title)}</title><style>{CSS}</style><script>{JS}</script></head><body>
<div class="wrap"><nav class="side"><div class="brand">{esc(title)}</div>{"".join(nav)}</nav><main>
<header class="hero"><h1>{esc(title)}</h1><p>{esc(subtitle)}</p><p style="margin-top:8px">内容更新于 {esc(updated)} · 网页生成于 {now}（北京时间） · 和同名 PDF 是同一份内容</p></header>
<div class="tools"><input type="search" placeholder="搜索：输入关键词，只显示包含它的小节（如 -Bag、scan_relay、Ctrl+C）" oninput="search(this.value)">
<span class="hits" id="hits"></span>
<button onclick="setAll('details.ch',true)">展开全部章</button><button onclick="setAll('details.ch',false)">收起全部章</button>
<button onclick="setAll('details.ch',true);setAll('details.sec',true)">展开全部小节</button><button onclick="setAll('details.sec',false)">收起全部小节</button>
<button onclick="setAll('details.ch',true);setAll('details.sec',true);setAll('details.issue',true)">展开全部卡片</button></div>
{"".join(body)}
<footer>由 文档\\源\\build.py 生成；内容源在 文档\\源\\*.py。命令块右上角“复制”可一键复制整块。</footer>
</main></div></body></html>"""
    path = os.path.join(OUT, os.path.splitext(fname)[0] + ".html")
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        f.write(page)
    print("OK", path)
