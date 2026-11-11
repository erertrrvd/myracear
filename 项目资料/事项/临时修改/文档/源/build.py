# -*- coding: utf-8 -*-
r"""生成四个 PDF：小车操作手册（最终指令）/ 改进与问题记录（调试排错）/ 项目进度 / 代码改动记录。
用法：python 文档\源\build.py        （输出到 文档\ 下；用系统 python，已装 reportlab）
内容在同目录 manual.py / issues.py / progress.py / changes.py 里改，改完重新运行即可。
最后还会运行 diffs.py，生成 代码改动记录.html（每个文件具体改了哪些行，可折叠；快照存在 源\改动快照\）。
另外三份 PDF 各生成一个同名 .html（htmldoc.py：章 / 小节可折叠、可搜索、命令一键复制，没网也能打开）。
每章对应 临时修改\ 下的一个模块（原来分文件夹放的 md），章内再分小节。

块类型：
  ('h2', 文字)                      小节标题
  ('p', 文字)                       正文
  ('ul', [文字, ...])               项目符号列表
  ('code', 文字)                    命令（只写实际用过的）
  ('note', 文字) ('warn', 文字)     提示 / 注意
  ('table', 表头列表, 行列表, 列宽比例)
  ('pkg', 行列表)                   本章配套使用的包 / 文件 / 代码（三列；文件自动补全为 projectliu 下的路径，见 PKG_PATH）
  ('paths',)                        模块 → 代码路径速查表
  ('issue', 标题, {'现象':..,'排查':..,'原因':..,'解决':..,'结果':..})   问题卡片，字段可缺
"""
import os, sys, re, zlib
from xml.sax.saxutils import escape
from reportlab.lib.pagesizes import A4
from reportlab.lib.units import mm
from reportlab.lib import colors
from reportlab.lib.styles import ParagraphStyle
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import (BaseDocTemplate, PageTemplate, Frame, Paragraph, Spacer,
                                Table, TableStyle, PageBreak, KeepTogether)
from reportlab.platypus.tableofcontents import TableOfContents

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.dirname(HERE)
sys.path.insert(0, HERE)

FD = r"C:\Windows\Fonts"
pdfmetrics.registerFont(TTFont("YH", os.path.join(FD, "msyh.ttc"), subfontIndex=0))
pdfmetrics.registerFont(TTFont("YHB", os.path.join(FD, "msyhbd.ttc"), subfontIndex=0))
pdfmetrics.registerFont(TTFont("MONO", os.path.join(FD, "consola.ttf")))

# ---- 配色：墨色文字 + 一个蓝灰强调色，其余用浅灰
INK = colors.HexColor("#1b2430")
MUTE = colors.HexColor("#6b7785")
ACC = colors.HexColor("#2f5d8a")
HAIR = colors.HexColor("#e3e8ee")
CODEBG = colors.HexColor("#f6f8fa")
CODEBAR = colors.HexColor("#c3ccd6")
NOTEBG = colors.HexColor("#eef3f8")
WARNBG = colors.HexColor("#fff6ea")
WARNBAR = colors.HexColor("#c77a1a")
GOODBAR = colors.HexColor("#3b8a5a")

W = 166 * mm  # 正文宽度


def S(name, **kw):
    base = dict(fontName="YH", fontSize=9.5, leading=16, textColor=INK, wordWrap="CJK")
    base.update(kw)
    return ParagraphStyle(name, **base)


ST = {
    "cover": S("cover", fontName="YHB", fontSize=30, leading=40),
    "sub": S("sub", fontSize=11.5, leading=18, textColor=MUTE),
    "date": S("date", fontSize=9, leading=14, textColor=MUTE),
    "tochead": S("tochead", fontName="YHB", fontSize=13, leading=20, textColor=ACC),
    "toc1": S("toc1", fontSize=10.5, leading=22),
    "label": S("label", fontName="YHB", fontSize=9, leading=12, textColor=ACC, spaceBefore=2, keepWithNext=1),
    "h1": S("h1", fontName="YHB", fontSize=22, leading=30, spaceAfter=4, keepWithNext=1),
    "intro": S("intro", fontSize=10, leading=17, textColor=MUTE, spaceAfter=10),
    "h2": S("h2", fontName="YHB", fontSize=12, leading=18, spaceBefore=12, spaceAfter=5, keepWithNext=1),
    "p": S("p", spaceAfter=6),
    "li": S("li", leftIndent=12, bulletIndent=2, spaceAfter=2),
    "cell": S("cell", fontSize=8.6, leading=13.5),
    "cellh": S("cellh", fontName="YHB", fontSize=8.4, leading=12, textColor=ACC),
    "box": S("box", fontSize=9, leading=14.5),
    "ik": S("ik", fontName="YHB", fontSize=8.6, leading=14, textColor=ACC),
    "iv": S("iv", fontSize=9, leading=14.5),
    "ititle": S("ititle", fontName="YHB", fontSize=10.5, leading=16),
}

CJK = re.compile(r"([\u2e80-\u9fff\u3000-\u303f\uff00-\uffef\u2018-\u201d\u2026\u00b7\u3001\u3002]+)")


def P(t, st="p"):
    return Paragraph(escape(t).replace("\n", "<br/>"), ST[st])


def mono(line):
    """代码行：英文用等宽字体，中文片段切回雅黑。"""
    e = escape(line).replace("  ", "&nbsp;&nbsp;")
    e = CJK.sub(r'<font name="YH">\1</font>', e)
    return Paragraph(e if line.strip() else "&nbsp;", S("m", fontName="MONO", fontSize=8.2, leading=12.2, wordWrap="CJK"))


def bar_box(rows, bg, bar, pad=7, width=W):
    t = Table([[r] for r in rows], colWidths=[width])
    t.setStyle(TableStyle([("BACKGROUND", (0, 0), (-1, -1), bg), ("LINEBEFORE", (0, 0), (0, -1), 2.4, bar),
                           ("LEFTPADDING", (0, 0), (-1, -1), pad + 3), ("RIGHTPADDING", (0, 0), (-1, -1), pad),
                           ("TOPPADDING", (0, 0), (-1, -1), 1.5), ("BOTTOMPADDING", (0, 0), (-1, -1), 1.5)]))
    t.setStyle(TableStyle([("TOPPADDING", (0, 0), (-1, 0), 6), ("BOTTOMPADDING", (0, -1), (-1, -1), 6)]))
    return t


def code_block(text):
    rows = [mono(l) for l in text.strip("\n").split("\n")]
    return [bar_box(rows, CODEBG, CODEBAR), Spacer(1, 8)]


def callout(text, bg, bar):
    return [bar_box([P(text, "box")], bg, bar), Spacer(1, 8)]


def table(head, rows, widths):
    data = [[Paragraph(escape(h), ST["cellh"]) for h in head]]
    for r in rows:
        data.append([Paragraph(escape(str(c)).replace("\n", "<br/>"), ST["cell"]) for c in r])
    tot = float(sum(widths))
    t = Table(data, colWidths=[W * w / tot for w in widths], repeatRows=1)
    t.setStyle(TableStyle([
        ("LINEBELOW", (0, 0), (-1, 0), 1.1, ACC), ("LINEBELOW", (0, 1), (-1, -1), 0.4, HAIR),
        ("VALIGN", (0, 0), (-1, -1), "TOP"),
        ("LEFTPADDING", (0, 0), (-1, -1), 4), ("RIGHTPADDING", (0, 0), (-1, -1), 6),
        ("TOPPADDING", (0, 0), (-1, -1), 4.5), ("BOTTOMPADDING", (0, 0), (-1, -1), 4.5)]))
    return [t, Spacer(1, 10)]


def issue_card(title, d):
    rows = [Paragraph(escape(title), ST["ititle"]), Spacer(1, 3)]
    for k in ("现象", "排查", "原因", "解决", "结果"):
        if k in d:
            v = d[k]
            if isinstance(v, (str, tuple)):
                v = [v]
            body = []
            for item in v:
                if isinstance(item, tuple) and item[0] == "code":
                    body.append(("code", item[1]))
                else:
                    body.append(("p", item))
            cell = []
            for kind, txt in body:
                if kind == "code":
                    cell += [bar_box([mono(l) for l in txt.strip("\n").split("\n")], colors.white, CODEBAR, pad=4, width=W - 24 * mm - 8), Spacer(1, 3)]
                else:
                    cell.append(Paragraph(escape(txt).replace("\n", "<br/>"), ST["iv"]))
            kc = ST["ik"]
            t = Table([[Paragraph(k, kc), cell]], colWidths=[14 * mm, W - 14 * mm - 22])
            t.setStyle(TableStyle([("VALIGN", (0, 0), (-1, -1), "TOP"), ("LEFTPADDING", (0, 0), (-1, -1), 0),
                                   ("RIGHTPADDING", (0, 0), (-1, -1), 0), ("TOPPADDING", (0, 0), (-1, -1), 1.5),
                                   ("BOTTOMPADDING", (0, 0), (-1, -1), 1.5)]))
            rows.append(t)
    bar = GOODBAR if "结果" in d else ACC
    box = bar_box(rows, colors.white, bar)
    box.setStyle(TableStyle([("NOSPLIT", (0, 0), (0, min(2, len(rows) - 1)))]))  # 标题不单独留在页底
    return [box, Spacer(1, 12)]


# 位置 → 实际路径（'pkg' 表第一列写这里的键，文件写相对路径即可）
PKG_PATH = {
    "项目": "C:\\Users\\lzc2\\Desktop\\my_racecar（复件）\\",
    "小车项目": "/home/racecar/src/my_racecar/",
    "小车": "/home/racecar/",
    "D:\\ubuntu\\slam": "D:\\ubuntu\\slam\\",
    "备份": "D:\\VSCode\\ros2技术集\\智能车总文件\\my_racecar（复件）\\事项\\临时修改\\",
}


def full_path(pkg, f):
    """pkg 表里的文件只写相对路径，前缀由第一列决定（见 PKG_PATH）。"""
    return f


def blocks_to_flow(blocks):
    out = []
    for b in blocks:
        k = b[0]
        if k == "h2":
            out.append(P(b[1], "h2"))
        elif k == "p":
            out.append(P(b[1]))
        elif k == "ul":
            for it in b[1]:
                out.append(Paragraph(escape(it).replace("\n", "<br/>"), ST["li"], bulletText="•"))
            out.append(Spacer(1, 5))
        elif k == "code":
            out += code_block(b[1])
        elif k == "note":
            out += callout(b[1], NOTEBG, ACC)
        elif k == "warn":
            out += callout(b[1], WARNBG, WARNBAR)
        elif k == "table":
            out += table(b[1], b[2], b[3] if len(b) > 3 else [1] * len(b[1]))
        elif k == "pkg":
            out.append(P("本章配套文件", "h2"))
            rows = [[r[0], full_path(r[0], r[1]), r[2]] for r in b[1]]
            out += table(["位置", "文件（相对位置）", "作用"], rows, [1.5, 5.0, 4.5])
        elif k == "paths":
            out += table(["位置", "实际路径"], [[n, p] for n, p in PKG_PATH.items()], [1.6, 8.4])
        elif k == "issue":
            out += issue_card(b[1], b[2])
    return out


class H1(Paragraph):
    toc_text = ""


class Doc(BaseDocTemplate):
    def __init__(self, fn, title, **kw):
        super().__init__(fn, pagesize=A4, leftMargin=22 * mm, rightMargin=22 * mm,
                         topMargin=22 * mm, bottomMargin=20 * mm, title=title, author="liu", **kw)
        self.doc_title = title
        fr = Frame(self.leftMargin, self.bottomMargin, self.width, self.height, id="f", leftPadding=0, rightPadding=0)
        self.addPageTemplates([PageTemplate(id="p", frames=[fr], onPage=self.deco)])

    def deco(self, c, d):
        c.saveState()
        c.setFont("YH", 7.8)
        c.setFillColor(MUTE)
        c.drawString(22 * mm, 12 * mm, self.doc_title)
        c.drawRightString(188 * mm, 12 * mm, str(d.page))
        c.restoreState()

    def afterFlowable(self, fl):
        if isinstance(fl, H1) and fl.toc_text:
            key = "ch%d" % zlib.crc32(fl.toc_text.encode("utf-8"))
            self.canv.bookmarkPage(key)
            self.canv.addOutlineEntry(fl.toc_text, key, 0, 0)
            self.notify("TOCEntry", (0, fl.toc_text, self.page, key))


def chapter_head(title):
    m = re.match(r"^(第\s*\d+\s*章|附录)\s*(.*)$", title)
    label, name = (m.group(1), m.group(2)) if m else ("", title)
    out = []
    if label:
        out.append(Paragraph(escape(label), ST["label"]))
    h = H1(escape(name or label), ST["h1"])
    h.toc_text = title
    out.append(h)
    return out


def build(fname, title, subtitle, chapters, updated):
    path = os.path.join(OUT, fname)
    doc = Doc(path, title)
    accent = Table([[""]], colWidths=[18 * mm], rowHeights=[2.2 * mm])
    accent.setStyle(TableStyle([("BACKGROUND", (0, 0), (-1, -1), ACC)]))
    accent.hAlign = "LEFT"
    story = [Spacer(1, 26 * mm), accent, Spacer(1, 7 * mm), P(title, "cover"), Spacer(1, 3 * mm),
             P(subtitle, "sub"), Spacer(1, 3 * mm), P("更新于 " + updated, "date"), Spacer(1, 22 * mm),
             P("目录", "tochead"), Spacer(1, 2 * mm)]
    toc = TableOfContents()
    toc.levelStyles = [ST["toc1"]]
    toc.dotsMinLevel = 0
    story += [toc, PageBreak()]
    for i, ch in enumerate(chapters):
        story += chapter_head(ch["title"])
        if ch.get("intro"):
            story.append(P(ch["intro"], "intro"))
        story += blocks_to_flow(ch["blocks"])
        # 章末的空白间距会在页满时单独占一页，再加上分页就出现整页空白，去掉
        while isinstance(story[-1], Spacer):
            story.pop()
        if i != len(chapters) - 1:
            story.append(PageBreak())
    doc.multiBuild(story)
    print("OK", path)


if __name__ == "__main__":
    # 平时只生成网页（.html）；每月大更新时加 --pdf 才重新生成四份 PDF（2026-10-22 用户要求）
    MAKE_PDF = "--pdf" in sys.argv
    if not MAKE_PDF:
        build = lambda *a, **k: None
    import manual, progress, issues, changes, htmldoc
    docs = [
        ("小车操作手册.pdf", "智能车 操作手册", "最终指令：环境 · git · 分布式建图 · 路径 · 红绿灯视觉 · 模型", manual.CHAPTERS, manual.UPDATED),
        ("改进与问题记录.pdf", "智能车 改进与问题记录", "调试与排错：遇到的问题、排查命令、原因和解决办法", issues.CHAPTERS, issues.UPDATED),
        ("项目进度.pdf", "智能车 项目进度", "各模块当前状态、时间线、待办", progress.CHAPTERS, progress.UPDATED),
    ]
    for d in docs:
        build(*d)
        htmldoc.build_html(*d, pkg_path=PKG_PATH)    # 同名 .html：可折叠、可搜索、命令一键复制，没网也能打开
    build("代码改动记录.pdf", "智能车 代码改动记录", "按模块列出新增 / 修改的文件和 git 提交", changes.CHAPTERS, changes.UPDATED)
    # 逐行改动（可折叠）：代码改动记录.html；有新改动时自动新增一个记录点，说明取 changes.py 总表最后一行
    import diffs
    diffs.build()
