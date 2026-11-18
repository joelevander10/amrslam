"""Build the AMR manual PDF: python3 build.py en|id <out.pdf>"""
import importlib
import os
import sys

from PIL import Image as PILImage
from reportlab.lib import colors
from reportlab.lib.fonts import addMapping
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib.units import mm
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import (BaseDocTemplate, CondPageBreak, Flowable, Frame, Image, KeepTogether, NextPageTemplate,
                                PageBreak, PageTemplate, Paragraph, Preformatted, Spacer, Table, TableStyle)
from reportlab.platypus.tableofcontents import TableOfContents

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import figs  # noqa: E402

FD = "/usr/share/fonts/truetype/dejavu/"
for n, f in (("DV", "DejaVuSans.ttf"), ("DVB", "DejaVuSans-Bold.ttf"), ("DVI", "DejaVuSans-Oblique.ttf"), ("DVM", "DejaVuSansMono.ttf")):
    pdfmetrics.registerFont(TTFont(n, FD + f))
addMapping("DV", 0, 0, "DV"); addMapping("DV", 1, 0, "DVB"); addMapping("DV", 0, 1, "DVI"); addMapping("DV", 1, 1, "DVB")

INK, INK2, GRID, PALE = colors.HexColor("#0b0b0b"), colors.HexColor("#52514e"), colors.HexColor("#e1e0d9"), colors.HexColor("#f4f3ef")
BLUE, CRIT, GOOD, WARN = colors.HexColor("#2a78d6"), colors.HexColor("#d03b3b"), colors.HexColor("#0ca30c"), colors.HexColor("#eda100")
NAVY = colors.HexColor("#1d3a66")
W = A4[0] - 40 * mm

S = {
    "h1": ParagraphStyle("h1", fontName="DVB", fontSize=17, leading=22, textColor=NAVY, spaceBefore=2, spaceAfter=8),
    "h2": ParagraphStyle("h2", fontName="DVB", fontSize=12, leading=16, textColor=INK, spaceBefore=9, spaceAfter=4),
    "p": ParagraphStyle("p", fontName="DV", fontSize=9.6, leading=13.6, textColor=INK, spaceAfter=5),
    "li": ParagraphStyle("li", fontName="DV", fontSize=9.6, leading=13.4, textColor=INK, leftIndent=13, bulletIndent=2, spaceAfter=2.5),
    "td": ParagraphStyle("td", fontName="DV", fontSize=8.4, leading=10.8, textColor=INK),
    "th": ParagraphStyle("th", fontName="DVB", fontSize=8.4, leading=10.8, textColor=colors.white),
    "cap": ParagraphStyle("cap", fontName="DVI", fontSize=8, leading=10, textColor=INK2, spaceAfter=8, alignment=1),
    "box": ParagraphStyle("box", fontName="DV", fontSize=9.2, leading=12.6, textColor=INK),
    "code": ParagraphStyle("code", fontName="DVM", fontSize=8.2, leading=11, textColor=INK),
    "toc1": ParagraphStyle("toc1", fontName="DVB", fontSize=10.5, leading=15, spaceBefore=6),
    "toc2": ParagraphStyle("toc2", fontName="DV", fontSize=9.6, leading=13, leftIndent=12),
}


class Doc(BaseDocTemplate):
    def __init__(self, path, meta, **kw):
        super().__init__(path, pagesize=A4, leftMargin=20 * mm, rightMargin=20 * mm, topMargin=20 * mm, bottomMargin=18 * mm,
                         title=meta["title"] + " - " + meta["subtitle"], author="AMR team", **kw)
        self.meta = meta
        fr = Frame(20 * mm, 18 * mm, W, A4[1] - 38 * mm, id="f")
        self.addPageTemplates([PageTemplate(id="cover", frames=[fr], onPage=self._cover_bg),
                               PageTemplate(id="body", frames=[fr], onPage=self._body),
                               PageTemplate(id="part", frames=[fr], onPage=self._part_bg)])

    def afterFlowable(self, f):
        if isinstance(f, Paragraph) and getattr(f, "_toc", None) is not None:
            lvl, text = f._toc
            key = f"k{id(f)}"
            self.canv.bookmarkPage(key)
            self.canv.addOutlineEntry(text, key, level=lvl, closed=False)
            self.notify("TOCEntry", (lvl, text, self.page, key))

    def _body(self, c, d):
        c.saveState()
        c.setStrokeColor(GRID); c.setLineWidth(0.6)
        c.line(20 * mm, A4[1] - 13 * mm, A4[0] - 20 * mm, A4[1] - 13 * mm)
        c.line(20 * mm, 12 * mm, A4[0] - 20 * mm, 12 * mm)
        c.setFont("DV", 7.5); c.setFillColor(INK2)
        c.drawString(20 * mm, A4[1] - 11 * mm, self.meta["title"])
        c.drawRightString(A4[0] - 20 * mm, A4[1] - 11 * mm, self.meta["subtitle"])
        c.drawString(20 * mm, 8 * mm, self.meta["footer"])
        c.drawRightString(A4[0] - 20 * mm, 8 * mm, f"{self.meta['page']} {d.page}")
        c.restoreState()

    def _cover_bg(self, c, d):
        c.saveState()
        c.setFillColor(NAVY); c.rect(0, A4[1] - 95 * mm, A4[0], 95 * mm, fill=1, stroke=0)
        c.setFillColor(BLUE); c.rect(0, A4[1] - 98 * mm, A4[0], 3 * mm, fill=1, stroke=0)
        c.setFillColor(colors.white)
        c.setFont("DVB", 30); c.drawString(20 * mm, A4[1] - 45 * mm, self.meta["title"])
        c.setFont("DV", 15); c.drawString(20 * mm, A4[1] - 58 * mm, self.meta["subtitle"])
        c.setFont("DV", 9); c.drawString(20 * mm, A4[1] - 80 * mm, self.meta["edition"])
        c.restoreState()

    def _part_bg(self, c, d):
        c.saveState()
        c.setFillColor(NAVY); c.rect(0, 0, 14 * mm, A4[1], fill=1, stroke=0)
        c.restoreState()
        self._body(c, d)


class PartTitle(Flowable):
    def __init__(self, num, title, sub, word):
        super().__init__(); self.num, self.title, self.sub, self.word = num, title, sub, word

    def wrap(self, aw, ah):
        return aw, 120 * mm

    def draw(self):
        c = self.canv
        c.setFillColor(BLUE); c.setFont("DVB", 60); c.drawString(0, 70 * mm, f"{self.word} {self.num}")
        c.setFillColor(NAVY); c.setFont("DVB", 26); c.drawString(0, 52 * mm, self.title)
        c.setFillColor(INK2); c.setFont("DV", 12); c.drawString(0, 42 * mm, self.sub)


def para(text, style):
    return Paragraph(text, S[style])


def table(header, rows, widths, has_header=True):
    data = []
    if has_header:
        data.append([para(h, "th") for h in header])
    data += [[para(str(c), "td") for c in r] for r in rows]
    t = Table(data, colWidths=[w * W for w in widths], repeatRows=1 if has_header else 0)
    st = [("VALIGN", (0, 0), (-1, -1), "TOP"), ("LEFTPADDING", (0, 0), (-1, -1), 4), ("RIGHTPADDING", (0, 0), (-1, -1), 4),
          ("TOPPADDING", (0, 0), (-1, -1), 3), ("BOTTOMPADDING", (0, 0), (-1, -1), 3), ("LINEBELOW", (0, 0), (-1, -1), 0.4, GRID)]
    if has_header:
        st += [("BACKGROUND", (0, 0), (-1, 0), NAVY)]
    start = 1 if has_header else 0
    for i in range(start, len(data)):
        if (i - start) % 2 == 1:
            st.append(("BACKGROUND", (0, i), (-1, i), PALE))
    if not has_header:
        st.append(("FONTNAME", (0, 0), (0, -1), "DVB"))
    t.setStyle(TableStyle(st))
    return t


def callout(text, kind):
    col = {"note": BLUE, "warn": CRIT, "tip": GOOD}[kind]
    bg = {"note": colors.HexColor("#eef3fb"), "warn": colors.HexColor("#fbeeee"), "tip": colors.HexColor("#eef8ee")}[kind]
    t = Table([[para(text, "box")]], colWidths=[W])
    t.setStyle(TableStyle([("BACKGROUND", (0, 0), (-1, -1), bg), ("LINEBEFORE", (0, 0), (0, -1), 3.5, col),
                           ("LEFTPADDING", (0, 0), (-1, -1), 9), ("RIGHTPADDING", (0, 0), (-1, -1), 8),
                           ("TOPPADDING", (0, 0), (-1, -1), 6), ("BOTTOMPADDING", (0, 0), (-1, -1), 6)]))
    return [Spacer(1, 3), t, Spacer(1, 6)]


def figure(path, caption, frac):
    im = PILImage.open(path)
    w = W * frac
    h = w * im.height / im.width
    out = [Image(path, width=w, height=h)]
    if caption:
        out.append(para(caption, "cap"))
    else:
        out.append(Spacer(1, 6))
    return KeepTogether(out)


def build(lang, out):
    mod = importlib.import_module(f"content_{lang}")
    meta, blocks = mod.META, mod.B
    fig = figs.make_all(lang, os.path.join(HERE, f"fig_{lang}"))
    doc = Doc(out, meta)
    story = [NextPageTemplate("body"), Spacer(1, 95 * mm)]
    # cover: fill-in table and who reads what
    story.append(para(meta["fill_title"], "h2"))
    ft = Table([[para(f"<b>{k}</b>", "td"), para(v or "", "td")] for k, v in meta["fill"]], colWidths=[0.38 * W, 0.62 * W], rowHeights=9 * mm)
    ft.setStyle(TableStyle([("GRID", (0, 0), (-1, -1), 0.6, INK2), ("VALIGN", (0, 0), (-1, -1), "MIDDLE")]))
    story += [ft, Spacer(1, 10), para(meta["who_title"], "h2"), table(["", ""], meta["who"], [0.45, 0.55], False), PageBreak()]
    toc = TableOfContents(); toc.levelStyles = [S["toc1"], S["toc2"]]
    story += [para(meta["contents"], "h1"), toc]
    n = 0
    first = True
    for b in blocks:
        k = b[0]
        if k == "part":
            story += [NextPageTemplate("part"), PageBreak(), Spacer(1, 40 * mm), PartTitle(b[1], b[2], b[3], meta["part"]),
                      NextPageTemplate("body")]
            # invisible anchor for the TOC
            hid = ParagraphStyle("hid", parent=S["p"], fontSize=1, leading=1, textColor=colors.white, spaceAfter=0)
            p = Paragraph(".", hid); p._toc = (0, f"{meta['part']} {b[1]} - {b[2]}")
            story.append(p)
        elif k == "h1":
            numbered = b[2]
            if numbered:
                n += 1
            title = f"{n}. {b[1]}" if numbered else b[1]
            story.append(PageBreak())
            p = para(title, "h1"); p._toc = (1 if numbered else 0, title)
            story.append(p)
            first = False
        elif k == "h2":
            story += [CondPageBreak(30 * mm), para(b[1], "h2")]
        elif k == "p":
            story.append(para(b[1], "p"))
        elif k == "bul":
            story += [Paragraph(x, S["li"], bulletText="•") for x in b[1]]
            story.append(Spacer(1, 4))
        elif k == "steps":
            story += [Paragraph(x, S["li"], bulletText=f"{i + 1}.") for i, x in enumerate(b[1])]
            story.append(Spacer(1, 4))
        elif k == "table":
            hdr = b[4] if len(b) > 4 else True
            story += [table(b[1], b[2], b[3], hdr), Spacer(1, 7)]
        elif k == "fig":
            story.append(figure(fig[b[1]], b[2], b[3]))
        elif k in ("note", "warn", "tip"):
            story += callout(b[1], k)
        elif k == "code":
            t = Table([[Preformatted(b[1], S["code"])]], colWidths=[W])
            t.setStyle(TableStyle([("BACKGROUND", (0, 0), (-1, -1), PALE), ("BOX", (0, 0), (-1, -1), 0.5, GRID),
                                   ("LEFTPADDING", (0, 0), (-1, -1), 7), ("TOPPADDING", (0, 0), (-1, -1), 5), ("BOTTOMPADDING", (0, 0), (-1, -1), 5)]))
            story += [t, Spacer(1, 6)]
    doc.multiBuild(story)
    return out


if __name__ == "__main__":
    print(build(sys.argv[1], sys.argv[2]))
