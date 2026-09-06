#!/usr/bin/env python3
import re
import shutil
import subprocess
import urllib.request
import zipfile
from pathlib import Path
from xml.sax.saxutils import escape


ROOT = Path(__file__).resolve().parent
MD_PATH = ROOT / "system_flowchart.md"
OUT_DIR = ROOT / "rendered_flowcharts"
DOCX_PATH = ROOT / "system_flowchart_rendered.docx"
PUPPETEER_CONFIG = ROOT / "puppeteer-config.json"
KROKI_URL = "https://kroki.io/mermaid/svg"


def w_text(text):
    return escape(text, {'"': '&quot;'})


def para(text="", style=None, bold=False, code=False):
    text = text.rstrip()
    ppr = f"<w:pPr><w:pStyle w:val=\"{style}\"/></w:pPr>" if style else ""
    rpr_bits = []
    if bold:
        rpr_bits.append("<w:b/>")
    if code:
        rpr_bits.append("<w:rFonts w:ascii=\"Consolas\" w:hAnsi=\"Consolas\"/><w:sz w:val=\"18\"/>")
    rpr = f"<w:rPr>{''.join(rpr_bits)}</w:rPr>" if rpr_bits else ""
    preserve = " xml:space=\"preserve\"" if text.startswith(" ") or text.endswith(" ") else ""
    return f"<w:p>{ppr}<w:r>{rpr}<w:t{preserve}>{w_text(text)}</w:t></w:r></w:p>"


def bullet(text):
    return (
        "<w:p><w:pPr><w:pStyle w:val=\"ListParagraph\"/>"
        "<w:numPr><w:ilvl w:val=\"0\"/><w:numId w:val=\"1\"/></w:numPr></w:pPr>"
        f"<w:r><w:t>{w_text(text)}</w:t></w:r></w:p>"
    )


def code_block(text, title=None):
    out = []
    if title:
        out.append(para(title, bold=True))
    for line in text.rstrip("\n").splitlines():
        out.append(para(line, code=True))
    return "".join(out)


def table(rows):
    if not rows:
        return ""
    grid = "".join("<w:gridCol w:w=\"2400\"/>" for _ in rows[0])
    out = [
        "<w:tbl><w:tblPr><w:tblStyle w:val=\"TableGrid\"/>"
        "<w:tblW w:w=\"0\" w:type=\"auto\"/></w:tblPr>"
        f"<w:tblGrid>{grid}</w:tblGrid>"
    ]
    for ri, row in enumerate(rows):
        out.append("<w:tr>")
        for cell in row:
            bg = "<w:shd w:fill=\"DDEBF7\"/>" if ri == 0 else ""
            out.append(
                "<w:tc><w:tcPr><w:tcW w:w=\"2400\" w:type=\"dxa\"/>"
                f"{bg}</w:tcPr>"
                f"{para(cell.strip(), bold=(ri == 0))}"
                "</w:tc>"
            )
        out.append("</w:tr>")
    out.append("</w:tbl>")
    return "".join(out)


def parse_table(lines, start):
    rows = []
    i = start
    while i < len(lines) and lines[i].strip().startswith("|"):
        raw = lines[i].strip()
        cells = [c.strip() for c in raw.strip("|").split("|")]
        if all(re.fullmatch(r":?-{3,}:?", c.replace(" ", "")) for c in cells):
            i += 1
            continue
        rows.append(cells)
        i += 1
    return rows, i


def extract_mermaid(md):
    blocks = []
    def repl(match):
        blocks.append(match.group(1).strip())
        return f"\n\n[[MERMAID_DIAGRAM_{len(blocks):02d}]]\n\n"
    return re.sub(r"```mermaid\n([\s\S]*?)```", repl, md), blocks


def sanitize_mermaid_for_kroki(code):
    if not code.lstrip().startswith("flowchart"):
        return code

    # Kroki's Mermaid parser is stricter with labels such as /home/... inside
    # bracket shapes. Quote labels for generated report images while leaving
    # the source Markdown untouched.
    def square(match):
        label = match.group(2).replace('"', "'")
        return f'{match.group(1)}["{label}"]'

    def curly(match):
        label = match.group(2).replace('"', "'")
        return f'{match.group(1)}{{"{label}"}}'

    code = re.sub(r"\b([A-Za-z][A-Za-z0-9_]*)\[([^\]\n]+)\]", square, code)
    code = re.sub(r"\b([A-Za-z][A-Za-z0-9_]*)\{([^}\n]+)\}", curly, code)
    return code


def render_diagrams(blocks):
    OUT_DIR.mkdir(exist_ok=True)
    for old in OUT_DIR.glob("*"):
        if old.is_file():
            old.unlink()
    svgs = []
    for idx, code in enumerate(blocks, 1):
        mmd = OUT_DIR / f"diagram_{idx:02d}.mmd"
        svg = OUT_DIR / f"diagram_{idx:02d}.svg"
        mmd.write_text(code + "\n", encoding="utf-8")
        request = urllib.request.Request(
            KROKI_URL,
            data=sanitize_mermaid_for_kroki(code).encode("utf-8"),
            headers={"Content-Type": "text/plain", "User-Agent": "Mozilla/5.0"},
        )
        with urllib.request.urlopen(request, timeout=30) as response:
            svg.write_bytes(response.read())
        svgs.append(svg)
    return svgs


def svg_size(svg_path):
    text = svg_path.read_text(encoding="utf-8", errors="ignore")
    view = re.search(r"viewBox=\"[^\"]*?([0-9.]+)\s+([0-9.]+)\"", text)
    if view:
        return float(view.group(1)), float(view.group(2))
    width = re.search(r"width=\"([0-9.]+)", text)
    height = re.search(r"height=\"([0-9.]+)", text)
    if width and height:
        return float(width.group(1)), float(height.group(1))
    return 1000.0, 650.0


def image_para(rid, svg_path, title):
    px_w, px_h = svg_size(svg_path)
    max_w_in = 9.8
    max_h_in = 6.4
    aspect = px_h / max(px_w, 1.0)
    w_in = max_w_in
    h_in = w_in * aspect
    if h_in > max_h_in:
        h_in = max_h_in
        w_in = h_in / max(aspect, 0.01)
    cx = int(w_in * 914400)
    cy = int(h_in * 914400)
    name = w_text(title)
    return f"""
<w:p>
  <w:pPr><w:jc w:val="center"/></w:pPr>
  <w:r>
    <w:drawing>
      <wp:inline xmlns:wp="http://schemas.openxmlformats.org/drawingml/2006/wordprocessingDrawing" distT="0" distB="0" distL="0" distR="0">
        <wp:extent cx="{cx}" cy="{cy}"/>
        <wp:docPr id="{rid}" name="{name}"/>
        <a:graphic xmlns:a="http://schemas.openxmlformats.org/drawingml/2006/main">
          <a:graphicData uri="http://schemas.openxmlformats.org/drawingml/2006/picture">
            <pic:pic xmlns:pic="http://schemas.openxmlformats.org/drawingml/2006/picture">
              <pic:nvPicPr>
                <pic:cNvPr id="{rid}" name="{name}"/>
                <pic:cNvPicPr/>
              </pic:nvPicPr>
              <pic:blipFill>
                <a:blip xmlns:r="http://schemas.openxmlformats.org/officeDocument/2006/relationships" r:embed="rId{rid}"/>
                <a:stretch><a:fillRect/></a:stretch>
              </pic:blipFill>
              <pic:spPr>
                <a:xfrm><a:off x="0" y="0"/><a:ext cx="{cx}" cy="{cy}"/></a:xfrm>
                <a:prstGeom prst="rect"><a:avLst/></a:prstGeom>
              </pic:spPr>
            </pic:pic>
          </a:graphicData>
        </a:graphic>
      </wp:inline>
    </w:drawing>
  </w:r>
</w:p>
"""


def md_to_body(md, svgs):
    lines = md.splitlines()
    body = []
    i = 0
    while i < len(lines):
        line = lines[i]
        stripped = line.strip()
        marker = re.fullmatch(r"\[\[MERMAID_DIAGRAM_(\d+)\]\]", stripped)
        if marker:
            idx = int(marker.group(1))
            body.append(para(f"Diagram {idx}", bold=True))
            body.append(image_para(idx, svgs[idx - 1], f"Diagram {idx}"))
            i += 1
            continue
        if stripped.startswith("```"):
            lang = stripped[3:].strip()
            block = []
            i += 1
            while i < len(lines) and not lines[i].strip().startswith("```"):
                block.append(lines[i])
                i += 1
            i += 1
            body.append(code_block("\n".join(block), title=f"Code block: {lang}" if lang else "Code block"))
            continue
        if not stripped:
            body.append(para(""))
            i += 1
            continue
        if stripped.startswith("|"):
            rows, i = parse_table(lines, i)
            body.append(table(rows))
            continue
        if stripped.startswith("# "):
            body.append(para(stripped[2:].strip(), style="Title"))
        elif stripped.startswith("## "):
            body.append(para(stripped[3:].strip(), style="Heading1"))
        elif stripped.startswith("### "):
            body.append(para(stripped[4:].strip(), style="Heading2"))
        elif stripped.startswith("- "):
            body.append(bullet(stripped[2:].strip()))
        else:
            body.append(para(stripped))
        i += 1
    return "".join(body)


def build_docx(body, svgs):
    content_types = """<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">
  <Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>
  <Default Extension="xml" ContentType="application/xml"/>
  <Default Extension="svg" ContentType="image/svg+xml"/>
  <Override PartName="/word/document.xml" ContentType="application/vnd.openxmlformats-officedocument.wordprocessingml.document.main+xml"/>
  <Override PartName="/word/styles.xml" ContentType="application/vnd.openxmlformats-officedocument.wordprocessingml.styles+xml"/>
  <Override PartName="/word/numbering.xml" ContentType="application/vnd.openxmlformats-officedocument.wordprocessingml.numbering+xml"/>
</Types>"""
    root_rels = """<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
  <Relationship Id="rDoc" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/officeDocument" Target="word/document.xml"/>
</Relationships>"""
    doc_rels_items = [
        f'<Relationship Id="rId{i}" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/image" Target="media/{svg.name}"/>'
        for i, svg in enumerate(svgs, 1)
    ]
    doc_rels = (
        '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
        '<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">'
        + "".join(doc_rels_items)
        + '</Relationships>'
    )
    styles = """<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<w:styles xmlns:w="http://schemas.openxmlformats.org/wordprocessingml/2006/main">
  <w:style w:type="paragraph" w:default="1" w:styleId="Normal"><w:name w:val="Normal"/><w:rPr><w:rFonts w:ascii="Arial" w:hAnsi="Arial" w:eastAsia="Tahoma"/><w:sz w:val="22"/></w:rPr></w:style>
  <w:style w:type="paragraph" w:styleId="Title"><w:name w:val="Title"/><w:rPr><w:b/><w:sz w:val="34"/></w:rPr></w:style>
  <w:style w:type="paragraph" w:styleId="Heading1"><w:name w:val="heading 1"/><w:pPr><w:spacing w:before="360" w:after="120"/></w:pPr><w:rPr><w:b/><w:sz w:val="28"/></w:rPr></w:style>
  <w:style w:type="paragraph" w:styleId="Heading2"><w:name w:val="heading 2"/><w:pPr><w:spacing w:before="240" w:after="80"/></w:pPr><w:rPr><w:b/><w:sz w:val="24"/></w:rPr></w:style>
  <w:style w:type="paragraph" w:styleId="ListParagraph"><w:name w:val="List Paragraph"/><w:pPr><w:ind w:left="720"/></w:pPr></w:style>
  <w:style w:type="table" w:styleId="TableGrid"><w:name w:val="Table Grid"/><w:tblPr><w:tblBorders><w:top w:val="single" w:sz="4"/><w:left w:val="single" w:sz="4"/><w:bottom w:val="single" w:sz="4"/><w:right w:val="single" w:sz="4"/><w:insideH w:val="single" w:sz="4"/><w:insideV w:val="single" w:sz="4"/></w:tblBorders></w:tblPr></w:style>
</w:styles>"""
    numbering = """<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<w:numbering xmlns:w="http://schemas.openxmlformats.org/wordprocessingml/2006/main">
  <w:abstractNum w:abstractNumId="0"><w:lvl w:ilvl="0"><w:start w:val="1"/><w:numFmt w:val="bullet"/><w:lvlText w:val="•"/><w:pPr><w:ind w:left="720" w:hanging="360"/></w:pPr></w:lvl></w:abstractNum>
  <w:num w:numId="1"><w:abstractNumId w:val="0"/></w:num>
</w:numbering>"""
    document = f"""<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<w:document
  xmlns:w="http://schemas.openxmlformats.org/wordprocessingml/2006/main"
  xmlns:r="http://schemas.openxmlformats.org/officeDocument/2006/relationships">
  <w:body>
    {body}
    <w:sectPr><w:pgSz w:w="16838" w:h="11906" w:orient="landscape"/><w:pgMar w:top="720" w:right="720" w:bottom="720" w:left="720" w:header="360" w:footer="360" w:gutter="0"/></w:sectPr>
  </w:body>
</w:document>"""
    with zipfile.ZipFile(DOCX_PATH, "w", zipfile.ZIP_DEFLATED) as z:
        z.writestr("[Content_Types].xml", content_types)
        z.writestr("_rels/.rels", root_rels)
        z.writestr("word/_rels/document.xml.rels", doc_rels)
        z.writestr("word/document.xml", document)
        z.writestr("word/styles.xml", styles)
        z.writestr("word/numbering.xml", numbering)
        for svg in svgs:
            z.write(svg, f"word/media/{svg.name}")


def main():
    md = MD_PATH.read_text(encoding="utf-8")
    md_marked, blocks = extract_mermaid(md)
    svgs = render_diagrams(blocks)
    body = md_to_body(md_marked, svgs)
    build_docx(body, svgs)
    print(DOCX_PATH)


if __name__ == "__main__":
    main()
