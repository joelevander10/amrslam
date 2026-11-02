# Render checklist

Run `agent/scripts/preview.ps1 <path-to-sheet.html>`, open the PNG it prints (`%TEMP%\jis-preview\<name>.png`), and check each item. Fix, preview again, re-check — at most two rounds. The preview PNG is throw-away; nothing but the HTML is written to the project.

## Content (compare against the source, line by line)
- [ ] Every branch / rung from the source is present, on the same line number.
- [ ] Device order along each branch matches the source.
- [ ] Every name, rating, wire spec and cross-reference matches the source or the confirmed interpretation.
- [ ] Every wire label written along a conductor in the source (e.g. `1P24` either side of a TB, at the device) is on the sheet, on the same segment.
- [ ] Junction dots only where the source has a connection; crossings without dots elsewhere.
- [ ] Buses start, end and continue (tee / arrow) as in the source.
- [ ] Title block fields match; sheet number and total are right.
- [ ] Every red item is either a genuine `?` doubt or a `«placeholder»` you will report — nothing red by accident.

## Legibility
- [ ] No text overlaps a line, symbol or other text (look closely at labels above breakers, contactors and cable marks).
- [ ] No text runs past the frame (right edge x ≈ 201 mm). Long descriptions → shorten only with the user's consent, otherwise reduce by moving to `desc` two lines.
- [ ] Nothing is cut off at the grid (y 240.5) or overlaps the title block. Title-block values fit their cell (the sheet-total cell is narrow: use `'«XX»'`, not `'«TOTAL»'`).
- [ ] Breaker ratings clear the thermal-element jog (single-pole `mccb` on a lone conductor is the usual offender).
- [ ] No Japanese text anywhere on the sheet (title block included); every source label is in English.

## Output
- [ ] Only the HTML file(s) were written to the project — no PNG/PDF unless the user asked.
- [ ] The HTML opens in a browser (relative `kit/` paths correct for its folder) and the toolbar status says nothing is red (or lists what is).
- [ ] If the user asked for PNG/PDF: `export/<name>.png` is 2480 × 3508 px and `export/<name>.pdf` is one A4 page.
