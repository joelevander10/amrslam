# Reading a photo or pen sketch

Most inputs are pen-on-paper sketches or phone photos of printed drawings. Reading them correctly is the hard part of the job — spend your care here, not on graphics.

## 1. Orient
- Find the title block or the line numbers. The true sheet has the title block at the bottom and line numbers down the left. If the photo is sideways, describe everything in the true orientation.
- Binder photos have perspective, holes and curl. Trust topology (what connects to what) over exact position.

## 2. Transcribe structure before values
Go in this order and write it down:
1. **Buses**: which vertical lines exist, their names, where they start and end, whether they continue (tee/arrow at the bottom).
2. **Branches / rungs**, top to bottom: the line number each starts on, where it taps from (which bus), and the device sequence left to right.
3. **Junctions**: a dot means connected; a crossing without a dot means not connected. In sketches, a hand-drawn blob or a small circle on a crossing is a junction; a plain crossing or a hop is not.
4. **Values**: names, ratings, wire specs, cross-reference numbers, descriptions.

## 3. Sketch-specific reading
Hand sketches are simplified. Expect and handle:
- **Abbreviated chains**: a box labelled "MS" with a motor circle means the standard motor chain. Use the full chain only if the user confirms it; otherwise draw only what is sketched.
- **"Same as above" marks** (ditto, arrows, "×3", "typ."): repeat the structure, but never copy values unless the sketch says so. Unknown values → `«placeholder»`.
- **Single-line sketches**: one line with "3φ" or a slash with 3 ticks means three phases. Draw three conductors.
- **Relative placement**: the sketch's layout is intent, not coordinates. Map to the nearest line numbers and the standard column plan.
- **Crossed-out or rewritten text**: take the latest writing; mention it if both versions are legible.
- **Net names written along a conductor** (`1P24` on both sides of a terminal, again at the device): these are **wire labels** — the marker text to attach to each cable. Transcribe every one onto its segment (bus → TB, TB → device, device end). Do not drop them because the bus name already implies the net, and do not move them into the device box. A name written next to a box edge that is not a net (A1, A2, 20/NET-VIN) is a terminal name and goes inside the box.
- **Hops**: a small arc where one line jumps another = crossing, **not** connected. Draw a plain crossing without a dot.
- **Cable specs as ellipse + arrow**: an ellipse around several conductors with an arrow to text (`NYAF 2,5mm² RED`) is one cable mark on those conductors. European decimal commas become dots (`2.5mm²`).
- **Arrow at a line end with a name** (`→ P24`) = conductor leaves the sheet; use `endLabels`. A short bar across a line end = bus continues (`'tee'`).
- **Stray marks** (a lone letter, a slash across a box corner): ask once in the questions; if the user says scribble, ignore them.
- **Title written as "PAGE: 01 / XX"**: the sheet total is unknown → `'«XX»'`.

## 4. Confidence
Mark every value as one of:
- **Read** — clearly legible. Use as is.
- **Doubtful** — probable reading. Use with a `?` prefix (renders red) and list it.
- **Missing** — not in the source. Leave `«placeholder»` and list it. Never fill from "typical" values.

## 5. The interpretation message (mandatory)
Fill in `agent/templates/interpretation.md` and send it before building. Keep it to what the user needs to verify:
- title block;
- bus table;
- branch/rung table (line number, tap from, devices in order, values);
- doubtful and missing items;
- assumptions (what you decided without asking);
- **at most 3 questions**, each one about something that changes the drawing.

Then wait. Build only after the user confirms or corrects.

## Conventions to suggest to the user for new sketches
If sketches keep causing questions, offer this short list:
- Write line numbers (00–21) in the left margin, one branch or rung per numbered line.
- Put a dot on every real junction; draw crossings without dots.
- Write every device name next to its symbol (MS1, OL1, CB3 …) and its value right after it (2.2A, 0.4kW).
- Write wire specs as "KIV2 BLACK" beside the line.
- Use "same as line 04" for repeated branches, but still write each branch's own names and values.
- Circle anything you are unsure about yourself, so it is flagged rather than guessed.
