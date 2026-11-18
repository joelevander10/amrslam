# AMR operator manual (source)

The PDFs `AMR_Manual_EN.pdf` and `AMR_Manual_ID.pdf` one folder up are built from these files.

- `content_en.py`, `content_id.py`: the text, chapter by chapter (English, Bahasa Indonesia).
- `figs.py`: the diagrams (labels for both languages in `L`).
- `build.py`: the PDF layout.

Rebuild (needs python3 with reportlab, matplotlib, pillow and the DejaVu fonts):

```bash
python3 build.py en ../AMR_Manual_EN.pdf
python3 build.py id ../AMR_Manual_ID.pdf
```
