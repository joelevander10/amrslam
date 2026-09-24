# Bundled typefaces

Both families are licensed under the **SIL Open Font License 1.1**, which
explicitly permits bundling and redistribution. Copies are committed here rather
than linked from a CDN because **the vehicle has no internet on the floor** — a
`<link>` to fonts.googleapis.com renders correctly at a desk and silently falls
back to system fonts on the AGV, which would mean two appearances for one page.

| Family | Weights | Upstream |
|---|---|---|
| Archivo | 600, 700 | https://github.com/Omnibus-Type/Archivo |
| IBM Plex Sans | 400, 500, 600 | https://github.com/IBM/plex |
| IBM Plex Mono | 400, 500, 600 | https://github.com/IBM/plex |

Files are the **latin subset only** (`U+0000-00FF` plus the usual punctuation
ranges), fetched from the Google Fonts CSS2 API on 2026-09-03. 246 KB total.

To refresh, or to add a weight, re-run the fetch with internet available:

    https://fonts.googleapis.com/css2?family=Archivo:wght@600;700
      &family=IBM+Plex+Mono:wght@400;500;600
      &family=IBM+Plex+Sans:wght@400;500;600&display=swap

request it with a modern browser User-Agent (otherwise Google serves ttf rather
than woff2), keep the blocks commented `/* latin */`, and name the files
`<slug>-<weight>.woff2` to match the `@font-face` rules at the top of app.css.

**Not usable by matplotlib.** `font_manager` reads ttf/otf/afm only, so the run
plots in core/plotrun.py stay on DejaVu rather than carrying a second copy of
Plex in another format for one consumer.
