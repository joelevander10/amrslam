"""The layout itself: filesystem anchors and the flat module namespace.

Two failure modes this refactor introduced, both silent:

  * a module that computes a path from its own __file__ points somewhere new
    the moment it moves directory. Nothing raises - run logs simply start
    landing in core/logs/ and the numbering restarts at 0001 beside the real
    runs. Only a check that pins the resolved path catches that.

  * putting core/, drivers/, drivers/canbus/ and app/ on sys.path instead of
    making them packages means module BASENAMES are one flat namespace. Two
    modules sharing a name, or one shadowing a stdlib module, resolves by
    sys.path order - which is to say, by accident.
"""
import pathlib
import sys

from helpers import ROOT, check

import config

# Every directory the app puts on sys.path. Keep in step with the loops in
# main.py, canworker.py, app/server.py and helpers.py.
LAYER_DIRS = ["core", "drivers", "drivers/canbus", "app"]


def test_path_anchors():
    """Directory-dependent paths must resolve to the repo root, not to the
    directory the module happens to live in today."""
    print("\nfilesystem anchors survive the move")

    check("config.PROFILE_DIR points at the repo root",
          pathlib.Path(config.PROFILE_DIR).resolve()
          == (ROOT / "profiles").resolve(), config.PROFILE_DIR)

    check("the profile resolves under profiles/",
          pathlib.Path(config.profile_path()).parent.resolve()
          == (ROOT / "profiles").resolve())


def test_flat_namespace_is_unambiguous():
    """No two modules on the flat sys.path may share a basename, and none may
    shadow a stdlib module."""
    print("\nthe flat module namespace stays unambiguous")

    owners = {}
    clashes = []
    for d in ["."] + LAYER_DIRS:
        for f in sorted((ROOT / d).glob("*.py")):
            if f.name.startswith("_"):
                continue
            name = f.stem
            if name in owners:
                clashes.append(f"{name}: {owners[name]} vs {d}")
            owners[name] = d
    check("no two modules share a basename", not clashes, "; ".join(clashes))

    # Shadowing a stdlib name would break imports in ways that depend on which
    # directory happened to be inserted first.
    shadowed = sorted(set(owners) & set(sys.stdlib_module_names))
    check("no module shadows a standard library module", not shadowed,
          str(shadowed))

    # The layer directories must not be importable as packages, or a module
    # could resolve twice under two names - the reason canbus/ was never made
    # one in the first place.
    inits = [d for d in LAYER_DIRS if (ROOT / d / "__init__.py").exists()]
    check("the layer directories are not packages", not inits, str(inits))

    check("every layer directory exists",
          all((ROOT / d).is_dir() for d in LAYER_DIRS))


def test_entry_point():
    """main.py is what systemd runs, so it has to work from the repo root and
    carry no logic of its own."""
    print("\nthe entry point resolves")

    src = (ROOT / "main.py").read_text(encoding="utf-8")
    check("main.py imports the server", "from server import main" in src)
    check("main.py stays thin", len(src.splitlines()) < 40,
          f"{len(src.splitlines())} lines")
    check("main.py puts the layer directories on sys.path",
          all(d.split("/")[-1] in src for d in LAYER_DIRS))

    # app/server.py, not app/app.py: with a directory named app/ on sys.path,
    # `import app` resolves to an empty namespace package or to app/app.py
    # depending on sys.path ORDER. `import server` cannot be ambiguous.
    check("the Flask module is not named app.py",
          not (ROOT / "app" / "app.py").exists())
    check("Flask finds its templates under app/",
          (ROOT / "app" / "templates" / "base.html").exists()
          and (ROOT / "app" / "static" / "common.js").exists())


def test_assets_are_offline():
    """Nothing the UI needs may be fetched over the network.

    The vehicle has no internet on the floor. A CDN font link renders correctly
    at a desk and silently falls back to system fonts on the AGV - the page
    still WORKS, so nobody notices until they are standing next to it, which is
    exactly the kind of regression worth a source scan.
    """
    import re
    print("\nweb assets are self-contained")

    static = ROOT / "app" / "static"
    tpl = ROOT / "app" / "templates"
    remote = re.compile(
        r'(<link[^>]*https?:)|(<script[^>]*src=["\']https?:)'
        r'|(@import[^;]*https?:)|(url\(\s*["\']?https?:)', re.I)

    for f in sorted(list(tpl.glob("*.html")) + list(static.glob("*.css"))
                    + list(static.glob("*.js"))):
        hits = remote.findall(f.read_text(encoding="utf-8"))
        check(f"{f.name} loads nothing remotely", not hits, str(hits[:1]))

    css = (static / "app.css").read_text(encoding="utf-8")
    faces = re.findall(r'src:\s*url\(["\']?([^"\')]+)', css)
    check("app.css declares the self-hosted faces", len(faces) >= 8, str(len(faces)))
    for rel in faces:
        check(f"{rel} is present", (static / rel).is_file())

    # The families named in the stack must be the ones actually shipped, or the
    # @font-face block is decoration and the browser silently uses the fallback.
    for fam in ("Archivo", "IBM Plex Sans", "IBM Plex Mono"):
        # Tolerate whitespace after the colon rather than stripping it out of
        # the whole file - the family names contain spaces themselves.
        check(f"{fam} is declared and bundled",
              re.search(rf'font-family:\s*"{re.escape(fam)}"', css) is not None)


TESTS = [
    test_path_anchors,
    test_flat_namespace_is_unambiguous,
    test_entry_point,
    test_assets_are_offline,
]
