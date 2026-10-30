"""Build the gallery's assets: run each example of docs/gallery/examples.yaml through the Python
bindings, in memory, and write the meshes it shows for the viewer (docs/viewer).

    python docs/tools/build_gallery.py                 # every example
    python docs/tools/build_gallery.py qslim tetwild   # some
    python docs/tools/build_gallery.py --posters       # and a poster image of each
    python docs/tools/build_gallery.py --check         # the pages name only examples that exist
    python docs/tools/build_gallery.py --cache-key     # the key CI caches the assets by

Needs the wildmeshing module (PYTHONPATH=build/bin, or installed) and data/models ($WMTK_DATA).
Writes docs/gallery/assets/<id>/: input.wmv.gz and output.wmv.gz (the views), viewer.json (what
<wmtk-viewer example="<id>"> shows), report.json (the component's report) and, with --posters,
poster.png, a screenshot of the example in the viewer itself taken with Playwright in headless
Chromium (`pip install playwright && playwright install chromium`, or set
WMTK_GALLERY_BROWSER=chrome to use an installed Chrome). The directory is not committed.

An example runs a JSON example of the tutorials (`json: <name>`, docs/examples/json/<name>.json,
in memory: every mesh file it names is read and passed by its stem), a component on meshes of
data/models (`component`, `inputs`, `options`), or a scene, a function of docs/gallery/scenes.py
that returns the meshes to show (`scene: <function>`), for pipelines.

The .wmv format: "WMV1", a little-endian uint32 header length, a JSON header, zero padding to a
multiple of 4 bytes, then the arrays the header lists (offset from the end of the padding, type,
count), all gzipped:

    positions      uint16, 3 per vertex, quantized over the header's bbox
    triangles      uint32, 3 per triangle (a surface)
    tets           uint32, 4 per tet
    tet_neighbors  int32, 4 per tet: the tet across the face opposite each corner, -1 for none
    cell_quality   float32, per tet or triangle: its AMIPS energy, when the mesh has it
    cell_region    uint8, per cell: the index of its regions (header "regions": "a", "a & b",
                   ...), 255 for none
    lines          uint32, 2 per segment
"""

import argparse
import gzip
import hashlib
import importlib.util
import json
import os
import re
import subprocess
import sys
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[2]
DOCS = ROOT / "docs"
EXAMPLES = DOCS / "gallery" / "examples.yaml"
SCENES = DOCS / "gallery" / "scenes.py"
ASSETS = DOCS / "gallery" / "assets"
DATA = Path(os.environ.get("WMTK_DATA", ROOT / "data" / "models"))

MESH_FILES = {".obj", ".stl", ".off", ".ply", ".msh"}
# Cell fields holding the element energy, in the order they are looked for.
QUALITY_FIELDS = ["t_energy", "energy", "quality", "amips", "tet_energy"]
# Tag columns that are not regions: the background, and the envelope's group.
NOT_REGIONS = {"ambient", "EnvelopeSurface"}

# The faces of a tet, opposite corners 0..3 (the viewer uses the same order).
TET_FACES = np.array([[1, 2, 3], [0, 3, 2], [0, 1, 3], [0, 2, 1]])


def tet_neighbors(T):
    """For each tet and corner, the tet across the opposite face, -1 on the boundary."""
    m = len(T)
    F = np.sort(T[:, TET_FACES].reshape(-1, 3), axis=1)
    order = np.lexsort((F[:, 2], F[:, 1], F[:, 0]))
    S = F[order]
    same = np.flatnonzero((S[1:] == S[:-1]).all(axis=1))
    i, j = order[same], order[same + 1]
    N = np.full(4 * m, -1, dtype=np.int32)
    N[i] = j // 4
    N[j] = i // 4
    return N


def region_columns(attributes):
    """The `tags` field's region columns (values, labels), without the background."""
    tags = attributes.get("tags")
    if tags is None:
        return None, []
    values = np.asarray(tags["values"]).astype(bool)
    labels = list(tags.get("labels", [])) or [f"tag_{i}" for i in range(values.shape[1])]
    keep = [i for i, label in enumerate(labels) if label not in NOT_REGIONS]
    return values[:, keep], [labels[i] for i in keep]


def regions(attributes):
    """Each cell's set of regions as an index into the distinct sets ("a", "a & b"), 255 for
    none, and the sets' names."""
    values, labels = region_columns(attributes)
    if values is None or values.shape[1] == 0:
        return None, []
    combos, index = np.unique(values, axis=0, return_inverse=True)
    names, ids = [], np.full(len(combos), 255, dtype=np.uint8)
    for k, combo in enumerate(combos):
        if combo.any() and len(names) < 254:
            ids[k] = len(names)
            names.append(" & ".join(l for l, on in zip(labels, combo) if on))
    return ids[np.ravel(index)], names


def cell_block(mesh):
    for kind in ("tetrahedron", "triangle"):
        if kind in mesh["cells"] and len(mesh["cells"][kind]["connectivity"]):
            return kind, mesh["cells"][kind]
    return None, None


def keep_regions(mesh):
    """The mesh without its background cells: those in no region."""
    kind, block = cell_block(mesh)
    if block is None:
        return mesh
    values, _ = region_columns(block.get("attributes", {}))
    if values is None:
        return mesh
    keep = values.any(axis=1)
    attrs = {}
    for name, a in block.get("attributes", {}).items():
        attrs[name] = dict(a, values=np.asarray(a["values"])[keep])
    cells = {kind: {"connectivity": np.asarray(block["connectivity"])[keep], "attributes": attrs}}
    if "edge" in mesh["cells"]:
        cells["edge"] = mesh["cells"]["edge"]
    return {"V": mesh["V"], "cells": cells}


def merge(meshes, transforms=None):
    """One mesh of several (the inputs of a component), each moved by its 4x4 or 3x3 transform.
    A single input, unmoved, is returned as it is, with its fields."""
    if len(meshes) == 1 and not (transforms and transforms[0]):
        return meshes[0]
    Vs, blocks, offset = [], {}, 0
    for i, mesh in enumerate(meshes):
        V = np.asarray(mesh["V"], dtype=np.float64)
        M = np.asarray(transforms[i]) if transforms and i < len(transforms) and transforms[i] else None
        if M is not None:
            d = M.shape[0] - 1
            V = V.copy()
            V[:, :d] = V[:, :d] @ M[:d, :d].T + M[:d, d]
        Vs.append(V)
        for kind, block in mesh["cells"].items():
            blocks.setdefault(kind, []).append(np.asarray(block["connectivity"]) + offset)
        offset += len(V)
    return {"V": np.vstack(Vs), "cells": {k: {"connectivity": np.vstack(v)} for k, v in blocks.items()}}


def write_wmv(path, mesh, title):
    V = np.asarray(mesh["V"], dtype=np.float64)
    lo, hi = V.min(axis=0), V.max(axis=0)
    extent = np.where(hi > lo, hi - lo, 1.0)
    arrays = {"positions": np.round((V - lo) / extent * 65535).astype(np.uint16).ravel()}
    header = {"title": title, "bbox": [lo.tolist(), hi.tolist()],
              "dim": 2 if np.ptp(V[:, 2]) == 0 else 3, "vertices": len(V)}
    kind, block = cell_block(mesh)
    n_cells = 0
    if kind == "tetrahedron":
        T = np.asarray(block["connectivity"], dtype=np.uint32)
        arrays["tets"] = T.ravel()
        arrays["tet_neighbors"] = tet_neighbors(T.astype(np.int64))
        n_cells = len(T)
    elif kind == "triangle":
        F = np.asarray(block["connectivity"], dtype=np.uint32)
        arrays["triangles"] = F.ravel()
        n_cells = len(F)
    if block is not None:
        attributes = block.get("attributes", {})
        for name in QUALITY_FIELDS:
            if name in attributes:
                q = np.asarray(attributes[name]["values"], dtype=np.float32)[:, 0]
                if np.all(np.isfinite(q)) and q.min() > 0:
                    arrays["cell_quality"] = q
                    header["quality"] = {"field": name, "min": float(q.min()), "max": float(q.max())}
                    break
        region, names = regions(attributes)
        if region is not None:
            arrays["cell_region"] = region
            header["regions"] = names
    if "edge" in mesh["cells"] and len(mesh["cells"]["edge"]["connectivity"]):
        arrays["lines"] = np.asarray(mesh["cells"]["edge"]["connectivity"], dtype=np.uint32).ravel()
        n_cells = n_cells or len(arrays["lines"]) // 2
    if n_cells == 0:
        raise ValueError(f"{path}: nothing to show")

    blobs, offset, header["arrays"] = [], 0, {}
    for name, a in arrays.items():
        a = np.ascontiguousarray(a)
        header["arrays"][name] = {"type": a.dtype.name, "offset": offset, "count": int(a.size)}
        blob = a.astype(a.dtype.newbyteorder("<")).tobytes()
        blob += b"\0" * (-len(blob) % 4)
        blobs.append(blob)
        offset += len(blob)
    head = json.dumps(header, separators=(",", ":")).encode()
    data = b"WMV1" + len(head).to_bytes(4, "little") + head
    data += b"\0" * (-len(data) % 4) + b"".join(blobs)
    path.parent.mkdir(parents=True, exist_ok=True)
    # mtime 0: the same mesh gives the same file.
    with open(path, "wb") as f:
        f.write(gzip.compress(data, compresslevel=9, mtime=0))
    return n_cells, path.stat().st_size


def run_json(name):
    """A JSON example of the tutorials, in memory: (input meshes in order, outputs, report)."""
    import wildmeshing as wm

    options = json.loads((DOCS / "examples" / "json" / f"{name}.json").read_text())
    component = options.pop("application")
    for key in ("output", "report", "log_file"):
        options.pop(key, None)
    meshes, order = {}, []

    def by_name(value, key):
        if isinstance(value, list):
            return [by_name(v, key) for v in value]
        if isinstance(value, str) and Path(value).suffix in MESH_FILES and (DATA / value).is_file():
            stem = Path(value).stem
            if stem not in meshes:
                meshes[stem] = wm.read_mesh(str(DATA / value))
            if key == "input":
                order.append(stem)
            return stem
        return value

    options = {k: by_name(v, k) for k, v in options.items()}
    outputs, report = getattr(wm, component)(options, meshes)
    inputs = [meshes[n] for n in order]
    return inputs, options.get("input_transform"), meshes, outputs, report


def run_component(example):
    """A component on meshes of data/models (`component`, `inputs`: name -> file, `options`)."""
    import wildmeshing as wm

    meshes = {name: wm.read_mesh(str(DATA / file)) for name, file in example["inputs"].items()}
    outputs, report = getattr(wm, example["component"])(example["options"], meshes)
    return list(meshes.values()), example["options"].get("input_transform"), meshes, outputs, report


def build(example):
    if "json" in example or "component" in example:
        run = run_json(example["json"]) if "json" in example else run_component(example)
        inputs, transforms, meshes, outputs, report = run
        views = {}
        for view, spec in example.get("views", {"input": {}, "output": {}}).items():
            source = spec.get("mesh", "input/*" if view == "input" else "output/mesh")
            side, name = source.split("/", 1)
            if side == "input":
                mesh = merge(inputs, transforms) if name == "*" else meshes[name]
            else:
                mesh = outputs[name]
            views[view] = (mesh, spec)
    else:
        spec = importlib.util.spec_from_file_location("scenes", SCENES)
        scenes = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(scenes)
        shown, report = getattr(scenes, example["scene"])(DATA)
        views = {v: (m, example.get("views", {}).get(v, {})) for v, m in shown.items()}

    out_dir = ASSETS / example["id"]
    manifest = {"title": example["title"], "views": {}, "labels": example.get("labels", "Input|Output"),
                "color": example.get("color", "quality"), "cut": example.get("cut", "x")}
    for key in ("camera", "zoom", "cut-position"):
        if key in example:
            manifest[key] = example[key]
    summary = {"views": {}}
    for view, (mesh, spec) in views.items():
        if view not in ("input", "output"):
            raise SystemExit(f"{example['id']}: a view is `input` or `output`, not `{view}`")
        if spec.get("keep") == "regions":
            mesh = keep_regions(mesh)
        n_cells, size = write_wmv(out_dir / f"{view}.wmv.gz", mesh, f"{example['title']}: {view}")
        if n_cells > example.get("max_cells", 250000):
            raise SystemExit(f"{example['id']}/{view}: {n_cells} cells, over the budget")
        manifest["views"][view] = f"{view}.wmv.gz"
        summary["views"][view] = {"cells": n_cells, "bytes": size}
        print(f"{example['id']}/{view}: {n_cells} cells, {size / 1024:.0f} KiB")
    (out_dir / "viewer.json").write_text(json.dumps(manifest, indent=2) + "\n")
    summary["report"] = {k: v for k, v in report.items() if isinstance(v, (int, float, str, bool))}
    (out_dir / "report.json").write_text(json.dumps(summary, indent=2) + "\n")


def posters(examples):
    """A screenshot of each example in the viewer, as the gallery shows it."""
    import functools
    import http.server
    import threading

    from playwright.sync_api import sync_playwright

    class Quiet(http.server.SimpleHTTPRequestHandler):
        def log_message(self, *args):
            pass

    handler = functools.partial(Quiet, directory=str(DOCS))
    server = http.server.ThreadingHTTPServer(("127.0.0.1", 0), handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    base = f"http://127.0.0.1:{server.server_address[1]}/gallery/"
    try:
        with sync_playwright() as p:
            channel = os.environ.get("WMTK_GALLERY_BROWSER") or None
            browser = p.chromium.launch(channel=channel, args=["--use-angle=swiftshader",
                                                               "--enable-unsafe-swiftshader"])
            page = browser.new_page(viewport={"width": 1000, "height": 800})
            for example in examples:
                html = ('<script type="module" src="../assets/viewer/wmtk-viewer.min.js"></script>'
                        f'<wmtk-viewer style="width: 960px; height: 600px" example="{example["id"]}">'
                        "</wmtk-viewer>")
                page.goto(base)
                page.set_content(html)
                page.wait_for_function("""() => document.querySelector('wmtk-viewer')
                    ?.shadowRoot?.querySelector('canvas')""", timeout=120000)
                page.wait_for_timeout(500)
                out = ASSETS / example["id"] / "poster.png"
                page.locator("wmtk-viewer").screenshot(path=str(out))
                print(f"{example['id']}: {out.relative_to(ROOT)}")
            browser.close()
    finally:
        server.shutdown()


def check(examples):
    """Every example a page shows is in examples.yaml, and every JSON example exists."""
    ids = {e["id"] for e in examples}
    errors = []
    for e in examples:
        if "json" in e and not (DOCS / "examples" / "json" / f"{e['json']}.json").is_file():
            errors.append(f"examples.yaml: {e['id']}: docs/examples/json/{e['json']}.json does not exist")
        if "tutorial" in e and not (DOCS / e["tutorial"]).is_file():
            errors.append(f"examples.yaml: {e['id']}: {e['tutorial']} does not exist")
    for page in DOCS.rglob("*.md"):
        for used in re.findall(r'<wmtk-viewer[^>]*\bexample="([^"]+)"', page.read_text()):
            if used not in ids:
                errors.append(f"{page.relative_to(ROOT)}: example {used} is not in examples.yaml")
    return errors


def cache_key():
    """What the assets depend on: the examples, the scenes, this script, the JSON examples, the
    viewer and the code (git tree hashes)."""
    h = hashlib.sha256(EXAMPLES.read_bytes() + SCENES.read_bytes() + Path(__file__).read_bytes())
    for path in ["src", "components", "python", "cmake", "docs/examples/json", "docs/viewer"]:
        tree = subprocess.run(["git", "rev-parse", f"HEAD:{path}"], cwd=ROOT, capture_output=True,
                              text=True, check=True).stdout
        h.update(tree.encode())
    return h.hexdigest()[:20]


def load_examples():
    return yaml.safe_load(EXAMPLES.read_text())["examples"]


def main():
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("ids", nargs="*", help="examples to build (default: all)")
    p.add_argument("--cache-key", action="store_true", help="print the cache key and exit")
    p.add_argument("--check", action="store_true", help="check the pages against examples.yaml")
    p.add_argument("--posters", action="store_true", help="also take a poster of each example")
    p.add_argument("--posters-only", action="store_true", help="only take the posters")
    args = p.parse_args()
    if args.cache_key:
        print(cache_key())
        return 0
    if args.check:
        errors = check(load_examples())
        for e in errors:
            print(e, file=sys.stderr)
        print(f"{len(load_examples())} examples, {len(errors)} problems")
        return 1 if errors else 0
    examples = [e for e in load_examples() if not args.ids or e["id"] in args.ids]
    if not args.posters_only:
        for example in examples:
            build(example)
    if args.posters or args.posters_only:
        posters(examples)
    return 0


if __name__ == "__main__":
    sys.exit(main())
