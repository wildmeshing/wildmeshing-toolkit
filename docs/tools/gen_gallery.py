"""Generate the gallery page (gallery/index.md) from docs/gallery/examples.yaml.

Run by mkdocs-gen-files on every build (mkdocs.yml). Every example gets its viewer
(<wmtk-viewer example="<id>">), a link to the page that explains it, and what it runs. The
viewer's assets come from docs/tools/build_gallery.py.
"""

import json
from pathlib import Path

import mkdocs_gen_files
import yaml

ROOT = Path(__file__).resolve().parents[2]
examples = yaml.safe_load((ROOT / "docs" / "gallery" / "examples.yaml").read_text())["examples"]

GROUPS = {
    "tutorials": ("The tutorials", "The first run of each tutorial."),
    "workflows": ("Workflows", "Pipelines of several components."),
    "robustness": ("Inputs from the wild",
                   "TetWild on inputs as they come: triangle soups with self-intersections, "
                   "holes, non-manifold edges and inconsistent orientations, from "
                   "[Thingi10K](https://ten-thousand-models.appspot.com/) among others."),
}

lines = [
    "# Gallery",
    "",
    "Each example runs a component, or a pipeline of them, on one of the example meshes, from "
    "the current code: CI runs them again whenever the code they depend on changes "
    "([`docs/gallery/examples.yaml`](https://github.com/wildmeshing/wildmeshing-toolkit/blob/main/docs/gallery/examples.yaml)).",
    "",
    "The input is on the left, the output on the right. Drag to rotate, scroll to zoom, "
    "right-drag to pan. On a tet mesh, the cut hides the tets on one side of a plane: pick its "
    "axis and move it with the slider. Cells are colored by element quality (the AMIPS energy, "
    "on a log scale from the mesh's best element to its worst) or by region; the faces of a tet "
    "mesh's boundary are gray.",
    "",
]
for group, (heading, blurb) in GROUPS.items():
    members = [e for e in examples if e.get("group") == group]
    if not members:
        continue
    lines += [f"## {heading}", "", blurb, ""]
    for e in members:
        lines += [f"### {e['title']}", "", "<div>", f'<wmtk-viewer example="{e["id"]}"></wmtk-viewer>',
                  "</div>", ""]
        if e.get("tutorial"):
            kind = "workflow" if group == "workflows" else "tutorial"
            lines += [f"Explained in the [{kind}](../{e['tutorial']}).", ""]
        if "json" in e:
            lines += [f'??? example "The options: `docs/examples/json/{e["json"]}.json`"', "",
                      "    ```json",
                      f'    --8<-- "docs/examples/json/{e["json"]}.json"', "    ```", ""]
        elif "component" in e:
            options = json.dumps(e["options"], indent=2).replace("\n", "\n    ")
            lines += [f'??? example "The options of `{e["component"]}`, on {", ".join(e["inputs"].values())}"',
                      "", "    ```json", f"    {options}", "    ```", ""]
        elif "scene" in e:
            lines += [f"The pipeline is `{e['scene']}` in "
                      "[`docs/gallery/scenes.py`](https://github.com/wildmeshing/wildmeshing-toolkit/blob/main/docs/gallery/scenes.py).",
                      ""]

with mkdocs_gen_files.open("gallery/index.md", "w") as f:
    f.write("\n".join(lines))
