"""The SimWild tutorial (docs/tutorials/simwild.md), editing regions: one run per operation, each on
a tagged mesh of the example data, in memory; the same runs as json/simwild_<operation>.json."""

import os
from pathlib import Path

import wildmeshing as wm

# The example meshes: $WMTK_DATA, or data/models from the repository root.
DATA = Path(os.environ.get("WMTK_DATA", "data/models"))


def regions(mesh):
    """How many cells each combination of regions has (see python/simwild.py)."""
    top = "tetrahedron" if "tetrahedron" in mesh["cells"] else "triangle"
    tags = mesh["cells"][top]["attributes"]["tags"]
    counts = {}
    for row in tags["values"]:
        key = " & ".join(label for label, member in zip(tags["labels"], row) if member)
        counts[key] = counts.get(key, 0) + 1
    return dict(sorted(counts.items()))


# Two overlapping spheres, regions tag_0 and tag_1; the same two with their overlap left empty;
# and a 2D label image, five regions on a 50 x 50 grid.
double_sphere = wm.read_mesh(str(DATA / "double_sphere.msh"))
double_sphere_hole = wm.read_mesh(str(DATA / "double_sphere_hole.msh"))
groups = wm.read_mesh(str(DATA / "topo_annots_groups.msh"))
for name, mesh in [("double_sphere", double_sphere), ("double_sphere_hole", double_sphere_hole),
                   ("topo_annots_groups", groups)]:
    print(name, regions(mesh))

# --8<-- [start:remeshing]
outputs, report = wm.simwild(
    {"operation": "remeshing", "input": ["m"],
     "sizing_field": [{"tags": "tag_0 & tag_1", "length_rel": 0.02}], "num_threads": 4},
    {"m": double_sphere},
)
print("remeshing:", report["#t"], "tets, max energy", round(report["max_energy"], 2))
print(regions(outputs["mesh"]))
# --8<-- [end:remeshing]

# --8<-- [start:fill_holes]
outputs, report = wm.simwild(
    {"operation": "fill_holes_topo", "input": ["m"], "fill_holes_tags": ["tag_1"]},
    {"m": groups},
)
print("fill_holes_topo:", regions(outputs["mesh"]))
# --8<-- [end:fill_holes]

# --8<-- [start:tight_seal]
outputs, report = wm.simwild(
    {"operation": "tight_seal_topo", "input": ["m"], "tight_seal_tag_sets": [["tag_0", "tag_1"]]},
    {"m": double_sphere_hole},
)
print("tight_seal_topo:", report["#t"], "tets, max energy", round(report["max_energy"], 2))
print(regions(outputs["mesh"]))
# --8<-- [end:tight_seal]

# --8<-- [start:keep_lcc]
outputs, report = wm.simwild(
    {"operation": "keep_lcc", "input": ["m"], "keep_lcc_tags": ["tag_0"], "keep_lcc_num": 1},
    {"m": groups},
)
print("keep_lcc:", regions(outputs["mesh"]))
# --8<-- [end:keep_lcc]

# --8<-- [start:resolve_overlaps]
resolved, report = wm.simwild(
    {"operation": "resolve_overlaps", "input": ["m"],
     "resolve_overlaps_tags": [["tag_0", "tag_1"]]},
    {"m": double_sphere},
)
print("resolve_overlaps:", report["#t"], "tets, max energy", round(report["max_energy"], 2))
print(regions(resolved["mesh"]))
# --8<-- [end:resolve_overlaps]

# --8<-- [start:resolve_then_remesh]
# The split leaves slivers along the new interface: a remeshing removes them.
remeshed, report = wm.simwild(
    {"operation": "remeshing", "input": ["m"], "num_threads": 4},
    {"m": resolved["mesh"]},
)
print("remeshed:", report["#t"], "tets, max energy", round(report["max_energy"], 2))
# --8<-- [end:resolve_then_remesh]

# --8<-- [start:replace_tags]
outputs, report = wm.simwild(
    {"operation": "replace_tags", "input": ["m"],
     "replace_tags_in": ["tag_0 & tag_1"], "replace_tags_out": ["overlap"]},
    {"m": double_sphere},
)
print("replace_tags:", regions(outputs["mesh"]))
# --8<-- [end:replace_tags]

# --8<-- [start:tag_priority]
outputs, report = wm.simwild(
    {"operation": "tag_priority", "input": ["m"], "tag_priority": ["tag_1", "tag_0"]},
    {"m": double_sphere},
)
print("tag_priority:", regions(outputs["mesh"]))
# --8<-- [end:tag_priority]
