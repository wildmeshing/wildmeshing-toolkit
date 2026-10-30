"""The multi-region workflow (docs/tutorials/workflows/regions.md): two overlapping spheres meshed
into one tet mesh with a region each, the overlap given to one of them, a thin gap carved around the
other, and a final remeshing; in memory, with a checkpoint after each step."""

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


# --8<-- [start:mesh]
# 1. Two copies of one sphere, the second moved 0.3 along x: regions a, b and their overlap.
sphere = wm.read_mesh(str(DATA / "sphere.obj"))
shift = [[1, 0, 0, 0.3], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
m1, report = wm.simwild(
    {"input": ["sphere", "sphere"], "input_names": ["a", "b"], "input_transform": [[], shift],
     "eps_rel": 0.01, "length_rel": 0.05, "num_threads": 4},
    {"sphere": sphere},
)
wm.save(m1, "regions/m1")  # regions/m1.msh and regions/m1_envelope.msh
print("1. simwild:", report["#t"], "tets", regions(m1["mesh"]))
# --8<-- [end:mesh]

# --8<-- [start:edit]
# 2. The overlap goes to b: its cells leave a and stay in b.
m2, report = wm.simwild(
    {"operation": "replace_tags", "input": ["m"],
     "replace_tags_in": ["a & b"], "replace_tags_out": ["b"]},
    {"m": m1["mesh"]},
)
wm.save(m2, "regions/m2")  # no envelope: a region edit returns none
print("2. replace_tags:", regions(m2["mesh"]))
# --8<-- [end:edit]

# --8<-- [start:offset]
# 3. A gap of thickness 0.01 around a, carved out of b and the background.
m3, report = wm.topological_offset(
    {"input": "m", "offset_selection": "a", "offset_output_tags": ["gap"],
     "target_distance": 0.01, "optimize_offset": False, "num_threads": 4},
    {"m": m2["mesh"]},
)
wm.save(m3, "regions/m3")
print("3. topological_offset:", report["after #t"], "tets", regions(m3["mesh"]))
# --8<-- [end:offset]

# --8<-- [start:remesh]
# 4. Improve the elements the offset cut; a thin envelope keeps the gap's walls in place.
m3 = wm.load("regions/m3")  # resume here without running steps 1 to 3 again
m4, report = wm.simwild(
    {"input": ["m"], "eps_rel": 0.001, "num_threads": 4},
    {"m": m3["mesh"]},
)
wm.save(m4, "regions/m4")
print("4. simwild remeshing:", report["#t"], "tets, max energy", round(report["max_energy"], 2),
      regions(m4["mesh"]))
# --8<-- [end:remesh]

# --8<-- [start:resolve]
# Step 2 the other way: split the overlap between a and b, each cell to the nearer one. The
# split leaves slivers along the new interface, which a remeshing removes.
resolved, report = wm.simwild(
    {"operation": "resolve_overlaps", "input": ["m"], "resolve_overlaps_tags": [["a", "b"]]},
    {"m": m1["mesh"]},
)
print("2'. resolve_overlaps:", regions(resolved["mesh"]),
      "max energy", round(report["max_energy"], 2))
resolved, report = wm.simwild(
    {"input": ["m"], "eps_rel": 0.01, "num_threads": 4},
    {"m": resolved["mesh"]},
)
print("    remeshed: max energy", round(report["max_energy"], 2), regions(resolved["mesh"]))
# --8<-- [end:resolve]
