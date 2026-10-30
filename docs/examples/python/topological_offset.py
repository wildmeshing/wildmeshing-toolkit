"""The topological offset tutorial (docs/tutorials/topological_offset.md): a band at a distance
around a body, inward and outward, around an expression of tags, and optimized."""

import os
from pathlib import Path

import numpy as np
import wildmeshing as wm

# The example meshes: $WMTK_DATA, or data/models from the repository root.
DATA = Path(os.environ.get("WMTK_DATA", "data/models"))


def count(mesh, tag):
    """The number of tets that carry `tag`."""
    t = mesh["cells"]["tetrahedron"]["attributes"]["tags"]
    if tag not in t["labels"]:
        return 0
    return int(np.asarray(t["values"])[:, t["labels"].index(tag)].sum())


# --8<-- [start:file]
wm.topological_offset({
    "input": str(DATA / "127891.msh"),
    "output": "127891_offset",
    "offset_selection": "127891",
    "offset_output_tags": ["offset"],
    "target_distance": 0.4,
    "optimize_offset": False,
})
# --8<-- [end:file]

# --8<-- [start:memory]
bar = wm.read_mesh(str(DATA / "127891.msh"))
outputs, report = wm.topological_offset(
    {"input": "bar", "offset_selection": "127891", "offset_output_tags": ["offset"],
     "target_distance": 0.4, "optimize_offset": False},
    {"bar": bar},
)
offset = outputs["mesh"]
print(report["before #t"], "->", report["after #t"], "tets,", count(offset, "offset"), "in the band")
# --8<-- [end:memory]

# --8<-- [start:sides]
# Outward (the default), inward, and both: the band is the cells tagged "offset".
for side in [{"offset_out": True}, {"offset_in": True, "offset_out": False},
             {"offset_in": True, "offset_out": True}]:
    out, _ = wm.topological_offset(
        {"input": "bar", "offset_selection": "127891", "offset_output_tags": ["offset"],
         "target_distance": 0.4, "optimize_offset": False, **side},
        {"bar": bar})
    print(side, count(out["mesh"], "offset"), "band tets")
# --8<-- [end:sides]

# --8<-- [start:expressions]
# Two overlapping spheres, tag_0 and tag_1. An expression selects the complex to offset.
spheres = wm.read_mesh(str(DATA / "double_sphere.msh"))
for selection in ["tag_0 & tag_1",             # the overlap, a lens
                  "ambient & tag_0 & tag_1"]:  # the circle where the two surfaces cross
    out, _ = wm.topological_offset(
        {"input": "spheres", "offset_selection": selection, "offset_output_tags": ["offset"],
         "target_distance": 0.05, "optimize_offset": False},
        {"spheres": spheres})
    print(f"{selection!r}: {count(out['mesh'], 'offset')} band tets")
# --8<-- [end:expressions]

# --8<-- [start:protected]
# The band around tag_0 runs through tag_1. Protected, tag_1 stays on the band's cells.
for protected in [[], ["tag_1"]]:
    out, _ = wm.topological_offset(
        {"input": "spheres", "offset_selection": "tag_0", "offset_output_tags": ["offset"],
         "protected_tags": protected, "target_distance": 0.05, "optimize_offset": False},
        {"spheres": spheres})
    m = out["mesh"]
    print(f"protected {protected}: offset {count(m, 'offset')}, tag_1 {count(m, 'tag_1')} tets")
# --8<-- [end:protected]

# --8<-- [start:optimize]
# The optimization, cut short: five turns, a bar of 20% of the target, four threads.
outputs, report = wm.topological_offset(
    {"input": "bar", "offset_selection": "127891", "offset_output_tags": ["offset"],
     "target_distance": 0.4, "front_conv_rel": 0.2, "max_rounds": 5, "num_threads": 4},
    {"bar": bar},
)
metrics = report["optimization_metrics"]
print("converged:", report["converged"], "|", report["after #t"], "tets |",
      f"front distance error: avg {metrics['avg_l2_dist_err'][-1]:.3g},"
      f" max {metrics['max_l2_dist_err'][-1]:.3g}")
# --8<-- [end:optimize]
