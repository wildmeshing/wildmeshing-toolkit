"""The SimWild tutorial (docs/tutorials/simwild.md): mesh two overlapping spheres into one tet mesh
with a region each, and five overlapping closed curves into one triangle mesh, on files and in
memory; then the options the page measures, and the envelope a run returns."""

import os
from pathlib import Path

import wildmeshing as wm

# The example meshes: $WMTK_DATA, or data/models from the repository root.
DATA = Path(os.environ.get("WMTK_DATA", "data/models"))


# --8<-- [start:regions]
def regions(mesh):
    """How many cells each combination of regions has: "a & b" counts the cells in a and b and
    in no other region, "ambient" the cells in none."""
    top = "tetrahedron" if "tetrahedron" in mesh["cells"] else "triangle"
    tags = mesh["cells"][top]["attributes"]["tags"]
    counts = {}
    for row in tags["values"]:
        key = " & ".join(label for label, member in zip(tags["labels"], row) if member)
        counts[key] = counts.get(key, 0) + 1
    return dict(sorted(counts.items()))
# --8<-- [end:regions]


# --8<-- [start:memory]
# Two copies of one sphere, the second moved 0.3 along x by a 4x4 matrix ([] is the identity).
SHIFT = [[1, 0, 0, 0.3], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
sphere = wm.read_mesh(str(DATA / "sphere.obj"))
outputs, report = wm.simwild(
    {"input": ["sphere", "sphere"], "input_names": ["a", "b"], "input_transform": [[], SHIFT],
     "eps_rel": 0.01, "length_rel": 0.05, "num_threads": 4},
    {"sphere": sphere},
)
spheres = outputs["mesh"]
print(report["#t"], "tets, max energy", round(report["max_energy"], 2))
print(regions(spheres))
# --8<-- [end:memory]

# --8<-- [start:file]
wm.simwild({
    "input": [str(DATA / "sphere.obj"), str(DATA / "sphere.obj")],
    "input_names": ["a", "b"],
    "input_transform": [[], SHIFT],
    "output": "two_spheres",
    "eps_rel": 0.01,
    "length_rel": 0.05,
    "num_threads": 4,
    "report": "two_spheres_report.json",
})
# --8<-- [end:file]

# --8<-- [start:curves]
# Five closed curves; the ellipse lies inside the rectangle, the others overlap in pairs.
NAMES = ["blob1", "blob2", "circle", "ellipse", "rectangle"]
curves = {name: wm.read_mesh(str(DATA / f"triwild_{name}.obj")) for name in NAMES}
outputs, report = wm.simwild(
    {"input": NAMES, "input_names": NAMES, "length_rel": 0.02, "num_threads": 4},
    curves,
)
shapes = outputs["mesh"]
print(report["#t"], "triangles, max energy", round(report["max_energy"], 2))
print(regions(shapes))
# --8<-- [end:curves]


def mesh_spheres(**change):
    """The first run, with some options changed."""
    options = {"input": ["sphere", "sphere"], "input_names": ["a", "b"],
               "input_transform": [[], SHIFT], "eps_rel": 0.01, "length_rel": 0.05,
               "num_threads": 4, **change}
    return wm.simwild(options, {"sphere": sphere})


def mesh_shapes(**change):
    """The 2D run, with some options changed."""
    options = {"input": NAMES, "input_names": NAMES, "length_rel": 0.02, "num_threads": 4,
               **change}
    return wm.simwild(options, curves)


# --8<-- [start:sweep]
for change in [{"length_rel": 0.1}, {}, {"length_rel": 0.025}, {"eps_rel": 0.002}]:
    outputs, report = mesh_spheres(**change)
    surface = outputs["surface"]["cells"]["triangle"]["connectivity"]
    print(f"{change}: {report['#t']} tets, {len(surface)} interface triangles, "
          f"eps {report['eps']:.4f}, max energy {report['max_energy']:.2f}")
# --8<-- [end:sweep]

# --8<-- [start:sizing]
# A coarser background, then a finer overlap.
for sizing in [[{"tags": "_", "length_rel": 0.1}], [{"tags": "a & b", "length_rel": 0.025}]]:
    outputs, report = mesh_spheres(sizing_field=sizing)
    print(sizing, report["#t"], "tets:", regions(outputs["mesh"]))
# --8<-- [end:sizing]

# --8<-- [start:quality]
# A stricter target inside the curves (every region: !_) than in the background.
for quality in [[], [{"tags": "!_", "quality": 4}]]:
    outputs, report = mesh_shapes(quality_field=quality)
    triangles = outputs["mesh"]["cells"]["triangle"]["attributes"]
    energy = triangles["quality"]["values"][:, 0]
    inside = triangles["tags"]["values"][:, triangles["tags"]["labels"].index("ambient")] == 0
    print(quality, report["#t"], "triangles, max energy inside the curves",
          round(energy[inside].max(), 2), "outside", round(energy[~inside].max(), 2))
# --8<-- [end:quality]

# --8<-- [start:envelope]
# The spheres again, saved with their envelope; then, from the checkpoint, a finer remeshing
# held to the same envelope.
outputs, report = mesh_spheres()
wm.save(outputs, "checkpoints/spheres")  # spheres.msh and spheres_envelope.msh
saved = wm.load("checkpoints/spheres")  # {"mesh": ..., "envelope": ...}
remeshed, report = wm.simwild(
    {"input": ["spheres"], "envelope": "envelope", "eps_rel": 0.01, "length_rel": 0.04,
     "num_threads": 4},
    {"spheres": saved["mesh"], "envelope": saved["envelope"]},
)
print(report["#t"], "tets after remeshing:", regions(remeshed["mesh"]))
# --8<-- [end:envelope]
