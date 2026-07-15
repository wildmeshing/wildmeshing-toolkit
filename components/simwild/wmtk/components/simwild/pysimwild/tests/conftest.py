"""Fixtures: synthetic multi-tag meshes and PolyFEM discovery.

Meshes are built directly with gmsh (Freudenthal/Kuhn tet grids, conforming
across cells) so tests are deterministic and need no external data.
"""
import os
from itertools import permutations
from pathlib import Path

import gmsh
import numpy as np
import pytest

# mesh_core delegates tag expressions to the wildmeshing bindings; when they
# are not pip-installed, fall back to the toolkit build tree.
try:
    import wildmeshing  # noqa: F401
except ImportError:
    import sys
    for up in Path(__file__).resolve().parents:
        cand = up / "build" / "bin"
        if cand.is_dir() and list(cand.glob("wildmeshing*.so")):
            sys.path.insert(0, str(cand))
            break


# --------------------------------------------------------------------------
# PolyFEM discovery
# --------------------------------------------------------------------------

POLYFEM_BIN = os.environ.get("POLYFEM_BIN")
if POLYFEM_BIN and not Path(POLYFEM_BIN).is_file():
    POLYFEM_BIN = None
needs_polyfem = pytest.mark.skipif(
    POLYFEM_BIN is None,
    reason="export POLYFEM_BIN=/path/to/PolyFEM_bin to run the polyfem tests")


# --------------------------------------------------------------------------
# Mesh writers
# --------------------------------------------------------------------------

def _write_groups_msh(path, dim, coords, groups):
    """Write a physical-groups .msh: coords (n,3) 1-based-node-tag order,
    groups = {name: [prim vertex tuples]} written in insertion order
    (first group owns the node block, matching wmtk's convention)."""
    gmsh.initialize()
    try:
        gmsh.model.add("synthetic")
        node_tags = list(range(1, len(coords) + 1))
        flat = [float(x) for c in coords for x in c]
        etype = 4 if dim == 3 else 2  # Tet or Triangle
        eid = 1
        first = True
        for name, prims in groups.items():
            ent = gmsh.model.addDiscreteEntity(dim)
            if first:
                gmsh.model.mesh.addNodes(dim, ent, node_tags, flat)
                first = False
            if prims:
                gmsh.model.mesh.addElementsByType(
                    ent, etype, list(range(eid, eid + len(prims))),
                    [v for p in prims for v in p])
                eid += len(prims)
            pg = gmsh.model.addPhysicalGroup(dim, [ent])
            gmsh.model.setPhysicalName(dim, pg, name)
        gmsh.option.setNumber("Mesh.MshFileVersion", 4.1)
        gmsh.write(str(path))
    finally:
        gmsh.finalize()


def _tet_grid(nx, ny, nz):
    """Freudenthal 6-tet-per-cube grid. Returns (coords, cube_tets) where
    cube_tets[(i,j,k)] = list of 4-tuples of 1-based node tags. Conforming
    across cubes (consistent main diagonal)."""
    def nid(i, j, k):
        return 1 + i + j * (nx + 1) + k * (nx + 1) * (ny + 1)

    coords = [(i, j, k)
              for k in range(nz + 1) for j in range(ny + 1) for i in range(nx + 1)]
    # coords list must be in node-tag order: tag = nid(i,j,k) -> index tag-1
    ordered = [None] * ((nx + 1) * (ny + 1) * (nz + 1))
    for k in range(nz + 1):
        for j in range(ny + 1):
            for i in range(nx + 1):
                ordered[nid(i, j, k) - 1] = (float(i), float(j), float(k))

    axes = {"x": (1, 0, 0), "y": (0, 1, 0), "z": (0, 0, 1)}
    cube_tets = {}
    for k in range(nz):
        for j in range(ny):
            for i in range(nx):
                tets = []
                for perm in permutations("xyz"):
                    p = np.array([i, j, k])
                    verts = [nid(*p)]
                    for ax in perm:
                        p = p + np.array(axes[ax])
                        verts.append(nid(*p))
                    # signed volume = permutation parity; flip odd perms so
                    # every tet is positively oriented (polyfem requires it)
                    if perm in (("x", "z", "y"), ("y", "x", "z"), ("z", "y", "x")):
                        verts[2], verts[3] = verts[3], verts[2]
                    tets.append(tuple(verts))
                cube_tets[(i, j, k)] = tets
    return ordered, cube_tets


def make_two_boxes_3d(path):
    """7x4x4 unit-cube tet grid: tag_0 = box x in [1,2], tag_1 = box x in
    [3,4] (both y,z in [1,3]), ambient elsewhere. The bodies are fully
    interior, separated by a 1-unit ambient gap along x."""
    coords, cube_tets = _tet_grid(7, 4, 4)

    def region(i, j, k):
        if 1 <= j < 3 and 1 <= k < 3:
            if i == 1:
                return "tag_0"
            if i == 3:
                return "tag_1"
        return "ambient"

    groups = {"ambient": [], "tag_0": [], "tag_1": []}
    for key, tets in cube_tets.items():
        groups[region(*key)].extend(tets)
    _write_groups_msh(path, 3, coords, groups)
    return path


def make_jagged_2d(path, nx=16, ny=8):
    """2D triangle grid with a staircase tag_0/ambient interface: tag_0 =
    squares with i < 4 + (j % 2). The interface zigzags between x=4 and
    x=5 — high turning angle, ideal smoothing target."""
    def nid(i, j):
        return 1 + i + j * (nx + 1)

    coords = []
    for j in range(ny + 1):
        for i in range(nx + 1):
            coords.append((float(i), float(j), 0.0))

    groups = {"ambient": [], "tag_0": []}
    for j in range(ny):
        for i in range(nx):
            a, b = nid(i, j), nid(i + 1, j)
            c, d = nid(i, j + 1), nid(i + 1, j + 1)
            tris = [(a, b, d), (a, d, c)]
            name = "tag_0" if i < 4 + (j % 2) else "ambient"
            groups[name].extend(tris)
    _write_groups_msh(path, 2, coords, groups)
    return path


@pytest.fixture()
def boxes3d(tmp_path):
    return make_two_boxes_3d(tmp_path / "boxes3d.msh")


@pytest.fixture()
def jagged2d(tmp_path):
    return make_jagged_2d(tmp_path / "jagged2d.msh")
