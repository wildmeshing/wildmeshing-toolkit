"""Byte parity between the Python engine and the C++ port of it.

The polyfem operations' glue — reading the tagged .msh, picking interface faces from region/filter
selections, orienting them, building the constraint matrices and writing the artifacts — was
written in `simwild.polyfem_ops` (the oracle) and ported to the wmtk component `polyfem_ops`
reached through `wildmeshing.wildmeshing(...)`. These tests hold the C++ engine to the Python
engine's results on the same mesh with the same options: byte for byte for the two text
artifacts, and dataset for dataset (same names, same dtypes, same shapes, exactly equal values)
for the HDF5s. That is the only check that catches a face written in a different order, a normal
flipped the other way, a float printed with one digit fewer, or a sum accumulated in another
order.

The Python engine's results were RECORDED (the goldens, see the next section) before the Python
engine was removed, so the C++ one is still checked against it.

The C++ engine writes its generated polyfem inputs under `<dirname(output)>/sep_input` (minimum
separation) or `<dirname(output)>/smooth_input` (smoothing) and returns without solving when
`inputs_only` is set; the Python side's artifacts were recorded from `make_interface_constraint`
and, for the protected-region pins, from the same calls `minimum_separation.run` made.

The values in these files are float64 all the way down, so "exactly equal" means every last bit:
the mass and stiffness matrices, the two normalizations (`np.sum` is a pairwise summation, not a
running total) and the Laplacian right-hand side (scipy accumulates a sparse-times-dense product
with a fused multiply-add) are all reproduced operation for operation on the C++ side.

The last section of this file goes further and runs the solver: the two outer loops, the deformed
mesh they write back, and the "already separated" probe. There the two engines can only be compared
to a tolerance, because polyfem itself is not reproducible bit for bit; that section's header says
what is asserted exactly, what is not, and the measurements behind both.
"""
import json
import math
import os
from pathlib import Path

import gmsh
import h5py
import numpy as np
import pytest

# The irrational-coordinate meshes below are the conftest fixtures with bent coordinates, built
# from the same two helpers; pytest puts this directory on sys.path, so conftest imports directly.
from conftest import _tet_grid, _write_groups_msh, needs_polyfem_ops, run_cpp

from simwild.polyfem_ops.polyfem_utils import get_mesh_info


# --------------------------------------------------------------------------
# The goldens: the Python engine's results, recorded
# --------------------------------------------------------------------------
#
# One directory per case under $POLYFEM_OPS_GOLDENS, holding exactly what that case compares plus
# the input mesh it was computed from. They live outside the repository, which keeps no binary
# test data, so only a machine that has them runs these comparisons; without the variable every
# test that needs them skips. They were recorded by this file's recording mode at f1f2f3953c, which
# ran the Python engine into each case's directory; re-recording needs a checkout of that commit
# (or of 8894367b56, the last one with the Python engine). metadata.json beside the case
# directories says what recorded them.

GOLDENS = os.environ.get("POLYFEM_OPS_GOLDENS")
needs_goldens = pytest.mark.skipif(
    not GOLDENS,
    reason="export POLYFEM_OPS_GOLDENS=/path/to/goldens to compare against the Python engine's "
           "recorded results")


def _golden(case, mesh):
    """The golden directory of `case`. The mesh this run built must be the one the goldens were
    computed from, or no comparison below would mean anything."""
    golden = Path(GOLDENS) / case
    assert (golden / "input.msh").is_file(), f"{case} has no recorded result in {GOLDENS}"
    recorded, built = _gmsh_reduced_mesh(golden / "input.msh"), _gmsh_reduced_mesh(mesh)
    _assert_same_mesh_structure(recorded, built, "recorded and current input meshes",
                                sides=("the recording", "this run"))
    assert np.array_equal(recorded["coords"], built["coords"]), (
        f"{case}: this run's input mesh is not the one the goldens were recorded on")
    return golden


BOTH_SKINS = [[{"region": "tag_0", "filter": "ambient"},
               {"region": "tag_1", "filter": "ambient"}]]
UNION_WITH_ITSELF = [[{"region": "tag_0 | tag_1", "filter": "ambient"},
                      {"region": "tag_0 | tag_1", "filter": "ambient"}]]

CONSTRAINT_FILES = ["interface_constraint.hdf5", "interface_constraint_laplacian.hdf5"]
LINEAR_MAP = "interface_linear_map.hdf5"

# Where each engine puts the generated polyfem inputs, and what the simulation JSON is called
# there. Both come from the operation's run(), not from a parameter.
SIM_DIRS = {"minimum_separation": ("sep_input", "sep_output", "separation.json"),
            "laplacian_smoothing": ("smooth_input", "smooth_output", "smoothing.json")}


# --------------------------------------------------------------------------
# Irrational-coordinate meshes
# --------------------------------------------------------------------------

def _bend(value, seed):
    """Scale by sqrt(2) and offset by a deterministic fraction: no coordinate is then exactly
    representable, every triangle is scalene, and no edge length, area or cotangent is exact. The
    offset stays well below the grid spacing, so the cells keep their shape and every orientation
    sign keeps its sense."""
    return value * math.sqrt(2.0) + 0.05 * math.sin(seed)


def make_two_boxes_3d_irrational(path):
    """make_two_boxes_3d with bent coordinates. The tags follow the cube indices, so the regions
    are the same ones."""
    coords, cube_tets = _tet_grid(7, 4, 4)
    coords = [tuple(_bend(c, 3 * i + 7 * k) for k, c in enumerate(p))
              for i, p in enumerate(coords)]

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


def make_jagged_2d_irrational(path, nx=16, ny=8):
    """make_jagged_2d with bent coordinates; the staircase interface is unchanged."""
    def nid(i, j):
        return 1 + i + j * (nx + 1)

    coords = []
    for j in range(ny + 1):
        for i in range(nx + 1):
            seed = 3 * len(coords)
            coords.append((_bend(float(i), seed), _bend(float(j), seed + 1), 0.0))

    groups = {"ambient": [], "tag_0": []}
    for j in range(ny):
        for i in range(nx):
            a, b = nid(i, j), nid(i + 1, j)
            c, d = nid(i, j + 1), nid(i + 1, j + 1)
            name = "tag_0" if i < 4 + (j % 2) else "ambient"
            groups[name].extend([(a, b, d), (a, d, c)])
    _write_groups_msh(path, 2, coords, groups)
    return path


@pytest.fixture()
def boxes3d_irrational(tmp_path):
    return make_two_boxes_3d_irrational(tmp_path / "boxes3d_irrational.msh")


@pytest.fixture()
def jagged2d_irrational(tmp_path):
    return make_jagged_2d_irrational(tmp_path / "jagged2d_irrational.msh")


# --------------------------------------------------------------------------
# Comparisons
# --------------------------------------------------------------------------

def _assert_identical(py_dir, cpp_dir, name):
    py_file = py_dir / name
    cpp_file = cpp_dir / name
    assert py_file.is_file(), f"the Python engine did not write {name}"
    assert cpp_file.is_file(), f"the C++ engine did not write {name}"
    py_lines = py_file.read_bytes().splitlines()
    cpp_lines = cpp_file.read_bytes().splitlines()
    for i, (a, b) in enumerate(zip(py_lines, cpp_lines), start=1):
        assert a == b, f"{name}: first difference on line {i}\n  python: {a!r}\n  c++   : {b!r}"
    assert len(py_lines) == len(cpp_lines), (
        f"{name}: {len(py_lines)} lines from python, {len(cpp_lines)} from c++")
    assert py_file.read_bytes() == cpp_file.read_bytes()


def _h5_objects(path):
    """Every dataset and every attribute in the file, keyed by kind and path. h5py hands back the
    on-disk dtype, which is part of the contract: polyfem reads local2global as vector<int>, so
    an int64 there would be silently misread."""
    objects = {}
    with h5py.File(path, "r") as f:
        def visit(name, obj):
            if isinstance(obj, h5py.Dataset):
                objects[f"dataset {name}"] = np.asarray(obj[()])
            for key, value in obj.attrs.items():
                objects[f"attribute {name}.{key}"] = np.asarray(value)
        f.visititems(visit)
        for key, value in f.attrs.items():
            objects[f"attribute /.{key}"] = np.asarray(value)
    return objects


def _assert_hdf5_identical(py_dir, cpp_dir, name):
    py_file = py_dir / name
    cpp_file = cpp_dir / name
    assert py_file.is_file(), f"the Python engine did not write {name}"
    assert cpp_file.is_file(), f"the C++ engine did not write {name}"
    py_objects = _h5_objects(py_file)
    cpp_objects = _h5_objects(cpp_file)
    assert sorted(py_objects) == sorted(cpp_objects), (
        f"{name}: python has {sorted(py_objects)}, c++ has {sorted(cpp_objects)}")
    for key in sorted(py_objects):
        a, b = py_objects[key], cpp_objects[key]
        assert a.dtype == b.dtype, f"{name}/{key}: dtype {a.dtype} from python, {b.dtype} from c++"
        assert a.shape == b.shape, f"{name}/{key}: shape {a.shape} from python, {b.shape} from c++"
        if np.array_equal(a, b):
            continue
        differing = np.nonzero(a != b)
        first = tuple(int(i[0]) for i in differing)
        detail = f"{name}/{key}: {len(differing[0])} of {a.size} values differ; first at {first}: "
        detail += f"python {a[first]!r} vs c++ {b[first]!r}"
        if np.issubdtype(a.dtype, np.floating):
            scale = np.where(np.abs(a) > 0, np.abs(a), 1.0)
            detail += f"; max relative difference {np.max(np.abs(a - b) / scale):.3e}"
        raise AssertionError(detail)


# --------------------------------------------------------------------------
# minimum separation
# --------------------------------------------------------------------------

@needs_polyfem_ops
@needs_goldens
@pytest.mark.parametrize("pairs, options, label", [
    (BOTH_SKINS, {}, "two_skins"),
    (UNION_WITH_ITSELF, {}, "union_with_itself"),
    (BOTH_SKINS, {"normalize_penalties": False}, "unnormalized"),
    (BOTH_SKINS, {"use_graph_laplacian": True}, "graph_laplacian"),
])
def test_minimum_separation_inputs_match(boxes3d, tmp_path, pairs, options, label):
    # Two explicit sides give two collision bodies (ids 1 and 2); the union region paired with
    # itself dedupes to one selection and one body (id 1), so both files must still agree.
    # `normalize_penalties` off drops both normalizations, and `use_graph_laplacian` on replaces
    # the igl mass and cotangent matrices with the identity and unit edge weights.
    golden = _golden(f"minimum_separation_inputs/{label}", boxes3d)
    cpp_root = tmp_path / f"cpp_{label}"
    run_cpp(boxes3d, "minimum_separation", cpp_root, collision_pairs=pairs, sep=1.5, **options)

    cpp_dir = cpp_root / "sep_input"
    _assert_identical(golden, cpp_dir, "interface_collision.obj")
    _assert_identical(golden, cpp_dir, "collision_body_ids.txt")
    for name in CONSTRAINT_FILES + [LINEAR_MAP]:
        _assert_hdf5_identical(golden, cpp_dir, name)


@needs_polyfem_ops
@needs_goldens
@pytest.mark.parametrize("protected, expected, label", [
    (["tag_1"], ["protected_pin.hdf5"], "bare_expression"),
    ([{"region": "tag_1", "axes": "z"}], ["protected_pin_z.hdf5"], "one_axis"),
    ([{"region": "tag_1", "axes": "xy"}, "tag_0"],
     ["protected_pin_xy.hdf5", "protected_pin.hdf5"], "two_groups"),
])
def test_minimum_separation_protected_pins_match(boxes3d, tmp_path, protected, expected, label):
    # A bare expression pins every component (b has `dim` columns); an `axes` subset pins
    # flattened degrees of freedom (b has one column). Entries with different axes land in
    # different files, which is why the third case produces two.
    golden = _golden(f"minimum_separation_protected_pins/{label}", boxes3d)
    cpp_root = tmp_path / f"cpp_{label}"
    run_cpp(boxes3d, "minimum_separation", cpp_root,
            collision_pairs=BOTH_SKINS, sep=1.5, protected_regions=protected)

    cpp_dir = cpp_root / "sep_input"
    for name in expected:
        _assert_hdf5_identical(golden, cpp_dir, name)


# --------------------------------------------------------------------------
# laplacian smoothing
# --------------------------------------------------------------------------

@needs_polyfem_ops
@needs_goldens
@pytest.mark.parametrize("mesh_fixture, interfaces, options, label", [
    ("jagged2d", [{"region": "tag_0", "filter": "ambient"}], {}, "2d_selected"),
    ("jagged2d", [{"region": "tag_0", "filter": "ambient"}],
     {"smooth_positions": False}, "2d_displacements"),
    ("jagged2d", [], {}, "2d_auto"),
    ("boxes3d", [], {}, "3d_auto"),
])
def test_laplacian_smoothing_inputs_match(request, tmp_path, mesh_fixture, interfaces,
                                          options, label):
    # An empty `interfaces` list is the legacy auto-detect: every material interface plus the
    # one-sided domain skins, oriented from the high physical tag outwards. The 2D cases also
    # exercise the edge-loop pass, which the 3D path does not have. Positions mode (the default)
    # puts -L (scale * rest coordinates) in `b`; displacement mode leaves it zero.
    mesh = request.getfixturevalue(mesh_fixture)
    golden = _golden(f"laplacian_smoothing_inputs/{label}", mesh)
    cpp_root = tmp_path / f"cpp_{label}"
    run_cpp(mesh, "laplacian_smoothing", cpp_root, interfaces=interfaces, **options)

    cpp_dir = cpp_root / "smooth_input"
    _assert_identical(golden, cpp_dir, "interface_collision.obj")
    for name in CONSTRAINT_FILES:
        _assert_hdf5_identical(golden, cpp_dir, name)
    # Smoothing mode writes no collision artifacts: polyfem does not read them there.
    for name in ("collision_body_ids.txt", LINEAR_MAP):
        assert not (golden / name).exists()
        assert not (cpp_dir / name).exists()


# --------------------------------------------------------------------------
# irrational coordinates
# --------------------------------------------------------------------------

@needs_polyfem_ops
@needs_goldens
def test_minimum_separation_irrational_coordinates_match(boxes3d_irrational, tmp_path):
    # The grid fixtures have integer coordinates, where every edge length, area and cotangent is
    # exact and a rounding difference cannot show. This one is where the two libigl builds are
    # actually compared: the C++ links libigl 2.6.0 and the Python wheel is 2.6.3, built with
    # different flags, and igl.massmatrix / igl.cotmatrix decide every value in both files.
    golden = _golden("minimum_separation_inputs/irrational_3d", boxes3d_irrational)
    cpp_root = tmp_path / "cpp_irrational_3d"
    run_cpp(boxes3d_irrational, "minimum_separation", cpp_root,
            collision_pairs=BOTH_SKINS, sep=1.5)

    cpp_dir = cpp_root / "sep_input"
    _assert_identical(golden, cpp_dir, "interface_collision.obj")
    for name in CONSTRAINT_FILES + [LINEAR_MAP]:
        _assert_hdf5_identical(golden, cpp_dir, name)


@needs_polyfem_ops
@needs_goldens
def test_laplacian_smoothing_irrational_coordinates_match(jagged2d_irrational, tmp_path):
    # 2D has no igl in it: the mass is half an edge length per endpoint and the stiffness weight
    # is its reciprocal, both from np.linalg.norm, and `b` is the sparse product. Irrational
    # coordinates are what make those roundings visible.
    golden = _golden("laplacian_smoothing_inputs/irrational_2d", jagged2d_irrational)
    cpp_root = tmp_path / "cpp_irrational_2d"
    run_cpp(jagged2d_irrational, "laplacian_smoothing", cpp_root, interfaces=[])

    cpp_dir = cpp_root / "smooth_input"
    _assert_identical(golden, cpp_dir, "interface_collision.obj")
    for name in CONSTRAINT_FILES:
        _assert_hdf5_identical(golden, cpp_dir, name)


# --------------------------------------------------------------------------
# Comparing the reduced mesh and the simulation JSON
# --------------------------------------------------------------------------

def _gmsh_reduced_mesh(path):
    """A reduced .msh as gmsh sees it: the nodes in file order, and per physical group its tag,
    its name and its elements (id and vertex tuple) in file order.

    The two engines write the file with different libraries -- gmsh on the Python side, mshio on
    the C++ one -- so the bytes cannot match and this is what "the same mesh" means instead. The
    node ORDER is part of it: it is what keeps polyfem's `in_node_to_node` the identity, so the
    collision artifacts index the original and the reduced mesh alike.
    """
    gmsh.initialize()
    try:
        gmsh.open(str(path))
        node_tags, coord_flat, _ = gmsh.model.mesh.getNodes()
        dim = 3 if len(gmsh.model.getPhysicalGroups(dim=3)) > 0 else 2
        elem_type = 4 if dim == 3 else 2   # gmsh: 4 = Tet, 2 = Triangle
        npp = 4 if dim == 3 else 3
        groups = []
        for d, ptag in gmsh.model.getPhysicalGroups(dim=dim):
            elements = []
            for ent in gmsh.model.getEntitiesForPhysicalGroup(d, ptag):
                etags, ntags = gmsh.model.mesh.getElementsByType(elem_type, ent)
                conn = np.array(ntags, dtype=np.int64).reshape(-1, npp)
                elements += [(int(e), tuple(int(v) for v in row))
                             for e, row in zip(etags, conn)]
            groups.append((int(ptag), gmsh.model.getPhysicalName(d, ptag), elements))
        return dict(dim=dim,
                    node_tags=[int(t) for t in node_tags],
                    coords=np.array(coord_flat, dtype=np.float64).reshape(-1, 3),
                    groups=groups)
    finally:
        gmsh.finalize()


def _assert_same_mesh_structure(py, cpp, what, sides=("python", "c++")):
    """Everything about two .msh files that does not depend on a coordinate: the node tags in file
    order, the physical groups, and each group's elements with their ids and vertex tuples.

    Both the reduced meshes and the deformed ones are held to this exactly; only their coordinates
    are compared differently, which is why that comparison stays with each caller. `what` names the
    pair in the failure messages ("reduced meshes", "deformed meshes"), and `sides` names the two
    meshes in them.
    """
    a_name, b_name = sides
    assert py["node_tags"] == cpp["node_tags"], f"the {what} list their nodes differently"
    assert [(t, n) for t, n, _ in py["groups"]] == [(t, n) for t, n, _ in cpp["groups"]], (
        f"the {what}' physical groups differ")
    for (_, name, py_elems), (_, _, cpp_elems) in zip(py["groups"], cpp["groups"]):
        assert len(py_elems) == len(cpp_elems), (
            f"group {name}: {len(py_elems)} elements from {a_name}, {len(cpp_elems)} from {b_name}")
        for i, (a, b) in enumerate(zip(py_elems, cpp_elems)):
            assert a == b, f"group {name}: element {i} is {a} from {a_name} and {b} from {b_name}"


def _assert_reduced_msh_identical(py_msh, cpp_msh):
    assert py_msh.is_file(), "the Python engine did not write the reduced mesh"
    assert cpp_msh.is_file(), "the C++ engine did not write the reduced mesh"
    py, cpp = _gmsh_reduced_mesh(py_msh), _gmsh_reduced_mesh(cpp_msh)
    assert py["dim"] == cpp["dim"]
    _assert_same_mesh_structure(py, cpp, "reduced meshes")
    assert np.array_equal(py["coords"], cpp["coords"]), (
        "the reduced meshes' coordinates differ; max relative difference "
        f"{np.max(np.abs(py['coords'] - cpp['coords']) / np.where(py['coords'] != 0, np.abs(py['coords']), 1.0)):.3e}")


def _normalise_paths(value, root):
    """Replace the run's own output directory with a fixed marker. The simulation JSON carries
    absolute paths into that directory, and the two engines are given different ones so that
    neither can read the other's artifacts."""
    marker = "<out>"
    if isinstance(value, str):
        return value.replace(str(root), marker)
    if isinstance(value, dict):
        return {k: _normalise_paths(v, root) for k, v in value.items()}
    if isinstance(value, list):
        return [_normalise_paths(v, root) for v in value]
    return value


def _assert_json_identical(py_doc, cpp_doc, path="doc"):
    """Same keys in the same ORDER at every level, same types, and values equal to the last bit.

    Key order matters because the file is `json.dumps` of an insertion-ordered dict and polyfem
    reads it back as one; `repr` rather than `==` decides the floats, because `==` cannot tell
    0.0 from -0.0 and an integer 1 from a float 1.0.
    """
    assert type(py_doc) is type(cpp_doc), (
        f"{path}: python has {type(py_doc).__name__}, c++ has {type(cpp_doc).__name__} "
        f"({py_doc!r} vs {cpp_doc!r})")
    if isinstance(py_doc, dict):
        assert list(py_doc) == list(cpp_doc), (
            f"{path}: python keys {list(py_doc)}, c++ keys {list(cpp_doc)}")
        for key in py_doc:
            _assert_json_identical(py_doc[key], cpp_doc[key], f"{path}.{key}")
    elif isinstance(py_doc, list):
        assert len(py_doc) == len(cpp_doc), (
            f"{path}: {len(py_doc)} entries from python, {len(cpp_doc)} from c++")
        for i, (a, b) in enumerate(zip(py_doc, cpp_doc)):
            _assert_json_identical(a, b, f"{path}[{i}]")
    else:
        assert repr(py_doc) == repr(cpp_doc), (
            f"{path}: python {py_doc!r}, c++ {cpp_doc!r}")


def _assert_polyfem_inputs_match(mesh, operation, options, tmp_path, label):
    """Compare the C++ engine's reduced mesh and simulation JSON with the Python engine's, for the
    same mesh and options. Returns the two JSON texts with the output directory normalised away."""
    sim_in_name, _, json_name = SIM_DIRS[operation]
    reduced_name = Path(mesh).stem + "_polyfem.msh"
    # The recorded JSON is what the Python engine WROTE (json.dumps of its document), with its
    # run's own directory replaced by the same marker, since the C++ run writes elsewhere.
    golden = _golden(f"{operation}_polyfem_inputs/{label}", mesh)
    cpp_root = tmp_path / f"cpp_{label}"
    run_cpp(mesh, operation, cpp_root, **options)

    cpp_dir = cpp_root / sim_in_name
    py_norm = (golden / json_name).read_text()
    cpp_norm = _normalise_paths((cpp_dir / json_name).read_text(), cpp_root.resolve())
    _assert_json_identical(json.loads(py_norm), json.loads(cpp_norm))
    _assert_reduced_msh_identical(golden / reduced_name, cpp_dir / reduced_name)
    # The contract above is the parsed document; this is the stronger statement that happens to
    # hold on every case here, and it is asserted so that a formatting drift -- nlohmann's float
    # serializer against CPython's repr, or a change of indent -- is reported as itself rather
    # than passing unnoticed.
    assert py_norm == cpp_norm, f"{json_name} differs byte for byte although it parses equal"
    return py_norm, cpp_norm


# --------------------------------------------------------------------------
# minimum separation: the reduced mesh and separation.json
# --------------------------------------------------------------------------

@needs_polyfem_ops
@needs_goldens
@pytest.mark.parametrize("options, label", [
    ({}, "default"),
    # Changes the constraint matrices, not the JSON: the key the JSON reads
    # (`amips_normalize_by_volume`) is a different, engine-only one.
    ({"normalize_penalties": False}, "unnormalized"),
    ({"save_vtu": True}, "save_vtu"),
    # A body tag counted as ambient moves tag_0's cells into the ambient group of the reduced
    # mesh, which changes both element counts and both AMIPS weights.
    ({"ambient_like_tags": ["tag_0"]}, "ambient_like_body_tag"),
    # The three spellings _resolve_amips_weights accepts. Only "ambient" is special: every other
    # key is "the body's weight", whether it is spelled as a group name or as a tag number, and
    # the numeric spelling is NOT a tag lookup -- the reduced mesh's groups are named, and the
    # dict is replaced by {"ambient", "body"} before the JSON builder ever sees it.
    ({"amips_weights": {"ambient": 2.5}}, "amips_ambient"),
    ({"amips_weights": {"tag_0": 3.0}}, "amips_group_name"),
    ({"amips_weights": {"1": 5.0}}, "amips_numeric_tag"),
    ({"amips_weights": {"ambient": 2.0, "tag_1": 4.0}}, "amips_both"),
    # Every knob that reaches the JSON, all off their defaults at once: scale also rescales both
    # AMIPS weights (the volume normalization carries a scale^dim factor).
    ({"scale": 0.01, "alpha_n": 0.25, "alpha_t": 0.35, "barrier_stiffness": 5e5,
      "dhat_growth": 1.5, "rtol": 0.05, "max_iterations": 3, "nl_max_iterations": 250},
     "non_default_knobs"),
])
def test_minimum_separation_polyfem_inputs_match(boxes3d, tmp_path, options, label):
    _assert_polyfem_inputs_match(
        boxes3d, "minimum_separation", {"collision_pairs": BOTH_SKINS, "sep": 1.5, **options},
        tmp_path, label)


@needs_polyfem_ops
@needs_goldens
@pytest.mark.parametrize("options, label", [
    ({"use_nh_body": True}, "nh_defaults"),
    ({"use_nh_body": True, "nh_youngs": 2.5e-3, "nh_poisson": 0.3}, "nh_non_default"),
])
def test_minimum_separation_neohookean_materials(boxes3d, tmp_path, options, label):
    """`use_nh_body` makes EVERY group NeoHookean -- identically on the two engines, which
    `_assert_polyfem_inputs_match` decides, and with a modulus per group that carries that group's
    own volume normalization, which is what the rest of this test checks.

    Ambient is NeoHookean too, rather than staying AMIPS, because polyfem cannot solve a mixed
    list: `State::formulation()` accepts an array of differing materials only when every entry is
    one of `AssemblerUtils::elastic_materials()`, AMIPS is not one of them, and an AMIPS ambient
    beside a NeoHookean body aborts the solve with "multimaterial supported only for
    LinearElasticity and NeoHookean".
    """
    # The defaults the engine fills in: the root-level ones of the operation's spec.json.
    spec = (Path(__file__).resolve().parents[1]
            / "simwild" / "polyfem_ops" / "minimum_separation" / "spec.json")
    defaults = {r["pointer"][1:]: r["default"] for r in json.loads(spec.read_text())
                if r["pointer"].count("/") == 1 and "default" in r}
    scale = defaults["scale"]
    _, cpp_text = _assert_polyfem_inputs_match(
        boxes3d, "minimum_separation",
        {"collision_pairs": BOTH_SKINS, "sep": 1.5, **options}, tmp_path, label)

    # The C++ engine's reduced mesh, which the call above found identical to the Python one.
    sim_in_name = SIM_DIRS["minimum_separation"][0]
    reduced = next((tmp_path / f"cpp_{label}" / sim_in_name).glob("*_polyfem.msh"))
    _, mesh_dim, name_to_tag, _, tag_to_volume = get_mesh_info(str(reduced))
    materials = {m["id"]: m for m in json.loads(cpp_text)["materials"]}
    assert sorted(materials) == sorted(name_to_tag.values()), (
        f"materials {sorted(materials)}, reduced-mesh tags {sorted(name_to_tag.values())}")

    # Every modulus is in the same normalized currency as the AMIPS weights it replaces: divided
    # by that group's own rest volume in solver units (mesh units * scale), which is what the
    # default `amips_normalize_by_volume` asks for. The two groups have very different volumes --
    # 624 ambient tets against 48 body ones -- so this is also what catches a shared divisor.
    for group in ("ambient", "body"):
        tag = name_to_tag[group]
        volume = tag_to_volume[tag] * scale ** mesh_dim
        assert materials[tag] == {
            "id": tag, "type": "NeoHookean",
            "E": options.get("nh_youngs", defaults["nh_youngs"]) / volume,
            "nu": options.get("nh_poisson", defaults["nh_poisson"]),
            "rho": 1.0}, f"the {group} material"


@needs_polyfem_ops
@needs_goldens
@pytest.mark.parametrize("protected, label", [
    (["tag_1"], "bare_expression"),
    ([{"region": "tag_1", "axes": "z"}], "one_axis"),
    ([{"region": "tag_1", "axes": "xy"}, "tag_0"], "two_groups"),
])
def test_minimum_separation_hard_constraints_match(boxes3d, tmp_path, protected, label):
    # `protected_regions` is the one part of the JSON that run() adds after the shared builder:
    # one pin file per axes group, listed under `constraints.hard` in group order.
    _, cpp_text = _assert_polyfem_inputs_match(
        boxes3d, "minimum_separation",
        {"collision_pairs": BOTH_SKINS, "sep": 1.5, "protected_regions": protected},
        tmp_path, f"hard_{label}")
    assert '"hard"' in cpp_text


# --------------------------------------------------------------------------
# laplacian smoothing: the reduced mesh and smoothing.json
# --------------------------------------------------------------------------

@needs_polyfem_ops
@needs_goldens
@pytest.mark.parametrize("options, label", [
    ({}, "default"),
    ({"smooth_positions": False}, "displacements"),
    ({"weight_laplacian": 2.5}, "weight_laplacian"),
    ({"use_graph_laplacian": True}, "graph_laplacian"),
    ({"save_vtu": True}, "save_vtu"),
    # Smoothing forces contact off and aliases max_iterations onto the polyfem nonlinear cap,
    # which is the one place the two operations read the same key differently.
    ({"max_iterations": 42}, "max_iterations"),
])
def test_laplacian_smoothing_polyfem_inputs_match(jagged2d, tmp_path, options, label):
    _assert_polyfem_inputs_match(jagged2d, "laplacian_smoothing", options, tmp_path, label)


# --------------------------------------------------------------------------
# irrational coordinates: where the volume sums are actually compared
# --------------------------------------------------------------------------

@needs_polyfem_ops
@needs_goldens
def test_minimum_separation_polyfem_inputs_irrational(boxes3d_irrational, tmp_path):
    # On the integer grids every tet volume is exact and the running sum cannot round. Here each
    # term is an absolute `np.linalg.det` over three edge vectors -- which numpy evaluates as
    # sign * exp(sum log|U_ii|) of a pivoted LU, not by cofactors -- and the sum is accumulated
    # left to right over the elements in file order. That sum divides both AMIPS weights, so a
    # single last-bit difference anywhere in it shows up in the materials block.
    _assert_polyfem_inputs_match(
        boxes3d_irrational, "minimum_separation",
        {"collision_pairs": BOTH_SKINS, "sep": 1.5}, tmp_path, "irrational_3d")


@needs_polyfem_ops
@needs_goldens
def test_laplacian_smoothing_polyfem_inputs_irrational(jagged2d_irrational, tmp_path):
    # 2D areas are half an absolute cross product of two edge vectors: two rounded products and a
    # subtraction, never a fused multiply-add, then the running sum over the file order.
    _assert_polyfem_inputs_match(
        jagged2d_irrational, "laplacian_smoothing", {}, tmp_path, "irrational_2d")


# --------------------------------------------------------------------------
# Driving the solver: the outer loops, the write-back and the deformed mesh
# --------------------------------------------------------------------------
#
# Everything above stops at the generated polyfem inputs, which are byte-identical. Everything
# below runs polyfem, and every case below compares two runs: the C++ engine's, which reaches the
# polyfem linked into this process, and the recorded one of the Python engine, which launched
# $POLYFEM_BIN as a child process. The two must agree, and that is where "identical" stops being
# available: the contact assembly sums per-collision-pair contributions in an order that depends
# on thread scheduling, so two runs of ONE binary on ONE machine do not agree bit for bit.
#
# The two tolerance constants below are measured. Each of the four solver cases -- the dhat ramp,
# the stiffness loop, the protected-region case and the smoothing case -- was run five times with
# the Python engine and five times with the C++ in-process engine, and every pair of runs was
# compared on exactly what the tests below compare: each iteration's active distance and dhat, read
# out of that solve's log, and the deformed mesh's coordinates. The widest relative difference over
# all of those pairs:
#
#   Python against Python   6.6e-11   dhat case, coordinates (active distances 4.6e-11)
#   C++ against C++         7.6e-11   dhat case, active distances (coordinates 5.8e-11)
#   Python against C++      7.1e-11   dhat case, active distances (coordinates 6.5e-11)
#
# Ten times the widest of those, rounded up to a power of ten, is 1e-9 -- which is where the
# tolerance already was, so the measurement says it was right and does not support loosening it.
# It applies ONLY to quantities the solver's output feeds. (The
# smoothing case has no contact and therefore no contact assembly; its deformed meshes agree to
# 2.9e-15 across the two engines, and the already-separated probe case moves nothing at all.)
#
# What IS asserted exactly:
#   - the decision trail: every line the loop prints, in order, from the marker onwards. That is
#     which solve overshot and was rolled back, which committed, what the next dhat or barrier
#     stiffness was and why the loop stopped, with every number to the seven significant digits the
#     two engines print. The iteration count is the number of per-solve logs, also exact.
#   - the probe gap, and with it the first dhat: the probe solves at zero barrier stiffness, so
#     nothing moves and the reported gap is a pure geometry measurement -- 0.001 exactly, on both
#     engines, in every dhat-strategy case here.
#   - every dhat of the stiffness strategy, which pins it at sep*(1+rtol) and never reads a
#     measurement.
# What is NOT, and why: after the first commit the dhat ramp sets the next dhat to
# dhat_growth * active_distance, so from there on dhat carries the solver's noise. Measured on the
# protected-region case, iteration 1: 0.002792526595410909 from python against 0.002792526595377208
# from c++, 1.2e-11 relative (2.5e-13 on a repeat). Those are compared to 1e-9 like the distances
# they come from.
#
# What the measurement does not say is that the Python-against-C++ bound is comfortable. The solver
# stops at a relative gradient of 1e-8, so the converged point of any single run is only pinned to
# about that, and once -- in a full-suite run made while the in-process backend was added -- the
# dhat case's iteration 1 came out as 0.0013792959880998634 from the Python engine against
# 0.00137929599125299 from the C++ one, 2.3e-9 relative, over the bound. Both engines had solved
# the same dhat (0.00145, equal to the last bit), taken the same 16 Newton steps and stopped on the
# same criterion, and the C++ engine's two backends of the time -- in process, and a child running
# the same binary -- agreed with each other to 1.1e-11 in that same run, so it was the Python run
# that drifted and not the port. That one observation is the reason the bound is measured rather
# than guessed, and the forty runs of the measurement above all stayed at least thirteen times
# inside it. It is not loosened past what those runs justify: a failure here is a report about the
# solver, not about the glue.

# The bound for the quantities the solver's output feeds -- the active distances, the dhat values
# that are computed from one, the deformed coordinates and the final barrier stiffness. Everything
# else in this section is compared exactly. See the measurement above.
RTOL_PYTHON_VS_CPP = 1e-9  # the Python engine against the C++ one


def _run_cpp_engine(mesh, operation, root, **options):
    """The C++ engine end to end, on the polyfem this component is linked against. Returns the
    deformed mesh."""
    run_cpp(mesh, operation, root, inputs_only=False, **options)
    return root / "out.msh"


# Every line the two loops print, keyed by the prefix that identifies it. The C++ engine sends them
# through `logger()` instead of `print`, which prefixes a timestamp and a level, so the comparison
# is on the text FROM the marker onwards -- that text is mirrored character for character, numeric
# formats included.
DECISION_MARKERS = (
    "Probe: initial gap ",
    "Starting dhat ramp at ",
    "Current active distance: ",
    "Overshot target separation.",
    "Updated dhat to ",
    "Separation grew <1%",
    "Desired separation achieved ",
    "Deficit ",
    "No active distance found in output",
    "Active distance is infinite",
    "Reached maximum iterations ",
)


def _decision_trail(captured):
    """The loop's decisions, in order, as [marker, text from the marker to the end of the line]
    (lists, so that a trail read back from a golden's JSON compares equal).

    This is the whole observable behaviour of the outer loop: which solve overshot and was rolled
    back, which committed, what dhat or barrier stiffness the next one got and why it stopped.
    """
    trail = []
    for line in captured.splitlines():
        for marker in DECISION_MARKERS:
            at = line.find(marker)
            if at >= 0:
                trail.append([marker, line[at:].rstrip()])
                break
    return trail


def _iteration_logs(sim_out_dir):
    """The per-solve logs in iteration order: polyfem_iter_0.log, polyfem_iter_1.log, ..."""
    return [sim_out_dir / f"polyfem_iter_{i}.log"
            for i in range(len(list(sim_out_dir.glob("polyfem_iter_*.log"))))]


def _log_values(log_path):
    """(active distance, dhat) of the last "active distance:" line in a solve's log, read exactly
    as both engines read it: the token after the marker, trailing comma stripped, `float`.

    polyfem prints both at full round-trip precision, so this recovers the very doubles the loop
    worked with -- the engines' own printed trail is rounded to seven significant digits.
    """
    last = None
    for line in log_path.read_text(errors="replace").splitlines():
        if "active distance:" in line:
            last = line
    assert last is not None, f"{log_path.name} has no active distance line"
    return [float(last.split("active distance:")[-1].split()[0].rstrip(",;").replace(",", "")),
            float(last.split("dhat:")[-1].split()[0].rstrip(",;").replace(",", ""))]


def _solve_values(root):
    """[active distance, dhat] of every solve of a minimum-separation loop, in order."""
    return [_log_values(log) for log in _iteration_logs(root / "sep_output")]


def _assert_deformed_meshes_close(py_msh, cpp_msh, rtol):
    """The two deformed meshes: identical structure, coordinates within `rtol`.

    The structure -- node tags in file order, physical groups and their elements -- is asserted
    exactly, because nothing about it depends on the solver's output. The coordinates cannot be:
    they are the solution, and the solution carries the contact assembly's run-to-run noise.
    """
    py, cpp = _gmsh_reduced_mesh(py_msh), _gmsh_reduced_mesh(cpp_msh)
    _assert_same_mesh_structure(py, cpp, "deformed meshes")
    diff = np.max(np.abs(py["coords"] - cpp["coords"])
                  / np.maximum(np.abs(py["coords"]), 1.0))
    assert diff <= rtol, f"deformed coordinates differ by {diff:.3e} relative (tolerance {rtol:.0e})"
    return diff


def _assert_loops_agree(py_run, cpp_run, rtol, dhat_pinned=False):
    """The two engines' solver loops, each given as {"trail": _decision_trail, "solves":
    _solve_values}: the same decisions, the same number of solves, and dhat and the active
    distances within `rtol` relative -- exactly where the section header above says dhat is exact.
    `dhat_pinned` is the stiffness strategy, which never moves dhat at all."""
    py_trail, cpp_trail = py_run["trail"], cpp_run["trail"]
    assert py_trail == cpp_trail, (
        "the loops decided differently\npython: {}\nc++   : {}".format(
            "\n        ".join(t for _, t in py_trail),
            "\n        ".join(t for _, t in cpp_trail)))

    py_solves, cpp_solves = py_run["solves"], cpp_run["solves"]
    assert len(py_solves) == len(cpp_solves) > 0, (
        f"{len(py_solves)} solves from python, {len(cpp_solves)} from c++")

    worst = 0.0
    for i, ((py_active, py_dhat), (cpp_active, cpp_dhat)) in enumerate(zip(py_solves, cpp_solves)):
        if dhat_pinned or i == 0:
            # Neither of these reads a solver measurement -- the stiffness strategy pins dhat at
            # sep*(1+rtol), and the first dhat of the ramp is dhat_growth times the probe's
            # geometric gap -- so a last-bit difference here would be a porting bug.
            assert py_dhat == cpp_dhat, (
                f"iteration {i}: dhat {py_dhat!r} from python, {cpp_dhat!r} from c++")
        else:
            rel = abs(py_dhat - cpp_dhat) / abs(py_dhat)
            assert rel <= rtol, f"iteration {i}: dhat values differ by {rel:.3e} relative"
        rel = abs(py_active - cpp_active) / abs(py_active)
        assert rel <= rtol, f"iteration {i}: active distances differ by {rel:.3e} relative"
        worst = max(worst, rel)
    return worst


def _assert_steered_on_the_logged_distance(root, trail):
    """The in-process backend takes the active distance from polyfem's contact form instead of from
    its log text. The two are asserted EQUAL, to every digit, in the C++ test that can see both at
    full precision (components/polyfem_ops/.../tests/test_polyfem_in_process.cpp); here the same
    claim is checked on the real cases at the six significant digits the loop prints -- the value
    the loop steered on, against the value polyfem wrote into that iteration's log.
    """
    logs = _iteration_logs(root / "sep_output")
    steered = [text for marker, text in trail if marker == "Current active distance: "]
    assert len(steered) == len(logs), (
        f"{len(steered)} active distances printed, {len(logs)} solves ran")
    for i, (log, text) in enumerate(zip(logs, steered)):
        active, _ = _log_values(log)
        expected = f"Current active distance: {active:.6e}"
        assert text.startswith(expected), (
            f"iteration {i}: the loop steered on {text!r}, the log carries {active!r}")


# --------------------------------------------------------------------------
# The write-back on its own: one solution applied exactly
# --------------------------------------------------------------------------

@needs_polyfem_ops
@pytest.mark.parametrize("mesh_fixture, dim", [("boxes3d", 3), ("jagged2d", 2)])
def test_deformed_msh_write_back_matches(request, tmp_path, mesh_fixture, dim):
    """The deformed mesh is the input mesh with the solution added, exactly: from the solution on,
    every step is deterministic, so the write-back is held to an exact value.

    The solution is a real one: the C++ engine runs the smoothing operation, and its write-back
    reads the solution.txt polyfem wrote, at 100 significant digits -- far past the 17 a double
    needs, so the file carries every value exactly.
    """
    mesh = request.getfixturevalue(mesh_fixture)
    scale = 1e-3
    cpp_out = _run_cpp_engine(mesh, "laplacian_smoothing", tmp_path / "cpp",
                              interfaces=[{"region": "tag_0", "filter": "ambient"}], scale=scale)
    solution = tmp_path / "cpp" / "smooth_output" / "solution.txt"

    original, cpp = _gmsh_reduced_mesh(mesh), _gmsh_reduced_mesh(cpp_out)
    _assert_same_mesh_structure(original, cpp, "input and deformed meshes",
                                sides=("the input", "the write-back"))

    # What the write-back has to produce: the original coordinate plus the displacement in mesh
    # units, `u / scale`, componentwise, with the components the solution does not carry (z in 2D)
    # left untouched. The C++ writes every coordinate as the shortest decimal that reads back as
    # the same double, so its file carries that value exactly. (gmsh's ASCII msh writer, which the
    # Python engine used, prints "%.16g" and loses the last bits of about 40% of them -- measured:
    # 122 of 300 pseudo-random values do not survive that round trip.)
    # The fixtures number their nodes 1..n, which is the row order of solution.txt.
    u = np.loadtxt(solution)[np.array(original["node_tags"]) - 1]
    exact = original["coords"].copy()
    exact[:, :dim] += u / scale
    assert np.array_equal(cpp["coords"], exact), (
        "the c++ coordinates are not original + u/scale; max relative difference {:.3e}".format(
            np.max(np.abs(cpp["coords"] - exact) / np.maximum(np.abs(exact), 1.0))))
    if dim == 2:
        # The solution has two columns, so the third coordinate is carried over untouched.
        assert np.array_equal(cpp["coords"][:, 2], np.zeros(len(cpp["coords"])))


# --------------------------------------------------------------------------
# End to end, with the real solver
# --------------------------------------------------------------------------

SEP_BASE = {"collision_pairs": BOTH_SKINS, "sep": 1.5e-3, "scale": 1e-3, "rtol": 1e-1,
            "max_iterations": 4}


def _final_kappa(root):
    """The barrier stiffness of the last solve: the last value written into the simulation JSON."""
    doc = json.loads((root / "sep_input" / "separation.json").read_text())
    return doc["solver"]["contact"]["barrier_stiffness"]


@needs_polyfem_ops
@needs_goldens
@pytest.mark.parametrize("options, label", [
    # The dhat ramp: a zero-stiffness probe, one overshoot that is rolled back and halved, a
    # commit that bisects the bracket, and a stop inside the tolerance band.
    ({"strategy": "dhat"}, "dhat"),
    # The stiffness loop: dhat pinned, every solve committed, the exponent refit from the last two
    # deficits and the multiplier clamped.
    ({"strategy": "stiffness"}, "stiffness"),
    # A hard-pinned region: the contact pushes only the unprotected side, and the extra
    # `constraints.hard` block has to reach polyfem identically from both engines.
    ({"strategy": "dhat", "protected_regions": ["tag_1"]}, "protected"),
    # Every group NeoHookean instead of AMIPS: a different materials block, hence a different
    # energy and a different ramp, and the check that polyfem accepts what that option writes.
    ({"strategy": "dhat", "use_nh_body": True}, "neohookean"),
])
def test_minimum_separation_end_to_end_matches(boxes3d, tmp_path, capfd, options, label):
    # A golden run is what this compares: the decision trail, every solve's (active distance,
    # dhat) at full precision, the final barrier stiffness, and the deformed mesh (out.msh).
    golden = _golden(f"minimum_separation_end_to_end/{label}", boxes3d)
    py_run = json.loads((golden / "run.json").read_text())
    assert py_run["trail"], "the recorded python run has no decisions"

    cpp_root = tmp_path / f"cpp_{label}"
    cpp_msh = _run_cpp_engine(boxes3d, "minimum_separation", cpp_root, **{**SEP_BASE, **options})
    cpp_trail = _decision_trail(capfd.readouterr().out)

    dhat_pinned = options["strategy"] == "stiffness"
    _assert_loops_agree(py_run, {"trail": cpp_trail, "solves": _solve_values(cpp_root)},
                        RTOL_PYTHON_VS_CPP, dhat_pinned=dhat_pinned)
    _assert_deformed_meshes_close(golden / "out.msh", cpp_msh, RTOL_PYTHON_VS_CPP)
    _assert_steered_on_the_logged_distance(cpp_root, cpp_trail)

    # The stiffness loop's kappa lives only in the trail above at seven digits; its full-precision
    # value is the last one written into the simulation JSON. It is a power law of the measured
    # deficit, so it inherits the noise: measured 1.9e-12 relative on this case, which stays inside
    # 1e-9 only because the multiplier hits its max_stiffness_multiplier clamp.
    if dhat_pinned:
        py_kappa, cpp_kappa = py_run["final_barrier_stiffness"], _final_kappa(cpp_root)
        rel = abs(py_kappa - cpp_kappa) / abs(py_kappa)
        assert rel <= RTOL_PYTHON_VS_CPP, (
            f"final barrier stiffness differs from python by {rel:.3e} relative")


def _group_volume(msh, group):
    """The volume of one physical group of a tet .msh, summed over its elements in file order."""
    mesh = _gmsh_reduced_mesh(msh)
    by_tag = dict(zip(mesh["node_tags"], mesh["coords"]))
    for _, name, elements in mesh["groups"]:
        if name != group:
            continue
        total = 0.0
        for _, verts in elements:
            a, b, c, d = (by_tag[v] for v in verts)
            total += abs(np.linalg.det(np.stack([b - a, c - a, d - a]))) / 6.0
        return total
    raise AssertionError(f"{msh} has no physical group {group}")


@needs_polyfem_ops
def test_minimum_separation_neohookean_holds_body_volume(boxes3d, tmp_path):
    """The measurement the `use_nh_body` spec entry claims, reproduced on this fixture.

    AMIPS scores element SHAPE only and is invariant under uniform scaling, so a body pushed on
    all sides can change volume for free; NeoHookean's volumetric term makes that cost energy. The
    number per body is its deformed volume over its rest volume at the end of the same dhat ramp
    the end-to-end test runs. One engine is enough here: the two write the same simulation JSON,
    which every test above this one asserts, so this runs the C++ one in this process.
    """
    ratios = {}
    for label, options in (("amips", {}), ("neohookean", {"use_nh_body": True})):
        msh = _run_cpp_engine(boxes3d, "minimum_separation", tmp_path / label,
                              **{**SEP_BASE, "strategy": "dhat", **options})
        ratios[label] = {body: _group_volume(msh, body) / _group_volume(boxes3d, body)
                         for body in ("tag_0", "tag_1")}

    # Measured on this fixture: AMIPS lets the two bodies shrink to 0.807 and 0.810 of their rest
    # volume, NeoHookean holds them at 0.985 and 0.989 (tag_0 and tag_1). The spec entry claims
    # 0.83 and 0.95-0.98 from an earlier run of the same experiment. Only the ORDER is asserted:
    # the ratios themselves depend on where the dhat ramp stops, which is a solver measurement.
    for body in ("tag_0", "tag_1"):
        assert ratios["neohookean"][body] > ratios["amips"][body], (
            f"{body}: NeoHookean kept {ratios['neohookean'][body]:.4f} of the rest volume, AMIPS "
            f"kept {ratios['amips'][body]:.4f}; the volumetric term is not doing its job")


@needs_polyfem_ops
@needs_goldens
def test_laplacian_smoothing_end_to_end_matches(jagged2d, tmp_path):
    """Smoothing is the single-solve path: no loop, no contact, one polyfem.log, then the
    write-back. The deformed mesh is the whole observable result, and the whole golden."""
    options = {"interfaces": [{"region": "tag_0", "filter": "ambient"}], "weight_laplacian": 1e3}
    golden = _golden("laplacian_smoothing_end_to_end", jagged2d)
    cpp_root = tmp_path / "cpp"
    cpp_msh = _run_cpp_engine(jagged2d, "laplacian_smoothing", cpp_root, **options)

    assert (cpp_root / "smooth_output" / "polyfem.log").is_file(), "no polyfem.log"
    assert not list((cpp_root / "smooth_output").glob("polyfem_iter_*.log")), (
        "smoothing must not run an outer loop")
    _assert_deformed_meshes_close(golden / "out.msh", cpp_msh, RTOL_PYTHON_VS_CPP)


@needs_polyfem_ops
@needs_goldens
def test_minimum_separation_probe_already_separated_matches(boxes3d, tmp_path, capfd):
    """The probe path: the bodies start 1 mesh unit apart, which at scale 1e-3 is a gap of 1e-3
    solver units, so a target of 5e-4 is already met. The probe measures the gap, reports it and
    the loop never starts -- nothing moves, on either engine. The golden is the trail and the
    deformed mesh."""
    options = {**SEP_BASE, "sep": 5e-4, "strategy": "dhat"}
    golden = _golden("minimum_separation_probe_already_separated", boxes3d)
    py_trail = json.loads((golden / "run.json").read_text())["trail"]
    py_msh = golden / "out.msh"
    cpp_root = tmp_path / "cpp"
    cpp_msh = _run_cpp_engine(boxes3d, "minimum_separation", cpp_root, **options)
    cpp_trail = _decision_trail(capfd.readouterr().out)

    assert py_trail == cpp_trail
    assert len(py_trail) == 1 and "already separated" in py_trail[0][1], py_trail
    assert (cpp_root / "sep_output" / "polyfem_probe.log").is_file()
    assert not list((cpp_root / "sep_output").glob("polyfem_iter_*.log")), (
        "the loop ran although the bodies were already separated")

    # Nothing moved: the probe solves at zero barrier stiffness, so its solution is the rest state
    # and the deformed mesh is the input mesh back again. No solver measurement reaches these
    # coordinates, and the `moved` check below pins each mesh to the input on its own.
    _assert_deformed_meshes_close(py_msh, cpp_msh, RTOL_PYTHON_VS_CPP)
    original = _gmsh_reduced_mesh(boxes3d)
    for msh in (py_msh, cpp_msh):
        moved = np.max(np.abs(_gmsh_reduced_mesh(msh)["coords"] - original["coords"]))
        assert moved <= 1e-9, f"{msh.name}: nodes moved by {moved:.3e} although nothing should"
