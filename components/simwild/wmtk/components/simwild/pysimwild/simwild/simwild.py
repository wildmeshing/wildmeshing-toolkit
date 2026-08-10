import os
import argparse
from pathlib import Path

from wildmeshing import *

"""
This module provides wrapper functions for SimWild operations.

This module requires the wildmeshing package to be installed.
To install wildmeshing, clone the WMTK repository, then execute:
python -m pip install .

To install the SimWild wrapper, execute:
python -m pip install -e /components/simwild/wmtk/components/simwild/pysimwild
"""


def _ensure_output_dir(output: str) -> None:
    """Make sure the parent directory of `output` (a path stem) exists."""
    parent = os.path.dirname(output)
    if parent:
        os.makedirs(parent, exist_ok=True)


def _run_wmtk(j: dict, output: str) -> None:
    """Final common step for every op: default the path-resolution root and
    invoke wmtk. wmtk resolves relative paths against j["input_dir"];
    libstdc++ throws on the empty default, so fall back to the CWD (same
    fix the old pywmtk_wrapper._wmtk_call applied via json_input_file)."""
    j.setdefault("input_dir", os.getcwd())
    _ensure_output_dir(output)
    wildmeshing(j)


def _ensure_msh_extension(path: str) -> str:
    """Ensure the path ends with .msh. If it has a different extension, raise ValueError."""
    p = Path(path)
    if p.suffix != ".msh":
        if p.suffix != "":
            raise ValueError(
                f"Input mesh file must be a .msh file. Got: {path}")
        return str(p.with_suffix(".msh"))
    return str(p)


def strip_envelope(msh_path, output_path=None):
    """Remove the WMTK envelope physical group from a `.msh` file.

    WMTK writes the input surface (3D) or curve (2D) as a separate physical
    group with its own vertex block. Those vertices duplicate coordinates
    already present in the volumetric mesh under fresh node tags and confuse
    downstream tools (polyfem's MeshNodes, deformed-msh writers, etc.). This
    util drops that envelope group, its entity, its element block, and the
    dangling envelope-only nodes — leaving the volumetric groups
    (ambient, tag_*) untouched.

    Dimension is auto-detected:
      - 3D mesh (any dim=3 phys group)  -> strip dim=2 envelope
      - 2D mesh (no dim=3, has dim=2)   -> strip dim=1 envelope

    Parameters:
    - msh_path: input `.msh` file path.
    - output_path: where to write the cleaned mesh. Defaults to msh_path
      (in-place).
    """

    msh_path = _ensure_msh_extension(msh_path)

    import gmsh
    if output_path is None:
        output_path = msh_path

    gmsh.initialize()
    try:
        gmsh.open(msh_path)

        phys_dims = {d for d, _ in gmsh.model.getPhysicalGroups()}
        # mesh dim is the highest phys group dim; envelope is one less.
        if 3 in phys_dims:
            envelope_dim = 2
        elif 2 in phys_dims:
            envelope_dim = 1
        else:
            raise ValueError(
                f"strip_envelope: no dim-2 or dim-3 physical groups in {msh_path}")

        groups_env = list(gmsh.model.getPhysicalGroups(dim=envelope_dim))
        if groups_env:
            gmsh.model.removePhysicalGroups(groups_env)

        ents_env = list(gmsh.model.getEntities(dim=envelope_dim))
        if ents_env:
            gmsh.model.removeEntities(ents_env, recursive=True)

        gmsh.option.setNumber("Mesh.MshFileVersion", 4.1)
        gmsh.write(output_path)
    finally:
        gmsh.finalize()


def surf_to_tet(meshes=[], tags=[], output="out", stop_energy=100, eps_rel=2e-3, eps_simplify_rel=2e-4, skip_simplify=False, preserve_topology=False, num_threads=0, others={}):
    """
    Convert surface meshes to a tetrahedral mesh.

    Parameters:
    - meshes: List of input surface mesh file paths (e.g., .obj, .ply).
    - tags: List of tags for each input mesh (e.g., ["tag_1", "tag_2"]).
    - output: Output file path for the generated tetrahedral mesh, without extension (e.g., "out" will generate "out.msh").
    - stop_energy: Energy threshold for mesh optimization (lower is more aggressive). We recommend to not go below 10.
    - eps_rel: Relative tolerance for mesh optimization.
    - eps_simplify_rel: Relative tolerance for simplification. Only relevant if skip_simplify is False.
    - skip_simplify: Whether to skip the simplification step.
    - preserve_topology: Whether to preserve the topology of the input meshes.
    - num_threads: Number of threads to use for parallel processing. 0 threads means single threaded execution.
    - others: Additional parameters (optional).
    """
    if len(meshes) == 0:
        raise ValueError("At least one mesh must be provided.")

    if len(tags) != len(meshes):
        raise ValueError("Number of tags must match the number of meshes.")

    j = {}
    j["application"] = "simwild"
    j["output"] = output
    j["input"] = meshes
    j["input_names"] = tags
    j["preserve_topology"] = preserve_topology
    j["eps_rel"] = eps_rel
    j["stop_energy"] = stop_energy
    j["eps_simplify_rel"] = eps_simplify_rel
    j["skip_simplify"] = skip_simplify
    j["num_threads"] = num_threads

    # copy any additional parameters from others into j
    for key, value in others.items():
        if key in j:
            raise ValueError(
                f"Key '{key}' already exists in the main parameters. Cannot set this key in others.")
        j[key] = value

    _run_wmtk(j, output)


def lines_to_tri(meshes=[], tags=[], output="out", stop_energy=10, eps_rel=2e-3, preserve_topology=False, num_threads=0, others={}):
    """
    Convert 2D lines to a triangle mesh.

    Parameters:
    - meshes: List of input lines file paths (.obj).
    - tags: List of tags for each input mesh (e.g., ["tag_1", "tag_2"]).
    - output: Output file path for the generated triangle mesh, without extension (e.g., "out" will generate "out.msh").
    - stop_energy: Energy threshold for mesh optimization (lower is more aggressive).
    - eps_rel: Relative tolerance for mesh optimization.
    - preserve_topology: Whether to preserve the topology of the input meshes.
    - num_threads: Number of threads to use for parallel processing. 0 threads means single threaded execution.
    - others: Additional parameters (optional).
    """
    if len(meshes) == 0:
        raise ValueError("At least one mesh must be provided.")

    if len(tags) != len(meshes):
        raise ValueError("Number of tags must match the number of meshes.")

    j = {}
    j["application"] = "triwild"
    j["output"] = output
    j["input"] = meshes
    j["input_names"] = tags
    j["preserve_topology"] = preserve_topology
    j["eps_rel"] = eps_rel
    j["stop_energy"] = stop_energy
    j["num_threads"] = num_threads

    # copy any additional parameters from others into j
    for key, value in others.items():
        if key in j:
            raise ValueError(
                f"Key '{key}' already exists in the main parameters. Cannot set this key in others.")
        j[key] = value

    _run_wmtk(j, output)


def remeshing(mesh, output="out", stop_energy=10, eps_rel=2e-3, length_rel=5e-2, sizing_field=[], preserve_topology=True, keep_envelope=False, num_threads=0, others={}):
    """
    Improve the quality of a tetrahedral mesh through remeshing.

    Parameters:
    - mesh: Input tetrahedral mesh file path (.msh). The extension can be omitted, and it will be automatically added, e.g. "mesh" will be treated as "mesh.msh". Other extensions will raise an error.
    - output: Output file path for the improved mesh, without extension (e.g., "out" will generate "out.msh").
    - stop_energy: Energy threshold for mesh optimization (lower is more aggressive). We recommend to not go below 10.
    - eps_rel: Relative tolerance for mesh optimization.
    - length_rel: Relative target edge length.
    - sizing_field: Prescribe a length for a specific region defined by tags. The length can be absolute (length) or relative (length_rel). Example: [{"tags": "tag_0 & tag_1", "length": 0.1}, {"tags": "tag_2", "length_rel": 0.05}]
    - preserve_topology: Whether to preserve the topology of the input meshes.
    - keep_envelope: Whether to keep the envelope of the input mesh.
    - num_threads: Number of threads to use for parallel processing. 0 threads means single threaded execution.
    - others: Additional parameters (optional).
    """

    mesh = _ensure_msh_extension(mesh)

    j = {}
    j["application"] = "simwild"
    j["output"] = output
    j["input"] = [mesh]
    j["preserve_topology"] = preserve_topology
    j["eps_rel"] = eps_rel
    j["stop_energy"] = stop_energy
    j["length_rel"] = length_rel
    j["sizing_field"] = sizing_field
    j["write_envelope"] = keep_envelope
    j["num_threads"] = num_threads

    # copy any additional parameters from others into j
    for key, value in others.items():
        if key in j:
            raise ValueError(
                f"Key '{key}' already exists in the main parameters. Cannot set this key in others.")
        j[key] = value

    _run_wmtk(j, output)


def resolve_overlaps(mesh, tags, output="out", others={}):
    """
    Resolve overlaps between two regions in the mesh.

    Parameters:
    - mesh: Input tetrahedral mesh file path (.msh). The extension can be omitted, and it will be automatically added, e.g. "mesh" will be treated as "mesh.msh". Other extensions will raise an error.
    - tags: List of intersecting regions. Each region is a Boolean expression, (e.g., [["tag_0 & tag_1", "tag_2"]]). For each intersection, two regions must be provided. Multiple intersections can be resolved in a single call by providing multiple pairs of regions (e.g., [["tag_0 & tag_1", "tag_2"], ["tag_3", "tag_4"]]).
    - output: Output file path for the resolved meshes, without extension (e.g., "out" will generate "out.msh").
    - others: Additional parameters (optional).
    """

    mesh = _ensure_msh_extension(mesh)

    j = {}
    j["application"] = "simwild"
    j["operation"] = "resolve_overlaps"
    j["input"] = [mesh]
    j["output"] = output
    j["resolve_overlaps_tags"] = tags

    # copy any additional parameters from others into j
    for key, value in others.items():
        if key in j:
            raise ValueError(
                f"Key '{key}' already exists in the main parameters. Cannot set this key in others.")
        j[key] = value

    _run_wmtk(j, output)


def replace_tags(mesh, tags_in, tags_out, output="out", others={}):
    """
    Replace tags in the mesh.

    Parameters:
    - mesh: Input mesh file path (must be .msh). The extension can be omitted, and it will be automatically added, e.g. "mesh" will be treated as "mesh.msh". Other extensions will raise an error.
    - tags_in: Intersection of tags to be replaced (e.g., ["tag_0 & tag_2"]).
    - tags_out: List of tags to replace with (e.g., ["tag_1"]). The ambient tag ("_") effectively removes the input tags.
    - output: Output file path for the modified mesh, without extension (e.g., "out" will generate "out.msh").
    - others: Additional parameters (optional).
    """

    mesh = _ensure_msh_extension(mesh)

    j = {}
    j["application"] = "simwild"
    j["operation"] = "replace_tags"
    j["input"] = [mesh]
    j["output"] = output
    j["replace_tags_in"] = tags_in
    j["replace_tags_out"] = tags_out

    # copy any additional parameters from others into j
    for key, value in others.items():
        if key in j:
            raise ValueError(
                f"Key '{key}' already exists in the main parameters. Cannot set this key in others.")
        j[key] = value

    _run_wmtk(j, output)


def tag_priority(mesh, tags, output="out", others={}):
    """
    Set tag priority in the mesh.

    Parameters:
    - mesh: Input mesh file path (must be .msh). The extension can be omitted, and it will be automatically added, e.g. "mesh" will be treated as "mesh.msh". Other extensions will raise an error.
    - tags: List of tags in order of priority (e.g., ["tag_1", "tag_0", "tag_2"]).
    - output: Output file path for the modified mesh, without extension (e.g., "out" will generate "out.msh").
    - others: Additional parameters (optional).
    """

    mesh = _ensure_msh_extension(mesh)

    j = {}
    j["application"] = "simwild"
    j["operation"] = "tag_priority"
    j["input"] = [mesh]
    j["output"] = output
    j["tag_priority"] = tags

    # copy any additional parameters from others into j
    for key, value in others.items():
        if key in j:
            raise ValueError(
                f"Key '{key}' already exists in the main parameters. Cannot set this key in others.")
        j[key] = value

    _run_wmtk(j, output)


def tight_seal_topo(mesh, tags, output="out", others={}):
    """
    Perform topological tight sealing of the mesh.

    Parameters:
    - mesh: Input mesh file path (must be .msh). The extension can be omitted, and it will be automatically added, e.g. "mesh" will be treated as "mesh.msh". Other extensions will raise an error.
    - tags: List of expression pairs; each pair [expr_A, expr_B] seals the holes
      between the regions selected by expr_A and expr_B (e.g. [["tag_0", "tag_1"]]).
    - output: Output file path for the sealed mesh, without extension (e.g., "out" will generate "out.msh").
    - others: Additional parameters (optional).
    """

    mesh = _ensure_msh_extension(mesh)

    j = {}
    j["application"] = "simwild"
    j["operation"] = "tight_seal_topo"
    j["input"] = [mesh]
    j["output"] = output
    j["tight_seal_tag_sets"] = tags

    # copy any additional parameters from others into j
    for key, value in others.items():
        if key in j:
            raise ValueError(
                f"Key '{key}' already exists in the main parameters. Cannot set this key in others.")
        j[key] = value

    _run_wmtk(j, output)


def keep_lcc(mesh, tags, lcc_num=1, output="out", others={}):
    """
    Keep only the largest connected component (LCC) of the mesh for specified tags.

    Parameters:
    - mesh: Input mesh file path (must be .msh). The extension can be omitted, and it will be automatically added, e.g. "mesh" will be treated as "mesh.msh". Other extensions will raise an error.
    - tags: Intersection of tags (e.g., ["tag_0 & tag_1"]).
    - lcc_num: Number of components to keep (default is 1).
    - output: Output file path for the modified mesh, without extension (e.g., "out" will generate "out.msh").
    - others: Additional parameters (optional).
    """

    mesh = _ensure_msh_extension(mesh)

    j = {}
    j["application"] = "simwild"
    j["operation"] = "keep_lcc"
    j["input"] = [mesh]
    j["output"] = output
    j["keep_lcc_tags"] = tags
    j["keep_lcc_num"] = lcc_num

    # copy any additional parameters from others into j
    for key, value in others.items():
        if key in j:
            raise ValueError(
                f"Key '{key}' already exists in the main parameters. Cannot set this key in others.")
        j[key] = value

    _run_wmtk(j, output)


def fill_holes_topo(mesh, tags, output="out", others={}):
    """
    Fill holes in the mesh based on topological information.

    Parameters:
    - mesh: Input mesh file path (must be .msh). The extension can be omitted, and it will be automatically added, e.g. "mesh" will be treated as "mesh.msh". Other extensions will raise an error.
    - tags: Intersection of tags (e.g., ["tag_0 & tag_1"]).
    - output: Output file path for the modified mesh, without extension (e.g., "out" will generate "out.msh").
    - others: Additional parameters (optional).
    """

    mesh = _ensure_msh_extension(mesh)

    j = {}
    j["application"] = "simwild"
    j["operation"] = "fill_holes_topo"
    j["input"] = [mesh]
    j["output"] = output
    j["fill_holes_tags"] = tags

    # copy any additional parameters from others into j
    for key, value in others.items():
        if key in j:
            raise ValueError(
                f"Key '{key}' already exists in the main parameters. Cannot set this key in others.")
        j[key] = value

    _run_wmtk(j, output)


def manifold_extraction(mesh, tag_selection, radius, fill_tags=[], output="out", others={}):
    """
    Make boundary of in_tag set manifold (Only 3D supported as of now).

    Parameters:
    - mesh: Input mesh file path (must be .msh). The extension can be omitted, and it will be automatically added, e.g. "mesh" will be treated as "mesh.msh". Other extensions will raise an error.
    - tag_selection: selection for elements considered inside the surface, e.g. "tag_0 | tag_1"
    - radius: Float, radius for offset created around nonmanifold components
    - fill_tags: Tag set for offset created around nonmanifold components, e.g. ["tag_0", "tag_1"]. Use [] for ambient.
    - output: Output file path for the modified mesh, without extension (e.g. "out" will generate "out.msh")
    - others: Additional parameters (optional).
    """

    mesh = _ensure_msh_extension(mesh)

    j = {}
    j["application"] = "manifold_extraction"
    j["input"] = mesh
    j["tag_selection"] = tag_selection
    j["radius"] = radius
    j["fill_tags"] = fill_tags
    j["output"] = output

    # copy any additional parameters from others into j
    for key, value in others.items():
        if key in j:
            raise ValueError(
                f"Key '{key}' already exists in the main parameters. Cannot set this key in others.")
        j[key] = value

    _run_wmtk(j, output)


def topological_offset(mesh, offset_selection, target_distance, offset_output_tags, overwrite_tags=False,
                       offset_in=False, offset_out=True, protected_tags=[], rel_ball_threshold=0.01,
                       output="out", others={}):
    """
    Create topological offset. NOTE: ambient is represented as 'ambient', not '_'.

    Parameters:
    - mesh: mesh: Input tetrahedral mesh file path (must be .msh). The extension can be omitted, and it will be automatically added, e.g. "mesh" will be treated as "mesh.msh". Other extensions will raise an error.
    - offset_selection: Str, boolean expression to identify simplicial complex to offset (e.g. "tag_0 & tag_1"). If one pure
        tag given (e.g. "tag_0"), single body mode is used. `_` denotes the ambient (empty tag-set) region, e.g. "!_" = every
        tagged cell; note "tag_0 & _" cannot select the body<->ambient skin (the incident-tag union is non-empty) — use
        single body mode with offset_in/offset_out for skins.
    - offset_in: Bool, only relevant if single body mode. If True, create offset inwards
    - offset_out: Bool, only relevant if single body mode. If True, create offset outwards (at least
        one of offset_in and offset_out must be true in single body mode)
    - offset_output_tags: List[Str], tag set for the newly createad offset volume (e.g. ["newtag", "newtag2"]).
    - protected_tags: List[Str], only relevant if overwrite_tags=True. Set of tags that will never be overwritten.
    - target_distance: Float, the target distance for the offset. If <= 0, midpoint splitting is used to create the offset.
    - rel_ball_threshold: Float, circle/sphere radius relative to target_distance to terminate conservative inside checks.
        (NOTE: for 3D, anything above 0.01 is not very effective. <0.01 is recommended)
    - output: Output file path for the modified mesh, without extension (e.g. "out" will generate "out.msh")
    - others: Additional parameters (optional).
    """

    mesh = _ensure_msh_extension(mesh)

    j = {}
    j["application"] = "topological_offset"
    j["input"] = mesh
    j["offset_selection"] = offset_selection
    j["offset_in"] = offset_in
    j["offset_out"] = offset_out
    j["offset_output_tags"] = offset_output_tags
    j["protected_tags"] = protected_tags
    j["target_distance"] = target_distance
    j["relative_ball_threshold"] = rel_ball_threshold
    j["output"] = output

    # copy any additional parameters from others into j
    for key, value in others.items():
        if key in j:
            raise ValueError(
                f"Key '{key}' already exists in the main parameters. Cannot set this key in others.")
        j[key] = value
    j.setdefault("edge_search_termination_len", min(1e-6, target_distance * 1e-3))

    _run_wmtk(j, output)


# ---------------------------------------------------------------------------
# Polyfem-based ops (validated against simwild/specs/*.json, same rule format
# as simwild_spec.json; engine: minimum_separation.py)
# ---------------------------------------------------------------------------

def _run_polyfem_op(op_name, params, engine):
    """Validate `params` against the op's spec.json and run the engine on
    the validated, defaults-filled dict. The PolyFEM binary comes from
    $POLYFEM_BIN (the engines raise if it is unset)."""
    from .polyfem_ops import spec as _spec
    p = _spec.validate(_spec.load_spec(op_name), params)
    _ensure_output_dir(p["output"])
    engine(p)
    return p


def minimum_separation(mesh, collision_pairs, sep, output="out", others={}):
    """
    Push collision bodies apart to a target separation (polyfem: AMIPS +
    fitting + Laplacian + GCP contact with a dhat line-search).

    Parameters:
    - mesh: Input multi-tag mesh file path (.msh).
    - collision_pairs: List of [side_A, side_B]; each side is a selection —
      the boundary of `region`, kept where the outside cell satisfies
      `filter`: {"region": expr, "filter": expr, "id": int} or a bare region
      string. Normals point out of the region. Identical selections dedupe
      to one collision body; reuse an explicit id to merge distinct
      selections into one body.
      E.g. [[{"region": "tag_0", "filter": "ambient"},
             {"region": "tag_1", "filter": "ambient"}]].
    - sep: Target minimum separation in solver units (mesh units * scale).
    - output: Output path stem; writes <output>.msh (artifacts next to it).
    - others: Additional parameters — see polyfem_ops/minimum_separation/spec.json
      (scale, use_laplacian, weight_*, rtol, max_iterations, strategy
      ["dhat" default | "stiffness" experimental], ...). PolyFEM binary:
      export POLYFEM_BIN.
    """
    from .polyfem_ops import minimum_separation as _op

    def engine(p):
        cfg = {
            "input_msh": p["input"],
            "collision_pairs": p["collision_pairs"],
            "sep": p["sep"],
            "scale": p["scale"],
            "useFitting": p["use_fitting"],
            "useLaplacian": p["use_laplacian"],
            "useGraphLaplacian": p["use_graph_laplacian"],
            "normalizePenalties": p["normalize_penalties"],
            "weight_fitting": p["weight_fitting"],
            "weight_laplacian": p["weight_laplacian"],
            "amips_weights": p["amips_weights"],
            "max_iterations": p["max_iterations"],
            "rtol": p["rtol"],
            "nl_max_iterations": p["nl_max_iterations"],
            "barrier_stiffness": p["barrier_stiffness"],
            "alpha_n": p["alpha_n"],
            "alpha_t": p["alpha_t"],
            "save_vtu": p["save_vtu"],
            "strategy": p["strategy"],
            "dhat_growth": p["dhat_growth"],
            "max_stiffness_multiplier": p["max_stiffness_multiplier"],
            "protected_regions": p["protected_regions"],
            "ambient_like_tags": p["ambient_like_tags"],
            "output_msh": f"{p['output']}.msh",
        }
        if p["init_dhat"] > 0:
            cfg["init_dhat"] = p["init_dhat"]
        out_dir = os.path.dirname(p["output"]) or "."
        _op.run(cfg, out_dir=Path(out_dir))

    j = {"input": mesh, "collision_pairs": collision_pairs, "sep": sep,
         "output": output, **others}
    _run_polyfem_op("minimum_separation", j, engine)


def laplacian_smoothing(mesh, interfaces=[], output="out", others={}):
    """
    Fair material interfaces with a single polyfem solve (AMIPS + fitting +
    Laplacian; no contact).

    Parameters:
    - mesh: Input multi-tag mesh file path (.msh).
    - interfaces: Selections to smooth — the boundary of `region`, kept
      where the outside satisfies `filter` (e.g. "tag_2" for tag_2's whole
      boundary, {"region": "tag_0", "filter": "tag_1"} for one interface).
      Empty = every material interface.
    - output: Output path stem; writes <output>.msh (artifacts next to it).
    - others: Additional parameters — see polyfem_ops/laplacian_smoothing/spec.json
      (weight_laplacian, smooth_positions, ...). PolyFEM binary: export POLYFEM_BIN.
    """
    from .polyfem_ops import laplacian_smoothing as _op

    def engine(p):
        cfg = {
            "input_msh": p["input"],
            "scale": p["scale"],
            "useFitting": p["use_fitting"],
            "useLaplacian": p["use_laplacian"],
            "useGraphLaplacian": p["use_graph_laplacian"],
            "normalizePenalties": p["normalize_penalties"],
            "weight_fitting": p["weight_fitting"],
            "weight_laplacian": p["weight_laplacian"],
            "max_iterations": p["max_iterations"],
            "smoothDisplacementsOrPositions": 1 if p["smooth_positions"] else 0,
            "save_vtu": p["save_vtu"],
            "ambient_like_tags": p["ambient_like_tags"],
            "output_msh": f"{p['output']}.msh",
        }
        if p["interfaces"]:
            cfg["interfaces"] = p["interfaces"]
        out_dir = os.path.dirname(p["output"]) or "."
        _op.run(cfg, out_dir=Path(out_dir))

    j = {"input": mesh, "interfaces": interfaces, "output": output, **others}
    _run_polyfem_op("laplacian_smoothing", j, engine)


if __name__ == "__main__":
    # Example usage of the wrapper functions for WMTK operations related to image simulation.

    parser = argparse.ArgumentParser(
        description="WMTK Image Simulation Wrapper. The main function demonstrates how to use the wrapper functions.")
    parser.add_argument("-2", "--d2", action="store_true",
                        help="Run 2D operations instead of 3D.")

    args = parser.parse_args()

    if args.d2:
        print("Running 2D operations...")

        # seal
        mesh = "topo_annots_groups.msh"
        tags = [["tag_0", "tag_1"]]
        tight_seal_topo(mesh=mesh, tags=tags, output="m1_2d")

        # resolve overlaps
        mesh = "m1_2d.msh"
        tags = [["tag_1", "tag_2"]]
        resolve_overlaps(mesh=mesh, tags=tags, output="m2_2d")

        # replace tags
        mesh = "m2_2d.msh"
        tags_in = ["tag_0 & tag_2"]
        tags_out = ["tag_2"]
        replace_tags(mesh=mesh, tags_in=tags_in,
                     tags_out=tags_out, output="m3_2d")

        # keep_lcc
        mesh = "m3_2d.msh"
        tags = ["tag_0"]
        keep_lcc(mesh=mesh, tags=tags, lcc_num=1, output="m4_2d")

        # fill holes
        mesh = "m4_2d.msh"
        tags = ["tag_1"]
        fill_holes_topo(mesh=mesh, tags=tags, output="m5_2d")

        # remeshing
        mesh = "m5_2d.msh"
        remeshing(mesh=mesh, output="m6_2d")

        # topological offset
        mesh = "m6_2d.msh"
        topological_offset(mesh=mesh,
                           offset_selection="(tag_1 & tag_2) | (tag_1 & tag_3) | (tag_1 & tag_4)",
                           offset_output_tags=["tag_5"],
                           target_distance=0.1, rel_ball_threshold=0.001, output="m7_2d")

        # # minimum separation — spec-validated op (engine: polyfem)
        # minimum_separation(
        #     mesh="m6.msh",
        #     collision_pairs=[[{"region": "tag_0", "filter": "ambient"},
        #                       {"region": "tag_1", "filter": "ambient"}]],
        #     sep=5e-5,
        #     output="output/sep/m7",
        #     others={"rtol": 1e-1},        # needs $POLYFEM_BIN exported
        # )

        # # laplacian smoothing — spec-validated op (engine: polyfem)
        # strip_envelope("m1.msh", output_path="m1_strip.msh")
        # laplacian_smoothing(
        #     mesh="m1_strip.msh",
        #     interfaces=[{"region": "tag_0", "filter": "tag_1"}],  # omit = all
        #     output="output/smooth/m1",
        #     others={"weight_laplacian": 1e3, "scale": 1.0},
        # )

    else:
        print("Running 3D operations...")

        # surf_to_tet — two unit spheres, second translated 0.3 in x so they overlap.
        meshes = ["sphere.obj", "sphere.obj"]
        others = {}
        others["input_transform"] = [
            [], [[1, 0, 0, 0.3], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]]
        surf_to_tet(meshes=meshes, tags=["tag_0", "tag_1"], output="m1",
                    stop_energy=100, eps_rel=1e-2, others=others)

        # replace_tags
        mesh = "m1.msh"
        tags_in = ["tag_0 & tag_1"]
        tags_out = ["tag_2"]
        replace_tags(mesh=mesh, tags_in=tags_in,
                     tags_out=tags_out, output="m3")

        # tight_seal_topo
        mesh = "m3.msh"
        tags = [["tag_0", "tag_1"]]
        tight_seal_topo(mesh=mesh, tags=tags, output="m4")

        # remeshing
        mesh = "m4.msh"
        remeshing(mesh=mesh, output="m5")

        # topological offset
        mesh = "m1.msh"
        topological_offset(mesh=mesh, offset_selection="tag_0 & tag_1",
                           offset_output_tags=["tag_2"],
                           target_distance=0.1, rel_ball_threshold=0.001, output="m6")

        # manifold extraction
        mesh = "m3.msh"
        manifold_extraction(mesh=mesh, in_tag=["tag_0", "tag_1"],
                            union=False, replace_tag=[], output="m7")

        # tag priority
        mesh = "m1.msh"
        tags = ["tag_1", "tag_0"]
        tag_priority(mesh=mesh, tags=tags, output="m8")
