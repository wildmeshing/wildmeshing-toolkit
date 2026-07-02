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


def _ensure_msh_extension(path: str) -> str:
    """Ensure the path ends with .msh. If it has a different extension, raise ValueError."""
    p = Path(path)
    if p.suffix != ".msh":
        if p.suffix != "":
            raise ValueError(
                f"Input mesh file must be a .msh file. Got: {path}")
        return str(p.with_suffix(".msh"))
    return str(p)


def _name_to_internal_tag(mesh_path: str) -> dict:
    """For a wmtk-compatible `.msh` file, return {phys_group_name: int}.

    Mirrors wmtk's `read_image_msh.cpp` Path A: physical groups are iterated
    in tag-id order; Fs[0] (typically `ambient`) is dropped; the remaining
    groups map to internal ids starting at 0 (so `tag_0` -> 0, `tag_1` -> 1,
    ...). The dropped group is intentionally not in the returned map -- it
    has no internal id.

    Mesh dimension is the highest physical-group dim present (3D mesh ->
    iterate dim-3 groups; 2D mesh -> iterate dim-2 groups).
    """
    import gmsh
    gmsh.initialize()
    try:
        gmsh.open(mesh_path)
        dim = 3 if gmsh.model.getPhysicalGroups(3) else 2
        groups = list(gmsh.model.getPhysicalGroups(dim))
        groups.sort(key=lambda dt: dt[1])  # by phys-tag id
        name_to_id = {}
        for fs_idx, (d, t) in enumerate(groups):
            if fs_idx == 0:
                continue  # ambient, dropped by wmtk
            name = gmsh.model.getPhysicalName(d, t)
            if name:
                name_to_id[name] = fs_idx - 1
        return name_to_id
    finally:
        gmsh.finalize()


_AMBIENT = object()  # sentinel: "ambient" (= no tag); dropped from its set


def _resolve_tags(tags, name_to_id: dict):
    """Recursively walk a (possibly nested) tag structure and convert string
    physical-group names to wmtk internal int ids. Integers pass through.
    Lists/tuples preserve their type.

    `"ambient"` (in any nesting depth) is treated as "no tag" and dropped
    from the enclosing list/tuple -- so e.g. `["ambient"]` becomes `[]`
    (ambient as a tag set), and `["tag_0", "ambient"]` becomes `[0]`. This
    matches wmtk's convention where the ambient region is represented as the
    empty tag set, e.g. `topological_offset(offset_tags=[[], [1, 2]])`.

    Unknown non-ambient names raise ValueError.
    """
    if isinstance(tags, str):
        if tags == "ambient":
            return _AMBIENT
        if tags not in name_to_id:
            raise ValueError(
                f"Unknown wmtk tag name '{tags}'. Available names: "
                f"{sorted(name_to_id)} (or 'ambient', which becomes [])."
            )
        return name_to_id[tags]
    if isinstance(tags, list):
        return [r for r in (_resolve_tags(t, name_to_id) for t in tags)
                if r is not _AMBIENT]
    if isinstance(tags, tuple):
        return tuple(r for r in (_resolve_tags(t, name_to_id) for t in tags)
                     if r is not _AMBIENT)
    return tags


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

    _ensure_output_dir(output)
    wildmeshing(j)


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

    _ensure_output_dir(output)
    wildmeshing(j)


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

    _ensure_output_dir(output)
    wildmeshing(j)


def resolve_intersections(mesh, tags, output="out", others={}):
    """
    Resolve intersections between two objects in the mesh.

    Parameters:
    - mesh: Input tetrahedral mesh file path (.msh). The extension can be omitted, and it will be automatically added, e.g. "mesh" will be treated as "mesh.msh". Other extensions will raise an error.
    - tags: List of tags corresponding to each input mesh (e.g., ["tag_0 & tag_1"]).
    - output: Output file path for the resolved meshes, without extension (e.g., "out" will generate "out.msh").
    - others: Additional parameters (optional).
    """

    mesh = _ensure_msh_extension(mesh)

    j = {}
    j["application"] = "simwild"
    j["operation"] = "resolve_intersections"
    j["input"] = [mesh]
    j["output"] = output
    j["resolve_intersections_tags"] = tags

    # copy any additional parameters from others into j
    for key, value in others.items():
        if key in j:
            raise ValueError(
                f"Key '{key}' already exists in the main parameters. Cannot set this key in others.")
        j[key] = value

    _ensure_output_dir(output)
    wildmeshing(j)


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

    _ensure_output_dir(output)
    wildmeshing(j)


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

    _ensure_output_dir(output)
    wildmeshing(j)


def tight_seal_topo(mesh, tags, output="out", others={}):
    """
    Perform topological tight sealing of the mesh.

    Parameters:
    - mesh: Input mesh file path (must be .msh). The extension can be omitted, and it will be automatically added, e.g. "mesh" will be treated as "mesh.msh". Other extensions will raise an error.
    - tags: Intersection of tags (e.g., ["tag_0 & tag_1"]).
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

    _ensure_output_dir(output)
    wildmeshing(j)


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

    _ensure_output_dir(output)
    wildmeshing(j)


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
    _ensure_output_dir(output)
    wildmeshing(j)


def manifold_extraction(mesh, in_tag, union, replace_tag=[], output="out", others={}):
    """
    Make boundary of in_tag set manifold (Only 3D supported as of now).

    Parameters:
    - mesh: Input mesh file path (must be .msh). The extension can be omitted, and it will be automatically added, e.g. "mesh" will be treated as "mesh.msh". Other extensions will raise an error.
    - in_tag: List of tags considered inside the input (e.g. ["tag_0", "tag_1"]). The boundary of the union of these tags is
        considered the surface to make manifold.
    - union: True to join the offset with in_tag, False to set to replace_tag
    - replace_tag: Only relevant if union=False. The tag set to fill offsets around non manifold simplices (e.g. [2])
    - output: Output file path for the modified mesh, without extension (e.g. "out" will generate "out.msh")
    - others: Additional parameters (optional).
    """

    mesh = _ensure_msh_extension(mesh)

    j = {}
    j["application"] = "manifold_extraction"
    j["input"] = mesh
    j["in_tag"] = in_tag
    j["manifold_union"] = union
    j["replace_tag"] = replace_tag
    j["output"] = output

    # copy any additional parameters from others into j
    for key, value in others.items():
        if key in j:
            raise ValueError(
                f"Key '{key}' already exists in the main parameters. Cannot set this key in others.")
        j[key] = value

    _ensure_output_dir(output)
    wildmeshing(j)


def topological_offset(mesh, offset_selection, target_distance, offset_output_tags, overwrite_tags,
                       offset_in=False, offset_out=True, protected_tags=[], rel_ball_threshold=0.01,
                       output="out", others={}):
    """
    Create topological offset.

    Parameters:
    - mesh: mesh: Input tetrahedral mesh file path (must be .msh). The extension can be omitted, and it will be automatically added, e.g. "mesh" will be treated as "mesh.msh". Other extensions will raise an error.
    - offset_selection: Str, boolean expression to identify simplicial complex to offset (e.g. "tag_0 & tag_1"). If one pure
        tag given (e.g. "tag_0"), single body mode is used.
    - offset_in: Bool, only relevant if single body mode. If True, create offset inwards
    - offset_out: Bool, only relevant if single body mode. If True, create offset outwards (at least
        one of offset_in and offset_out must be true in single body mode)
    - offset_output_tags: List[Str], tag set for the newly createad offset volume (e.g. ["newtag", "newtag2"]).
    - overwrite_tags: Bool, if True then offset_output_tags will overwrite tags in offset region
    - protected_tags: List[Str], only relevant if overwrite_tags=True. Set of tags that will never be overwritten.
    - target_distance: Float, the target distance for the offset. If <= 0, midpoint splitting is used to create the offset.
    - rel_ball_threshold: Float, circle/sphere radius relative to target_distance to terminate conservative inside checks.
        (NOTE: for 3D, anything above 0.01 is not very effective. <0.01 is recommended)
    - output: Output file path for the modified mesh, without extension (e.g. "out" will generate "out.msh")
    - others: Additional parameters (optional).
    """

    mesh = _ensure_msh_extension(mesh)

    j = {}
    if "edge_search_termination_len" not in j.keys():
        j["edge_search_termination_len"] = min(1e-6, target_distance*1e-3)

    j["application"] = "topological_offset"
    j["input"] = mesh
    j["offset_selection"] = offset_selection
    j["offset_in"] = offset_in
    j["offset_out"] = offset_out
    j["offset_output_tags"] = offset_output_tags
    j["overwrite_tags"] = overwrite_tags
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

    _ensure_output_dir(output)
    wildmeshing(j)


# ---------------------------------------------------------------------------
# Minimum separation
# ---------------------------------------------------------------------------
# `ms.run(cfg)` (from `minimum_separation` at image-simulation/) is NOT a
# WMTK op — it drives polyfem to push specified bodies apart by a target
# distance. Typically chained after the WMTK ops above as the next pipeline
# stage (see example_py/aeropress/aeropress.py).
#
# Minimum cfg fields:
#   input_msh        : path to .msh
#   sep              : target minimum separation distance (mesh units)
#   collision_pairs  : list of [side_A, side_B] pairs; each side is either a
#                      list of tag names (id auto-assigned) or
#                      {"oriented_tagset": [...], "id": int}. The first tag
#                      in each oriented_tagset is the body interior — face
#                      normals point from interior outward. Reuse the same
#                      `id` across pairs to mark the same collision body. Specify explicity `id`
#                      when the same selection is used in multiple pairs (e.g. A-B and A-C) to
#                      ensure consistent collision body ids across pairs.
#
# Common optional fields:
#   scale            : mesh-to-solver unit scale (default 0.001)
#   useFitting       : add fitting penalty (default True)
#   useLaplacian     : add Laplacian smoothness penalty (default False)
#   normalizePenalties : normalize penalty matrices (default False)
#   amips_weights    : {"tag_name": float} — per-tag AMIPS weight overrides
#   rtol             : relative tolerance for separation convergence
#   output_msh       : path for the deformed mesh (default <stem>_separated.msh)
#
# Run with `ms.run(cfg, out_dir=Path("output/sep"))`. Outputs in `out_dir`:
#   interface_collision.obj     — collision proxy mesh
#   collision_body_ids.txt      — per-triangle body id lists
#   interface_constraint.hdf5   — fitting constraint
#   interface_constraint_laplacian.hdf5 — Laplacian smoothness
#   <output_msh>                — deformed .msh


# ---------------------------------------------------------------------------
# Laplacian smoothing
# ---------------------------------------------------------------------------
# `ms.run_smoothing(cfg)` (also from `minimum_separation`) is the
# smoothing-only sibling of `ms.run`: a single polyfem solve that fairs
# material-interface manifolds with AMIPS + fitting + Laplacian. No contact,
# no `sep`, no dhat loop. Use for surface fairing across per-tag boundaries
# (see example_py/aeropress/aeropress.py stage -1).
#
# Required cfg fields:
#   input_msh        : path to .msh
#
# Common optional fields:
#   useLaplacian     : add Laplacian smoothness penalty (default False;
#                      turn ON for "laplacian smoothing")
#   useFitting       : fitting penalty anchors interface vertices near rest
#                      pose (default True; set False to let the smoother
#                      move them freely)
#   weight_laplacian : soft-constraint weight on Laplacian (default 1)
#   weight_fitting   : soft-constraint weight on fitting (default 1)
#   normalizePenalties : normalize penalty matrices (default False)
#   useGraphLaplacian : use combinatorial Laplacian instead of geometric
#                       (default False)
#   amips_weights    : {"tag_name": float} — per-tag AMIPS weight overrides
#   scale            : mesh-to-solver unit scale (default 0.001)
#   output_msh       : path for the deformed mesh (default <stem>_smoothed.msh)
#   save_vtu         : emit paraview .vtu/.pvd output (slow; default False)
#   smoothDisplacementsOrPositions : 0 = displacement-mode (L·u=0), 1 =
#                       positions-mode (L·x=0, RHS pulls toward rest).
#                       Forced to 1 by default — displacement-mode is
#                       already at zero gradient at rest and does nothing.
#
# Optional scoping:
#   interfaces : flat list of oriented_tagsets scoping which manifolds are
#                smoothed. Each entry is a list of tag names or
#                {"oriented_tagset": [...]}. First tag is the interior; all
#                selected face sets are unioned into one triangle patch.
#                If omitted, every material interface in the mesh is
#                smoothed (legacy auto-detect path).
#
# Ignored (separation-only fields print a warning if passed): sep, init_dhat,
#   max_iterations, rtol, barrier_stiffness, alpha_n, alpha_t,
#   use_rest_shape_measure, use_adaptive_dhat. contact_enabled is forced False.
#
# Run with `ms.run_smoothing(cfg, out_dir=Path("output/smooth"))`. Outputs:
#   interface_constraint.hdf5            — fitting (when useFitting=True)
#   interface_constraint_laplacian.hdf5  — Laplacian smoothness
#   smoothing.json                       — polyfem JSON
#   solution.txt                         — per-node displacement
#   <output_msh>                         — deformed .msh
#   smooth_output/                       — polyfem run dir (paraview when save_vtu)


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

        # resolve intersections
        mesh = "m1_2d.msh"
        tags = ["tag_1 & tag_2"]
        resolve_intersections(mesh=mesh, tags=tags, output="m2_2d")

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
        topological_offset(mesh=mesh, offset_tags=[[1], [2, 3, 4]], offset_output_tag=[5],
                           target_distance=0.1, rel_ball_threshold=0.001, output="m7_2d")

        # # minimum separation (not a WMTK op — see docstring above the __main__ block)
        # from pathlib import Path
        # import minimum_separation as ms
        # ms_cfg = {
        #     "polyfem": "/path/to/PolyFEM_bin",
        #     "input_msh": "m6.msh",
        #     "collision_pairs": [
        #         [
        #             {"oriented_tagset": ["tag_0", "ambient"], "id": 1},
        #             {"oriented_tagset": ["tag_1", "ambient"], "id": 2},
        #         ],
        #     ],
        #     "sep": 5e-5,
        #     "useFitting": True,
        #     "useLaplacian": True,
        #     "normalizePenalties": True,
        #     "amips_weights": {"ambient": 1e-6, "tag_0": 1, "tag_1": 1},
        #     "scale": 0.001,
        #     "rtol": 1e-1,
        #     "output_msh": "m7.msh",
        # }
        # ms.run(ms_cfg, out_dir=Path("output/sep"))

        # laplacian smoothing (not a WMTK op — see docstring above the __main__ block)
        # Strip the WMTK envelope group before polyfem (it duplicates surface
        # vertices and confuses MeshNodes), then smooth the tag_0 ↔ tag_1
        # intersection surface produced by resolve_intersections above.
        # from pathlib import Path
        # import sys
        # sys.path.append('..')
        # import minimum_separation as ms
        # strip_envelope("m1.msh", output_path="m1_strip.msh")
        # smooth_cfg = {
        #     "polyfem": "/Users/uday/Research/simwild/polyfem/build/PolyFEM_bin",
        #     "input_msh": "m1_strip.msh",
        #     "useFitting": True,
        #     "useLaplacian": True,
        #     "weight_laplacian": 1e3,
        #     "scale": 1,
        #     "save_vtu": True,
        #     # Optional: scope which manifolds get smoothed (omit to smooth all).
        #     "interfaces": [
        #         ["tag_0", "tag_1"]
        #     ],
        # }
        # ms.run_smoothing(smooth_cfg, out_dir=Path("output/smooth"))

    else:
        print("Running 3D operations...")

        # surf_to_tet — two unit spheres, second translated 0.3 in x so they overlap.
        meshes = ["sphere.obj", "sphere.obj"]
        others = {}
        others["input_transform"] = [
            [], [[1, 0, 0, 0.3], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]]
        surf_to_tet(meshes=meshes, output="m1",
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
        topological_offset(mesh=mesh, offset_tags=[[0], [1]], offset_output_tag=[2],
                           target_distance=0.1, rel_ball_threshold=0.001, output="m6")

        # manifold extraction
        mesh = "m3.msh"
        manifold_extraction(mesh=mesh, in_tag=[
                            0, 1], union=False, replace_tag=[], output="m7")

        # tag priority
        mesh = "m1.msh"
        tags = ["tag_1", "tag_0"]
        tag_priority(mesh=mesh, tags=tags, output="m8")
