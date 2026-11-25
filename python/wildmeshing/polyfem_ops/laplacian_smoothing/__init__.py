"""laplacian_smoothing — fair material interfaces with a single polyfem
solve (AMIPS + fitting + Laplacian; no contact, no iteration).

Options: spec.json here. laplacian_smoothing(options) runs on a .msh through
PolyFEM_bin; smooth(options, meshes) runs on a mesh dictionary in this process
(a wildmeshing built with WMTK_WITH_POLYFEM), as a component's memory
interface does. run(cfg) is the engine both use, on engine keys.
Selections use the region/filter format from wildmeshing.polyfem_ops.mesh_core.
"""

import os
from pathlib import Path

import numpy as np

from .. import spec as _spec
from ..constraints import interface_resources
from ..mesh_core import TaggedMesh, assign_selection_ids
from ..polyfem_utils import (OPT_DEFAULTS, build_polyfem_json,
                             get_mesh_info, mesh_group_resources, polyfem_bin,
                             reduced_mesh,
                             step_make_interface_constraint,
                             step_run_polyfem_single, step_write_deformed_msh,
                             _resolve_amips_weights,
                             _write_polyfem_reduced_msh)


_SMOOTHING_IGNORED_KEYS = (
    "sep", "init_dhat", "rtol",
    "barrier_stiffness", "alpha_n", "alpha_t",
    "use_adaptive_dhat",
)


def _engine_cfg(p: dict) -> dict:
    """The engine keys run() reads, from validated options (spec.json); as the
    pywmtk wrapper translated them before #1121."""
    cfg = {
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
    }
    if p["interfaces"]:
        cfg["interfaces"] = p["interfaces"]
    return cfg


def laplacian_smoothing(options: dict) -> dict:
    """Smooth the .msh options["input"] through PolyFEM_bin ($POLYFEM_BIN);
    writes <options["output"]>.msh, the artifacts next to it. Options:
    spec.json. Returns the validated options."""
    p = _spec.validate(_spec.load_spec("laplacian_smoothing"), options)
    cfg = dict(_engine_cfg(p), input_msh=p["input"], output_msh=f"{p['output']}.msh")
    run(cfg, out_dir=Path(os.path.dirname(p["output"]) or "."))
    return p


# The name the smoothing JSON's geometry gives the mesh smooth() hands PolyFEM.
_MESH = "smoothing_mesh"


def smooth(options: dict, meshes: dict) -> tuple:
    """Smooth the tagged mesh dictionary meshes[options["input"]] in this
    process (a wildmeshing built with WMTK_WITH_POLYFEM), as run() smooths its
    .msh: the same constraints, reduced mesh and PolyFEM JSON, from memory.
    Options: spec.json; "output" is unused. Returns ({"mesh": the input mesh,
    its vertices moved}, report), as a component's memory interface; writes
    no files."""
    from ... import _core
    if not hasattr(_core, "polyfem_simulate"):
        raise RuntimeError("laplacian_smoothing.smooth: this wildmeshing is built without "
                           "PolyFEM (WMTK_WITH_POLYFEM); use laplacian_smoothing()")
    p = _spec.validate(_spec.load_spec("laplacian_smoothing"), options)
    if p["save_vtu"]:
        raise ValueError("laplacian_smoothing.smooth: save_vtu writes files; smooth writes none")
    if p["input"] not in meshes:
        raise KeyError(f"laplacian_smoothing.smooth: no mesh '{p['input']}' in meshes")
    mesh = meshes[p["input"]]
    cfg, interfaces_norm = _prepare(_engine_cfg(p))
    scale = cfg.get("scale", OPT_DEFAULTS["scale"])

    tagged = TaggedMesh.from_mesh(mesh)
    constraints, constraint_attributes = interface_resources(
        tagged, selections=interfaces_norm if interfaces_norm else None,
        use_graph=cfg.get("useGraphLaplacian", OPT_DEFAULTS["useGraphLaplacian"]),
        normalize=cfg.get("normalizePenalties", OPT_DEFAULTS["normalizePenalties"]),
        scale=scale,
        smooth_positions=cfg.get("smoothDisplacementsOrPositions", 1) == 1)
    group, material_tags, name_to_tag, tag_to_count, tag_to_volume = reduced_mesh(
        tagged, cfg.get("ambient_like_tags", []))
    cfg["amips_weights"] = _resolve_amips_weights(cfg)
    # out_dir "": the constraints' names are the files' names, here resources.
    sim_json = build_polyfem_json(
        cfg, Path(_MESH), Path(""), material_tags, tagged.mesh_dim, Path(""),
        name_to_tag=name_to_tag, tag_to_count=tag_to_count, tag_to_volume=tag_to_volume)
    sim_json["geometry"][0]["mesh"] = _MESH
    sim_json["output"] = {"advanced": {"save_time_sequence": False}}

    resources, attributes = mesh_group_resources(_MESH, group)
    resources.update(constraints)
    attributes.update(constraint_attributes)
    u = _core.polyfem_simulate(sim_json, resources, attributes)["displacement"]

    V = np.array(mesh["V"], dtype=np.float64)
    V[:, :tagged.mesh_dim] += u / scale          # solver units -> mesh units
    return {"mesh": dict(mesh, V=V)}, {"max_displacement": float(np.abs(u).max() / scale)}


def _prepare(cfg: dict) -> tuple:
    """run()'s smoothing settings on engine keys: (cfg, normalized interfaces)."""
    cfg = dict(cfg)
    ignored_present = [k for k in _SMOOTHING_IGNORED_KEYS if k in cfg]
    if ignored_present:
        print(f"[run_smoothing] ignoring separation-only cfg keys: {ignored_present}")
    if cfg.get("contact_enabled"):
        print("[run_smoothing] forcing contact_enabled=False (smoothing mode)")
    cfg["contact_enabled"] = False
    # In smoothing there's no outer dhat loop, so `max_iterations` is unambiguous —
    # alias it to `nl_max_iterations` for the polyfem nonlinear solver.
    if "max_iterations" in cfg and "nl_max_iterations" not in cfg:
        cfg["nl_max_iterations"] = cfg["max_iterations"]

    # With volume-normalized AMIPS the weights share the Laplacian's
    # currency; smoothing wants the fairing to win, so the body's
    # element-quality guard defaults low (pre-normalization it was
    # effectively ~1e-9 relative to the Laplacian).
    if "amips_body_weight" not in cfg and not any(
            k != "ambient" for k in (cfg.get("amips_weights") or {})):
        cfg["amips_body_weight"] = 1e-4

    interfaces_norm, _ = assign_selection_ids(cfg.get("interfaces", []))
    return cfg, interfaces_norm


def run(cfg: dict, out_dir=None) -> None:
    """Smooth material interfaces with a single polyfem solve — AMIPS +
    fitting + Laplacian, no contact, no dhat loop; for body-body separation
    use run(). See the module docstring for cfg fields. `interfaces` scopes
    the smoothed manifolds via the boundary-of-S rule (face selected iff
    (A ∪ B) ⊇ S AND NOT (A ⊇ S AND B ⊇ S)), unioned into one triangle patch;
    `max_iterations`/`nl_max_iterations` both cap the nonlinear solver
    (default 1000). Writes the generated polyfem inputs (smoothing.json,
    constraint hdf5s, reduced mesh) to out_dir/smooth_input, polyfem output
    (logs, solution.txt) to out_dir/smooth_output, and the deformed mesh to
    output_msh (default <stem>_smoothed.msh); out_dir defaults to the mesh's
    folder.
    """
    cfg, interfaces_norm = _prepare(cfg)

    msh_path = Path(cfg["input_msh"]).resolve()
    polyfem = polyfem_bin()
    if out_dir is None:
        out_dir = msh_path.parent
    out_dir = Path(out_dir).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    scale = cfg.get("scale", OPT_DEFAULTS["scale"])
    use_graph = cfg.get("useGraphLaplacian", OPT_DEFAULTS["useGraphLaplacian"])
    normalize = cfg.get("normalizePenalties", OPT_DEFAULTS["normalizePenalties"])
    # Default to positions mode in smoothing: in displacement mode (L·u=0) the
    # rest configuration is already a minimum and the solver terminates with
    # zero gradient, so nothing happens. Positions mode (L·x=0, b=-L·X_rest)
    # makes the Laplacian actually do work.
    smooth_positions = cfg.get("smoothDisplacementsOrPositions", 1) == 1

    output_msh = Path(cfg.get(
        "output_msh", str(out_dir / (msh_path.stem + "_smoothed.msh"))
    )).resolve()
    sim_in_dir = (out_dir / "smooth_input").resolve()
    sim_out_dir = (out_dir / "smooth_output").resolve()
    sim_json_path = (sim_in_dir / "smoothing.json").resolve()
    sol_path = (sim_out_dir / "solution.txt").resolve()

    print(f"Input  : {msh_path}")
    print(f"Output : {output_msh}")
    print(f"scale={scale}")

    sim_in_dir.mkdir(parents=True, exist_ok=True)
    sim_out_dir.mkdir(parents=True, exist_ok=True)

    # Interface constraint uses the ORIGINAL mesh — we want the multi-tag
    # info (e.g. {tag_0, tag_1} overlap tets) preserved so the boundary-of-S
    # selector picks the right faces. Generated polyfem inputs go to
    # smooth_input/.
    step_make_interface_constraint(
        str(msh_path), sim_in_dir, use_graph, normalize, scale,
        selections=interfaces_norm if interfaces_norm else None,
        smooth_positions=smooth_positions,
        skip_collision_artifacts=True,
    )

    # WMTK's `write_msh_groups` writes a separate copy of every multi-tagged
    # tet for each tag in its tag-set. Polyfem reads these as distinct
    # elements, double-counting AMIPS at shared vertices and corrupting
    # assembly (segfault during constraint setup). Reduce to a 2-body mesh.
    polyfem_msh_path = (sim_in_dir / (msh_path.stem + "_polyfem.msh")).resolve()
    print(f"\n[reduce mesh for polyfem]")
    _write_polyfem_reduced_msh(str(msh_path), str(polyfem_msh_path),
                               ambient_like_tags=cfg.get("ambient_like_tags", []))

    material_tags, mesh_dim, name_to_tag, tag_to_count, tag_to_volume = \
        get_mesh_info(str(polyfem_msh_path))
    print(f"\nReduced material tags : {material_tags}  dim={mesh_dim}")
    print(f"Group name → tag map: {name_to_tag}")
    print(f"Element counts per tag: {tag_to_count}")

    # The reduced mesh's two phys groups are "ambient" and "body" — the
    # existing `build_polyfem_json` looks up amips weights by group name,
    # so we just override `cfg["amips_weights"]` to that 2-key dict.
    cfg["amips_weights"] = _resolve_amips_weights(cfg)

    sim_json = build_polyfem_json(
        cfg, polyfem_msh_path, sim_in_dir, material_tags, mesh_dim, sol_path,
        name_to_tag=name_to_tag, tag_to_count=tag_to_count,
        tag_to_volume=tag_to_volume)

    step_run_polyfem_single(polyfem, sim_json, sim_json_path, sim_out_dir)

    print("\n[write deformed msh]")
    # Apply displacements to the ORIGINAL mesh — preserves the user's full
    # tag set on output. Node tags match between original and reduced (we
    # kept all of them in the reduce step), so solution.txt indexes
    # consistently against either.
    step_write_deformed_msh(msh_path, sol_path, output_msh, scale)


