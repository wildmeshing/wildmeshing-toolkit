"""laplacian_smoothing — fair material interfaces with a single polyfem
solve (AMIPS + fitting + Laplacian; no contact, no iteration).

cfg reference: see spec.json here and the simwild.laplacian_smoothing op.
Selections use the region/filter format from simwild.polyfem_ops.mesh_core.
"""

from pathlib import Path

from ..mesh_core import assign_selection_ids
from ..polyfem_utils import (OPT_DEFAULTS, build_polyfem_json,
                             get_mesh_info, polyfem_bin,
                             step_make_interface_constraint,
                             step_run_polyfem_single, step_write_deformed_msh,
                             _resolve_amips_weights,
                             _write_polyfem_reduced_msh)


_SMOOTHING_IGNORED_KEYS = (
    "sep", "init_dhat", "rtol",
    "barrier_stiffness", "alpha_n", "alpha_t",
    "use_rest_shape_measure", "use_adaptive_dhat",
)


def run(cfg: dict, out_dir=None) -> None:
    """Smooth material interfaces with a single polyfem solve — AMIPS +
    fitting + Laplacian, no contact, no dhat loop; for body-body separation
    use run(). See the module docstring for cfg fields. `interfaces` scopes
    the smoothed manifolds via the boundary-of-S rule (face selected iff
    (A ∪ B) ⊇ S AND NOT (A ⊇ S AND B ⊇ S)), unioned into one triangle patch;
    `max_iterations`/`nl_max_iterations` both cap the nonlinear solver
    (default 1000). Writes constraint artifacts + smoothing.json +
    solution.txt to out_dir (default: the mesh's folder), polyfem output to
    out_dir/smooth_output, and the deformed mesh to output_msh (default
    <stem>_smoothed.msh).
    """
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

    interfaces_norm, _ = assign_selection_ids(cfg.get("interfaces", []))

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
    sim_out_dir = (out_dir / "smooth_output").resolve()
    sim_json_path = (out_dir / "smoothing.json").resolve()
    sol_path = (out_dir / "solution.txt").resolve()

    print(f"Input  : {msh_path}")
    print(f"Output : {output_msh}")
    print(f"scale={scale}")

    sim_out_dir.mkdir(parents=True, exist_ok=True)

    # Interface constraint uses the ORIGINAL mesh — we want the multi-tag
    # info (e.g. {tag_0, tag_1} overlap tets) preserved so the boundary-of-S
    # selector picks the right faces.
    step_make_interface_constraint(
        str(msh_path), out_dir, use_graph, normalize, scale,
        selections=interfaces_norm if interfaces_norm else None,
        smooth_positions=smooth_positions,
        skip_collision_artifacts=True,
    )

    # WMTK's `write_msh_groups` writes a separate copy of every multi-tagged
    # tet for each tag in its tag-set. Polyfem reads these as distinct
    # elements, double-counting AMIPS at shared vertices and corrupting
    # assembly (segfault during constraint setup). Reduce to a 2-body mesh.
    polyfem_msh_path = (out_dir / (msh_path.stem + "_polyfem.msh")).resolve()
    print(f"\n[reduce mesh for polyfem]")
    _write_polyfem_reduced_msh(str(msh_path), str(polyfem_msh_path))

    material_tags, mesh_dim, name_to_tag, tag_to_count = get_mesh_info(str(polyfem_msh_path))
    print(f"\nReduced material tags : {material_tags}  dim={mesh_dim}")
    print(f"Group name → tag map: {name_to_tag}")
    print(f"Element counts per tag: {tag_to_count}")

    # The reduced mesh's two phys groups are "ambient" and "body" — the
    # existing `build_polyfem_json` looks up amips weights by group name,
    # so we just override `cfg["amips_weights"]` to that 2-key dict.
    cfg["amips_weights"] = _resolve_amips_weights(cfg)

    sim_json = build_polyfem_json(
        cfg, polyfem_msh_path, out_dir, material_tags, mesh_dim, sol_path,
        name_to_tag=name_to_tag, tag_to_count=tag_to_count)

    step_run_polyfem_single(polyfem, sim_json, sim_json_path, sim_out_dir)

    print("\n[write deformed msh]")
    # Apply displacements to the ORIGINAL mesh — preserves the user's full
    # tag set on output. Node tags match between original and reduced (we
    # kept all of them in the reduce step), so solution.txt indexes
    # consistently against either.
    step_write_deformed_msh(msh_path, sol_path, output_msh, scale)


