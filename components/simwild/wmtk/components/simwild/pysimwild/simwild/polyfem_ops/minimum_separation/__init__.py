"""minimum_separation — push collision bodies apart to a target separation
(AMIPS + fitting + Laplacian + GCP contact, with a dhat line-search loop).

cfg reference: see spec.json here and the simwild.minimum_separation op.
Selections use the region/filter format from simwild.polyfem_ops.mesh_core.

Usage: python -m simwild.polyfem_ops.minimum_separation config.json
"""

import argparse
import json
from pathlib import Path

import numpy as np

from ..mesh_core import assign_selection_ids
from ..polyfem_utils import (OPT_DEFAULTS, build_polyfem_json,
                             check_polyfem_success, get_mesh_info,
                             polyfem_bin, run_streaming,
                             step_make_interface_constraint,
                             step_write_deformed_msh,
                             _resolve_amips_weights,
                             _write_polyfem_reduced_msh)


def step_run_polyfem(polyfem_bin: str, sep_json: dict, sep_json_path: Path,
                     sim_out_dir: Path, cfg: dict) -> None:
    """Iterate polyfem solves, line-searching dhat until `sep` is reached:
    each solve's "active distance" is parsed from stdout; undershoot commits
    the state (warm start) and grows dhat by 2*(sep - active), overshoot rolls
    back and halves the step. Mutates sep_json per iteration, logs each solve
    to sim_out_dir/polyfem_iter_<i>.log, and leaves the accepted solve's
    solution.txt for the caller. Raises RuntimeError if a solve fails."""
    sim_out_dir.mkdir(parents=True, exist_ok=True)

    active_dist = -np.inf
    sep = cfg["sep"]
    init_dhat = cfg.get("init_dhat", sep)

    sep_json["contact"]["dhat"] = init_dhat

    curr_json = sep_json.copy()
    curr_state_path = sim_out_dir / "curr_state.hdf5"
    prev_state_path = sim_out_dir / "prev_state.hdf5"
    # Clear stale state from any previous (possibly crashed) run, otherwise
    # the iteration loop would warm-start from prev_state.hdf5 and silently
    # use the wrong initial configuration.
    curr_state_path.unlink(missing_ok=True)
    prev_state_path.unlink(missing_ok=True)
    committed_dhat = init_dhat
    curr_json["output"]["data"]["state"] = str(curr_state_path)
    alpha = 1.0
    line_search_step = None
    for iter in range(cfg.get("max_iterations", OPT_DEFAULTS["max_iterations"])):
        if prev_state_path.exists():
            curr_json["input"]["data"]["state"] = str(prev_state_path)
        sep_json_path.write_text(json.dumps(curr_json, indent=4))

        # Run polyfem through a pty so spdlog keeps its ANSI colors.
        returncode, stdout_lines = run_streaming(
            [polyfem_bin, "-j", str(sep_json_path), "-o", str(sim_out_dir)],
            log_path=sim_out_dir / f"polyfem_iter_{iter}.log")
        check_polyfem_success(
            returncode, stdout_lines,
            allow_out_of_iterations=curr_json.get("solver", {}).get(
                "nonlinear", {}).get("allow_out_of_iterations", False),
        )

        active_dist_line = next((l for l in reversed(stdout_lines) if "active distance:" in l), None)
        if active_dist_line is None:
            print("No active distance found in output — contact not triggered. Stopping.")
            break
        active_dist = float(active_dist_line.split("active distance:")[-1].split()[0].rstrip(',;').replace(",", ""))

        print(f"Current active distance: {active_dist:.6e}")

        if (active_dist == np.inf):
            print("Active distance is infinite — no contact. Stopping.")
            break
        if np.isclose(active_dist, sep, rtol=cfg.get("rtol", OPT_DEFAULTS["rtol"])) and active_dist > sep:
            print(f"Desired separation achieved (active distance {active_dist:.6e} >= {sep:.6e}). Stopping.")
            break
        
        if active_dist > sep:
            # Revert to previous state if we overshot
            assert line_search_step is not None, "Cannot overshoot on first iteration if init_dhat=sep"
            alpha *= 0.5
            curr_json["contact"]["dhat"] = committed_dhat + line_search_step * alpha
            print(f"Overshot target separation. Reverting to previous state. Reducing line search alpha to {alpha:.6f}")
            print(f"Updated dhat to {curr_json['contact']['dhat']:.6e} for next iteration")
            
            continue
        else: 
            prev_state_path.unlink(missing_ok=True)
            curr_state_path.rename(prev_state_path)
            # step is approved
            committed_dhat = curr_json["contact"]["dhat"]
            alpha = 1.0 # reset line search
            line_search_step = (sep - active_dist)*2
            curr_json["contact"]["dhat"] = committed_dhat + line_search_step * alpha
            print(f"Updated dhat to {curr_json['contact']['dhat']:.6e} for next iteration")

    if iter == cfg.get("max_iterations", OPT_DEFAULTS["max_iterations"]) - 1:
        print(f"Reached maximum iterations ({iter + 1}) without achieving desired separation. Final active distance: {active_dist:.6e}")

    curr_state_path.unlink(missing_ok=True)
    prev_state_path.unlink(missing_ok=True)


def _normalize_collision_pairs(raw_pairs: list) -> tuple[list, list]:
    """Turn cfg["collision_pairs"] ([[side_A, side_B], ...], sides per
    mesh_core.normalize_selection) into (unique_selections, polyfem_pairs).
    Identical selections are deduped to one collision body; duplicate id
    pairs collapse."""
    for pair in raw_pairs:
        if not isinstance(pair, (list, tuple)) or len(pair) != 2:
            raise ValueError(
                f"collision_pairs entries must be [side_A, side_B], got {pair!r}")
    flat = [side for pair in raw_pairs for side in pair]
    unique, ids = assign_selection_ids(flat)
    polyfem_pairs, seen = [], set()
    for i in range(len(raw_pairs)):
        pair = [ids[2 * i], ids[2 * i + 1]]
        key = tuple(sorted(pair))
        if key not in seen:
            seen.add(key)
            polyfem_pairs.append(pair)
    return unique, polyfem_pairs


def run(cfg: dict, out_dir=None) -> None:
    """Run the full minimum-separation pipeline: extract interface
    constraints, reduce the mesh for polyfem, iterate the dhat line-search
    solve, and write the deformed mesh. See the module docstring for cfg
    fields. Writes constraint artifacts + separation.json + solution.txt to
    out_dir (default: the mesh's folder), polyfem logs to out_dir/sep_output,
    and the deformed mesh to output_msh (default <stem>_separated.msh).
    """
    cfg = dict(cfg)  # shallow copy — we may mutate

    sides_flat, polyfem_collision_pairs = _normalize_collision_pairs(
        cfg.get("collision_pairs", []))
    cfg["collision_pairs"] = polyfem_collision_pairs

    msh_path = Path(cfg["input_msh"]).resolve()
    polyfem = polyfem_bin()
    if out_dir is None:
        out_dir = msh_path.parent
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    scale = cfg.get("scale", OPT_DEFAULTS["scale"])
    use_graph = cfg.get("useGraphLaplacian", OPT_DEFAULTS["useGraphLaplacian"])
    normalize = cfg.get("normalizePenalties", OPT_DEFAULTS["normalizePenalties"])
    smooth_positions = cfg.get(
        "smoothDisplacementsOrPositions", OPT_DEFAULTS["smoothDisplacementsOrPositions"]
    ) == 1
    out_dir = out_dir.resolve()
    output_msh = Path(cfg.get(
        "output_msh", str(out_dir / (msh_path.stem + "_separated.msh"))
    )).resolve()
    sim_out_dir = (out_dir / "sep_output").resolve()
    sep_json_path = (out_dir / "separation.json").resolve()
    sol_path = (out_dir / "solution.txt").resolve()

    print(f"Input  : {msh_path}")
    print(f"Output : {output_msh}")
    print(f"separation={cfg['sep']}  scale={scale}")

    sim_out_dir.mkdir(parents=True, exist_ok=True)

    # Interface extraction uses the ORIGINAL mesh — needs the multi-tag
    # info to identify body-body manifolds for the collision proxy.
    step_make_interface_constraint(
        str(msh_path), out_dir, use_graph, normalize, scale,
        selections=sides_flat if sides_flat else None,
        smooth_positions=smooth_positions,
    )

    # Same WMTK-duplication issue as smoothing — polyfem reads each
    # multi-tag tet as two separate elements and breaks. Reduce to a 2-body
    # mesh for the volumetric solve. Collision filtering still works because
    # `contact.collision_pairs` + `collision_body_ids.txt` operate on the
    # collision proxy mesh (vertex/face level, not volumetric body ids).
    polyfem_msh_path = (out_dir / (msh_path.stem + "_polyfem.msh")).resolve()
    print(f"\n[reduce mesh for polyfem]")
    _write_polyfem_reduced_msh(str(msh_path), str(polyfem_msh_path))

    material_tags, mesh_dim, name_to_tag, tag_to_count = get_mesh_info(str(polyfem_msh_path))
    print(f"\nReduced material tags : {material_tags}  dim={mesh_dim}")
    print(f"Group name → tag map: {name_to_tag}")
    print(f"Element counts per tag: {tag_to_count}")

    cfg["amips_weights"] = _resolve_amips_weights(cfg)

    sep_json = build_polyfem_json(
        cfg, polyfem_msh_path, out_dir, material_tags, mesh_dim, sol_path,
        name_to_tag=name_to_tag, tag_to_count=tag_to_count)

    step_run_polyfem(polyfem, sep_json, sep_json_path, sim_out_dir, cfg)

    print("\n[write deformed msh]")
    # Apply displacements to the ORIGINAL mesh — preserves the user's full
    # tag set on output. Node tags match between original and reduced so
    # solution.txt indexes consistently against either.
    step_write_deformed_msh(msh_path, sol_path, output_msh, scale)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("config", help="Path to config JSON")
    args = ap.parse_args()

    cfg = json.loads(Path(args.config).read_text())
    out_dir = Path(args.config).parent.resolve()
    run(cfg, out_dir=out_dir)


if __name__ == "__main__":
    main()
