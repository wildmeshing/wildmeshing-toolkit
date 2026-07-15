"""minimum_separation — push collision bodies apart to a target separation
(AMIPS + fitting + Laplacian + GCP contact). Two outer-loop strategies via
cfg["strategy"]: "dhat" (default) ramps dhat multiplicatively from the
measured geometric gap (next dhat = dhat_growth * active) at fixed barrier
stiffness, binary-searching on overshoot; "stiffness" (experimental) pins
dhat at sep*(1+rtol) and raises the barrier stiffness until the separation
clears sep.

cfg reference: see spec.json here and the simwild.minimum_separation op.
Selections use the region/filter format from simwild.polyfem_ops.mesh_core.

Usage: python -m simwild.polyfem_ops.minimum_separation config.json
"""

import argparse
import json
from pathlib import Path

import numpy as np

from ..constraints import write_pin_constraint_hdf5
from ..mesh_core import TaggedMesh, assign_selection_ids, select_region_nodes
from ..polyfem_utils import (OPT_DEFAULTS, build_polyfem_json,
                             check_polyfem_success, get_mesh_info,
                             polyfem_bin, run_streaming,
                             step_make_interface_constraint,
                             step_write_deformed_msh,
                             _resolve_amips_weights,
                             _write_polyfem_reduced_msh)


def step_run_polyfem(polyfem_bin: str, sep_json: dict, sep_json_path: Path,
                     sim_out_dir: Path, cfg: dict) -> None:
    """Iterate polyfem solves, ramping dhat until `sep` is reached: each
    solve's "active distance" is parsed from stdout; undershoot commits the
    state (warm start) and sets the next dhat to dhat_growth * active
    (default 1.9 — the measured gap then sits at x = active/dhat ~ 0.53,
    i.e. on the exact (4/3)(1-x)^3 outer branch of the GCP barrier, and the
    gap grows multiplicatively); overshoot rolls back and halves the step
    (binary search between the last undershoot dhat and the overshooting
    one). The smallest overshooting dhat is kept as a bracket upper bound:
    later undershoots bisect toward it instead of letting the growth rule
    jump past a dhat already known to overshoot. Unless cfg["init_dhat"] is given, the initial gap is measured by
    a zero-stiffness probe solve (nothing moves; polyfem reports the gap as
    "active distance") and the ramp starts at growth*gap0, with the
    overshoot line search anchored at gap0 (where the barrier exerts no
    force). Fixed-stiffness ceiling (progress stalls because the penalties
    balance the barrier) is warned about once. Mutates sep_json per
    iteration, logs each solve to sim_out_dir/polyfem_iter_<i>.log, and
    leaves the accepted solve's solution.txt for the caller. Raises
    RuntimeError if a solve fails."""
    sim_out_dir.mkdir(parents=True, exist_ok=True)
    # a shorter rerun must not leave stale iteration logs from a longer one
    for stale in sim_out_dir.glob("polyfem_*.log"):
        stale.unlink()

    active_dist = -np.inf
    sep = cfg["sep"]
    growth = cfg.get("dhat_growth", OPT_DEFAULTS["dhat_growth"])
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
    prev_active = None
    stall_warned = False
    dhat_high = None  # smallest dhat known to overshoot (bracket upper bound)

    if "init_dhat" not in cfg:
        # Zero-stiffness probe: no contact force, so nothing moves and the
        # reported "active distance" IS the initial gap (measured through
        # the same collision proxy the real solves use). Seed the ramp at
        # growth*gap0 and anchor the overshoot line search at gap0, where
        # the barrier exerts no force — a first-step overshoot then halves
        # back toward the measured gap instead of tripping the assert.
        probe_json = json.loads(json.dumps(sep_json))
        probe_json["solver"]["contact"]["barrier_stiffness"] = 0.0
        sep_json_path.write_text(json.dumps(probe_json, indent=4))
        returncode, stdout_lines = run_streaming(
            [polyfem_bin, "-j", str(sep_json_path), "-o", str(sim_out_dir)],
            log_path=sim_out_dir / "polyfem_probe.log")
        check_polyfem_success(returncode, stdout_lines, allow_out_of_iterations=True)
        gap_line = next((l for l in reversed(stdout_lines) if "active distance:" in l), None)
        gap0 = (float(gap_line.split("active distance:")[-1].split()[0].rstrip(',;').replace(",", ""))
                if gap_line is not None else np.inf)
        if not (gap0 < sep):
            print(f"Probe: initial gap "
                  f"{'not within dhat' if gap_line is None else f'{gap0:.6e}'}"
                  f" >= sep {sep:.6e} — already separated. Stopping.")
            curr_state_path.unlink(missing_ok=True)
            prev_state_path.unlink(missing_ok=True)
            return
        print(f"Probe: initial gap {gap0:.6e}")
        committed_dhat = gap0
        line_search_step = (growth - 1.0) * gap0
        prev_active = gap0
        curr_json["contact"]["dhat"] = growth * gap0
        print(f"Starting dhat ramp at {curr_json['contact']['dhat']:.6e}")

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
            dhat_high = min(dhat_high or np.inf, curr_json["contact"]["dhat"])
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
            if (prev_active is not None and active_dist < prev_active * 1.01
                    and not stall_warned):
                print("Separation grew <1% this iteration — likely at the fixed-"
                      "stiffness ceiling (penalties balance the barrier); consider "
                      "raising barrier_stiffness or strategy=\"stiffness\".")
                stall_warned = True
            prev_active = active_dist
            if dhat_high is not None and dhat_high - committed_dhat < 1e-3 * dhat_high:
                # bracket collapsed but still undershooting: the state has
                # drifted since the overshoot was recorded — drop the bound
                dhat_high = None
            dhat_next = growth * active_dist
            if dhat_high is not None and dhat_next >= dhat_high:
                # a dhat >= dhat_high is already known to overshoot: bisect
                # the bracket instead of re-discovering it
                dhat_next = 0.5 * (committed_dhat + dhat_high)
            line_search_step = dhat_next - committed_dhat
            curr_json["contact"]["dhat"] = committed_dhat + line_search_step * alpha
            print(f"Updated dhat to {curr_json['contact']['dhat']:.6e} for next iteration")

    if iter == cfg.get("max_iterations", OPT_DEFAULTS["max_iterations"]) - 1:
        print(f"Reached maximum iterations ({iter + 1}) without achieving desired separation. Final active distance: {active_dist:.6e}")

    curr_state_path.unlink(missing_ok=True)
    prev_state_path.unlink(missing_ok=True)


def step_run_polyfem_stiffness(polyfem_bin: str, sep_json: dict,
                               sep_json_path: Path, sim_out_dir: Path,
                               cfg: dict) -> None:
    """strategy="stiffness": pin dhat at sep*(1+rtol) and raise the barrier
    stiffness kappa until the active distance clears sep. The contact force
    vanishes at distances >= dhat, so the equilibrium can never exceed dhat —
    the rtol overshoot bound holds by construction and no rollback is needed
    (raising kappa only pushes the distance up, monotonically).

    Update rule: near dhat the barrier force scales like
    kappa * (dhat - d)^2 / dhat, and the penalty force resisting it is
    locally ~constant, so the deficit delta = dhat - active follows
    delta ~ kappa^(-1/2). Each iteration solves that power law for the kappa
    that lands at half the tolerance band (delta = (dhat - sep)/2); once two
    solves exist the exponent is re-fit from them in log-log (the GCP
    potential may not vanish exactly quadratically at dhat). The multiplier
    is clamped to max_stiffness_multiplier per step to protect Newton
    conditioning. Same warm-start/state/log conventions as
    step_run_polyfem."""
    sim_out_dir.mkdir(parents=True, exist_ok=True)
    for stale in sim_out_dir.glob("polyfem_*.log"):
        stale.unlink()

    active_dist = -np.inf
    sep = cfg["sep"]
    rtol = cfg.get("rtol", OPT_DEFAULTS["rtol"])
    dhat = sep * (1.0 + rtol)
    kappa = cfg.get("barrier_stiffness", OPT_DEFAULTS["barrier_stiffness"])
    max_mult = cfg.get("max_stiffness_multiplier", 100.0)

    sep_json["contact"]["dhat"] = dhat

    curr_json = sep_json.copy()
    curr_state_path = sim_out_dir / "curr_state.hdf5"
    prev_state_path = sim_out_dir / "prev_state.hdf5"
    curr_state_path.unlink(missing_ok=True)
    prev_state_path.unlink(missing_ok=True)
    curr_json["output"]["data"]["state"] = str(curr_state_path)
    prev_kappa, prev_deficit = None, None
    for iter in range(cfg.get("max_iterations", OPT_DEFAULTS["max_iterations"])):
        if prev_state_path.exists():
            curr_json["input"]["data"]["state"] = str(prev_state_path)
        curr_json["solver"]["contact"]["barrier_stiffness"] = kappa
        sep_json_path.write_text(json.dumps(curr_json, indent=4))

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

        print(f"Current active distance: {active_dist:.6e}  (kappa={kappa:.6e})")

        if (active_dist == np.inf):
            print("Active distance is infinite — no contact. Stopping.")
            break
        if active_dist >= sep:
            print(f"Desired separation achieved (active distance {active_dist:.6e} >= {sep:.6e}, "
                  f"bounded above by dhat={dhat:.6e}). Stopping.")
            break

        # Always commit: the distance is monotone in kappa, no rollback.
        prev_state_path.unlink(missing_ok=True)
        curr_state_path.rename(prev_state_path)

        deficit = dhat - active_dist          # > dhat - sep since active < sep
        target = 0.5 * (dhat - sep)           # aim mid-tolerance for margin
        exponent = -0.5                       # theory: force ~ kappa*delta^2
        if prev_deficit is not None and deficit < prev_deficit and kappa > prev_kappa:
            measured = np.log(deficit / prev_deficit) / np.log(kappa / prev_kappa)
            if measured < -1e-3:
                exponent = measured
        prev_kappa, prev_deficit = kappa, deficit
        mult = min((target / deficit) ** (1.0 / exponent), max_mult)
        kappa *= mult
        print(f"Deficit {deficit:.6e} (tolerance {dhat - sep:.6e}, exponent {exponent:.3f}); "
              f"raising barrier stiffness x{mult:.3g} to {kappa:.6e}")

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
    fields. Writes the generated polyfem inputs (separation.json, constraint
    hdf5s, collision proxy, reduced mesh) to out_dir/sep_input, polyfem
    outputs (logs, solution.txt) to out_dir/sep_output, and the deformed
    mesh to output_msh (default <stem>_separated.msh); out_dir defaults to
    the mesh's folder.
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

    strategy = cfg.get("strategy", "dhat")
    if cfg.get("barrier_stiffness", -1.0) <= 0:
        # auto (<=0/unset): the dhat ramp never adjusts kappa, and too-soft
        # stalls at its fixed point, so it starts generous; the stiffness
        # loop only ever raises kappa, and a soft start does the bulk of the
        # displacement on a cheap landscape (measured 3x fewer total Newton
        # iterations at tight rtol) before the capped power-law jumps
        # stiffen the endgame.
        cfg["barrier_stiffness"] = (
            1.0 if strategy == "stiffness" else OPT_DEFAULTS["barrier_stiffness"])

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
    sim_in_dir = (out_dir / "sep_input").resolve()
    sim_out_dir = (out_dir / "sep_output").resolve()
    sep_json_path = (sim_in_dir / "separation.json").resolve()
    sol_path = (sim_out_dir / "solution.txt").resolve()

    print(f"Input  : {msh_path}")
    print(f"Output : {output_msh}")
    print(f"separation={cfg['sep']}  scale={scale}")

    sim_in_dir.mkdir(parents=True, exist_ok=True)
    sim_out_dir.mkdir(parents=True, exist_ok=True)

    # Interface extraction uses the ORIGINAL mesh — needs the multi-tag
    # info to identify body-body manifolds for the collision proxy. All
    # generated polyfem inputs (constraint hdf5s, collision proxy, reduced
    # mesh) go to sep_input/.
    step_make_interface_constraint(
        str(msh_path), sim_in_dir, use_graph, normalize, scale,
        selections=sides_flat if sides_flat else None,
        smooth_positions=smooth_positions,
    )

    # Same WMTK-duplication issue as smoothing — polyfem reads each
    # multi-tag tet as two separate elements and breaks. Reduce to a 2-body
    # mesh for the volumetric solve. Collision filtering still works because
    # `contact.collision_pairs` + `collision_body_ids.txt` operate on the
    # collision proxy mesh (vertex/face level, not volumetric body ids).
    polyfem_msh_path = (sim_in_dir / (msh_path.stem + "_polyfem.msh")).resolve()
    print(f"\n[reduce mesh for polyfem]")
    _write_polyfem_reduced_msh(str(msh_path), str(polyfem_msh_path),
                               ambient_like_tags=cfg.get("ambient_like_tags", []))

    material_tags, mesh_dim, name_to_tag, tag_to_count, tag_to_volume = \
        get_mesh_info(str(polyfem_msh_path))
    print(f"\nReduced material tags : {material_tags}  dim={mesh_dim}")
    print(f"Group name → tag map: {name_to_tag}")
    print(f"Element counts per tag: {tag_to_count}")

    cfg["amips_weights"] = _resolve_amips_weights(cfg)

    sep_json = build_polyfem_json(
        cfg, polyfem_msh_path, sim_in_dir, material_tags, mesh_dim, sol_path,
        name_to_tag=name_to_tag, tag_to_count=tag_to_count,
        tag_to_volume=tag_to_volume)

    # protected_regions: pin every node of the matching cells exactly at
    # rest (hard constraint via augmented Lagrangian) so the contact pushes
    # only the unprotected side and protected geometry stays bit-identical.
    protected = cfg.get("protected_regions", [])
    if protected:
        pin_ids = select_region_nodes(TaggedMesh(str(msh_path)), protected)
        pin_path = (sim_in_dir / "protected_pin.hdf5").resolve()
        write_pin_constraint_hdf5(str(pin_path), pin_ids, mesh_dim)
        sep_json.setdefault("constraints", {})["hard"] = [str(pin_path)]

    if strategy == "dhat":
        step_run_polyfem(polyfem, sep_json, sep_json_path, sim_out_dir, cfg)
    elif strategy == "stiffness":
        step_run_polyfem_stiffness(polyfem, sep_json, sep_json_path,
                                   sim_out_dir, cfg)
    else:
        raise ValueError(f'strategy must be "dhat" or "stiffness", got {strategy!r}')

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
