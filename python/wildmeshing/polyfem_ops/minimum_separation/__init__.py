"""minimum_separation — push collision bodies apart to a target separation
(AMIPS + fitting + Laplacian + GCP contact). Two outer-loop strategies via
options["strategy"]: "dhat" (default) ramps dhat multiplicatively from the
measured geometric gap (next dhat = dhat_growth * active) at fixed barrier
stiffness, binary-searching on overshoot; "stiffness" (experimental) pins
dhat at sep*(1+rtol) and raises the barrier stiffness until the separation
clears sep.

Options: spec.json here. separate(options, meshes) runs on a mesh dictionary,
as a component's memory interface does; minimum_separation(options) runs on a
.msh. Both solve in this process (a wildmeshing built with WMTK_WITH_POLYFEM):
each outer iteration starts from the last accepted solve, and PolyFEM has no
JSON key for that start in a static solve, so there is no PolyFEM_bin route.
Selections use the region/filter format from wildmeshing.polyfem_ops.mesh_core.

Usage: python -m wildmeshing.polyfem_ops.minimum_separation options.json
"""

import argparse
import copy
import json
import os
from pathlib import Path

import numpy as np

from .. import spec as _spec
from ..constraints import interface_resources, parse_axes, pin_constraint
from ..mesh_core import TaggedMesh, assign_selection_ids, select_region_nodes
from ..polyfem_utils import (OPT_DEFAULTS, build_polyfem_json,
                             check_polyfem_success, mesh_group_resources,
                             reduced_mesh, _resolve_amips_weights)


def _active_distance(log_lines):
    """The active distance of a solve: the last "active distance:" its log
    reports (GCPContactForm), or None when contact never triggered."""
    line = next((l for l in reversed(log_lines) if "active distance:" in l), None)
    if line is None:
        return None
    return float(line.split("active distance:")[-1].split()[0].rstrip(',;').replace(",", ""))


def _ramp_dhat(solve, sep_json: dict, cfg: dict) -> tuple:
    """strategy="dhat": iterate solves, ramping dhat until `sep` is reached.
    Each solve's "active distance" is read from its log; undershoot commits
    the solve (the next one starts from it) and sets the next dhat to
    dhat_growth * active (default 1.9 — the measured gap then sits at
    x = active/dhat ~ 0.53, i.e. on the exact (4/3)(1-x)^3 outer branch of the
    GCP barrier, and the gap grows multiplicatively); overshoot discards the
    solve and halves the step (binary search between the last undershoot dhat
    and the overshooting one). The smallest overshooting dhat is kept as a
    bracket upper bound: later undershoots bisect toward it instead of letting
    the growth rule jump past a dhat already known to overshoot. Unless
    cfg["init_dhat"] is given, the initial gap is measured by a zero-stiffness
    probe solve (nothing moves; polyfem reports the gap as "active distance")
    and the ramp starts at growth*gap0, with the overshoot line search
    anchored at gap0 (where the barrier exerts no force). Fixed-stiffness
    ceiling (progress stalls because the penalties balance the barrier) is
    warned about once.

    solve(json, start) runs one solve from `start` (an earlier solve's
    "solution"; None: the rest pose) and returns _core.polyfem_simulate's
    dictionary. Returns (the last solve's result, its active distance)."""
    sep = cfg["sep"]
    growth = cfg.get("dhat_growth", OPT_DEFAULTS["dhat_growth"])
    init_dhat = cfg.get("init_dhat", sep)
    j = copy.deepcopy(sep_json)
    j["contact"]["dhat"] = init_dhat

    active_dist = -np.inf
    committed = None   # the last accepted solve's solution: where the next one starts
    committed_dhat = init_dhat
    alpha = 1.0
    line_search_step = None
    prev_active = None
    stall_warned = False
    dhat_high = None   # smallest dhat known to overshoot (bracket upper bound)
    result = None

    if "init_dhat" not in cfg:
        # Zero-stiffness probe: no contact force, so nothing moves and the
        # reported "active distance" IS the initial gap (measured through
        # the same collision proxy the real solves use). Seed the ramp at
        # growth*gap0 and anchor the overshoot line search at gap0, where
        # the barrier exerts no force — a first-step overshoot then halves
        # back toward the measured gap instead of tripping the assert.
        probe = copy.deepcopy(j)
        probe["solver"]["contact"]["barrier_stiffness"] = 0.0
        result = solve(probe, None)
        gap0 = _active_distance(result["log"])
        if gap0 is None:
            gap0 = np.inf
        if not (gap0 < sep):
            print(f"Probe: initial gap "
                  f"{'not within dhat' if gap0 == np.inf else f'{gap0:.6e}'}"
                  f" >= sep {sep:.6e} — already separated. Stopping.")
            return result, gap0
        print(f"Probe: initial gap {gap0:.6e}")
        committed_dhat = gap0
        line_search_step = (growth - 1.0) * gap0
        prev_active = gap0
        j["contact"]["dhat"] = growth * gap0
        print(f"Starting dhat ramp at {j['contact']['dhat']:.6e}")

    max_iterations = cfg.get("max_iterations", OPT_DEFAULTS["max_iterations"])
    for _ in range(max_iterations):
        result = solve(j, committed)
        active = _active_distance(result["log"])
        if active is None:
            print("No active distance found in output — contact not triggered. Stopping.")
            break
        active_dist = active
        print(f"Current active distance: {active_dist:.6e}")

        if active_dist == np.inf:
            print("Active distance is infinite — no contact. Stopping.")
            break
        if np.isclose(active_dist, sep, rtol=cfg.get("rtol", OPT_DEFAULTS["rtol"])) and active_dist > sep:
            print(f"Desired separation achieved (active distance {active_dist:.6e} >= {sep:.6e}). Stopping.")
            break

        if active_dist > sep:
            # Overshot: discard this solve, start the next from the last accepted one
            assert line_search_step is not None, "Cannot overshoot on first iteration if init_dhat=sep"
            dhat_high = min(dhat_high or np.inf, j["contact"]["dhat"])
            alpha *= 0.5
            j["contact"]["dhat"] = committed_dhat + line_search_step * alpha
            print(f"Overshot target separation. Reverting to previous state. Reducing line search alpha to {alpha:.6f}")
            print(f"Updated dhat to {j['contact']['dhat']:.6e} for next iteration")
            continue

        # step is approved
        committed = result["solution"]
        committed_dhat = j["contact"]["dhat"]
        alpha = 1.0  # reset line search
        if prev_active is not None and active_dist < prev_active * 1.01 and not stall_warned:
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
        j["contact"]["dhat"] = committed_dhat + line_search_step * alpha
        print(f"Updated dhat to {j['contact']['dhat']:.6e} for next iteration")
    else:
        print(f"Reached maximum iterations ({max_iterations}) without achieving desired "
              f"separation. Final active distance: {active_dist:.6e}")
    return result, active_dist


def _raise_stiffness(solve, sep_json: dict, cfg: dict) -> tuple:
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
    conditioning. solve and the result as in _ramp_dhat."""
    sep = cfg["sep"]
    rtol = cfg.get("rtol", OPT_DEFAULTS["rtol"])
    dhat = sep * (1.0 + rtol)
    kappa = cfg.get("barrier_stiffness", OPT_DEFAULTS["barrier_stiffness"])
    max_mult = cfg.get("max_stiffness_multiplier", 100.0)
    j = copy.deepcopy(sep_json)
    j["contact"]["dhat"] = dhat

    active_dist = -np.inf
    committed = None
    result = None
    prev_kappa, prev_deficit = None, None
    max_iterations = cfg.get("max_iterations", OPT_DEFAULTS["max_iterations"])
    for _ in range(max_iterations):
        j["solver"]["contact"]["barrier_stiffness"] = kappa
        result = solve(j, committed)
        active = _active_distance(result["log"])
        if active is None:
            print("No active distance found in output — contact not triggered. Stopping.")
            break
        active_dist = active
        print(f"Current active distance: {active_dist:.6e}  (kappa={kappa:.6e})")

        if active_dist == np.inf:
            print("Active distance is infinite — no contact. Stopping.")
            break
        if active_dist >= sep:
            print(f"Desired separation achieved (active distance {active_dist:.6e} >= {sep:.6e}, "
                  f"bounded above by dhat={dhat:.6e}). Stopping.")
            break

        # Always commit: the distance is monotone in kappa, no rollback.
        committed = result["solution"]

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
    else:
        print(f"Reached maximum iterations ({max_iterations}) without achieving desired "
              f"separation. Final active distance: {active_dist:.6e}")
    return result, active_dist


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


def _engine_cfg(p: dict) -> dict:
    """The engine keys the loops and build_polyfem_json read, from validated
    options (spec.json); as the pywmtk wrapper translated them before #1121,
    plus the NeoHookean body options added since."""
    cfg = {
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
        "use_nh_body": p["use_nh_body"],
        "nh_youngs": p["nh_youngs"],
        "nh_poisson": p["nh_poisson"],
        "protected_regions": p["protected_regions"],
        "ambient_like_tags": p["ambient_like_tags"],
    }
    if p["init_dhat"] > 0:
        cfg["init_dhat"] = p["init_dhat"]
    return cfg


def _protected_pins(protected: list, tagged: TaggedMesh, dim: int) -> dict:
    """protected_regions as hard constraints: {constraint name: its datasets}.
    Pin every node of the matching cells exactly at rest (hard constraint via
    augmented Lagrangian) so the contact pushes only the unprotected side and
    protected geometry stays bit-identical. Entries are either a bare
    expression (pin every component) or {"region": expr, "axes": "z"} to hold
    only some components, which leaves the region free to slide along the
    others. Regions sharing an axes spec go into one constraint."""
    groups: dict = {}
    for entry in protected:
        if isinstance(entry, dict):
            expr = entry.get("region")
            if expr is None:
                raise ValueError(f"protected_regions entry {entry!r} needs 'region'")
            extra = set(entry) - {"region", "axes"}
            if extra:
                raise ValueError(
                    f"protected_regions entry {entry!r}: unknown key(s) {sorted(extra)}")
            axes = parse_axes(entry.get("axes"), dim)
        else:
            expr, axes = entry, None
        groups.setdefault(axes, []).append(expr)
    pins = {}
    for axes, exprs in groups.items():
        suffix = "" if axes is None else "_" + "".join("xyz"[a] for a in axes)
        data, _ = pin_constraint(select_region_nodes(tagged, exprs), dim, axes=axes)
        pins[f"protected_pin{suffix}.hdf5"] = data
    return pins


# The name the separation JSON's geometry gives the mesh separate() hands PolyFEM.
_MESH = "separation_mesh"


def _inputs(p: dict, mesh: dict) -> tuple:
    """The PolyFEM inputs of a separation from validated options and a mesh
    dictionary: (engine cfg, the separation JSON before its outer loop, its
    resources and attributes for _core.polyfem_simulate, the mesh's
    TaggedMesh). The JSON writes nothing: no solution file, no paraview."""
    cfg = _engine_cfg(p)
    sides_flat, cfg["collision_pairs"] = _normalize_collision_pairs(cfg["collision_pairs"])
    if cfg["barrier_stiffness"] <= 0:
        # auto (<=0/unset): the dhat ramp never adjusts kappa, and too-soft
        # stalls at its fixed point, so it starts generous; the stiffness
        # loop only ever raises kappa, and a soft start does the bulk of the
        # displacement on a cheap landscape (measured 3x fewer total Newton
        # iterations at tight rtol) before the capped power-law jumps
        # stiffen the endgame.
        cfg["barrier_stiffness"] = (
            1.0 if cfg["strategy"] == "stiffness" else OPT_DEFAULTS["barrier_stiffness"])
    scale = cfg["scale"]

    # Interface extraction uses the ORIGINAL mesh — needs the multi-tag info
    # to identify body-body manifolds for the collision proxy.
    tagged = TaggedMesh.from_mesh(mesh)
    resources, attributes = interface_resources(
        tagged, selections=sides_flat if sides_flat else None,
        use_graph=cfg["useGraphLaplacian"], normalize=cfg["normalizePenalties"], scale=scale,
        smooth_positions=cfg.get("smoothDisplacementsOrPositions",
                                 OPT_DEFAULTS["smoothDisplacementsOrPositions"]) == 1,
        collision=True)
    # PolyFEM reads each multi-tag tet as separate elements: the volumetric
    # solve runs on the 2-body reduction. Collision filtering still works because
    # `contact.collision_pairs` + the collision body ids operate on the
    # collision proxy (vertex/face level, not volumetric body ids).
    group, material_tags, name_to_tag, tag_to_count, tag_to_volume = reduced_mesh(
        tagged, cfg["ambient_like_tags"])
    cfg["amips_weights"] = _resolve_amips_weights(cfg)
    # out_dir "": the constraints' names are the files' names, here resources.
    sep_json = build_polyfem_json(
        cfg, Path(_MESH), Path(""), material_tags, tagged.mesh_dim, Path(""),
        name_to_tag=name_to_tag, tag_to_count=tag_to_count, tag_to_volume=tag_to_volume)
    sep_json["geometry"][0]["mesh"] = _MESH
    sep_json["output"] = {"advanced": {"save_time_sequence": False}}
    r, a = mesh_group_resources(_MESH, group)
    resources.update(r)
    attributes.update(a)

    pins = _protected_pins(cfg["protected_regions"], tagged, tagged.mesh_dim)
    if pins:
        resources.update({f"{name}/{field}": values
                          for name, data in pins.items() for field, values in data.items()})
        sep_json.setdefault("constraints", {})["hard"] = list(pins)
    return cfg, sep_json, resources, attributes, tagged


def separate(options: dict, meshes: dict) -> tuple:
    """Separate the tagged mesh dictionary meshes[options["input"]] in this
    process (a wildmeshing built with WMTK_WITH_POLYFEM). Options: spec.json;
    "output" is unused. Returns ({"mesh": the input mesh, its vertices
    moved}, report: the final active distance "gap" in solver units, as
    options["sep"], the number of PolyFEM "solves", and the "max_displacement"
    in mesh units), as a component's memory interface; writes no files."""
    from ... import _core
    if not hasattr(_core, "polyfem_simulate"):
        raise RuntimeError("minimum_separation: this wildmeshing is built without PolyFEM "
                           "(WMTK_WITH_POLYFEM)")
    p = _spec.validate(_spec.load_spec("minimum_separation"), options)
    if p["save_vtu"]:
        raise ValueError("minimum_separation: save_vtu is not supported in process")
    if p["input"] not in meshes:
        raise KeyError(f"minimum_separation: no mesh '{p['input']}' in meshes")
    mesh = meshes[p["input"]]
    cfg, sep_json, resources, attributes, tagged = _inputs(p, mesh)

    solves = 0

    def solve(j, start):
        nonlocal solves
        solves += 1
        result = _core.polyfem_simulate(j, resources, attributes, initial_solution=start)
        check_polyfem_success(
            0, result["log"],
            allow_out_of_iterations=j["solver"]["nonlinear"].get("allow_out_of_iterations", False))
        return result

    if cfg["strategy"] == "dhat":
        result, gap = _ramp_dhat(solve, sep_json, cfg)
    elif cfg["strategy"] == "stiffness":
        result, gap = _raise_stiffness(solve, sep_json, cfg)
    else:
        raise ValueError(f'strategy must be "dhat" or "stiffness", got {cfg["strategy"]!r}')

    u = result["displacement"] / cfg["scale"]   # solver units -> mesh units
    V = np.array(mesh["V"], dtype=np.float64)
    V[:, :tagged.mesh_dim] += u
    return {"mesh": dict(mesh, V=V)}, {
        "gap": float(gap), "solves": solves, "max_displacement": float(np.abs(u).max())}


def minimum_separation(options: dict) -> dict:
    """Separate the .msh options["input"] as separate() separates a mesh
    dictionary, and write <options["output"]>.msh. Options: spec.json.
    Returns the report."""
    from ... import _core
    p = _spec.validate(_spec.load_spec("minimum_separation"), options)
    meshes, report = separate(p, {p["input"]: _core.read_mesh(p["input"])})
    out = f"{p['output']}.msh"
    if os.path.dirname(out):
        os.makedirs(os.path.dirname(out), exist_ok=True)
    _core.write_mesh(out, meshes["mesh"])
    return report


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("options", help="Path to the options JSON (spec.json)")
    args = ap.parse_args()
    minimum_separation(json.loads(Path(args.options).read_text()))


if __name__ == "__main__":
    main()
