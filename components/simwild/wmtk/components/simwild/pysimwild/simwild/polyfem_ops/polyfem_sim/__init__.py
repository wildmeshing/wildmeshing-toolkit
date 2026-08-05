#!/usr/bin/env python3
"""
polyfem_sim — build a PolyFEM simulation.json from a Python config
and run PolyFEM on it. Pairs with the boundary-extraction outputs
(`m_sim.msh` + `m_bdry_selection.txt` + `m_bdry_selection_id_map.txt`):
materials and BCs are given by physical-group / tag NAME (e.g. "tag_0",
"tag_0 & ambient") and resolved to polyfem integer ids via the id_map.

Config (dict):
  input_msh          : str   — boundary-extracted _sim.msh
  input_selection    : str   — boundary selection file (polyfem
                               surface_selection); default
                               <stem>_bdry_selection.txt next to input_msh.
                               `selection_file` is a legacy alias.
  id_map             : str   — default <stem>_bdry_selection_id_map.txt
  scale              : float — geometry scale (default 0.001); shorthand for
                               transformation.scale = [scale]*mesh_dim
  transformation     : dict  — polyfem geometry transform, deep-merged over
                               the scale (so {"scale": [0.001,-0.001,0.001]}
                               flips y; translation/rotation also allowed)
  materials          : dict  — {tag_name|phys_tag: polyfem material fields}
  boundary_conditions: dict  — pass-through, except dirichlet/neumann items
                               may use "selection": "tag_0 & ambient" (or a
                               list of names) instead of "id"
  initial_conditions : dict  — pass-through; "id" is the volumetric body id
                               from the boundary-extraction output_groups
  contact / time / solver / output / space : dict — optional pass-through
  save_vtu           : bool  — emit paraview .vtu/.pvd (default True)
  save_restart       : bool  — write state.hdf5/restart.json checkpoints so
                               a later run can resume (default True)
  resume             : bool  — resume from restart files found in out_dir/sim_output
                               (default True); set False or delete them to
                               start fresh

Example:
  cfg = {
    "input_msh": "output/m_sim.msh",
    "materials": {"tag_0": {"type": "NeoHookean", "E": 1e9, "nu": 0.3, "rho": 1000}},
    "boundary_conditions": {
        "rhs": [0, -9.81, 0],
        "dirichlet_boundary": [{"selection": "tag_0 & ambient", "value": [0, 0, 0]}],
    },
    "time": {"integrator": "ImplicitEuler", "tend": 2.0, "dt": 0.05},
  }
  polyfem_sim.run(cfg, out_dir=Path("output/sim"))
"""

import json
import re
from pathlib import Path

import gmsh

from ..polyfem_utils import (PARAVIEW_DEFAULTS, deep_merge,
                             polyfem_bin as _polyfem_bin,
                             geometry_block, get_mesh_info,
                             run_streaming)


def _diff_json_keys(a, b, path: str = "") -> list:
    """Return dotted-path strings of fields where two json structures differ.
    Recurses into dicts; lists and scalars compared atomically."""
    if isinstance(a, dict) and isinstance(b, dict):
        out = []
        for k in sorted(set(a) | set(b)):
            out.extend(_diff_json_keys(a.get(k), b.get(k), f"{path}.{k}" if path else k))
        return out
    return [] if a == b else [path or "<root>"]


# Fields that can safely differ between a resumed run and the saved
# simulation.json. Extending tend, tweaking output bookkeeping, retuning the
# solver, or providing initial conditions (only applied at t=0) does not
# invalidate the state vectors saved at t > 0.
_RESUME_SAFE_PREFIXES = (
    "time.tend",
    "output.",
    "solver.",
    "initial_conditions.",
)


def _filter_drift(diffs: list) -> list:
    return [d for d in diffs if not any(
        d == p.rstrip(".") or d.startswith(p) for p in _RESUME_SAFE_PREFIXES
    )]


SIM_DEFAULTS = {
    "scale":        0.001,
    "save_vtu":     True,
    "save_restart": True,
}

# Polyfem JSON section defaults — match the in-tree reference simulation.json
# (lego-head-3d-coarse). These are merged shallowly with whatever the user
# passes in cfg[<section>], so user values override individual fields.
SIM_SECTION_DEFAULTS = {
    "space": {
        "discr_order": 1,
        "advanced": {"bc_method": "sample"},
    },
    "time": {
        "tend": 0.5,
        "dt": 0.01,
        "integrator": "ImplicitEuler",
    },
    "contact": {
        "enabled": True,
        "dhat": 1e-5,
        "friction_coefficient": 0.0,
        "epsv": 1e-3,
        "use_gcp_formulation": True,
        "use_adaptive_dhat": False,
    },
    "solver": {
        "linear": {
            "solver": ["Eigen::AccelerateLDLT", "Eigen::PardisoLDLT", "Eigen::CholmodDecomposition"],
        },
        "nonlinear": {
            "x_delta_tol": 0,
            "grad_norm_tol": 1e-6,
            "max_iterations": 500,
        },
        "advanced": {
            "lump_mass_matrix": False,
        },
        "contact": {
            "friction_convergence_tol": 0.01,
            "friction_iterations": 1,
            "barrier_stiffness": 1e2,
            "CCD": {
                "broad_phase": "BVH",
            },
        },
        "augmented_lagrangian": {
            "nonlinear": {
                "max_iterations": 500,
            },
        },
    },
}


# ---------------------------------------------------------------------------
# Parsers
# ---------------------------------------------------------------------------

def parse_id_map(path: str) -> dict:
    """Parse the *_id_map.txt written by msh_boundary_extractor. Returns
    {"bodies": {phys tag: group name}, "selections": {(region, filter|None):
    surface id}} — selections keyed by their exact expression strings."""
    bodies: dict = {}
    selections: dict = {}
    section = None
    body_re = re.compile(r"^\s*(\d+)\s*←\s*'([^']*)'\s*$")
    sel_re = re.compile(
        r"^\s*(\d+)\s*←\s*region='([^']*)'(?:\s+filter='([^']*)')?\s*$")
    with open(path) as f:
        for line in f:
            stripped = line.strip()
            if not stripped:
                continue
            if stripped.startswith("#"):
                low = stripped.lower()
                section = ("bodies" if "body" in low
                           else "selections" if "surface selection" in low
                           else None)
                continue
            if section == "bodies":
                m = body_re.match(line)
                if m:
                    bodies[int(m.group(1))] = m.group(2)
            elif section == "selections":
                m = sel_re.match(line)
                if m:
                    selections[(m.group(2), m.group(3))] = int(m.group(1))
    return {"bodies": bodies, "selections": selections}

def _resolve_bc_list(bc_list: list, selection_map: dict) -> list:
    """Resolve named surface selections to polyfem ids in a list of BC items.
    Items may carry `"selection": {"region": expr, "filter": expr?}` instead
    of `"id"`; the pair is matched against the id_map written by
    msh_boundary_extractor (exact expression strings)."""
    out = []
    for item in bc_list:
        if not isinstance(item, dict):
            out.append(item)
            continue
        item = dict(item)
        if "selection" in item and "id" not in item:
            sel = item.pop("selection")
            if not isinstance(sel, dict) or "region" not in sel:
                raise ValueError(
                    f"BC selection must be {{'region': expr, 'filter': expr?}} "
                    f"matching the id_map, got {sel!r} (name lists and "
                    f"'a & b' strings are no longer supported)")
            key = (sel["region"], sel.get("filter"))
            if key not in selection_map:
                avail = [f"region='{r}'" + (f" filter='{f}'" if f else "")
                         for r, f in sorted(selection_map,
                                            key=lambda k: (k[0], k[1] or ""))]
                raise ValueError(
                    f"BC selection region='{key[0]}' filter='{key[1]}' not in "
                    f"id_map. Available: {avail}")
            item["id"] = selection_map[key]
        out.append(item)
    return out

def build_polyfem_sim_json(cfg: dict, msh_path: Path, sim_out_dir: Path,
                            name_to_tag: dict, id_map: dict,
                            mesh_dim: int) -> dict:
    scale = cfg.get("scale", SIM_DEFAULTS["scale"])
    save_vtu = cfg.get("save_vtu", SIM_DEFAULTS["save_vtu"])
    save_restart = cfg.get("save_restart", SIM_DEFAULTS["save_restart"])

    # ---- scale: scalar -> per-dim list (matches reference simulation.json)
    if isinstance(scale, (int, float)):
        scale_val = [float(scale)] * mesh_dim
    else:
        scale_val = list(scale)

    # ---- materials: keyed by tag name OR phys_tag int
    raw_mats = cfg.get("materials", {})
    materials = []
    for k, mat_def in raw_mats.items():
        if isinstance(k, str) and not k.lstrip("-").isdigit():
            if k not in name_to_tag:
                raise ValueError(
                    f"materials key '{k}' is not a physical group in the mesh. "
                    f"Available: {sorted(name_to_tag)}")
            phys_tag = name_to_tag[k]
        else:
            phys_tag = int(k)
        item = {"id": phys_tag}
        item.update(mat_def)
        materials.append(item)

    # ---- boundary conditions: resolve "selection" -> "id" and default
    # `dimension` to all-true at mesh_dim if user omits it.
    raw_bc = dict(cfg.get("boundary_conditions", {}))
    sel_map = id_map["selections"]
    for key in ("dirichlet_boundary", "neumann_boundary",
                "pressure_boundary", "obstacle_displacements"):
        if key in raw_bc and isinstance(raw_bc[key], list):
            raw_bc[key] = _resolve_bc_list(raw_bc[key], sel_map)
    for item in raw_bc.get("dirichlet_boundary", []):
        if isinstance(item, dict) and "dimension" not in item:
            item["dimension"] = [True] * mesh_dim

    # ---- geometry: include the boundary selection file if it exists alongside
    # `transformation` starts from cfg["scale"] (or the default) and is
    # deep-merged with cfg["transformation"]. So `sim_cfg["scale"]` still
    # works as before, and users who want translation / rotation / a flipped
    # axis (e.g. {"scale": [1, -1, 1]}) just set transformation directly.
    sel_file = cfg.get("input_selection") or cfg.get("selection_file")
    if sel_file is None:
        sel_default = msh_path.parent / (
            msh_path.stem.removesuffix("_sim") + "_bdry_selection.txt"
            if msh_path.stem.endswith("_sim")
            else msh_path.stem + "_bdry_selection.txt"
        )
        if sel_default.exists():
            sel_file = str(sel_default)
    geom = geometry_block(msh_path, scale_val,
                          transformation=cfg.get("transformation"),
                          surface_selection=sel_file)

    j: dict = {
        "geometry": [geom],
        "materials": materials,
        "space": deep_merge(SIM_SECTION_DEFAULTS["space"], cfg.get("space", {})),
        "time": deep_merge(SIM_SECTION_DEFAULTS["time"], cfg.get("time", {})),
        "boundary_conditions": raw_bc,
        "contact": deep_merge(SIM_SECTION_DEFAULTS["contact"], cfg.get("contact", {})),
        "solver": deep_merge(SIM_SECTION_DEFAULTS["solver"], cfg.get("solver", {})),
    }
    if "initial_conditions" in cfg:
        j["initial_conditions"] = cfg["initial_conditions"]

    # Pass-through sections (skip the ones with defaults; keep this loop for
    # any future top-level section the user wants to inject).
    for section in ():
        if section in cfg:
            j[section] = cfg[section]

    # Output: only emit the `paraview` block when save_vtu is on (matches
    # the reference simulation.json which only has the paraview section).
    output: dict = dict(cfg.get("output", {}))
    if save_vtu:
        output["paraview"] = deep_merge(
            deep_merge(PARAVIEW_DEFAULTS, {"skip_frame": 1}),
            output.get("paraview", {}))
    # Restart checkpoint: polyfem fmt::format-substitutes `{}` with the global
    # timestep index. Omitting `{}` makes fmt return the literal string each
    # step, so polyfem overwrites the same `state.hdf5` / `restart.json`
    # instead of accumulating one pair per timestep.
    if save_restart:
        data = dict(output.get("data", {}))
        data.setdefault("state", "state.hdf5")
        output["data"] = data
        output.setdefault("restart_json", "restart.json")
    j["output"] = output

    return j


# ---------------------------------------------------------------------------
# Runner
# ---------------------------------------------------------------------------

def run(cfg: dict, out_dir=None, in_dirname: str = "sim_input",
        out_dirname: str = "sim_output") -> Path:
    msh_path = Path(cfg["input_msh"]).resolve()
    if not msh_path.exists():
        raise FileNotFoundError(f"input_msh not found: {msh_path}")

    out_dir = Path(out_dir) if out_dir is not None else (
        msh_path.parent / "sim")
    # Mirror minimum_separation's layout: generated polyfem inputs go to
    # in_dirname/, everything polyfem writes (vtu/pvd, log, restart/state)
    # to out_dirname/. Override the names e.g. to "input"/"output" when
    # out_dir is already a per-simulation folder.
    sim_in_dir = out_dir / in_dirname
    sim_out_dir = out_dir / out_dirname
    sim_in_dir.mkdir(parents=True, exist_ok=True)
    sim_out_dir.mkdir(parents=True, exist_ok=True)

    polyfem = _polyfem_bin()

    # id_map: explicit cfg["id_map"] wins; else derive from input_selection
    # (replace ".txt" with "_id_map.txt"); else fall back to the mesh stem.
    id_map_path = cfg.get("id_map")
    sel_cfg = cfg.get("input_selection") or cfg.get("selection_file")
    if id_map_path is None and sel_cfg is not None:
        sel_path = Path(sel_cfg)
        id_map_path = sel_path.with_name(sel_path.stem + "_id_map.txt")
    if id_map_path is None:
        stem = msh_path.stem
        if stem.endswith("_sim"):
            stem = stem[: -len("_sim")]
        id_map_path = msh_path.parent / f"{stem}_bdry_selection_id_map.txt"
    id_map_path = Path(id_map_path)
    id_map = (parse_id_map(str(id_map_path))
              if id_map_path.exists() else {"bodies": {}, "selections": {}})

    _, mesh_dim, name_to_tag, _, _ = get_mesh_info(str(msh_path))

    print(f"\n[polyfem_sim] mesh: {msh_path} (dim={mesh_dim})")
    if name_to_tag:
        print(f"  bodies: {name_to_tag}")
    if id_map["selections"]:
        print(f"  surface selections: "
              f"{ {' ∩ '.join(sorted(k)): v for k, v in id_map['selections'].items()} }")

    sim_json = build_polyfem_sim_json(
        cfg, msh_path, out_dir, name_to_tag, id_map, mesh_dim)

    sim_json_path = sim_in_dir / "simulation.json"

    # Resume from the latest polyfem-written restart_<N>.json if one is in
    # sim_output/. Polyfem's restart file references simulation.json via `common`,
    # so the saved state must match the cfg used to produce it — bail out if
    # the freshly-built cfg has drifted from the saved simulation.json.
    resume = cfg.get("resume", True)
    json_to_run = sim_json_path
    if resume:
        # Prefer the overwrite-style `restart.json`; fall back to legacy
        # per-timestep `restart_<N>.json` from older runs.
        restart_single = sim_out_dir / "restart.json"
        if restart_single.exists():
            restarts = [restart_single]
        else:
            restarts = sorted(
                sim_out_dir.glob("restart_*.json"),
                key=lambda p: int(p.stem.split("_")[-1]) if p.stem.split("_")[-1].isdigit() else -1,
            )
        if restarts and sim_json_path.exists():
            try:
                prior_json = json.loads(sim_json_path.read_text())
            except json.JSONDecodeError:
                prior_json = None
            if prior_json != sim_json:
                all_diffs = _diff_json_keys(prior_json or {}, sim_json)
                unsafe = _filter_drift(all_diffs)
                safe = [d for d in all_diffs if d not in unsafe]
                if safe:
                    print(f"  resume: applying safe cfg changes to "
                          f"{sim_json_path.name}: {safe}")
                if unsafe and not cfg.get("force_resume", False):
                    raise RuntimeError(
                        f"cfg has drifted from {sim_json_path.name} on fields that affect saved state.\n"
                        f"  Unsafe diffs: {unsafe}\n"
                        f"  (safe diffs would have been auto-applied: {safe or 'none'})\n"
                        f"  The saved state hdf5 was produced under the OLD cfg, so resuming would\n"
                        f"  silently mix old state vectors with new settings.\n"
                        f"  Fix one of:\n"
                        f"    - cfg['resume'] = False  (start fresh, keeps the old files around)\n"
                        f"    - delete {sim_out_dir}/restart*.json and state*.hdf5  (clean slate)\n"
                        f"    - cfg['force_resume'] = True  (override at your own risk)\n"
                        f"    - revert cfg so it matches {sim_json_path}")
                if unsafe:
                    print(f"  resume: WARNING force_resume=True; ignoring unsafe diffs: {unsafe}")
        if restarts:
            json_to_run = restarts[-1]
            print(f"  resuming from {json_to_run.name} "
                  f"(set cfg['resume']=False or delete state*.hdf5/restart*.json to start fresh)")

    sim_json_path.write_text(json.dumps(sim_json, indent=4))
    print(f"  wrote {sim_json_path}")

    cmd = [polyfem, "-j", str(json_to_run), "-o", str(sim_out_dir)]
    print(f"  running: {' '.join(cmd)}\n")
    returncode, stdout_lines = run_streaming(cmd, log_path=sim_out_dir / "polyfem.log")

    if returncode != 0:
        last = next((l.strip() for l in reversed(stdout_lines) if "Finished:" in l),
                    None)
        banner = "=" * 72
        print(f"\n{banner}\nPOLYFEM SIM FAILED (rc={returncode})", flush=True)
        if last:
            print(f"  last 'Finished:' line: {last}", flush=True)
        print(f"{banner}\n", flush=True)
        raise RuntimeError(
            f"PolyFEM sim failed (return code {returncode}); "
            f"last Finished line: {last!r}")

    return out_dir


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main():
    import argparse
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("config", help="Path to config JSON")
    ap.add_argument("--out-dir", default=None, help="Output directory")
    args = ap.parse_args()

    with open(args.config) as f:
        cfg = json.load(f)
    run(cfg, out_dir=args.out_dir)


if __name__ == "__main__":
    main()
