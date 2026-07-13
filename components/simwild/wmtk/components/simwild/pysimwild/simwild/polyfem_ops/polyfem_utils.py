"""Shared polyfem machinery for the polyfem ops: process streaming and
convergence checking, mesh queries, the optimization-JSON builder, the
2-body polyfem mesh reduction, the interface-constraint step, single solves,
deformed-mesh write-back, and small helpers (deep merge, tag centroids)."""
import json
import os
import re
import subprocess
import sys
from pathlib import Path

import numpy as np
import gmsh


# ---------------------------------------------------------------------------
# Polyfem process handling
# ---------------------------------------------------------------------------

_ANSI_RE = re.compile(r"\x1B(?:[@-Z\\-_]|\[[0-?]*[ -/]*[@-~])")


def polyfem_bin() -> str:
    """The PolyFEM binary from $POLYFEM_BIN — the single supported way to
    point the polyfem ops at a build. Raises if unset or missing."""
    p = os.environ.get("POLYFEM_BIN", "")
    if not p:
        raise RuntimeError(
            "POLYFEM_BIN is not set — export POLYFEM_BIN=/path/to/PolyFEM_bin")
    if not Path(p).is_file():
        raise RuntimeError(f"POLYFEM_BIN points to a missing file: {p}")
    return p


def run_streaming(cmd: list, cwd: Path | None = None,
                  log_path: Path | None = None) -> tuple[int, list]:
    """Run `cmd`, streaming output live (through a pty so polyfem keeps its
    colors) and tee-ing it, ANSI-stripped, to optional `log_path`.
    Returns (returncode, output lines)."""
    log_file = None
    log_pending = ""
    if log_path is not None:
        log_path = Path(log_path)
        log_path.parent.mkdir(parents=True, exist_ok=True)
        log_file = open(log_path, "w", buffering=1)

    def _log_chunk(text: str) -> None:
        # Buffer until a complete line so an ANSI escape split across reads
        # isn't mangled; flush per line so `tail -f` works.
        nonlocal log_pending
        if log_file is None:
            return
        log_pending += text
        last_nl = log_pending.rfind("\n")
        if last_nl < 0:
            return
        complete = log_pending[:last_nl + 1]
        log_pending = log_pending[last_nl + 1:]
        log_file.write(_ANSI_RE.sub("", complete))
        log_file.flush()

    def _flush_log() -> None:
        nonlocal log_pending
        if log_file is None:
            return
        if log_pending:
            log_file.write(_ANSI_RE.sub("", log_pending))
            log_pending = ""
        log_file.flush()

    try:
        try:
            import pty
            import select
            import errno
        except ImportError:
            # Fallback: plain pipe (loses colors).
            proc = subprocess.Popen(
                cmd, cwd=str(cwd) if cwd else None,
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
            lines = []
            for line in proc.stdout:
                print(line, end="", flush=True)
                lines.append(line)
                _log_chunk(line)
            proc.wait()
            _flush_log()
            return proc.returncode, lines

        master_fd, slave_fd = pty.openpty()
        try:
            proc = subprocess.Popen(
                cmd, cwd=str(cwd) if cwd else None,
                stdout=slave_fd, stderr=slave_fd, stdin=subprocess.DEVNULL,
                close_fds=True)
            os.close(slave_fd)
            slave_fd = -1

            captured = bytearray()

            while True:
                try:
                    rlist, _, _ = select.select([master_fd], [], [], 0.1)
                except (InterruptedError, OSError):
                    continue

                if master_fd in rlist:
                    try:
                        data = os.read(master_fd, 4096)
                    except OSError as e:
                        if e.errno == errno.EIO:
                            # PTY closed by child
                            break
                        raise
                    if not data:
                        break
                    # Raw bytes to stdout so ANSI sequences pass through.
                    sys.stdout.buffer.write(data)
                    sys.stdout.buffer.flush()
                    captured.extend(data)
                    _log_chunk(data.decode("utf-8", errors="replace"))
                elif proc.poll() is not None:
                    # Child exited and no more data buffered.
                    break

            # Drain any final unread bytes after the child exits.
            try:
                while True:
                    data = os.read(master_fd, 4096)
                    if not data:
                        break
                    sys.stdout.buffer.write(data)
                    sys.stdout.buffer.flush()
                    captured.extend(data)
                    _log_chunk(data.decode("utf-8", errors="replace"))
            except OSError:
                pass

            proc.wait()
        finally:
            if slave_fd != -1:
                try:
                    os.close(slave_fd)
                except OSError:
                    pass
            try:
                os.close(master_fd)
            except OSError:
                pass

        text = captured.decode("utf-8", errors="replace")
        lines = [l + "\n" for l in text.split("\n")]
        if lines and lines[-1] == "\n":
            lines.pop()
        _flush_log()
        return proc.returncode, lines
    finally:
        if log_file is not None:
            log_file.close()


def check_polyfem_success(returncode: int, stdout_lines: list[str],
                          allow_out_of_iterations: bool = False) -> None:
    """Raise RuntimeError (with a banner) unless polyfem converged;
    allow_out_of_iterations=True also accepts "Iteration limit reached"."""
    stdout = "".join(stdout_lines)
    success_phrases = [
        "Finished: Gradient vector norm too small",
        "Finished: Relative gradient vector too small",
    ]
    if allow_out_of_iterations:
        success_phrases.append("Finished: Iteration limit reached")
    if returncode == 0 and any(p in stdout for p in success_phrases):
        return
    finished_line = next(
        (l.strip() for l in reversed(stdout_lines) if "Finished:" in l),
        None)
    banner = "=" * 72
    print(f"\n{banner}\nPOLYFEM SOLVE FAILED (return code {returncode})", flush=True)
    if finished_line:
        print(f"  last 'Finished:' line: {finished_line}", flush=True)
    print(f"{banner}\n", flush=True)
    raise RuntimeError(
        f"PolyFEM failed (return code {returncode}); "
        f"last Finished line: {finished_line!r}")


# ---------------------------------------------------------------------------
# Mesh queries
# ---------------------------------------------------------------------------

def tag_centroid(msh_path, tag_name):
    """Volume-weighted centroid (length-3 array) of all tets in the named
    3D physical group; ValueError if the group is not found."""
    gmsh.initialize()
    gmsh.open(msh_path)
    node_tags, coord_flat, _ = gmsh.model.mesh.getNodes()
    coords = np.array(coord_flat).reshape(-1, 3)
    nt2idx = {int(t): i for i, t in enumerate(node_tags)}

    target_id = None
    for d, p in gmsh.model.getPhysicalGroups(3):
        if gmsh.model.getPhysicalName(d, p) == tag_name:
            target_id = p
            break
    if target_id is None:
        gmsh.finalize()
        raise ValueError(f"physical group {tag_name!r} not found")

    centroid_sum = np.zeros(3)
    vol_sum = 0.0
    for ent in gmsh.model.getEntitiesForPhysicalGroup(3, target_id):
        etags, ntags = gmsh.model.mesh.getElementsByType(4, ent)
        if len(etags) == 0:
            continue
        arr = np.array(ntags, dtype=np.int64).reshape(-1, 4)
        for row in arr:
            v = np.array([coords[nt2idx[int(t)]] for t in row])
            tet_centroid = v.mean(axis=0)
            tet_vol = abs(np.linalg.det(np.column_stack([v[1]-v[0], v[2]-v[0], v[3]-v[0]]))) / 6
            centroid_sum += tet_centroid * tet_vol
            vol_sum += tet_vol
    gmsh.finalize()
    return centroid_sum / vol_sum


# ---------------------------------------------------------------------------
# Config defaults (all overridable via config JSON)
# ---------------------------------------------------------------------------
OPT_DEFAULTS = {
    "scale":              0.001,
    "useFitting":         True,
    "useLaplacian":       True,
    "useGraphLaplacian":  False,
    "normalizePenalties": True,
    "amips_weight_bg":    1e-6,   # tag 0 (background)
    "amips_weight_body":  1e0,    # all other tags
    "barrier_stiffness":  1e3,
    "alpha_n":            0.1,
    "alpha_t":            0.1,
    "sep":                None,
    "weight_fitting":     1,
    "weight_laplacian":   1,
    "smoothDisplacementsOrPositions": 0,
    "max_iterations":     5,
    "rtol":              1e-2,
    "amips_normalize_by_count": True,
    "save_vtu":           False,   # paraview/vtu output -- slow; on for debugging
    "contact_enabled":     True,
}
# OPT_DEFAULTS["init_dhat"] = OPT_DEFAULTS["sep"]


# ---------------------------------------------------------------------------
# Mesh helpers
# ---------------------------------------------------------------------------

def get_mesh_info(msh_path: str) -> tuple[list[int], int, dict, dict]:
    """Read a .msh's material physical groups; returns (sorted material tags,
    mesh dimension 2|3, {group name: tag}, {tag: element count}). Elements are
    counted once per group even when a group spans multi-tag entities."""
    gmsh.initialize()
    gmsh.open(msh_path)
    has_3d = len(gmsh.model.getPhysicalGroups(dim=3)) > 0
    dim = 3 if has_3d else 2
    elem_type = 4 if dim == 3 else 2  # gmsh: 4=Tet, 2=Triangle
    name_to_tag: dict = {}
    tag_to_count: dict = {}
    tags: set = set()
    for d, ptag in gmsh.model.getPhysicalGroups(dim=dim):
        name = gmsh.model.getPhysicalName(d, ptag)
        if name:
            name_to_tag[name] = int(ptag)
        tags.add(int(ptag))
        seen = set()
        for ent in gmsh.model.getEntitiesForPhysicalGroup(d, ptag):
            etags, _ = gmsh.model.mesh.getElementsByType(elem_type, ent)
            for e in etags:
                seen.add(int(e))
        tag_to_count[int(ptag)] = len(seen)
    gmsh.finalize()
    return sorted(tags), dim, name_to_tag, tag_to_count


def read_msh_nodes(msh_path: str) -> tuple[list[int], np.ndarray, dict]:
    """Read all nodes of a .msh; returns (int node tags, (n,3) float64 coords
    indexed via the map, {node tag: row index})."""
    gmsh.initialize()
    gmsh.open(msh_path)
    node_tags_raw, coord_flat, _ = gmsh.model.mesh.getNodes()
    gmsh.finalize()

    int_tags = [int(t) for t in node_tags_raw]
    n = len(int_tags)
    max_tag = max(int_tags)
    if max_tag != n:
        node_tag_to_idx = {t: i for i, t in enumerate(sorted(int_tags))}
    else:
        node_tag_to_idx = {t: t - 1 for t in int_tags}

    coords_raw = np.array(coord_flat, dtype=np.float64).reshape(-1, 3)
    coords = np.zeros((n, 3), dtype=np.float64)
    for i, t in enumerate(int_tags):
        coords[node_tag_to_idx[t]] = coords_raw[i]

    return int_tags, coords, node_tag_to_idx


# ---------------------------------------------------------------------------
# Pipeline steps
# ---------------------------------------------------------------------------

def step_make_interface_constraint(msh_path: str, out_dir: Path, use_graph: bool,
                                    normalize: bool, scale: float,
                                    selections: list | None = None,
                                    smooth_positions: bool = False,
                                    skip_collision_artifacts: bool = False) -> None:
    """Run make_interface_constraint on the ORIGINAL multi-tag .msh
    (selections=None auto-detects all interfaces), writing
    interface_constraint*.hdf5, interface_collision.obj and (unless skipped)
    the linear-map/body-id artifacts into out_dir."""
    print(f"\n[make_interface_constraint] {msh_path}")
    from . import constraints as mic
    mic.make_interface_constraint(
        mesh_path=msh_path,
        selections=selections,
        out_dir=str(out_dir),
        use_graph=use_graph,
        normalize=normalize,
        scale=scale,
        smooth_positions=smooth_positions,
        skip_collision_artifacts=skip_collision_artifacts,
    )


def step_run_polyfem_single(polyfem_bin: str, sim_json: dict,
                             sim_json_path: Path, sim_out_dir: Path) -> None:
    """Run polyfem exactly once (smoothing mode: no contact, no dhat loop);
    output and polyfem.log go to sim_out_dir. Raises RuntimeError if the
    solve fails."""
    sim_out_dir.mkdir(parents=True, exist_ok=True)
    sim_json_path.write_text(json.dumps(sim_json, indent=4))

    returncode, stdout_lines = run_streaming(
        [polyfem_bin, "-j", str(sim_json_path), "-o", str(sim_out_dir)],
        log_path=sim_out_dir / "polyfem.log")
    check_polyfem_success(
        returncode, stdout_lines,
        allow_out_of_iterations=sim_json.get("solver", {}).get(
            "nonlinear", {}).get("allow_out_of_iterations", False),
    )


def step_write_deformed_msh(msh_path: Path, sol_path: Path,
                             output_msh: Path, scale: float) -> None:
    """Apply solution.txt displacements (input-node order, solver units —
    divided by `scale` to get mesh units) to the original .msh and write the
    deformed mesh to output_msh with tags/groups preserved."""
    print(f"  sol : {sol_path}")
    u = np.loadtxt(str(sol_path))   # (n_nodes, dim), reorder_nodes=true → input order
    if u.ndim == 1:
        u = u[:, None]
    dim = u.shape[1]

    _, orig_coords, node_tag_to_idx = read_msh_nodes(str(msh_path))
    n_nodes = len(node_tag_to_idx)

    if u.shape[0] != n_nodes:
        raise ValueError(
            f"solution.txt has {u.shape[0]} rows but .msh has {n_nodes} nodes"
        )

    deformed = orig_coords.copy()
    u_mesh = u / scale              # solver units → mesh units
    for d in range(dim):
        deformed[:, d] += u_mesh[:, d]

    _write_deformed_msh(str(msh_path), deformed, node_tag_to_idx, output_msh)
    print(f"  Out : {output_msh}")


# ---------------------------------------------------------------------------
# JSON builder
# ---------------------------------------------------------------------------

PARAVIEW_DEFAULTS = {
    "file_name": "sim.pvd",
    "surface": True,
    "vismesh_rel_area": 1e7,
    "options": {
        "material": True,
        "body_ids": True,
        "tensor_values": False,
        "nodes": False,
    },
}


def geometry_block(mesh_path, scale, transformation=None,
                   surface_selection=None) -> dict:
    """Polyfem geometry entry: mesh + transformation ({"scale": scale}
    deep-merged with overrides) + optional surface_selection file."""
    geom = {"mesh": str(mesh_path),
            "transformation": deep_merge({"scale": scale}, transformation or {})}
    if surface_selection:
        geom["surface_selection"] = str(surface_selection)
    return geom


def build_polyfem_json(cfg: dict, msh_path: Path, out_dir: Path,
                        material_tags: list[int], mesh_dim: int,
                        sol_path: Path,
                        name_to_tag: dict | None = None,
                        tag_to_count: dict | None = None) -> dict:
    """Build the polyfem simulation JSON shared by run() and run_smoothing():
    per-tag AMIPS materials, soft fitting/Laplacian constraints from out_dir,
    an optional contact block, quasistatic time, and an all-fixed Dirichlet
    BC. msh_path must be the REDUCED 2-body mesh; the tag/dim/name/count args
    come from get_mesh_info on it."""
    scale = cfg.get("scale", OPT_DEFAULTS["scale"])
    sep = cfg.get("sep")
    init_dhat = cfg.get("init_dhat", sep)
    collision_pairs = cfg.get("collision_pairs")
    amips_weights_cfg = cfg.get("amips_weights", {})
    use_fitting = cfg.get("useFitting", OPT_DEFAULTS["useFitting"])
    use_laplacian = (cfg.get("useLaplacian", OPT_DEFAULTS["useLaplacian"])
                     or cfg.get("useGraphLaplacian", OPT_DEFAULTS["useGraphLaplacian"]))
    # AMIPS is summed over all elements; without normalization a denser mesh
    # contributes proportionally more energy. Divide each material's weight by
    # the number of elements it owns so total per-material AMIPS is
    # mesh-density-independent (same scaling philosophy as normalizePenalties
    # for fitting/laplacian).
    amips_norm_by_count = cfg.get("amips_normalize_by_count",
                                   OPT_DEFAULTS["amips_normalize_by_count"])

    # Build a tag -> physical-group-name map so amips_weights can be keyed by
    # either the numeric tag (int / numeric str) or the group name (e.g.
    # "ambient", "tag_0").
    tag_to_name = {int(t): n for n, t in (name_to_tag or {}).items()}
    tag_to_count = tag_to_count or {}

    materials = []
    for tag in material_tags:
        name = tag_to_name.get(int(tag))
        w = (amips_weights_cfg.get(str(tag))
             if str(tag) in amips_weights_cfg
             else amips_weights_cfg.get(tag,
                  amips_weights_cfg.get(name) if name else None))
        if w is None:
            w = OPT_DEFAULTS["amips_weight_bg"] if tag == 0 else OPT_DEFAULTS["amips_weight_body"]
        if amips_norm_by_count:
            n_elems = tag_to_count.get(int(tag), 0)
            if n_elems > 0:
                w = w / n_elems
        materials.append({
            "id": tag, "type": "AMIPS", "weight": w, "use_rest_pose": True
        })

    weight_fitting = cfg.get("weight_fitting", OPT_DEFAULTS["weight_fitting"])
    weight_laplacian = cfg.get("weight_laplacian", OPT_DEFAULTS["weight_laplacian"])

    soft = []
    if use_fitting:
        soft.append({
            "data": str(out_dir / "interface_constraint.hdf5"),
            "weight": weight_fitting
        })
    if use_laplacian:
        soft.append({
            "data": str(out_dir / "interface_constraint_laplacian.hdf5"),
            "weight": weight_laplacian
        })

    zero_val = [0.0] * mesh_dim
    dim_flags = [True] * mesh_dim

    doc = {
        "geometry": [geometry_block(msh_path.resolve(), scale)],
        "materials": materials,
        "space": {
            "advanced": {"bc_method": "sample"}
        },
        **({"contact": {
            "enabled": True,
            "friction_coefficient": 0.0,
            "epsv": 1e-3,
            "use_gcp_formulation": True,
            "dhat": init_dhat,
            "alpha_t": cfg.get("alpha_t", OPT_DEFAULTS["alpha_t"]),
            "alpha_n": cfg.get("alpha_n", OPT_DEFAULTS["alpha_n"]),
            "use_rest_shape_measure": cfg.get("use_rest_shape_measure", True),
            "use_adaptive_dhat": cfg.get("use_adaptive_dhat", False),
            "collision_pairs": collision_pairs,
            "collision_mesh": {
                "enabled": True,
                "mesh": str(out_dir / "interface_collision.obj"),
                "linear_map": str(out_dir / "interface_linear_map.hdf5"),
                "collision_body_ids": str(out_dir / "collision_body_ids.txt")
            }
        }} if cfg.get("contact_enabled", OPT_DEFAULTS["contact_enabled"]) else {}),
        "solver": {
            "max_threads": 0,
            "linear": {
                "solver": ["Eigen::PardisoLDLT", "Eigen::CholmodDecomposition", "Eigen::AccelerateLDLT"]
            },
            "nonlinear": {
                "grad_norm_tol": 1e-10,
                "rel_grad_norm_tol": 1e-8,
                "max_iterations": cfg.get("nl_max_iterations", 1000),
                "allow_out_of_iterations": True,
                "Newton": {"residual_tolerance": 1e-3}
            },
            "advanced": {"lump_mass_matrix": True},
            "contact": {
                "friction_convergence_tol": 0.01,
                "friction_iterations": 1,
                "barrier_stiffness": cfg.get("barrier_stiffness", OPT_DEFAULTS["barrier_stiffness"])
            }
        },
        "time": {
            "quasistatic": True,
            "dt": 1,
            "time_steps": 1
        },
        "boundary_conditions": {
            "rhs": [0.0] * 3,
            "dirichlet_boundary": [{
                "id": "all",
                "value": zero_val,
                "dimension": dim_flags
            }]
        },
        "output": {
            **({"paraview": deep_merge(PARAVIEW_DEFAULTS, {})}
               if cfg.get("save_vtu", OPT_DEFAULTS["save_vtu"]) else {}),
            "data": {
                "solution": str(sol_path),
                "advanced": {"reorder_nodes": True}
            }
        },
        "input": {
            "data": {}
        }
    }

    if soft:
        doc["constraints"] = {"soft": soft}

    return doc


# ---------------------------------------------------------------------------
# MSH writer
# ---------------------------------------------------------------------------

def _write_deformed_msh(msh_path: str, deformed: np.ndarray,
                         node_tag_to_idx: dict, output_msh: Path) -> None:
    """Replace node coordinates in .msh with deformed positions.

    Uses gmsh so it works for both ASCII and binary .msh formats.
    """
    gmsh.initialize()
    try:
        gmsh.open(str(msh_path))
        all_tags, _, _ = gmsh.model.mesh.getNodes()
        for t in all_tags:
            idx = node_tag_to_idx[int(t)]
            x, y, z = deformed[idx]
            gmsh.model.mesh.setNode(int(t), [float(x), float(y), float(z)], [])
        gmsh.option.setNumber("Mesh.MshFileVersion", 4.1)
        gmsh.write(str(output_msh))
    finally:
        gmsh.finalize()


# ---------------------------------------------------------------------------
# Polyfem mesh reduction (used by both run() and run_smoothing())
# ---------------------------------------------------------------------------

_RED = "\033[31m"
_RESET = "\033[0m"


def _write_polyfem_reduced_msh(input_msh: str, output_msh: str) -> None:
    """Reduce a multi-tag mesh to a 2-body ("ambient"/"body") polyfem mesh.
    WMTK's `write_msh_groups` writes one copy of each multi-tagged tet per
    tag; polyfem reads the copies as distinct elements, double-counting AMIPS
    and corrupting assembly. Tets are deduped by sorted-vertex-tuple and
    classified by their unioned tag set (ambient → body id 1, other tags →
    body id 2, both → ValueError, tagless → skipped). All original node tags
    are kept, sorted, so geogram's input-vertex-id stays `tag - 1` for both
    the original and reduced mesh (collision artifacts index either)."""
    from collections import defaultdict

    gmsh.initialize()
    try:
        gmsh.open(input_msh)

        # Auto-detect mesh dimension (3D if any volume phys-group exists,
        # else 2D). Set the corresponding gmsh element type and nodes/prim.
        has_3d = len(gmsh.model.getPhysicalGroups(dim=3)) > 0
        prim_dim = 3 if has_3d else 2
        elem_type = 4 if prim_dim == 3 else 2  # gmsh: 4=Tet, 2=Triangle
        nodes_per_prim = 4 if prim_dim == 3 else 3

        node_tags_raw, coord_flat, _ = gmsh.model.mesh.getNodes()
        # Sort by tag so the reduced mesh's storage order is monotonic.
        # This guarantees polyfem's `in_node_to_node` is the identity
        # (input vertex id i ↔ gmsh tag i+1) regardless of how getNodes
        # ordered things internally.
        coords_arr = np.array(coord_flat, dtype=np.float64).reshape(-1, 3)
        order = np.argsort([int(t) for t in node_tags_raw])
        node_tags = [int(node_tags_raw[i]) for i in order]
        coord_list = coords_arr[order].flatten().tolist()

        vtuple_to_tags: dict = defaultdict(set)
        vtuple_to_nodes: dict = {}
        for d, ptag in gmsh.model.getPhysicalGroups(dim=prim_dim):
            name = gmsh.model.getPhysicalName(d, ptag)
            for ent in gmsh.model.getEntitiesForPhysicalGroup(d, ptag):
                etags, ntags = gmsh.model.mesh.getElementsByType(elem_type, ent)
                if len(etags) == 0:
                    continue
                arr = np.array(ntags, dtype=np.int64).reshape(-1, nodes_per_prim)
                for j in range(len(etags)):
                    ordered = tuple(int(x) for x in arr[j])
                    vt = frozenset(ordered)
                    vtuple_to_tags[vt].add(name)
                    if vt not in vtuple_to_nodes:
                        vtuple_to_nodes[vt] = ordered

        ambient_prims: list = []
        body_prims: list = []
        for vt, tags in vtuple_to_tags.items():
            has_ambient = "ambient" in tags
            non_ambient = tags - {"ambient"}
            if has_ambient and non_ambient:
                msg = (
                    f"Element with vertices {sorted(vt)} has both 'ambient' "
                    f"and non-ambient tags {sorted(non_ambient)} — forbidden "
                    f"in polyfem mesh reduction"
                )
                print(f"{_RED}ERROR:{_RESET} {msg}", file=sys.stderr)
                raise ValueError(msg)
            ordered = vtuple_to_nodes[vt]
            if has_ambient:
                ambient_prims.append(ordered)
            elif non_ambient:
                body_prims.append(ordered)
            # else: tagless prim — skip

        gmsh.clear()
        gmsh.model.add("polyfem_reduced_input")
        ent_ambient = gmsh.model.addDiscreteEntity(prim_dim)
        ent_body = gmsh.model.addDiscreteEntity(prim_dim)
        # Keep all original node tags. Entity ownership of nodes doesn't
        # affect element-vertex references; element-add below uses tags.
        gmsh.model.mesh.addNodes(prim_dim, ent_ambient, node_tags, coord_list)

        next_id = 1
        if ambient_prims:
            n = len(ambient_prims)
            gmsh.model.mesh.addElementsByType(
                ent_ambient, elem_type,
                list(range(next_id, next_id + n)),
                [int(x) for prim in ambient_prims for x in prim],
            )
            next_id += n
        if body_prims:
            n = len(body_prims)
            gmsh.model.mesh.addElementsByType(
                ent_body, elem_type,
                list(range(next_id, next_id + n)),
                [int(x) for prim in body_prims for x in prim],
            )
            next_id += n

        pg_amb = gmsh.model.addPhysicalGroup(prim_dim, [ent_ambient], 1)
        gmsh.model.setPhysicalName(prim_dim, pg_amb, "ambient")
        pg_body = gmsh.model.addPhysicalGroup(prim_dim, [ent_body], 2)
        gmsh.model.setPhysicalName(prim_dim, pg_body, "body")

        gmsh.option.setNumber("Mesh.MshFileVersion", 4.1)
        gmsh.write(output_msh)
        kind = "tets" if prim_dim == 3 else "triangles"
        print(f"  reduced  : {output_msh}  "
              f"({len(ambient_prims)} ambient + {len(body_prims)} body {kind})")
    finally:
        gmsh.finalize()


def _resolve_amips_weights(cfg: dict) -> dict:
    """Resolve cfg AMIPS weights to the reduced mesh's {'ambient', 'body'}
    scheme: dedicated `amips_ambient_weight`/`amips_body_weight` keys win,
    then the legacy `amips_weights` dict (its "ambient" entry / first
    non-ambient value), then OPT_DEFAULTS."""
    legacy = cfg.get("amips_weights", {}) or {}
    ambient_w = cfg.get("amips_ambient_weight",
                        legacy.get("ambient", OPT_DEFAULTS["amips_weight_bg"]))
    body_w = cfg.get("amips_body_weight")
    if body_w is None:
        body_vals = [v for k, v in legacy.items() if k != "ambient"]
        body_w = body_vals[0] if body_vals else OPT_DEFAULTS["amips_weight_body"]
    return {"ambient": ambient_w, "body": body_w}


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------



def deep_merge(default: dict, override: dict) -> dict:
    """Recursive dict merge: override wins, nested dicts merge."""
    out = dict(default)
    for k, v in (override or {}).items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = deep_merge(out[k], v)
        else:
            out[k] = v
    return out
