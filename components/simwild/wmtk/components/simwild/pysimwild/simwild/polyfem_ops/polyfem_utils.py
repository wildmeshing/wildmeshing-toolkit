"""Polyfem helpers used by polyfem_sim: running the PolyFEM binary with its
output streamed and logged, reading a mesh's material groups, and the pieces
of a polyfem JSON (paraview output, geometry entry, deep merge). The rest of
this module was the Python engine of minimum_separation and
laplacian_smoothing, which now run only in the C++ component polyfem_ops."""
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


# ---------------------------------------------------------------------------
# Mesh helpers
# ---------------------------------------------------------------------------

def get_mesh_info(msh_path: str) -> tuple[list[int], int, dict, dict, dict]:
    """Read a .msh's material physical groups; returns (sorted material tags,
    mesh dimension 2|3, {group name: tag}, {tag: element count},
    {tag: rest area/volume in mesh units}). Elements are counted once per
    group even when a group spans multi-tag entities."""
    gmsh.initialize()
    gmsh.open(msh_path)
    has_3d = len(gmsh.model.getPhysicalGroups(dim=3)) > 0
    dim = 3 if has_3d else 2
    elem_type = 4 if dim == 3 else 2  # gmsh: 4=Tet, 2=Triangle
    node_tags, coord_flat, _ = gmsh.model.mesh.getNodes()
    idx = {int(t): i for i, t in enumerate(node_tags)}
    P = np.array(coord_flat, dtype=np.float64).reshape(-1, 3)
    name_to_tag: dict = {}
    tag_to_count: dict = {}
    tag_to_volume: dict = {}
    tags: set = set()
    npp = 4 if dim == 3 else 3
    for d, ptag in gmsh.model.getPhysicalGroups(dim=dim):
        name = gmsh.model.getPhysicalName(d, ptag)
        if name:
            name_to_tag[name] = int(ptag)
        tags.add(int(ptag))
        seen = {}
        for ent in gmsh.model.getEntitiesForPhysicalGroup(d, ptag):
            etags, ntags = gmsh.model.mesh.getElementsByType(elem_type, ent)
            conn = np.array(ntags, dtype=np.int64).reshape(-1, npp)
            for e, row in zip(etags, conn):
                seen[int(e)] = row
        vol = 0.0
        for row in seen.values():
            v = P[[idx[int(t)] for t in row]]
            if dim == 3:
                vol += abs(np.linalg.det(np.stack(
                    [v[1] - v[0], v[2] - v[0], v[3] - v[0]]))) / 6.0
            else:
                e1, e2 = v[1] - v[0], v[2] - v[0]
                vol += 0.5 * abs(e1[0] * e2[1] - e1[1] * e2[0])
        tag_to_count[int(ptag)] = len(seen)
        tag_to_volume[int(ptag)] = vol
    gmsh.finalize()
    return sorted(tags), dim, name_to_tag, tag_to_count, tag_to_volume


# ---------------------------------------------------------------------------
# JSON helpers
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


def deep_merge(default: dict, override: dict) -> dict:
    """Recursive dict merge: override wins, nested dicts merge."""
    out = dict(default)
    for k, v in (override or {}).items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = deep_merge(out[k], v)
        else:
            out[k] = v
    return out
