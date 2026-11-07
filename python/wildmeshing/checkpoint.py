"""Checkpoints of memory results.

A memory result keeps its envelope as an entry of its own; a .msh holds one
mesh. A checkpoint is therefore <stem>.msh for the output `mesh` and
<stem>_<name>.msh for every other output (<stem>_envelope.msh for the
envelope), <stem>_<key>.history for each bytes value of the report (a
history, the bytes write_history writes and read_history returns unchanged),
and the record <stem>.json: every file with its sha256, the other report
values, and what the caller adds (a pipeline's stage key, component and
options). <stem>.log is the stage's log, when one is given.

load() reads only the files the record lists and refuses one whose sha256
differs, so a leftover file of an older save is never read and a file edited
since is never taken for the saved one; it returns exactly the arrays save()
wrote, in the same order. A checkpoint written before the record existed
(<stem>.msh, <stem>_envelope.msh, no <stem>.json) loads as it always did.
"""
import hashlib
import json
import os

from ._core import read_history, read_mesh, write_history, write_mesh


def _sha256(path):
    with open(path, "rb") as f:
        return hashlib.sha256(f.read()).hexdigest()


def _path(stem, name, suffix):
    return f"{stem}{suffix}" if name == "mesh" and suffix == ".msh" else f"{stem}_{name}{suffix}"


def save(result, stem, report=None, log=None, **record):
    """Write a memory result as the checkpoint <stem>: its outputs, the report `report` and the
    log text `log` when given, and `record`'s entries in <stem>.json."""
    parent = os.path.dirname(stem)
    if parent:
        os.makedirs(parent, exist_ok=True)
    files = {}
    for name, mesh in result.items():
        files[name] = _path(stem, name, ".msh")
        write_mesh(files[name], mesh)
    envelope = f"{stem}_envelope.msh"
    if "envelope" not in result and os.path.exists(envelope):
        os.remove(envelope)  # else a load without the record would return an older envelope
    histories, values = {}, {}
    for key, value in (report or {}).items():
        if isinstance(value, bytes):
            histories[key] = _path(stem, key, ".history")
            write_history(histories[key], value)
        else:
            values[key] = value
    if log is not None:
        with open(f"{stem}.log", "w") as f:
            f.write(log)
    entry = lambda path: {"file": os.path.basename(path), "sha256": _sha256(path)}
    record = {**record, "outputs": {n: entry(p) for n, p in files.items()},
              "histories": {k: entry(p) for k, p in histories.items()}, "report": values}
    # The record last, replaced in one step: a save cut short leaves no record, or the older one,
    # whose hashes the new files fail.
    with open(f"{stem}.json.tmp", "w") as f:
        json.dump(record, f, indent=1)
    os.replace(f"{stem}.json.tmp", f"{stem}.json")


def record(stem):
    """The record of the checkpoint <stem>, or None without one."""
    try:
        with open(f"{stem}.json") as f:
            return json.load(f)
    except FileNotFoundError:
        return None


def _checked(stem, entry):
    path = os.path.join(os.path.dirname(stem), entry["file"])
    if _sha256(path) != entry["sha256"]:
        raise ValueError(f"{path} changed since it was saved")
    return path


def verified(stem):
    """The record of the checkpoint <stem> if every file it lists still hashes as saved, else
    None (also without a record)."""
    rec = record(stem)
    try:
        for group in ["outputs", "histories"]:
            for e in (rec or {}).get(group, {}).values():
                _checked(stem, e)
    except (ValueError, OSError):
        return None
    return rec


def load(stem):
    """Read the checkpoint <stem> back into a memory result."""
    rec = record(stem)
    if rec is None:
        result = {"mesh": read_mesh(f"{stem}.msh")}
        envelope = f"{stem}_envelope.msh"
        if os.path.exists(envelope):
            result["envelope"] = read_mesh(envelope)
        return result
    return {name: read_mesh(_checked(stem, e)) for name, e in rec["outputs"].items()}


def load_report(stem):
    """The report saved with the checkpoint <stem>, histories included; {} without a record."""
    rec = record(stem)
    if rec is None:
        return {}
    return {**rec["report"],
            **{k: read_history(_checked(stem, e)) for k, e in rec["histories"].items()}}
