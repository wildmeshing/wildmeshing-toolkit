"""Checkpoints of memory results.

A memory result keeps its envelope as an entry of its own; a .msh holds one
mesh. A checkpoint is therefore <stem>.msh, plus <stem>_envelope.msh when the
result has an envelope, and load() returns exactly the arrays save() wrote, in
the same order.
"""
import os

from ._core import read_mesh, write_mesh


def save(result, stem):
    """Write a memory result as the checkpoint <stem>."""
    parent = os.path.dirname(stem)
    if parent:
        os.makedirs(parent, exist_ok=True)
    write_mesh(f"{stem}.msh", result["mesh"])
    envelope = f"{stem}_envelope.msh"
    if "envelope" in result:
        write_mesh(envelope, result["envelope"])
    elif os.path.exists(envelope):
        os.remove(envelope)  # else load() would return an older save's envelope


def load(stem):
    """Read the checkpoint <stem> back into a memory result."""
    result = {"mesh": read_mesh(f"{stem}.msh")}
    envelope = f"{stem}_envelope.msh"
    if os.path.exists(envelope):
        result["envelope"] = read_mesh(envelope)
    return result
