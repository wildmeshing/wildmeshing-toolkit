"""A pipeline of components: its inputs, named mesh files, and a JSON list of stages, each calling
one component, whose option values may refer to an input or to an earlier stage's output mesh,
report value or option.

    {"cache": "<folder>", "inputs": {"box": "box.obj"}, "stages": [
        {"name": "remesh", "component": "isotropic_remeshing", "save": "out/remesh",
         "options": {"input": {"from": "box", "output": "mesh"}, "history_output": "h"}},
        {"name": "map", "component": "tracking",
         "options": {"history": {"from": "remesh", "report": "history"}}}]}

    from wildmeshing.pipeline import Pipeline
    results = Pipeline.load("pipeline.json").run()

A reference {"from": S, "output": O} passes the mesh O of stage S under the mesh name S (O is
"mesh") or S.O -- an input S has one output, its mesh; {"from": S, "report": K} passes report[K]
of S as it is; {"from": S, "option": K} passes S's option K -- as S sets it, else its spec
default -- known before anything runs. A stage refers only to the inputs and the stages above it,
so the list order is a run order. Names -- of inputs and stages, one set -- are letters, digits,
'_' and '-', as a tag expression reads a label. Paths are relative to the pipeline file; the
cache defaults to <pipeline stem>_cache next to it.

An input's key is the sha256 of its file's bytes; a stage's key is the sha256 of its component,
its options with each option reference resolved and each other reference replaced by (key of S,
kind, O or K, and for an output the mesh name it is passed under), and the sha256 of the
installed compiled module -- not the version string, which a hand-copied install leaves stale.
A stage takes, in order, the result in memory with its key, a checkpoint
(wildmeshing.checkpoint: the cache entry <cache>/<key>, or the stage's `save` stem) whose record
holds its key and whose files still hash as saved, else it runs; the cache entry and the `save`
stem then hold its checkpoint. A file used for a stage was so written for that stage's key and
has not changed since. A rerun need not give the same bits: with num_threads > 1 the scheduler's
order varies; the cache keeps one valid run.

While a component runs, the process's stdout and stderr -- where the C++ log goes -- are
redirected into <cache>/<key>.log. A failed stage raises StageFailed, keeps its log, and the
stages after it do not run.
"""
import contextlib
import copy
import ctypes
import hashlib
import json
import os
import re
import sys

import numpy as np

import wildmeshing

from . import checkpoint

# prismatic_mesh reads fixed names: the regions tag_0 (the input volume) and offset (the band), and
# vertex labels 1 input / 2 offset / 0 background. Until it takes its regions by label, a pipeline
# selects them with two tag expressions its C++ spec does not have yet, and the input is rebuilt
# here: tag_0 and offset the tets each expression selects, the labels from the offsets'
# correspondence (corr_input_vid, by vid). The JSON stays as it is once the C++ side reads these
# options.
OPTIONS = {"prismatic_mesh": [
    {"pointer": "/input_tags", "type": "string", "default": "",
     "doc": "The input volume (tag_0), a tag expression. Read by wildmeshing.pipeline for now."},
    {"pointer": "/offset_tags", "type": "string", "default": "",
     "doc": "The band (offset), a tag expression. Read by wildmeshing.pipeline for now."}]}


def _prismatic_input(mesh, input_tags, offset_tags):
    """The mesh as prismatic_mesh reads it: regions tag_0, offset and ambient, and vertex labels 1
    on the input volume and on every correspondence target, 2 on every vertex with one."""
    if not input_tags or not offset_tags:
        raise ValueError("prismatic_mesh: give both input_tags and offset_tags")
    attributes = mesh.get("vertex_attributes", {})
    if "corr_input_vid" not in attributes:
        raise ValueError("prismatic_mesh: the input has no corr_input_vid (run the offsets with"
                         " save_offset_correspondence)")
    tets = mesh["cells"]["tetrahedron"]
    tags = tets["attributes"]["tags"]
    values, labels = np.asarray(tags["values"]), list(tags["labels"])

    def selected(expr):
        # simwild's parser, as every component reads an expression; one evaluation per distinct
        # set of regions a tet is in.
        e = wildmeshing.Expression(expr, labels)
        if e.has_unknown_names():
            raise ValueError(f"prismatic_mesh: {expr!r} names a region the input does not have"
                             f" (it has {', '.join(labels)})")
        rows, inverse = np.unique(values == 1, axis=0, return_inverse=True)
        chosen = [e.eval({n for n, x in zip(labels, row) if x}) for row in rows]
        return np.array(chosen, bool)[inverse.ravel()]
    tag_0, band = selected(input_tags), selected(offset_tags)
    vid = np.asarray(attributes["vid"]["values"]).ravel()
    corr = np.asarray(attributes["corr_input_vid"]["values"]).ravel()
    row = {v: k for k, v in enumerate(vid)}
    vertex = np.zeros(len(vid), np.int64)
    vertex[np.unique(tets["connectivity"][tag_0])] = 1
    vertex[[row[c] for c in corr[corr >= 0]]] = 1
    vertex[corr >= 0] = 2
    return {"V": mesh["V"],
            "vertex_attributes": {"vid": {"values": vid}, "corr_input_vid": {"values": corr},
                                  "labels": {"values": vertex}},
            "cells": {"tetrahedron": {"connectivity": tets["connectivity"], "attributes": {
                "tags": {"values": np.stack([tag_0, band, ~(tag_0 | band)], 1).astype(np.uint8),
                         "labels": ["tag_0", "offset", "ambient"]}}}}}

_module_hash = None


def _module_sha256():
    global _module_hash
    if _module_hash is None:
        with open(wildmeshing._core.__file__, "rb") as f:
            _module_hash = hashlib.sha256(f.read()).hexdigest()
    return _module_hash


class StageFailed(RuntimeError):
    """A stage's component raised; `log` is the path of its log."""

    def __init__(self, stage, log):
        super().__init__(f"stage {stage} failed, log: {log}")
        self.stage, self.log = stage, log


def _is_ref(value):
    return isinstance(value, dict) and set(value) in (
        {"from", "output"}, {"from", "report"}, {"from", "option"})


def _refs(value):
    """Every reference in an option value, lists and objects included."""
    if _is_ref(value):
        yield value
    elif isinstance(value, dict):
        for v in value.values():
            yield from _refs(v)
    elif isinstance(value, list):
        for v in value:
            yield from _refs(v)


def _replace(value, f):
    """`value` with every reference r replaced by f(r)."""
    if _is_ref(value):
        return f(value)
    if isinstance(value, dict):
        return {k: _replace(v, f) for k, v in value.items()}
    if isinstance(value, list):
        return [_replace(v, f) for v in value]
    return value


def _mesh_name(ref):
    return ref["from"] if ref["output"] == "mesh" else f"{ref['from']}.{ref['output']}"


@contextlib.contextmanager
def _captured(path):
    """fds 1 and 2 into the file `path`, restored on the way out, exceptions included."""
    libc = ctypes.CDLL(None)
    sys.stdout.flush()
    sys.stderr.flush()
    saved = os.dup(1), os.dup(2)
    with open(path, "wb") as f:
        try:
            os.dup2(f.fileno(), 1)
            os.dup2(f.fileno(), 2)
            yield
        finally:
            sys.stdout.flush()
            sys.stderr.flush()
            libc.fflush(None)
            os.dup2(saved[0], 1)
            os.dup2(saved[1], 2)
            os.close(saved[0])
            os.close(saved[1])


NAME = re.compile(r"[A-Za-z0-9_-]+")


def check_name(name, taken):
    """Refuse a name already taken, or one a tag expression could not read as a label."""
    if not NAME.fullmatch(name or "") or name in taken:
        raise ValueError(f"name {name!r}: names are unique, of letters, digits, '_' and '-'")


class Pipeline:
    """Inputs, stages and their results in memory, kept across run() calls; edit `inputs` and
    `stages` in place and run again, and only the stages whose keys changed rerun."""

    def __init__(self, stages, root=".", cache=None, inputs=None):
        self.stages, self.root, self.inputs = stages, root, dict(inputs or {})
        self.cache = os.path.join(root, cache or "pipeline_cache")
        self.results = {}
        self.running = None  # (stage, log path) while a component runs, for a live log
        names = set()
        for name in self.inputs:
            check_name(name, names)
            names.add(name)
        for stage in stages:
            name = stage["name"]
            check_name(name, names)
            if "component" not in stage:
                raise ValueError(f"stage {name}: a stage calls a component; a file is an input")
            if stage["component"] not in wildmeshing.components():
                raise ValueError(f"stage {name}: unknown component {stage['component']}")
            for ref in _refs(stage.get("options", {})):
                if ref["from"] not in names:
                    raise ValueError(f"stage {name}: {ref['from']} is not an input or a stage"
                                     " above it")
            names.add(name)

    @classmethod
    def load(cls, path):
        with open(path) as f:
            spec = json.load(f)
        root = os.path.dirname(os.path.abspath(path))
        stem = os.path.splitext(os.path.basename(path))[0]
        return cls(spec["stages"], root, spec.get("cache", f"{stem}_cache"), spec.get("inputs"))

    def _resolved(self, stage):
        """The stage's options with each option reference replaced by the value it names: that
        stage's option as written (its own option references resolved), else the option's spec
        default. Known before anything runs."""
        def value(ref):
            if ref["from"] in self.inputs:
                raise ValueError(f"stage {stage['name']}: {ref['from']} is an input, no options")
            source = next(s for s in self.stages if s["name"] == ref["from"])
            options = self._resolved(source)
            if ref["option"] in options:
                return options[ref["option"]]
            rule = next((r for r in wildmeshing.spec(source["component"])
                         if r["pointer"] == "/" + ref["option"]), {})
            if "default" not in rule:
                raise ValueError(f"stage {stage['name']}: {ref['from']} sets no {ref['option']}"
                                 " and it has no default")
            return copy.deepcopy(rule["default"])
        return _replace(stage["options"], lambda r: value(r) if "option" in r else r)

    def _key(self, stage, keys):
        def hashed(r):
            # An output goes in under its mesh name, which a component may take as a label: the
            # name is in the key, so that renaming a stage reruns the stages that read it.
            if "output" in r:
                return [keys[r["from"]], "output", r["output"], _mesh_name(r)]
            return [keys[r["from"]], "report", r["report"]]
        options = _replace(self._resolved(stage), hashed)
        text = json.dumps([stage["component"], options, _module_sha256()], sort_keys=True)
        return hashlib.sha256(text.encode()).hexdigest()

    def _call(self, stage, key):
        """Run the stage's component on its references, its log into <cache>/<key>.log, and save
        its checkpoint in the cache."""
        meshes = {}

        def resolve(ref):
            source = self.results[ref["from"]]
            kind = "output" if "output" in ref else "report"
            found = source["outputs" if kind == "output" else "report"]
            if ref[kind] not in found:
                raise ValueError(f"stage {stage['name']}: {ref['from']} has no {kind} {ref[kind]}")
            if kind == "report":
                return found[ref["report"]]
            meshes[_mesh_name(ref)] = found[ref["output"]]
            return _mesh_name(ref)

        def shown(ref):
            # A history as the cache file of the stage it came from.
            value = resolve(ref)
            if isinstance(value, bytes):
                return f"{self.results[ref['from']]['key']}_{ref['report']}.history"
            return value

        resolved = self._resolved(stage)
        options = _replace(resolved, resolve)
        os.makedirs(self.cache, exist_ok=True)
        log = os.path.join(self.cache, f"{key}.log")
        self.running = stage["name"], log
        try:
            with _captured(log):
                if stage["component"] == "prismatic_mesh" and (
                        "input_tags" in options or "offset_tags" in options):
                    meshes[options["input"]] = _prismatic_input(
                        meshes[options["input"]], options.pop("input_tags", []),
                        options.pop("offset_tags", []))
                outputs, report = getattr(wildmeshing, stage["component"])(options, meshes)
        except Exception as e:
            with open(log, "a") as f:  # the error, which the C++ log need not hold
                f.write(f"\n{type(e).__name__}: {e}\n")
            raise StageFailed(stage["name"], log) from e
        finally:
            self.running = None
        checkpoint.save(outputs, os.path.join(self.cache, key), report, key=key,
                        component=stage["component"], options=_replace(resolved, shown))
        return outputs, report

    def _component(self, stage, key):
        """The result of a component stage: from the first of its cache entry and its save stem
        saved for `key` whose files hash as saved, else from running it; both then hold it."""
        stems = {"cache": os.path.join(self.cache, key)}
        if "save" in stage:
            stems["save"] = os.path.join(self.root, stage["save"])
        fresh = lambda stem: (checkpoint.verified(stem) or {}).get("key") == key
        result = None
        for source, stem in stems.items():
            if fresh(stem):
                try:
                    result = {"outputs": checkpoint.load(stem),
                              "report": checkpoint.load_report(stem)}
                except (ValueError, OSError):  # changed since it was verified
                    continue
                origin = stem
                result.update(key=key, source=source, log=f"{stem}.log")
                break
        if result is None:
            outputs, report = self._call(stage, key)
            origin = stems["cache"]
            result = {"outputs": outputs, "report": report, "key": key, "source": "run",
                      "log": f"{origin}.log"}
        rec = checkpoint.record(origin)
        for stem in stems.values():
            if stem != origin and not fresh(stem):
                log = None
                if os.path.exists(result["log"]):
                    with open(result["log"]) as f:
                        log = f.read()
                checkpoint.save(result["outputs"], stem, result["report"], log, key=key,
                                component=rec["component"], options=rec["options"])
        return result

    def run(self, until=None, on_stage=None):
        """Read the inputs, then run the stages in order, through the stage named `until` (all
        without), and return {name: {"outputs", "report", "key", "source", "log"}} for each input
        and stage: source is memory, file, cache, save or run, log the path of the stage's log
        (None for an input). on_stage(name, result) is called after each."""
        keys = {}

        def done(name, result):
            self.results[name] = result
            if on_stage:
                on_stage(name, result)

        for name, file in self.inputs.items():
            path = os.path.join(self.root, file)
            with open(path, "rb") as f:
                key = keys[name] = hashlib.sha256(f.read()).hexdigest()
            old = self.results.get(name)
            done(name, {**old, "source": "memory"} if old is not None and old["key"] == key else
                 {"outputs": {"mesh": wildmeshing.read_mesh(path)}, "report": {}, "key": key,
                  "source": "file", "log": None})
        for stage in self.stages:
            name = stage["name"]
            key = keys[name] = self._key(stage, keys)
            old = self.results.get(name)
            done(name, {**old, "source": "memory"} if old is not None and old["key"] == key else
                 self._component(stage, key))
            if name == until:
                break
        return {name: self.results[name] for name in keys}
