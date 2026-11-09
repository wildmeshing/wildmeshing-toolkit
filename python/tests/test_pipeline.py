"""wildmeshing.pipeline on a unit box: a remeshing recording its history, two simplifications of its
result side by side, and a tracking run on its history.

- An unchanged pipeline runs no component again: from memory in the same Pipeline, from the
  cache in a new one, with the same arrays and history bytes.
- Editing a stage reruns it and the stages below that read it, nothing else.
- A save is used only for the key it was saved for and while its files hash as saved.
- A checkpoint without a record loads as before; a stage's log holds the C++ log lines.

Needs the wildmeshing bindings; no external data. Each run takes milliseconds."""
import collections
import json
import os
import shutil

import numpy as np
import pytest

import wildmeshing as wm
from wildmeshing import checkpoint
from wildmeshing.pipeline import Pipeline, StageFailed

COMPONENTS = ["isotropic_remeshing", "qslim", "tracking"]


def ref(stage, output="mesh"):
    return {"from": stage, "output": output}


def stages(max_iterations=2, target_a=0.6):
    return [
        {"name": "remesh", "component": "isotropic_remeshing", "save": "out/remesh",
         "options": {"input": ref("box"), "length_rel": 0.3, "eps_rel": 1e-2,
                     "max_iterations": max_iterations, "num_threads": 0,
                     "history_output": "h"}},
        {"name": "simplify_a", "component": "qslim",
         "options": {"input": ref("remesh"), "eps_rel": 1e-2, "target_rel": target_a}},
        {"name": "simplify_b", "component": "qslim",
         "options": {"input": ref("remesh"), "eps_rel": 1e-2, "target_rel": 0.9}},
        {"name": "map", "component": "tracking",
         "options": {"operation": "info", "history": {"from": "remesh", "report": "history"}}}]


def write(tmp_path, stage_list):
    """The unit box as box.obj, the input box, and the pipeline as p.json, in tmp_path."""
    v = [(0, 0, 0), (1, 0, 0), (1, 1, 0), (0, 1, 0), (0, 0, 1), (1, 0, 1), (1, 1, 1), (0, 1, 1)]
    f = [(1, 4, 3), (1, 3, 2), (5, 6, 7), (5, 7, 8), (1, 2, 6), (1, 6, 5),
         (4, 8, 7), (4, 7, 3), (1, 5, 8), (1, 8, 4), (2, 3, 7), (2, 7, 6)]
    (tmp_path / "box.obj").write_text("".join(f"v {a} {b} {c}\n" for a, b, c in v)
                                      + "".join(f"f {a} {b} {c}\n" for a, b, c in f))
    path = tmp_path / "p.json"
    path.write_text(json.dumps({"inputs": {"box": "box.obj"}, "stages": stage_list}))
    return str(path)


@pytest.fixture
def calls(monkeypatch):
    """Calls per component, counted through the module attribute the pipeline calls."""
    count = collections.Counter()
    for name in COMPONENTS:
        def counted(*args, _f=getattr(wm, name), _name=name):
            count[_name] += 1
            return _f(*args)
        monkeypatch.setattr(wm, name, counted)
    return count


def flat(d, prefix=""):
    for k, v in d.items():
        if isinstance(v, dict):
            yield from flat(v, f"{prefix}{k}/")
        else:
            yield prefix + k, np.asarray(v)


def same(a, b):
    """Same keys, dtypes, shapes and bytes."""
    fa, fb = dict(flat(a)), dict(flat(b))
    return fa.keys() == fb.keys() and all(
        fa[k].dtype == fb[k].dtype and fa[k].shape == fb[k].shape
        and fa[k].tobytes() == fb[k].tobytes() for k in fa)


def sources(results):
    return {name: r["source"] for name, r in results.items()}


def test_an_unchanged_pipeline_runs_no_component_again(tmp_path, calls):
    path = write(tmp_path, stages())
    p = Pipeline.load(path)
    first = p.run()
    assert calls == {"isotropic_remeshing": 1, "qslim": 2, "tracking": 1}
    assert set(sources(first).values()) == {"file", "run"}
    calls.clear()
    assert set(sources(p.run()).values()) == {"memory"}
    again = Pipeline.load(path).run()
    assert not calls
    assert sources(again) == {"box": "file", "remesh": "cache", "simplify_a": "cache",
                              "simplify_b": "cache", "map": "cache"}
    for name in ["remesh", "simplify_a", "simplify_b"]:
        assert same(first[name]["outputs"], again[name]["outputs"]), name
    assert again["remesh"]["report"]["history"] == first["remesh"]["report"]["history"]


def test_an_edit_reruns_the_stage_and_the_stages_reading_it(tmp_path, calls):
    p = Pipeline.load(write(tmp_path, stages()))
    first = p.run()
    calls.clear()
    p.stages[1]["options"]["target_rel"] = 0.5
    r = p.run()
    assert calls == {"qslim": 1}
    assert sources(r)["simplify_a"] == "run" and sources(r)["simplify_b"] == "memory"
    calls.clear()
    p.stages[0]["options"]["max_iterations"] = 3
    r = p.run()
    assert calls == {"isotropic_remeshing": 1, "qslim": 2, "tracking": 1}
    assert all(r[n]["key"] != first[n]["key"] for n in ["remesh", "simplify_b", "map"])


def test_two_stages_read_one_result(tmp_path):
    r = Pipeline.load(write(tmp_path, stages())).run()
    a, b = (r[n]["outputs"]["mesh"]["V"] for n in ["simplify_a", "simplify_b"])
    assert len(a) < len(b) <= len(r["remesh"]["outputs"]["mesh"]["V"])


def test_a_report_value_reaches_the_next_stage(tmp_path):
    p = Pipeline.load(write(tmp_path, stages()))
    r = p.run()
    history = r["remesh"]["report"]["history"]
    direct = wm.tracking({"operation": "info", "history": history}, {})[1]
    assert r["map"]["report"]["edits"] == direct["edits"] > 0
    # The record names the history by the file of the stage it came from.
    shown = checkpoint.record(os.path.join(p.cache, r["map"]["key"]))["options"]["history"]
    assert shown == f"{r['remesh']['key']}_history.history"
    assert os.path.exists(os.path.join(p.cache, shown))


def test_a_history_round_trips_through_the_checkpoint(tmp_path):
    p = Pipeline.load(write(tmp_path, stages()))
    r = p.run()["remesh"]
    for stem in [os.path.join(p.cache, r["key"]), str(tmp_path / "out" / "remesh")]:
        assert checkpoint.load_report(stem)["history"] == r["report"]["history"]
        assert same(checkpoint.load(stem), r["outputs"])
        assert checkpoint.load_report(stem)["#v"] == r["report"]["#v"]


def test_a_save_for_another_key_is_not_used(tmp_path, calls):
    Pipeline.load(write(tmp_path, stages(max_iterations=2))).run()
    shutil.rmtree(tmp_path / "p_cache")
    calls.clear()
    # out/remesh holds the result of 2 iterations: not the stage's with 3.
    r = Pipeline.load(write(tmp_path, stages(max_iterations=3))).run()
    assert sources(r)["remesh"] == "run" and calls["isotropic_remeshing"] == 1
    shutil.rmtree(tmp_path / "p_cache")
    calls.clear()
    # Now it does.
    r = Pipeline.load(write(tmp_path, stages(max_iterations=3))).run()
    assert sources(r)["remesh"] == "save" and calls["isotropic_remeshing"] == 0


def test_an_edited_save_is_refused_and_the_stage_reruns(tmp_path, calls):
    Pipeline.load(write(tmp_path, stages())).run()
    shutil.rmtree(tmp_path / "p_cache")
    stem = str(tmp_path / "out" / "remesh")
    mesh = wm.read_mesh(f"{stem}.msh")
    mesh["V"] = mesh["V"] + 1e-3
    wm.write_mesh(f"{stem}.msh", mesh)
    with pytest.raises(ValueError, match="changed since it was saved"):
        checkpoint.load(stem)
    calls.clear()
    r = Pipeline.load(write(tmp_path, stages())).run()
    assert sources(r)["remesh"] == "run" and calls["isotropic_remeshing"] == 1
    assert not same(checkpoint.load(stem), {"mesh": mesh})


def test_a_checkpoint_without_a_record_loads_as_before(tmp_path):
    write(tmp_path, [])
    box = wm.read_mesh(str(tmp_path / "box.obj"))
    stem = str(tmp_path / "old")
    wm.save({"mesh": box, "envelope": box}, stem)
    os.remove(f"{stem}.json")
    back = wm.load(stem)
    assert list(back) == ["mesh", "envelope"]
    assert same(back["mesh"], wm.read_mesh(f"{stem}.msh")) and checkpoint.load_report(stem) == {}


def test_the_log_holds_the_component_log(tmp_path):
    r = Pipeline.load(write(tmp_path, stages())).run()
    with open(r["remesh"]["log"]) as f:
        log = f.read()
    assert "[wmtk] [info] remeshing on box" in log  # the mesh passed under its stage's name
    assert r["box"]["log"] is None


def test_a_failed_stage_stops_the_run_and_keeps_its_log(tmp_path, calls):
    bad = {"name": "bad", "component": "tracking", "options": {"operation": "bogus"}}
    p = Pipeline.load(write(tmp_path, [bad, stages()[0]]))
    with pytest.raises(StageFailed) as failed:
        p.run()
    assert failed.value.stage == "bad"
    with open(failed.value.log) as f:
        assert '"bogus"' in f.read()
    assert calls["isotropic_remeshing"] == 0 and "remesh" not in p.results


def test_a_stage_refers_only_to_the_inputs_and_the_stages_above_it(tmp_path):
    s = stages()
    with pytest.raises(ValueError, match="not an input or a stage above it"):
        Pipeline.load(write(tmp_path, [s[1], s[0]]))
    with pytest.raises(ValueError, match="a file is an input"):
        Pipeline.load(write(tmp_path, [{"name": "box2", "file": "box.obj"}]))
    for name in ["a.b", "a b", "box"]:  # a label a tag expression cannot read; one taken
        with pytest.raises(ValueError, match="letters, digits"):
            Pipeline([{"name": name, "component": "qslim", "options": {}}], str(tmp_path),
                     inputs={"box": "box.obj"})


def test_an_option_reference_takes_the_value_or_the_default(tmp_path, calls):
    s = stages()
    s[2]["options"]["target_rel"] = {"from": "simplify_a", "option": "target_rel"}
    s[2]["options"]["num_threads"] = {"from": "simplify_a", "option": "num_threads"}  # unset
    p = Pipeline.load(write(tmp_path, s))
    resolved = p._resolved(p.stages[2])
    assert resolved["target_rel"] == 0.6
    assert resolved["num_threads"] == next(r["default"] for r in wm.spec("qslim")
                                           if r["pointer"] == "/num_threads")
    first = p.run()
    assert sources(first)["simplify_b"] == "run"
    calls.clear()
    p.stages[1]["options"]["target_rel"] = 0.5  # the option it refers to: both rerun
    r = p.run()
    assert calls == {"qslim": 2} and r["simplify_b"]["key"] != first["simplify_b"]["key"]


def test_the_prismatic_input_is_rebuilt_from_the_regions_and_the_correspondence():
    from wildmeshing.pipeline import _prismatic_input
    # Two tets: the volume's and the band's; vertex 4, on the band, came from vertex 1.
    mesh = {"V": np.eye(5, 3), "vertex_attributes": {"vid": {"values": np.arange(5)},
                                                     "corr_input_vid": {"values": [-1] * 4 + [1]}},
            "cells": {"tetrahedron": {"connectivity": np.array([[0, 1, 2, 3], [1, 2, 3, 4]]),
                                      "attributes": {"tags": {"values": [[1, 0, 0], [0, 1, 0]],
                                                              "labels": ["cube", "b", "ambient"]}}}}}
    m = _prismatic_input(mesh, ["cube"], ["b"])
    tags = m["cells"]["tetrahedron"]["attributes"]["tags"]
    assert tags["labels"] == ["tag_0", "offset", "ambient"]
    assert tags["values"].tolist() == [[1, 0, 0], [0, 1, 0]]
    assert m["vertex_attributes"]["labels"]["values"].tolist() == [1, 1, 1, 1, 2]
    with pytest.raises(ValueError, match="no region c"):
        _prismatic_input(mesh, ["cube"], ["c"])
    with pytest.raises(ValueError, match="both"):
        _prismatic_input(mesh, ["cube"], [])


def test_prismatic_adapter_preserves_original_curves_faces_and_layer_links():
    from wildmeshing.pipeline import _prismatic_input
    mesh = {"V": np.eye(6, 3),
            "vertex_attributes": {"vid": {"values": np.arange(6)},
                                  "corr_input_vid": {"values": [-1,-1,-1,-1,0,4]}},
            "cells": {
                "edge": {"connectivity": np.array([[0,1]]), "attributes": {}},
                "triangle": {"connectivity": np.array([[1,2,4]]), "attributes": {}},
                "tetrahedron": {"connectivity": np.array([[0,1,2,3],[0,1,4,5]]),
                    "attributes": {"tags": {"values": [[1,0],[0,1]], "labels": ["cube","b"]}}}}}
    converted = _prismatic_input(mesh, ["cube"], ["b"])
    assert set(converted['cells']) == {'edge','triangle','tetrahedron'}
    assert np.array_equal(converted['cells']['edge']['connectivity'], [[0,1]])
    assert np.array_equal(converted['cells']['triangle']['connectivity'], [[1,2,4]])
    # The adapter leaves the two layer links intact for C++ junction handling.
    assert converted['vertex_attributes']['corr_input_vid']['values'].tolist() == [-1,-1,-1,-1,0,4]
    assert converted['vertex_attributes']['labels']['values'][4] == 2
