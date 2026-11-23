"""wildmeshing.gui without a window (polyscope's mock backend): a pipeline built stage by stage
through the GUI's methods, run in its background thread, each stage reaching the viewer; an
edit marking the stage and its readers, and the rerun running only those; every panel widget
drawn, each tree node open, for each kind of option value (number, flag, choice, text, list,
reference)."""
import os
import time

import numpy as np
import pytest

polyscope = pytest.importorskip("polyscope")
import wildmeshing as wm  # noqa: E402
from wildmeshing import checkpoint, gui, viewer  # noqa: E402
from wildmeshing.pipeline import Pipeline  # noqa: E402


def box(tmp_path):
    v = [(0, 0, 0), (1, 0, 0), (1, 1, 0), (0, 1, 0), (0, 0, 1), (1, 0, 1), (1, 1, 1), (0, 1, 1)]
    f = [(1, 4, 3), (1, 3, 2), (5, 6, 7), (5, 7, 8), (1, 2, 6), (1, 6, 5),
         (4, 8, 7), (4, 7, 3), (1, 5, 8), (1, 8, 4), (2, 3, 7), (2, 7, 6)]
    (tmp_path / "box.obj").write_text("".join(f"v {a} {b} {c}\n" for a, b, c in v)
                                      + "".join(f"f {a} {b} {c}\n" for a, b, c in f))


def run(b):
    b.start()
    b.thread.join()
    polyscope.show(1)  # one frame: the panel takes in what the run finished


def test_build_run_edit_and_draw(tmp_path, monkeypatch):
    monkeypatch.setattr(viewer, "OPEN_ALL", True)
    box(tmp_path)
    b = gui.Gui(Pipeline([], str(tmp_path)), str(tmp_path / "p.json"))
    b.add_input("box", "box.obj")
    b.add_stage("remesh", "isotropic_remeshing")
    # The required options; the first stage reads the input.
    assert b.stage("remesh")["options"] == {"input": {"from": "box", "output": "mesh"}}
    for key, value in {"length_rel": 0.3,
                       "eps_rel": 1e-2, "max_iterations": 2, "num_threads": 0,
                       "use_sample_envelope": False, "feature_fields": [],
                       "history_output": "h"}.items():
        b.set_option("remesh", key, value)
    b.add_stage("simplify", "qslim")
    assert b.stage("simplify")["options"]["input"] == {"from": "remesh", "output": "mesh"}
    b.set_option("simplify", "target_rel", 0.6)
    b.set_option("simplify", "eps_rel", {"from": "remesh", "option": "eps_rel"})
    b.add_stage("map", "tracking")
    assert "input" not in b.stage("map")["options"]  # tracking's is not the stage before
    b.set_option("map", "history", {"from": "remesh", "report": "history"})
    b.set_option("map", "operation", "info")
    with pytest.raises(ValueError):
        b.add_stage("remesh", "qslim")  # names are unique
    b.add_input("box2", "box.obj")
    assert b.names()[:2] == ["box", "box2"]  # the inputs come before the stages
    b.remove("box2")

    drawn = set()
    value = b._value
    monkeypatch.setattr(b, "_value", lambda name, label, v, pointer, rules, unset=None: (
        drawn.add((name, pointer)), value(name, label, v, pointer, rules, unset))[1])
    gui.run_window(b, "mock")  # three frames, nothing run yet
    assert drawn >= {("remesh", p) for p in ["/input", "/length_rel", "/max_iterations",
                                             "/use_sample_envelope", "/history_output"]} \
        | {("map", "/history"), ("map", "/operation"), ("simplify", "/eps_rel")}
    run(b)
    assert b.status == {"box": "file", "remesh": "run", "simplify": "run", "map": "run"}
    assert [s["name"] for s in b.viewer.stages] == ["box", "remesh", "simplify", "map"]
    # The last stage with a mesh is shown alone; the tracking run after it has none to show.
    assert [s["shown"] for s in b.viewer.stages] == [False, False, True, True]
    assert [s["folder"] for s in b.viewer.stages] == ["inputs", None, None, None]
    remesh = b.viewer.stages[1]
    (entry,) = remesh["correspondences"]  # the history's, its input found through the reference
    b.viewer._build(remesh, entry)
    # A stage added now reads simplify: the tracking run after it returned no mesh, and simplify
    # no envelope.
    assert b._chained("topological_offset") == {"input": {"from": "simplify", "output": "mesh"}}
    assert entry["built"] and entry["error"] is None
    b.log_stage = "remesh"
    assert b._log_text()

    b.set_option("simplify", "target_rel", 0.8)
    assert b.status["simplify"] == "edited" and b.status["map"] == "run"
    with pytest.raises(ValueError):
        b.remove("remesh")  # read by simplify and map
    polyscope.show(1)  # the panel with an edited stage, the references' keys now known
    run(b)
    assert b.status == {"box": "memory", "remesh": "memory", "simplify": "run", "map": "memory"}

    # A rename: every reference follows, the readers rerun (the mesh name is in their keys), the
    # input itself stays where it is, in the pipeline and in the view.
    b.rename("box", "cube")
    assert b.stage("remesh")["options"]["input"] == {"from": "cube", "output": "mesh"}
    assert b.viewer.stages[0]["name"] == "cube"
    run(b)
    assert b.status == {"cube": "memory", "remesh": "run", "simplify": "run", "map": "run"}
    with pytest.raises(ValueError):
        b.rename("cube", "remesh")

    b.save(str(tmp_path / "p.json"))
    assert Pipeline.load(str(tmp_path / "p.json")).stages == b.pipeline.stages
    b.remove("map")
    assert [s["name"] for s in b.viewer.stages] == ["cube", "remesh", "simplify"]


def test_label_options_and_the_inputs_table(tmp_path, monkeypatch):
    """A label option names the labels of the mesh the stage reads, marked once that mesh is
    made; a stage naming its inputs lists their names and marks nothing; its per-input lists
    are one table with `input`, a row added or removed in all of them."""
    monkeypatch.setattr(viewer, "OPEN_ALL", True)
    box(tmp_path)
    b = gui.Gui(Pipeline([], str(tmp_path)))
    b.add_input("box", "box.obj")
    b.add_input("wire", "box.obj")
    b.add_stage("mesh", "simwild")
    b.set_option("mesh", "input_type", ["volume"])  # shorter than input: a "+" in its column
    assert b.labels("mesh") == (["box", "wire"], False)  # its inputs' names, never all known
    priority = b.spec("simwild")[0]["/tag_priority/*"]
    assert priority["labels"] == "existing"  # the C++ spec's key, through wm.spec
    assert b.label_problem("mesh", priority, "zz") is None
    b.add_stage("offset", "topological_offset")
    rules = b.spec("topological_offset")[0]
    selection, protected = rules["/offsets/*/selection"], rules["/protected_tags"]
    assert b.labels("offset") == ([], False)  # mesh has not run
    assert b.label_problem("offset", selection, "zz") is None
    # mesh made: a tet tagged box, a curve tagged wire.
    b.pipeline.results["mesh"] = {"key": "k", "report": {}, "outputs": {"mesh": {"cells": {
        "tetrahedron": {"attributes": {"tags": {"values": [[1, 0]], "labels": ["box", "ambient"]}}},
        "edge": {"attributes": {"tags": {"values": [[1]], "labels": ["wire"]}}}}}}}
    assert b.labels("offset") == (["box", "ambient", "wire"], True)
    assert b.label_problem("offset", selection, "box | !wire") is None
    assert b.label_problem("offset", selection, "box | zz") == "unknown label"
    assert b.label_problem("offset", selection, "box |").startswith("not a tag expression")
    assert b.label_problem("offset", protected, "zz") == "unknown label"
    b.set_option("offset", "offsets", [{"selection": "box | zz"}])
    b.set_option("offset", "protected_tags", "wire | zz")

    drawn, red = set(), set()
    value = b._value
    monkeypatch.setattr(b, "_value", lambda name, label, v, pointer, rules, unset=None: (
        drawn.add((name, pointer)), value(name, label, v, pointer, rules, unset))[1])
    imgui = pytest.importorskip("polyscope.imgui")
    coloured = imgui.TextColored
    monkeypatch.setattr(imgui, "TextColored", lambda c, text: (red.add(text), coloured(c, text)))
    gui.run_window(b, "mock")
    assert drawn >= {("mesh", "/input/*"), ("mesh", "/input_names/*"), ("mesh", "/input_type/*"),
                     ("offset", "/offsets"), ("offset", "/protected_tags")}
    assert ("mesh", "/input") not in drawn  # the table draws its entries
    assert red == {"unknown label: the stage can name box, ambient, wire"}

    b.append_stage_input("mesh")
    options = b.stage("mesh")["options"]
    assert options["input"][2] == {"from": "wire", "output": "mesh"}
    assert options["input_names"] == ["box", "wire", ""] and options["input_type"] == ["volume"]
    b.remove_stage_input("mesh", 0)
    assert [r["from"] for r in options["input"]] == ["wire", "wire"]
    assert options["input_names"] == ["wire", ""] and options["input_type"] == []


def test_nested_values_follow_the_spec(tmp_path, monkeypatch):
    """A value the spec types below its top -- pairs, objects, rows of a matrix -- is drawn item
    by item, each as its rule types it, a label item with its dropdown of labels; a new item is
    one its rule takes; "x" and "+" keep a list within its min and max."""
    monkeypatch.setattr(viewer, "OPEN_ALL", True)
    box(tmp_path)
    b = gui.Gui(Pipeline([], str(tmp_path)))
    b.add_input("box", "box.obj")
    b.add_stage("mesh", "simwild")
    b.add_stage("fix", "simwild")
    rules = b.spec("simwild")[0]
    assert gui._new(rules, "/resolve_overlaps_tags/*") == ["", ""]  # min 2
    assert gui._new(rules, "/quality_field/*") == {"tags": "", "quality": 0.0}  # its required
    b.set_option("mesh", "input_transform", [[[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0]]])
    b.set_option("fix", "input", [{"from": "mesh", "output": "mesh"}])
    b.set_option("fix", "resolve_overlaps_tags", [["box", "zz"]])
    b.set_option("fix", "quality_field", [{"tags": "box", "quality": 2.0}])
    b.set_option("fix", "sizing_field", [{"tags": "box", "length_rel": 0.1}])
    b.pipeline.results["mesh"] = {"key": "k", "report": {}, "outputs": {"mesh": {"cells": {
        "tetrahedron": {"attributes": {"tags": {"values": [[1, 0]],
                                                "labels": ["box", "ambient"]}}}}}}}

    drawn, buttons, dropdowns = set(), [], {}
    value = b._value
    monkeypatch.setattr(b, "_value", lambda name, label, v, pointer, rules, unset=None: (
        drawn.add((name, pointer)), value(name, label, v, pointer, rules, unset))[1])
    imgui = pytest.importorskip("polyscope.imgui")
    small, combo = imgui.SmallButton, imgui.Combo
    clicks = {"fix/quality_field/append", "fix/sizing_field/0/add"}  # once each

    def clicked(label):
        hit = next((c for c in clicks if label.endswith(c)), None)
        clicks.discard(hit)
        return hit is not None

    monkeypatch.setattr(imgui, "SmallButton", lambda label: (
        buttons.append(label), small(label))[1] or clicked(label))
    monkeypatch.setattr(imgui, "Combo", lambda label, i, items: (
        dropdowns.__setitem__(label, list(items)),
        (True, 0) if clicked(label) else combo(label, i, items))[1])
    gui.run_window(b, "mock")
    assert drawn >= {("fix", "/resolve_overlaps_tags/*/*"), ("fix", "/quality_field/*/tags"),
                     ("fix", "/quality_field/*/quality"), ("fix", "/sizing_field/*/length_rel"),
                     ("mesh", "/input_transform/*/*/*")}
    # Both items of the pair have the labels of the mesh fix reads.
    pair = [k for k in dropdowns if k.startswith("##labels") and "resolve_overlaps_tags/0/" in k]
    assert len(pair) == 2 and all(dropdowns[k] == ["box", "ambient"] for k in pair)
    # A pair has neither "x" nor "+" of its own (min = max = 2); the list of pairs has both.
    assert not [x for x in buttons if "resolve_overlaps_tags/0/" in x]
    assert any(x.endswith("fix/resolve_overlaps_tags/append") for x in buttons)
    # A matrix row of 4 (3 to 4): "x", no "+".
    assert any(x.endswith("mesh/input_transform/0/0/remove") for x in buttons)
    assert not any(x.endswith("mesh/input_transform/0/0/append") for x in buttons)
    options = b.stage("fix")["options"]
    assert options["quality_field"] == [{"tags": "box", "quality": 2.0},
                                        {"tags": "", "quality": 0.0}]
    assert options["sizing_field"] == [{"tags": "box", "length_rel": 0.1, "length": -1.0}]
    assert not clicks


def test_a_window_error_is_shown_and_the_window_stays(tmp_path, monkeypatch, capsys):
    monkeypatch.setattr(viewer, "OPEN_ALL", True)
    b = gui.Gui(Pipeline([], str(tmp_path)))
    b.add_input("box", "box.obj")
    b.add_stage("remesh", "isotropic_remeshing")

    def broken(stage):
        raise RuntimeError("drawn wrong")
    monkeypatch.setattr(b, "_options", broken)
    gui.run_window(b, "mock")  # three frames, each failing inside the stage's tree node
    assert b.message == "window error: RuntimeError: drawn wrong"
    assert capsys.readouterr().err.count("RuntimeError: drawn wrong") == 1  # once, repeated


def test_save_asks_before_overwriting_a_file_changed_since_it_was_read(tmp_path, monkeypatch):
    path = str(tmp_path / "p.json")
    b = gui.Gui(Pipeline([], str(tmp_path)), path)
    b.save(path)
    b.save(path)  # the file as this window wrote it
    other = gui.Gui(Pipeline.load(path), path)  # a second window on the same file
    other.add_input("box", "box.obj")
    other.save(path)
    with pytest.raises(ValueError, match="changed on disk"):
        b.save(path)
    assert Pipeline.load(path).inputs == {"box": "box.obj"}  # the other window's save kept
    (tmp_path / "q.json").write_text("{}")
    with pytest.raises(ValueError, match="has not read"):
        b.save(str(tmp_path / "q.json"))
    assert (tmp_path / "q.json").read_text() == "{}"
    b.save(str(tmp_path / "q.json"), overwrite=True)
    assert Pipeline.load(str(tmp_path / "q.json")).inputs == {}
    b.save(str(tmp_path / "r.json"))  # a new file
    b.save(str(tmp_path / "r.json"))
    # A window opened on no file has read none: not the pipeline.json in the current folder.
    monkeypatch.chdir(tmp_path)
    (tmp_path / "pipeline.json").write_text("{}")
    with pytest.raises(ValueError, match="has not read"):
        gui.Gui(Pipeline([], str(tmp_path))).save("pipeline.json")

    # In the window: "save" over r.json, changed since, asks; "cancel" keeps the file as it is,
    # "overwrite" writes it.
    path = b.path
    other = gui.Gui(Pipeline.load(path), path)
    other.add_input("box", "box.obj")
    other.save(path)
    imgui = pytest.importorskip("polyscope.imgui")
    button = imgui.Button
    clicks, seen = ["save", "cancel", "save", "overwrite"], []

    def click(label):
        pressed = button(label)
        if clicks and label == clicks[0]:
            seen.append((clicks.pop(0), b.asking, b.message, Pipeline.load(path).inputs))
            return True
        return pressed
    monkeypatch.setattr(imgui, "Button", click)
    gui.run_window(b, "mock")
    assert not clicks
    assert seen[1][1] == "changed on disk since this window read or saved it"
    assert seen[2][2:] == ("not saved", {"box": "box.obj"})
    assert b.message == f"saved {path}" and Pipeline.load(path).inputs == {}


def test_a_stage_line_has_its_buttons_folded_or_not(tmp_path, monkeypatch):
    b = gui.Gui(Pipeline([], str(tmp_path)))
    b.add_input("box", "box.obj")
    b.add_stage("a", "isotropic_remeshing")
    b.add_stage("b", "isotropic_remeshing")
    imgui = pytest.importorskip("polyscope.imgui")
    small, buttons = imgui.SmallButton, []
    monkeypatch.setattr(imgui, "SmallButton", lambda label: (
        buttons.append(label), small(label))[1] or label in ("remove##removeb", "log##loga"))
    collapse, folds = imgui.SetNextWindowCollapsed, []
    monkeypatch.setattr(imgui, "SetNextWindowCollapsed", lambda *a: (
        folds.append(a[0]), collapse(*a))[1])
    gui.run_window(b, "mock")  # every stage folded
    assert {"run##runa", "log##loga", "remove##removea"} <= set(buttons)
    assert [s["name"] for s in b.pipeline.stages] == ["a"]
    assert b.log_stage == "a" and False in folds  # "log" unfolds the log window
    # Unfolded, a stage removed from its line draws nothing more.
    monkeypatch.setattr(viewer, "OPEN_ALL", True)
    b.add_stage("b", "isotropic_remeshing")
    gui.run_window(b, "mock")
    assert [s["name"] for s in b.pipeline.stages] == ["a"]
    assert not b.message.startswith("window error")


def test_save_checkpoint_writes_the_shown_result_then_and_there(tmp_path, monkeypatch):
    monkeypatch.setattr(viewer, "OPEN_ALL", True)
    box(tmp_path)
    b = gui.Gui(Pipeline([], str(tmp_path)), str(tmp_path / "p.json"))
    b.add_input("box", "box.obj")
    b.add_stage("remesh", "isotropic_remeshing")
    for key, value in {"length_rel": 0.3, "eps_rel": 1e-2, "max_iterations": 2,
                       "num_threads": 0}.items():
        b.set_option("remesh", key, value)
    with pytest.raises(ValueError, match="run it first"):
        b.save_checkpoint("remesh", "remesh.msh")
    run(b)
    result = b.pipeline.results["remesh"]
    b.save_checkpoint("remesh", "out/remesh.msh")
    stem = str(tmp_path / "out" / "remesh")
    record = checkpoint.verified(stem)
    assert record["key"] == result["key"] and record["component"] == "isotropic_remeshing"
    assert np.array_equal(checkpoint.load(stem)["mesh"]["V"],
                          result["outputs"]["mesh"]["V"])
    assert os.path.exists(f"{stem}.log")
    assert "save" not in b.stage("remesh") and not (tmp_path / "p.json").exists()
    with pytest.raises(ValueError, match="already there"):
        b.save_checkpoint("remesh", "out/remesh")  # with or without .msh
    b.save_checkpoint("remesh", "out/remesh", overwrite=True)

    # In the window: the field starts at the stage's name; a second "save checkpoint" asks,
    # and "overwrite" writes.
    imgui = pytest.importorskip("polyscope.imgui")
    button, text = imgui.Button, imgui.InputText
    clicks, seen, fields = ["save checkpoint##checkpointremesh"] * 2 + ["overwrite"], [], {}

    def click(label):
        pressed = button(label)
        if clicks and label == clicks[0]:
            seen.append((clicks.pop(0), b.message))
            return True
        return pressed
    monkeypatch.setattr(imgui, "Button", click)
    monkeypatch.setattr(imgui, "InputText", lambda label, v: (
        fields.__setitem__(label, v), text(label, v))[1])
    gui.run_window(b, "mock")
    assert fields["##checkpointremesh"] == "remesh.msh"
    assert not clicks
    assert seen[1][1] == "saved remesh.msh"  # the first: nothing was there
    assert b.message == "saved remesh.msh"
    assert checkpoint.verified(str(tmp_path / "remesh"))["key"] == result["key"]


def slow(b, name):
    """A remeshing of the box that runs for seconds: one to stop, or to end, while it runs."""
    b.add_stage(name, "isotropic_remeshing")
    for key, value in {"length_rel": 0.01, "eps_rel": 1e-2, "max_iterations": 10,
                       "num_threads": 0}.items():
        b.set_option(name, key, value)


def started(b):
    """The run's component process, once it is up."""
    while b.pipeline.child is None or not b.pipeline.child.is_alive():
        assert b.running()
        time.sleep(0.01)
    return b.pipeline.child


def test_stop_ends_the_running_stage_and_the_run(tmp_path):
    box(tmp_path)
    b = gui.Gui(Pipeline([], str(tmp_path)))
    b.add_input("box", "box.obj")
    slow(b, "slow")
    b.add_stage("simplify", "qslim")
    gui.run_window(b, "mock")
    b.start()
    child = started(b)
    b.pipeline.stop()
    b.thread.join()
    polyscope.show(1)
    assert not child.is_alive()
    assert b.status == {"box": "file", "slow": "stopped"} and b.message == "stopped at slow"
    with open(b.logs["slow"]) as f:
        assert f.read().endswith("stopped\n")
    assert [f for f in os.listdir(b.pipeline.cache) if not f.endswith(".log")] == []


def test_a_crash_fails_only_its_stage(tmp_path):
    # A process ending without a result -- here killed, as an abort in C++ ends it.
    box(tmp_path)
    b = gui.Gui(Pipeline([], str(tmp_path)))
    b.add_input("box", "box.obj")
    slow(b, "slow")
    gui.run_window(b, "mock")
    b.start()
    started(b).kill()
    b.thread.join()
    polyscope.show(1)
    assert b.status["slow"] == "failed" and b.log_stage == "slow"
    with open(b.logs["slow"]) as f:
        assert "without a result" in f.read()


def test_a_failed_stage_is_marked_and_its_log_chosen(tmp_path, monkeypatch):
    box(tmp_path)
    b = gui.Gui(Pipeline([], str(tmp_path)))
    b.add_input("box", "box.obj")
    b.add_stage("remesh", "isotropic_remeshing")
    b.set_option("remesh", "max_iterations", "two")  # the spec refuses a string
    gui.run_window(b, "mock")
    run(b)
    assert b.status["remesh"] == "failed" and b.log_stage == "remesh"


def test_a_picked_file_and_the_default_names(tmp_path, monkeypatch):
    box(tmp_path)
    b = gui.Gui(Pipeline([], str(tmp_path)))
    gui.run_window(b, "mock")
    assert b.new_input["name"] == ""  # no file yet
    picked = type("Done", (), {"stdout": str(tmp_path / "box.obj") + "\n"})
    monkeypatch.setattr(gui.subprocess, "run", lambda *a, **k: picked)
    b.pick_file("new").join()
    polyscope.show(1)
    assert b.new_input == {"name": "box", "file": "box.obj", "default": "box"}  # relative, stem
    b.add_input(b.new_input["name"], b.new_input["file"])
    b.new["kind"] = wm.components().index("qslim")
    polyscope.show(1)
    assert b.new["name"] == "1_qslim"
    b.add_stage(b.new["name"], "qslim")
    polyscope.show(1)
    assert b.new["name"] == "2_qslim"


def test_a_tracking_stage_reads_the_last_history(tmp_path):
    # Its history is the last one recorded above it, of a stage with history_output set; with
    # none recorded, the option's default.
    b = gui.Gui(Pipeline([], str(tmp_path)))
    b.add_input("box", "box.obj")
    b.add_stage("simplify", "qslim")
    b.add_stage("map", "tracking")
    assert b.stage("map")["options"]["history"] == ""
    b.set_option("simplify", "history_output", "h")
    b.add_stage("remesh", "isotropic_remeshing")
    b.set_option("remesh", "history_output", "h")
    b.add_stage("collapse", "shortest_edge_collapse")
    b.add_stage("map_again", "tracking")
    assert b.stage("map_again")["options"]["history"] == {"from": "remesh", "report": "history"}
