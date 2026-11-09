"""wildmeshing.gui without a window (polyscope's mock backend): a pipeline built stage by stage
through the GUI's methods, run in its background thread, each stage reaching the viewer; an
edit marking the stage and its readers, and the rerun running only those; every panel widget
drawn, each tree node open, for each kind of option value (number, flag, choice, text, list,
reference)."""
import pytest

polyscope = pytest.importorskip("polyscope")
import wildmeshing as wm  # noqa: E402
from wildmeshing import gui, viewer  # noqa: E402
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
