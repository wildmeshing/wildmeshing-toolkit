"""polyfem_sim.run up to PolyFEM: the simulation it writes, run by a stand-in POLYFEM_BIN."""
import json
import stat
import sys

import pytest

from wildmeshing.polyfem_ops.polyfem_sim import _selection_desc, run


def test_selection_desc():
    assert _selection_desc("bottom", None) == "region='bottom'"
    assert _selection_desc("tag_0", "ambient") == "region='tag_0' filter='ambient'"


@pytest.mark.skipif(sys.platform == "win32", reason="the stand-in PolyFEM is a shell script")
def test_run_with_selection_without_filter(jagged2d, tmp_path, monkeypatch, capsys):
    """A selection without a filter -- `region='...'` alone in the id_map, a region's whole
    boundary -- used to stop run() before PolyFEM: its log sorted the key (region, None)."""
    msh = jagged2d
    msh.with_name(msh.stem + "_bdry_selection.txt").write_text("1 0 1\n")
    msh.with_name(msh.stem + "_bdry_selection_id_map.txt").write_text(
        "# Body IDs  (polyfem materials[].id  ←  mesh physical group tag)\n"
        "     1  ←  'ambient'\n"
        "     2  ←  'tag_0'\n\n"
        "# Surface selection IDs  (polyfem boundary_conditions[].id  ←  selection)\n"
        "     1  ←  region='bottom'\n"
        "     2  ←  region='tag_0' filter='ambient'\n", encoding="utf-8")
    fake = tmp_path / "PolyFEM_bin"
    fake.write_text("#!/bin/sh\nexit 0\n")
    fake.chmod(fake.stat().st_mode | stat.S_IEXEC)
    monkeypatch.setenv("POLYFEM_BIN", str(fake))

    cfg = {"input_msh": str(msh),
           "materials": {"tag_0": {"type": "NeoHookean", "E": 1e5, "nu": 0.3, "rho": 1}},
           "boundary_conditions": {"dirichlet_boundary": [
               {"selection": {"region": "bottom"}, "value": [0, 0]},
               {"selection": {"region": "tag_0", "filter": "ambient"}, "value": [0, 0]}]},
           "save_restart": False}
    out = run(cfg, out_dir=tmp_path / "sim")

    sim = json.loads((out / "sim_input" / "simulation.json").read_text())
    assert [bc["id"] for bc in sim["boundary_conditions"]["dirichlet_boundary"]] == [1, 2]
    log = capsys.readouterr().out
    assert "1 ← region='bottom'" in log
    assert "2 ← region='tag_0' filter='ambient'" in log


def _stand_in(tmp_path, monkeypatch):
    """A stand-in POLYFEM_BIN: logs its arguments, and writes the checkpoint a JSON asks for."""
    calls = tmp_path / "calls.txt"
    fake = tmp_path / "PolyFEM_bin"
    fake.write_text(f"#!/bin/sh\necho \"$@\" >> '{calls}'\n"
                    "grep -q '\"checkpoint\"' \"$2\" && touch \"$4/checkpoint.hdf5\"\nexit 0\n")
    fake.chmod(fake.stat().st_mode | stat.S_IEXEC)
    monkeypatch.setenv("POLYFEM_BIN", str(fake))
    return calls


def _cfg(msh, **kw):
    msh.with_name(msh.stem + "_bdry_selection.txt").write_text("1 0 1\n")
    msh.with_name(msh.stem + "_bdry_selection_id_map.txt").write_text(
        "# Body IDs\n     1  ←  'ambient'\n     2  ←  'tag_0'\n\n"
        "# Surface selection IDs\n     1  ←  region='bottom'\n",
        encoding="utf-8")
    cfg = {"input_msh": str(msh),
           "materials": {"tag_0": {"type": "NeoHookean", "E": 1e5, "nu": 0.3, "rho": 1}},
           "boundary_conditions": {"dirichlet_boundary": [
               {"selection": {"region": "bottom"}, "value": [0, 0]}]}}
    cfg.update(kw)
    return cfg


@pytest.mark.skipif(sys.platform == "win32", reason="the stand-in PolyFEM is a shell script")
def test_save_restart_writes_polyfem_checkpoint(jagged2d, tmp_path, monkeypatch):
    """save_restart is off by default; on, it asks PolyFEM main for its checkpoint (one file,
    overwritten every step), not the /output/data/state and /output/restart_json it rejects."""
    _stand_in(tmp_path, monkeypatch)
    out = run(_cfg(jagged2d), out_dir=tmp_path / "off")
    assert "checkpoint" not in json.loads((out / "sim_input" / "simulation.json").read_text())["output"]
    assert not (out / "sim_output" / "checkpoint.hdf5").exists()

    out = run(_cfg(jagged2d, save_restart=True), out_dir=tmp_path / "on")
    output = json.loads((out / "sim_input" / "simulation.json").read_text())["output"]
    assert output["checkpoint"] == {"path": "checkpoint.hdf5"}
    assert "data" not in output and "restart_json" not in output
    assert (out / "sim_output" / "checkpoint.hdf5").exists()


@pytest.mark.skipif(sys.platform == "win32", reason="the stand-in PolyFEM is a shell script")
def test_resume_from_checkpoint(jagged2d, tmp_path, monkeypatch):
    """resume (off by default) runs PolyFEM on the checkpoint, only for the simulation.json that
    wrote it: PolyFEM takes its settings from the checkpoint, so a changed cfg is refused, and
    force_resume resumes on the checkpoint's settings."""
    calls = _stand_in(tmp_path, monkeypatch)
    out_dir = tmp_path / "sim"
    cfg = _cfg(jagged2d, save_restart=True)
    out = run(cfg, out_dir=out_dir)
    sim_json = out / "sim_input" / "simulation.json"
    checkpoint = out / "sim_output" / "checkpoint.hdf5"
    written = sim_json.read_text()

    def last_json():
        return calls.read_text().splitlines()[-1].split()[1]

    run(cfg, out_dir=out_dir)  # resume off: from the start
    assert last_json() == str(sim_json)

    run(dict(cfg, resume=True), out_dir=out_dir)
    resume_json = out / "sim_input" / "resume.json"
    assert last_json() == str(resume_json)
    assert json.loads(resume_json.read_text()) == {
        "input": {"checkpoint": {"path": str(checkpoint.resolve())}}}

    changed = dict(cfg, resume=True, materials={
        "tag_0": {"type": "NeoHookean", "E": 2e5, "nu": 0.3, "rho": 1}})
    n = len(calls.read_text().splitlines())
    with pytest.raises(RuntimeError, match="materials"):
        run(changed, out_dir=out_dir)
    assert len(calls.read_text().splitlines()) == n

    run(dict(changed, force_resume=True), out_dir=out_dir)
    assert last_json() == str(resume_json)
    assert sim_json.read_text() == written
