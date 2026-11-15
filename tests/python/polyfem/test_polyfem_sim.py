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
