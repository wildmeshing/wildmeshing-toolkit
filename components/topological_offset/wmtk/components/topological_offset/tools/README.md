# topological_offset viewing tools

Two small dependency-light scripts for looking at 2D offset results.

## Setup

Polyscope and numpy only, and nothing else in the repo needs them:

```sh
python3 -m venv .venv
.venv/bin/pip install polyscope numpy
```

## `view_offset.py` — inspect results in polyscope

```sh
.venv/bin/python view_offset.py <outdir> [<outdir> ...]
```

Each `<outdir>` is scanned for `*/out.vtu`, so point it at a directory of runs. Pass two
directories to compare them in one window -- construction against optimized, say:

```sh
.venv/bin/python view_offset.py runs_construction runs_optimized
```

Every layer is registered separately and **starts hidden**, so the window opens empty and you
switch on exactly what you want:

| layer | what it is |
|---|---|
| `input <tag>` | boundary of each input tag region, registered once |
| `<dir>/<run> offset` | the offset boundary, one colour per run |
| `<dir>/<run> band` | the offset band as a filled surface |

Nothing about which tag means what is hardcoded: `offset_tag` is written by the offset itself
and names the band, and every other `tag_*` cell array becomes its own input layer. It reads
the `.vtu` directly (VTK inline-base64), so `save_vtu: true` is the only requirement.

The view is orthographic along z with planar navigation, since the meshes are 2D at z=0.

## `make_disk2d.py` — the simplest possible test input

```sh
python3 make_disk2d.py [rings] [R_inner] [R_domain] [out.msh]
python3 make_disk2d.py 300 0.20 0.28 disk.msh
```

Writes a MSH 4.1 ASCII disk domain with a concentric disk tagged `tag_0`. Everything is
radially symmetric, so the offset is exactly an annulus and the closed region has area
`pi*(R_inner + d)^2` -- which makes it easy to check a result rather than eyeball it. Use it
with `offset_selection: "tag_0"`.

Note the mesh has to resolve the offset: at `rings=300` over `R_domain=0.28` the spacing is
~9.3e-4, so an offset of 2e-3 is about two cells wide. Ask for less and the result is
dominated by discretization.
