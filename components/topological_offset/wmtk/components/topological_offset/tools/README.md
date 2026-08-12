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

## Worked example: offsetting a disk

End to end, from nothing to a picture. Run it from this directory.

```sh
# 1. the input: a disk domain of radius 0.28 with a concentric disk of radius 0.20 tagged tag_0
python3 make_disk2d.py 300 0.20 0.28 disk.msh

# 2. the offset, at 10% of the input radius (target_distance 0.02)
<build>/app/wmtk_app -j disk_offset_example.json

# 3. look at it
mkdir -p runs/d10 && mv disk_offset.vtu runs/d10/out.vtu
.venv/bin/python view_offset.py runs
```

`disk_offset_example.json` is the config; edit `target_distance` for other widths. To compare
several, give each its own `runs/<name>/out.vtu` and pass `runs` once -- every run becomes its
own toggleable layer.

### Checking the answer

The exact result is an annulus, so the closed region (input disk plus offset band) has area
`pi*(R+d)^2`. For `R=0.20, d=0.02` that is `0.152053`. Measuring the tagged faces in the output
gives an effective outer radius of 0.21956 against the exact 0.22 -- 0.4% low, which is
discretization and nothing else.

### Two things that will bite you

**`relative_ball_threshold` defaults to 0.1, which is far too large.** The conservative growth
then rejects nearly every candidate and the offset stops after a single cell layer, returning
the same result no matter how large a `target_distance` you ask for -- measured, `d=0.14` and
`d=0.21` produced byte-identical output, both ~60-73% short. `0.01` is correct to under 1%
across a sweep from 1% to 25% of the radius. The warning the run prints does say to decrease
it, but it also fires on runs that are perfectly fine, so it is not a reliable signal.

**The mesh has to resolve the offset.** At `rings=300` over `R_domain=0.28` the spacing is
~9.3e-4, so `target_distance` below ~2e-3 is only a couple of cells wide and the result is
dominated by discretization. Raise `rings` for thinner offsets; cost grows quadratically.
