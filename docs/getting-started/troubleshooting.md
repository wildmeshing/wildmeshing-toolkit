# Troubleshooting

## Building

**CMake cannot find GMP.** Install it (`apt install libgmp-dev`, `brew install gmp`); on Windows,
CMake downloads it. A GMP outside the default paths is found through the `GMP_DIR` environment
variable.

**A component takes minutes where it should take seconds.** Check the build type: Debug builds
are 10 to 100 times slower than Release. Configure with `-DCMAKE_BUILD_TYPE=Release`
(`pip install .` always builds in Release).

**`import wildmeshing` fails from a CMake build.** The module in `build/bin/wildmeshing` is built
for the Python interpreter CMake found when it configured (`Python_EXECUTABLE` in
`build/CMakeCache.txt`), and only that version imports it.

**PolyFEM fails to link with `access beyond end of merged section`.** A binutils bug with
debug-info merging: add `-DCMAKE_EXE_LINKER_FLAGS="-fuse-ld=lld"`.

## Running

**An option has no effect.** Check its spelling: most components ignore an option they do not
know and keep its default, after a warning (`qslim has no option ...`). `topological_offset`
refuses unknown options instead. `wmtk_app --options <component>` and
`help(wildmeshing.<component>)` list the options; the [JSON Schemas](../json-schemas.md) let an
editor check a file before it runs.

**Results change slightly from run to run.** With `num_threads` above 0, the order of the
operations depends on the schedule. `num_threads: 0` runs sequentially and gives the same result
every time. See [Concepts: scheduling](../concepts/scheduling.md).

**Simplification stops before the target.** The envelope (`eps_rel`) refuses the remaining
collapses: thicken it, or accept the larger mesh. See the [QSlim tutorial](../tutorials/qslim.md#the-envelope-eps_rel).

**Simplification is very slow on some inputs.** Inputs where one triangle overlaps millions of
small ones (some Thingi10K models) make the exact envelope test expensive. Try
`use_sample_envelope: true`: the same envelope, tested approximately and much faster there.

**TetWild returns an empty mesh.** `filter: flood` needs a closed input; on an open one the flood
fill reaches everything. Use `filter: input` (the input's winding number), or `none`. See the
[TetWild tutorial](../tutorials/tetwild.md).

**TetWild or TriWild runs to `max_iterations` without reaching `stop_energy`.** Some inputs keep
elements of high energy that the operations cannot improve. The stall refinement
(`stuck_refine_*`) refines around the worst elements when the improvement stalls; the output is
valid either way, with its quality in the report (`max_energy`, `avg_energy`). See
[Concepts: quality](../concepts/quality.md).

## Getting help

Open an issue on [GitHub](https://github.com/wildmeshing/wildmeshing-toolkit/issues) with the
JSON options, the input mesh if you can share it, and the log.
