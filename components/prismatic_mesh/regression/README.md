# Prismatic mesh differential regression

Build the current app, the comparator, and the abandoned thick-shell executable,
then run:

```sh
python3 run_differential.py \
  --old-binary /path/to/old/build/applications/thick_shells_app \
  --new-binary /path/to/current/build/app/wmtk_app \
  --compare-binary /path/to/current/build/components/prismatic_mesh/wmtk/components/prismatic_mesh/tests/wmtk_prismatic_regression_compare \
  --work-dir /tmp/prismatic-differential \
  --runs 3 \
  --timeout 10
```

The runner generates all sixteen tetrahedral fixtures, executes both binaries
three times in isolated directories, detects unstable output, bounds abandoned
branch hangs, and writes `summary.json`. A failure in one executable never
prevents the other executable from running. Raw meshes and logs remain in the
selected work directory.

The summary records the pinned old commit, current commit and dirty-worktree
state, CMake build type, compiler, executable hashes, and the exact per-case
configuration paths. A dirty run is useful during development but must be
repeated from a clean commit before recording a release baseline.

Determinism is based on the comparator's canonical cell hash, independently of
validity. This keeps an invalid-but-byte-identical output from being
misdiagnosed as nondeterministic; validity remains a separate hard failure.

The corpus contains genuine planar fans, closed and open embedded cubes, an
embedded tetrahedral surface, disconnected panels, a cylindrical patch, a
non-manifold T-junction, two nearby sheets, and a shell attached to a tagged
solid. Fixture tetrahedra are orientation-checked before either executable is
started.

The old application only accepts `side: out` when `tag_input` is a surface, so
the generated old contract records that limitation explicitly. The annotated
MSH used by the new application and the volume-only MSH plus OBJ used by the old
application are generated from the same indexed fixture.

`baseline_status.json` records the current combined three-run result. All sixteen
new outputs are deterministic, valid, and conforming, including cells recovered
as pyramids/tetrahedra from residual offset obstructions. It remains a failing
differential baseline because no old/new pair met its declared equivalence rule:
the pinned reference also crashes, times out, changes connectivity between runs,
omits solid preservation, and emits an invalid open-cube result. These facts are
kept as regression evidence rather than hidden behind relaxed tolerances.
