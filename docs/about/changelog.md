# Changelog

The toolkit has no numbered release since `1.0`; `main` is the current version. This page
summarizes what changed since `1.0`. The merged pull requests on
[GitHub](https://github.com/wildmeshing/wildmeshing-toolkit/pulls?q=is%3Apr+is%3Amerged) have the
details.

## Since 1.0 (September 2023)

`1.0` is the version the SIGGRAPH Asia 2022 paper describes. Since then:

**Components.** The algorithms are components: each a library with a JSON spec of its options
(validated, with defaults, and embedded in the build, [#926](https://github.com/wildmeshing/wildmeshing-toolkit/pull/926)),
one command line (`wmtk_app`) and the same function in C++ and Python. New components: SimWild
(multi-region meshing and region editing), topological offsets, manifold extraction, tracking,
and, in progress, prismatic meshes and C1 simplification.

**Python.** A Python package, `wildmeshing`, with every component on files and on NumPy arrays
([#877](https://github.com/wildmeshing/wildmeshing-toolkit/pull/877),
[#922](https://github.com/wildmeshing/wildmeshing-toolkit/pull/922)), now built from `python/src`
([#1138](https://github.com/wildmeshing/wildmeshing-toolkit/pull/1138)).

**Robust geometry.** Inputs are embedded exactly with VolumeRemesher
([#932](https://github.com/wildmeshing/wildmeshing-toolkit/pull/932),
[#1075](https://github.com/wildmeshing/wildmeshing-toolkit/pull/1075)); geogram and TBB are no
longer dependencies ([#951](https://github.com/wildmeshing/wildmeshing-toolkit/pull/951)); the
envelope is one class, exact by default ([#1104](https://github.com/wildmeshing/wildmeshing-toolkit/pull/1104)).

**TetWild and TriWild.** Feature curves and points; surface swaps
([#979](https://github.com/wildmeshing/wildmeshing-toolkit/pull/979)); vertex placement by a
projected line search ([#1027](https://github.com/wildmeshing/wildmeshing-toolkit/pull/1027));
refinement when the quality stalls ([#944](https://github.com/wildmeshing/wildmeshing-toolkit/pull/944));
input simplification for curves ([#960](https://github.com/wildmeshing/wildmeshing-toolkit/pull/960));
a coarsening pass, off by default ([#1047](https://github.com/wildmeshing/wildmeshing-toolkit/pull/1047));
parallel smoothing by color class ([#1079](https://github.com/wildmeshing/wildmeshing-toolkit/pull/1079));
less memory ([#1077](https://github.com/wildmeshing/wildmeshing-toolkit/pull/1077)).

**Tracking.** Applications record a history of their operations, and the tracking component
rebuilds bijective maps between input and output, for surfaces, curves and volumes
([#1126](https://github.com/wildmeshing/wildmeshing-toolkit/pull/1126)).

**Documentation.** This site: tutorials with tested examples, a reference generated from the
specs, JSON Schemas, and the interactive viewer.
