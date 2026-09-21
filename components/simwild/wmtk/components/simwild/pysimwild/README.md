# simwild

Python package for the image-to-simulation pipeline: WMTK mesh operations
(surf_to_tet, lines_to_tri, remeshing, resolve_overlaps, tight_seal,
topological offset, ...) plus the PolyFEM-backed ops (minimum separation,
Laplacian interface smoothing, full simulation).

Full documentation — installation, the operation catalog with examples, and
testing — lives in the [SimWild component README](../../../../README.md)
(`components/simwild/README.md`).

Quick start: install the `wildmeshing` bindings, then `pip install -e .`
from this directory. The PolyFEM-backed ops need the bindings built with
`-DWMTK_WITH_POLYFEM=ON`; their Python engine (`engine="python"`) and the full
simulation instead require
`export POLYFEM_BIN=/path/to/polyfem/build/PolyFEM_bin`.
