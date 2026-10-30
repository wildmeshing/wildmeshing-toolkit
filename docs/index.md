# Wildmeshing Toolkit

Robust, parallel triangle and tetrahedral mesh generation and editing.

<a href="gallery/"><img src="gallery/assets/tetwild_octocat/poster.png" alt="The Octocat: the input surface on the left, and the tet mesh TetWild makes of it, cut open and colored by element quality, on the right" style="width: 100%"></a>

The toolkit writes mesh algorithms as local operations — split, collapse, swap, smooth — together
with the invariants every operation must keep and the rules that update attributes. It schedules
the operations in parallel, checks the invariants, and rolls back any operation that would break
them. On this foundation it implements TetWild, TriWild, QSlim, isotropic remeshing,
multi-region meshing, topological offsets and more, each callable from the command line, Python
and C++ with the same JSON options.

```bash
python -m pip install wildmeshing
```

```python
--8<-- "docs/examples/python/quickstart.py:quickstart"
```

## Where to go

- **[Getting started](getting-started/index.md)**: install, run a first component, and learn the
  two ways to call one.
- **[Tutorials](tutorials/index.md)**: each component, its key settings, with examples in JSON,
  Python and C++.
- **[Gallery](gallery/index.md)**: the components' results, interactive.
- **[Reference](reference/index.md)**: every option of every component, the
  [Python API](python-api.md), the [C++ API](https://wildmeshing.github.io/wildmeshing-toolkit/api/).
- **[Concepts](concepts/index.md)**: local operations, scheduling, robust geometry, quality.
- **[Developer guide](developer/index.md)**: the code, and how to add an operation or a component.

The toolkit is described in *Declarative Specification for Unstructured Mesh Editing Algorithms*
(ACM Transactions on Graphics, SIGGRAPH Asia 2022): see [Citing](about/citing.md). It is
[MIT licensed](about/license.md).
