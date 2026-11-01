# Getting started

The toolkit's algorithms are **components**: TetWild, TriWild, QSlim, isotropic remeshing and the
others in the [component reference](../reference/index.md). Each one takes its options as JSON,
and runs three ways:

- from the command line, `wmtk_app -j options.json`, on files;
- from Python, `wildmeshing.<component>(options)` on files, or
  `wildmeshing.<component>(options, meshes)` on NumPy arrays;
- from C++, `wmtk::components::<component>::<component>(options, meshes)` on `MeshData`.

The options are the same in all three, and so are the results.

1. [Installation](installation.md): build the library, the command line and the Python package.
2. [A first run](first-run.md): the same remeshing three ways.
3. [Calling components](calling-components.md): files or memory, mesh data, reports, histories.
4. [Troubleshooting](troubleshooting.md): what goes wrong, and what to do.

Then the [tutorials](../tutorials/index.md) go through each component, and
[Concepts](../concepts/index.md) explains what they are built on.
