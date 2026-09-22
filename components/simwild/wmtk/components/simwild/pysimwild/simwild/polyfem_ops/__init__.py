"""simwild.polyfem_ops — the polyfem-backed pipeline stages.

minimum_separation and laplacian_smoothing run in the wmtk component
polyfem_ops (C++), reached through simwild.minimum_separation and
simwild.laplacian_smoothing. Their parameter rules are the spec.json files in
the two directories of the same names here, which the C++ build embeds.

In Python:
    polyfem_sim     : full simulation of a finished mesh (+ its
                      msh_boundary_extractor preprocessing); runs the binary
                      named by $POLYFEM_BIN
    mesh_core       : tag expressions (via the wildmeshing bindings'
                      Expression), selection normalization and ids
    polyfem_utils   : polyfem_sim's helpers: process runner, mesh query,
                      JSON pieces, deep merge

Selections everywhere use the region/filter format from mesh_core.
"""
