"""simwild.polyfem_ops — the polyfem-backed pipeline stages.

Shared:
    mesh_core       : TaggedMesh, tag expressions (via the wildmeshing
                      bindings' Expression), region/filter face selection
    constraints     : interface selection -> polyfem constraint artifacts
    polyfem_utils   : process runner + convergence check, mesh queries,
                      optimization-JSON builder, 2-body mesh reduction,
                      solve steps, deformed-mesh write-back, deep merge
    spec            : validator for the per-op spec.json rule files

Ops (one package each, spec.json alongside the runner):
    minimum_separation, laplacian_smoothing, polyfem_sim (+ its
    msh_boundary_extractor preprocessing).

Selections everywhere use the region/filter format from mesh_core.
"""
