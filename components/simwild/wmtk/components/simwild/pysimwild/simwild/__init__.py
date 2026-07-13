"""simwild — image-to-simulation pipeline.

WMTK op wrappers plus the polyfem-side pipeline stages. Import submodules
explicitly so heavy dependencies (wildmeshing, h5py, ...) load only when
needed:

    from simwild import simwild as wm               # WMTK ops (needs wildmeshing)
    from simwild import minimum_separation as ms    # polyfem separation/smoothing
    from simwild import simulation_wrapper as sw    # polyfem simulation builder
    from simwild import msh_boundary_extractor      # BC surface selections
    from simwild import sim_utils                   # shared helpers

Tag selections everywhere use the expression format "tag_0 & ambient"
(first name = body interior where orientation matters).
"""

__version__ = "0.1.0"
