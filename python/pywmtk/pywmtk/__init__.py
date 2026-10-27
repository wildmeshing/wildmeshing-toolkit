"""pywmtk — Python layer over the wildmeshing bindings.

WMTK op wrappers plus the polyfem-side pipeline stages. Import submodules
explicitly so heavy dependencies (wildmeshing, h5py, ...) load only when
needed:

    from pywmtk import simwild as wm                          # WMTK ops (needs wildmeshing)
    from pywmtk.polyfem_ops import minimum_separation as ms   # polyfem separation
    from pywmtk.polyfem_ops import laplacian_smoothing as lsm # polyfem smoothing
    from pywmtk.polyfem_ops import polyfem_sim as sw          # polyfem simulation builder
    from pywmtk.polyfem_ops.polyfem_sim import msh_boundary_extractor  # BC surface selections
    from pywmtk.polyfem_ops import polyfem_utils              # shared helpers

Tag selections everywhere use the expression format "tag_0 & ambient"
(first name = body interior where orientation matters).
"""

__version__ = "0.1.0"
