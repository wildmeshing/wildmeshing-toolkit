# PolyFEM (https://github.com/polyfem/polyfem)
# License: MIT
#
# Built in with WMTK_WITH_POLYFEM, for the Python module's in-process simulation. The recipes
# both projects have (polysolve, libigl, ipc_toolkit, geogram, onetbb, ...) are wmtk's in polyfem
# too: polyfem appends its recipe directory to CMAKE_MODULE_PATH after wmtk's, so its include()s
# find wmtk's first, and the build has one of each.

if(TARGET polyfem::polyfem)
    return()
endif()

message(STATUS "Third-party: creating target 'polyfem::polyfem'")

include(polyfem_source)

# The library only. Its tests and app are off already: polyfem is not the top-level project.
# Adjoint optimization is unused here and would add MMG; the embedded Python interpreter would
# link libpython into a Python extension module.
set(POLYFEM_WITH_OPTIMIZATION OFF CACHE BOOL "Build adjoint optimization support")
set(POLYFEM_WITH_PYTHON OFF CACHE BOOL "Enable Python interpreter support")

# One floating-point model in the binary: polyfem's tree is compiled with the options wmtk's code
# is, those of its exact predicates (VolumeRemesher: -frounding-math, -ffp-contract=off, its -O
# and -m flags), which reach wmtk's code through wmtk::toolkit's links. polyfem instantiates the
# same templates (Eigen's, the standard library's), a binary keeps one copy of each, and copies
# compiled otherwise changed wmtk's results: the Python module's tetwild output differed from
# wmtk_app's (tests/python/test_integration.py compares them byte for byte). As compile options,
# so they come after the configuration's flags, as they do on wmtk's code (its -O2 after
# Release's -O3). The targets defined after this, wmtk's, have them already.
get_target_property(WMTK_VOLUME_REMESHER VolumeRemesher::VolumeRemesher ALIASED_TARGET)
get_target_property(WMTK_FP_OPTIONS ${WMTK_VOLUME_REMESHER} INTERFACE_COMPILE_OPTIONS)
if(NOT WMTK_FP_OPTIONS)
    message(FATAL_ERROR "polyfem: VolumeRemesher has no compile options to build polyfem with")
endif()
add_compile_options(${WMTK_FP_OPTIONS})
add_subdirectory("${polyfem_SOURCE_DIR}" "${polyfem_BINARY_DIR}")
