# polyfem_ops: the polyfem-backed simwild operations (minimum separation, interface smoothing),
# ported from pysimwild's Python glue. Links polyfem as a library; cmake/recipes/polyfem.cmake
# carries the pin and the reasons. Built only with WMTK_WITH_POLYFEM (components/CMakeLists.txt).

set(COMPONENT_NAME polyfem_ops)
add_component(${COMPONENT_NAME})

if(NOT ${WMTK_ENABLE_COMPONENT_${COMPONENT_NAME}})
	return()
endif()

set(SRC_FILES
	polyfem_ops.hpp
	polyfem_ops.cpp
	PythonFormat.hpp
	PythonFormat.cpp
	NumpyCompat.hpp
	NumpyCompat.cpp
	TaggedMesh.hpp
	TaggedMesh.cpp
	InterfaceSelection.hpp
	InterfaceSelection.cpp
	ConstraintMatrices.hpp
	ConstraintMatrices.cpp
	Hdf5Writers.hpp
	Hdf5Writers.cpp
	MeshReduction.hpp
	MeshReduction.cpp
	PolyfemJson.hpp
	PolyfemJson.cpp
	PolyfemRunner.hpp
	PolyfemRunner.cpp
	PolyfemInProcess.cpp
	OuterLoops.hpp
	OuterLoops.cpp
	DeformedMesh.hpp
	DeformedMesh.cpp
)

target_sources(wmtk_${COMPONENT_NAME} PRIVATE ${SRC_FILES})

# The two operations are validated against the pysimwild spec.json files themselves, embedded at
# build time: those files are the single source of the parameter rules, and a copy here would be
# a second source that silently drifts. jse_add_spec exposes each as
# jse::embed::<target>::<header stem>::spec().
jse_add_spec(wmtk_${COMPONENT_NAME}_minimum_separation_spec
	INPUT "${CMAKE_SOURCE_DIR}/components/simwild/wmtk/components/simwild/pysimwild/simwild/polyfem_ops/minimum_separation/spec.json"
	OUTPUT "${WMTK_GENERATED_INCLUDE_DIR}/${COMPONENT_NAME}/minimum_separation_spec.hpp"
)
jse_add_spec(wmtk_${COMPONENT_NAME}_laplacian_smoothing_spec
	INPUT "${CMAKE_SOURCE_DIR}/components/simwild/wmtk/components/simwild/pysimwild/simwild/polyfem_ops/laplacian_smoothing/spec.json"
	OUTPUT "${WMTK_GENERATED_INCLUDE_DIR}/${COMPONENT_NAME}/laplacian_smoothing_spec.hpp"
)

# Linked HERE and not into wmtk::toolkit, like ipc-toolkit in topological_offset: this component
# is the only consumer, and polyfem brings geogram, TBB, HDF5, nanospline, tinyexpr and more with
# it, none of which tetwild / triwild / simwild have any use for. It also carries the two headers
# the constraint matrices need: igl (massmatrix, cotmatrix -- the same library the Python calls
# through its bindings) and h5pp (via paraviewo, built with HDF5 on), so neither needs a link
# target of its own here.
include(polyfem)

target_link_libraries(wmtk_${COMPONENT_NAME} PUBLIC
	wmtk::toolkit
	wmtk::simwild
	jse::jse
	polyfem::polyfem
	wmtk_${COMPONENT_NAME}_minimum_separation_spec
	wmtk_${COMPONENT_NAME}_laplacian_smoothing_spec
)

# #####################
# # TESTS
# #####################
if(${WMTK_COMPONENT_TESTS})
	add_subdirectory(tests)
endif()
