set(COMPONENT_NAME simwild)
add_component(${COMPONENT_NAME})

if(NOT ${WMTK_ENABLE_COMPONENT_${COMPONENT_NAME}})
    return()
endif()

set(SRC_FILES
    SimWildMesh.cpp
	SimWildMesh.h

	EmbedSurface.hpp
	EmbedSurface.cpp

	EmbedCurves.hpp
	EmbedCurves.cpp

	Parameters.h

	QualityStats.hpp

	EdgeSplitting.cpp
	EdgeCollapsing.cpp
	EdgeSwapping.cpp
	Smooth.cpp
	
	VolumemesherInsertion.cpp

	simwild.hpp
	simwild.cpp

	read_image_msh.hpp
	read_image_msh.cpp

	SimWildMeshTri.hpp
	SimWildMeshTri.cpp
	ConnectedComponent.hpp
	AnnotationsTri.cpp
	AnnotationsTet.cpp

	expression_parser/Expression.hpp
	expression_parser/Parser.hpp
	expression_parser/Parser.cpp
    )

target_sources(wmtk_${COMPONENT_NAME} PRIVATE ${SRC_FILES})

target_link_libraries(wmtk_${COMPONENT_NAME} PUBLIC
	wmtk::toolkit
	wmtk::data
	wmtk::shortest_edge_collapse
	jse::jse
)

# The polyfem-facing helpers of the polyfem_ops component (components/polyfem_ops), which is the
# JSON entry point and calls them. Built only with WMTK_WITH_POLYFEM (top-level CMakeLists), which
# is the same switch that gates the polyfem_ops component in components/CMakeLists.txt.
if(WMTK_WITH_POLYFEM)
	set(POLYFEM_HELPERS_SRC_FILES
		polyfem_helpers/PythonFormat.hpp
		polyfem_helpers/PythonFormat.cpp
		polyfem_helpers/NumpyCompat.hpp
		polyfem_helpers/NumpyCompat.cpp
		polyfem_helpers/TaggedMesh.hpp
		polyfem_helpers/TaggedMesh.cpp
		polyfem_helpers/InterfaceSelection.hpp
		polyfem_helpers/InterfaceSelection.cpp
		polyfem_helpers/ConstraintMatrices.hpp
		polyfem_helpers/ConstraintMatrices.cpp
		polyfem_helpers/Hdf5Writers.hpp
		polyfem_helpers/Hdf5Writers.cpp
		polyfem_helpers/MeshReduction.hpp
		polyfem_helpers/MeshReduction.cpp
		polyfem_helpers/PolyfemJson.hpp
		polyfem_helpers/PolyfemJson.cpp
		polyfem_helpers/PolyfemRunner.hpp
		polyfem_helpers/PolyfemRunner.cpp
		polyfem_helpers/PolyfemInProcess.cpp
		polyfem_helpers/OuterLoops.hpp
		polyfem_helpers/OuterLoops.cpp
		polyfem_helpers/DeformedMesh.hpp
		polyfem_helpers/DeformedMesh.cpp
	)

	target_sources(wmtk_${COMPONENT_NAME} PRIVATE ${POLYFEM_HELPERS_SRC_FILES})

	# The two operations are validated against the pysimwild spec.json files themselves, embedded at
	# build time: those files are the single source of the parameter rules, and a copy here would be
	# a second source that silently drifts. jse_add_spec exposes each as
	# jse::embed::<target>::<header stem>::spec().
	jse_add_spec(wmtk_${COMPONENT_NAME}_minimum_separation_spec
		INPUT "pysimwild/simwild/polyfem_ops/minimum_separation/spec.json"
		OUTPUT "${WMTK_GENERATED_INCLUDE_DIR}/${COMPONENT_NAME}/minimum_separation_spec.hpp"
	)
	jse_add_spec(wmtk_${COMPONENT_NAME}_laplacian_smoothing_spec
		INPUT "pysimwild/simwild/polyfem_ops/laplacian_smoothing/spec.json"
		OUTPUT "${WMTK_GENERATED_INCLUDE_DIR}/${COMPONENT_NAME}/laplacian_smoothing_spec.hpp"
	)

	# Linked HERE and not into wmtk::toolkit, like ipc-toolkit in topological_offset: these helpers
	# and the polyfem_ops component on top of them are the only consumers, and polyfem brings
	# geogram, TBB, HDF5, nanospline, tinyexpr and more with it, none of which tetwild / triwild /
	# the rest of simwild have any use for. It also carries the two headers the constraint matrices
	# need: igl (massmatrix, cotmatrix -- the same library the Python calls through its bindings)
	# and h5pp (via paraviewo, built with HDF5 on), so neither needs a link target of its own here.
	#
	# PUBLIC because the helper headers are part of this component's interface: PolyfemRunner.hpp
	# includes polysolve headers, and polyfem_ops includes it.
	include(polyfem)

	target_link_libraries(wmtk_${COMPONENT_NAME} PUBLIC
		polyfem::polyfem
		wmtk_${COMPONENT_NAME}_minimum_separation_spec
		wmtk_${COMPONENT_NAME}_laplacian_smoothing_spec
	)
endif()

######################
## TESTS
######################

if(${WMTK_COMPONENT_TESTS})
    add_subdirectory(tests)
endif()

