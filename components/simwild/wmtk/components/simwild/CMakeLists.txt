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

# The polyfem-backed operations minimum_separation and laplacian_smoothing, and the helpers they are
# built from; simwild.cpp dispatches to them. Built only with WMTK_WITH_POLYFEM (top-level
# CMakeLists).
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
		polyfem_helpers/PolyfemOperation.hpp
		polyfem_helpers/PolyfemOperation.cpp
		polyfem_helpers/MinimumSeparation.hpp
		polyfem_helpers/MinimumSeparation.cpp
		polyfem_helpers/LaplacianSmoothing.hpp
		polyfem_helpers/LaplacianSmoothing.cpp
		polyfem_helpers/DeformedMesh.hpp
		polyfem_helpers/DeformedMesh.cpp
	)

	target_sources(wmtk_${COMPONENT_NAME} PRIVATE ${POLYFEM_HELPERS_SRC_FILES})

	# simwild.cpp tests this to reach the two operations; a build without it refuses them with an
	# error that names the option, and the rest of simwild is compiled exactly as before.
	target_compile_definitions(wmtk_${COMPONENT_NAME} PUBLIC WMTK_SIMWILD_WITH_POLYFEM)

	# Linked HERE and not into wmtk::toolkit, like ipc-toolkit in topological_offset: these helpers
	# are the only consumers, and polyfem brings geogram, TBB, HDF5, nanospline, tinyexpr and more
	# with it, none of which tetwild / triwild / the rest of simwild have any use for. It also
	# carries the two headers the constraint matrices need: igl (massmatrix, cotmatrix -- the same
	# library the Python calls through its bindings) and h5pp (via paraviewo, built with HDF5 on),
	# so neither needs a link target of its own here.
	#
	# PUBLIC because the helper headers are part of this component's interface: PolyfemRunner.hpp
	# includes polysolve headers, and the component's tests include it.
	include(polyfem)

	target_link_libraries(wmtk_${COMPONENT_NAME} PUBLIC
		polyfem::polyfem
	)
endif()

######################
## TESTS
######################

if(${WMTK_COMPONENT_TESTS})
    add_subdirectory(tests)
endif()

