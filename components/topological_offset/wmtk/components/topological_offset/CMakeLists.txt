# topological offset

set(COMPONENT_NAME topological_offset)
add_component(${COMPONENT_NAME})

if(NOT ${WMTK_ENABLE_COMPONENT_${COMPONENT_NAME}})
	return()
endif()

set(SRC_FILES
	TopoOffsetTetMesh.h
	TagEnvelopes.hpp
	TopoOffsetTetMesh.cpp
	Optimize2d.cpp
	Smooth.cpp
	Collapse.cpp
	Swap.cpp

	TopoOffsetTriMesh.h
	TopoOffsetTriMesh.cpp

	Parameters.h

	EdgeSplittingTet.cpp
	EdgeSplittingTri.cpp
	SimplicialComplexBVH.hpp
	Spatial.cpp
	Circle.hpp
	Sphere.hpp

	OffsetPotential.hpp
	OffsetPotential.cpp

	topological_offset.hpp
	topological_offset.cpp
)

target_sources(wmtk_${COMPONENT_NAME} PRIVATE ${SRC_FILES})

# PYTHON BINDINGS FOR THE 2D POTENTIALS. scripts/visualize_error_field.py draws E = (Phi - c)^2,
# and the `smooth` field is the ipc-toolkit OGC potential -- not something a script can
# reimplement, so the script calls this and gets the same field the run uses. Optional: the
# component builds and the app runs whether or not this target exists.
if(WMTK_PYBIND AND TARGET pybind11::module)
    pybind11_add_module(wmtk_offset_field python/offset_field_py.cpp)
    target_link_libraries(wmtk_offset_field PRIVATE wmtk_${COMPONENT_NAME})
    # The component and everything under it are static; a shared module can only link them if
    # they were compiled position-independent.
    set_property(TARGET wmtk_${COMPONENT_NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)
    set_target_properties(wmtk_offset_field PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
endif()

# The smooth offset potential. Deliberately linked HERE and not into wmtk::toolkit: this is the
# only consumer, and ipc-toolkit brings geogram, filib, TBB and tight_inclusion with it, none of
# which tetwild / triwild / simwild have any use for.
include(ipc_toolkit)

target_link_libraries(wmtk_${COMPONENT_NAME} PUBLIC
	wmtk::toolkit
	wmtk::simwild
	jse::jse
	ipc::toolkit
)

# #####################
# # TESTS
# #####################
if(${WMTK_COMPONENT_TESTS})
	add_subdirectory(tests)
endif()
