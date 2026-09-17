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
)

target_sources(wmtk_${COMPONENT_NAME} PRIVATE ${SRC_FILES})

# Linked HERE and not into wmtk::toolkit, like ipc-toolkit in topological_offset: this component
# is the only consumer, and polyfem brings geogram, TBB, HDF5, nanospline, tinyexpr and more with
# it, none of which tetwild / triwild / simwild have any use for.
include(polyfem)

target_link_libraries(wmtk_${COMPONENT_NAME} PUBLIC
	wmtk::toolkit
	wmtk::simwild
	jse::jse
	polyfem::polyfem
)

# #####################
# # TESTS
# #####################
if(${WMTK_COMPONENT_TESTS})
	add_subdirectory(tests)
endif()
