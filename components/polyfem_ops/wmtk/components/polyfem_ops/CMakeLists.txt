# polyfem_ops: the polyfem-backed simwild operations (minimum separation, interface smoothing),
# ported from pysimwild's Python glue. This component is the JSON entry ("application":
# "polyfem_ops"); the polyfem-facing code it calls lives in wmtk::simwild, under
# components/simwild/wmtk/components/simwild/polyfem_helpers, which is also where polyfem is
# linked and the two pysimwild operation specs are embedded. Built only with WMTK_WITH_POLYFEM
# (components/CMakeLists.txt).

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

target_link_libraries(wmtk_${COMPONENT_NAME} PUBLIC
	wmtk::toolkit
	wmtk::simwild
	jse::jse
)
