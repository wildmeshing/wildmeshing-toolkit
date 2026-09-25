# polyfem_ops: the old JSON entry ("application": "polyfem_ops") of the polyfem-backed simwild
# operations (minimum separation, interface smoothing), kept as a forwarder while it is compared
# with the simwild entry. The operations live in wmtk::simwild, under
# components/simwild/wmtk/components/simwild/polyfem_helpers, which is also where polyfem is linked
# and where their parameters are specified (simwild_spec.json). Built only with WMTK_WITH_POLYFEM
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
