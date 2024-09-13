set(COMPONENT_NAME isotropic_remeshing)
add_component(${COMPONENT_NAME})
if(NOT ${WMTK_ENABLE_COMPONENT_${COMPONENT_NAME}})
    return()
endif()

set(SRC_FILES
    IsotropicRemeshingOptions.hpp
	isotropic_remeshing.hpp
    isotropic_remeshing.cpp
    )


#CURRENT_COMPONENT_LIB_NAME is set from the main cmake
target_sources(wmtk_${COMPONENT_NAME} PRIVATE ${SRC_FILES})
target_link_libraries(wmtk_${COMPONENT_NAME} PRIVATE geogram::delaunay_psm)
