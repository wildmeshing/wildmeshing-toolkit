set(COMPONENT_NAME isotropic_remeshing)
add_component(${COMPONENT_NAME})
if(NOT ${WMTK_ENABLE_COMPONENT_${COMPONENT_NAME}})
    return()
endif()

set(SRC_FILES
    IsotropicRemeshingOptions.cpp
    IsotropicRemeshingOptions.hpp
	isotropic_remeshing.hpp
    isotropic_remeshing.cpp
    RegulatedIsotropicRemeshingOptions.cpp
    RegulatedIsotropicRemeshingOptions.hpp
    isotropic_remeshing_regulated.hpp
    isotropic_remeshing_regulated.cpp
    )


#CURRENT_COMPONENT_LIB_NAME is set from the main cmake
target_sources(wmtk_${COMPONENT_NAME} PRIVATE ${SRC_FILES})
target_link_libraries(wmtk_${COMPONENT_NAME} PRIVATE nlohmann_json)
