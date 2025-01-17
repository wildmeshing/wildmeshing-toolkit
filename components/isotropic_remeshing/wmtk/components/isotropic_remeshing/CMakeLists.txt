set(COMPONENT_NAME isotropic_remeshing)
add_component(${COMPONENT_NAME})
if(NOT ${WMTK_ENABLE_COMPONENT_${COMPONENT_NAME}})
    return()
endif()

set(SRC_FILES
    IsotropicRemeshingOptions.cpp
    IsotropicRemeshingOptions.hpp

    IsotropicRemeshing.cpp
    IsotropicRemeshing.hpp

    IsotropicRemeshing_split.cpp
    IsotropicRemeshing_collapse.cpp
    IsotropicRemeshing_smooth.cpp
    IsotropicRemeshing_swap.cpp

	isotropic_remeshing.hpp
    isotropic_remeshing.cpp

    internal/configure_collapse.hpp
    internal/configure_collapse.cpp

    internal/configure_split.hpp
    internal/configure_split.cpp

    internal/configure_swap.hpp
    internal/configure_swap.cpp
    internal/configure_swap_tetmesh.cpp

    )


#CURRENT_COMPONENT_LIB_NAME is set from the main cmake
target_sources(wmtk_${COMPONENT_NAME} PRIVATE ${SRC_FILES})
target_link_libraries(wmtk_${COMPONENT_NAME} PRIVATE nlohmann_json wmtk::output)
