
set(SRC_FILES
    internal/IsotropicRemeshingOptions.hpp
    internal/IsotropicRemeshing.hpp
    internal/IsotropicRemeshing.cpp
	isotropic_remeshing.hpp
    isotropic_remeshing.cpp
    )


#CURRENT_COMPONENT_LIB_NAME is set from the main cmake
target_sources(${CURRENT_COMPONENT_LIB_NAME} PRIVATE ${SRC_FILES})
