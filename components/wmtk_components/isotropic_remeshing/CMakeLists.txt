
set(SRC_FILES
    internal/IsotropicRemeshingOptions.hpp
    internal/IsotropicRemeshing.hpp
    internal/IsotropicRemeshing.cpp
	isotropic_remeshing.hpp
    isotropic_remeshing.cpp
    )
target_sources(wildmeshing_components PRIVATE ${SRC_FILES})
