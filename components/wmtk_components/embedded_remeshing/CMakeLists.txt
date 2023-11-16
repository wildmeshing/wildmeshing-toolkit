
set(SRC_FILES
    internal/EmbeddedRemeshingOptions.hpp
    internal/EmbeddedRemeshing.hpp
    internal/EmbeddedRemeshing.cpp
    internal/HelperFunctions.hpp
    internal/HelperFunctions.cpp
    internal/ModelLoader.hpp
    internal/ModelLoader.cpp
	embedded_remeshing.hpp
    embedded_remeshing.cpp
    )
target_sources(wildmeshing_components PRIVATE ${SRC_FILES})
