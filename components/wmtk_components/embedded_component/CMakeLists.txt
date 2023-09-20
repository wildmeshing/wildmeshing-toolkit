
set(SRC_FILES
    internal/EmbeddedComponentOptions.hpp
    internal/EmbeddedComponent.hpp
    internal/EmbeddedComponent.cpp
	embedded_component.hpp
    embedded_component.cpp
    )
    
target_sources(wildmeshing_components PRIVATE ${SRC_FILES})
