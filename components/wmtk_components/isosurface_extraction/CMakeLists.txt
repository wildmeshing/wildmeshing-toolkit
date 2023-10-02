
set(SRC_FILES
    internal/IsosurfaceExtractionOptions.hpp
    internal/IsosurfaceExtraction.hpp
    internal/IsosurfaceExtraction.cpp
	isosurface_extraction.hpp
    isosurface_extraction.cpp
    )
target_sources(wildmeshing_components PRIVATE ${SRC_FILES})
