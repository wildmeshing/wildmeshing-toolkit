
set(SRC_FILES
	wildmeshing.hpp
    wildmeshing.cpp
    WildmeshingOptions.hpp
    )


#CURRENT_COMPONENT_LIB_NAME is set from the main cmake
target_sources(${CURRENT_COMPONENT_LIB_NAME} PRIVATE ${SRC_FILES})
