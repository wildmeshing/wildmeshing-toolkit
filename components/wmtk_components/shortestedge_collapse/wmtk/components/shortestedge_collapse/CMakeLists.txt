
set(SRC_FILES
    internal/SECOptions.hpp
    shortestedge_collapse.hpp
    shortestedge_collapse.cpp
    )


#CURRENT_COMPONENT_LIB_NAME is set from the main cmake
target_sources(${CURRENT_COMPONENT_LIB_NAME} PRIVATE ${SRC_FILES})
