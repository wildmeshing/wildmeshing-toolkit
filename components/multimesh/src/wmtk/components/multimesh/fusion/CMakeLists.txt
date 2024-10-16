set(SRC_FILES
	fusion.hpp
    fusion.cpp
    FusionOptions.hpp
    )

#CURRENT_COMPONENT_LIB_NAME is set from the main cmake
target_sources(${CURRENT_COMPONENT_LIB_NAME} PRIVATE ${SRC_FILES})