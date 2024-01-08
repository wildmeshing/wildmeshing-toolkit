
set(SRC_FILES
    internal/OutputOptions.hpp
    output.cpp
	output.hpp)


#CURRENT_COMPONENT_LIB_NAME is set from the main cmake
target_sources(${CURRENT_COMPONENT_LIB_NAME} PRIVATE ${SRC_FILES})
