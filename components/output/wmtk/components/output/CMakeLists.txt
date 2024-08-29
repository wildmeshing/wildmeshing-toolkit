
set(SRC_FILES
    internal/OutputOptions.hpp
    output.cpp
	output.hpp)


#COMPONENT_TARGET_NAME is set from the main cmake
target_sources(${COMPONENT_TARGET_NAME} PRIVATE ${SRC_FILES})
target_link_libraries(${COMPONENT_TARGET_NAME} PRIVATE wmtk::component_utils)