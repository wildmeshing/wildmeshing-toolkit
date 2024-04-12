
set(SRC_FILES
    internal/ExtractOptions.hpp
    internal/Extract.cpp
    internal/Extract.hpp
    extract.cpp
	extract.hpp)


include(stb)
#CURRENT_COMPONENT_LIB_NAME is set from the main cmake
target_sources(${CURRENT_COMPONENT_LIB_NAME} PRIVATE ${SRC_FILES})
target_link_libraries(${CURRENT_COMPONENT_LIB_NAME} PRIVATE stb::image)

