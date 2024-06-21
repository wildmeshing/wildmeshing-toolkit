
set(SRC_FILES
    internal/ExtractOptions.hpp
    internal/Extract_Soup.cpp
    internal/Extract_Soup.hpp
    extract_soup.cpp
	extract_soup.hpp)


include(stb)
#CURRENT_COMPONENT_LIB_NAME is set from the main cmake
target_sources(${CURRENT_COMPONENT_LIB_NAME} PRIVATE ${SRC_FILES})
target_link_libraries(${CURRENT_COMPONENT_LIB_NAME} PRIVATE stb::image)
target_link_libraries(${CURRENT_COMPONENT_LIB_NAME} PRIVATE wmtkc_multimesh_from_tag)
