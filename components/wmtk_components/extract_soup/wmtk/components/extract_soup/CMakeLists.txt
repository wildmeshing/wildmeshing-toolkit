
set(SRC_FILES
    internal/ExtractOptions.hpp
    internal/Extract_Soup.cpp
    internal/Extract_Soup.hpp
    extract_soup.cpp
	extract_soup.hpp)


include(libigl)

target_sources(${CURRENT_COMPONENT_LIB_NAME} PRIVATE ${SRC_FILES})
target_link_libraries(${CURRENT_COMPONENT_LIB_NAME} PRIVATE igl::core)
