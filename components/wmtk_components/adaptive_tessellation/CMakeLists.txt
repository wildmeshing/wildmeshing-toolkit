# add this component's recipes
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/recipes/")


include(stb)
include(tinyexr)


set(SRC_FILES
    adaptive_tessellation.hpp
    adaptive_tessellation.cpp
    )

    
add_library(adaptive_tessellation)
target_sources(adaptive_tessellation PRIVATE ${SRC_FILES})
    
add_subdirectory(src)
add_subdirectory(tests)

# Compile definitions
#target_compile_definitions(adaptive_tessellation PRIVATE TINYEXR_USE_STB_ZLIB 1)
#target_compile_definitions(adaptive_tessellation PRIVATE TINYEXR_USE_MINIZ 0)
#target_compile_definitions(adaptive_tessellation PRIVATE TINYEXR_IMPLEMENTATION)


# publicly include the main toolkit, which provides compilation features / definitions like using cpp17, NOMINMAX, etc
target_link_libraries(adaptive_tessellation PUBLIC wmtk::toolkit)

# private libraries that will just be used in internal implementation details, not public APIs
target_link_libraries(adaptive_tessellation PRIVATE wmtk::warnings)
target_link_libraries(adaptive_tessellation PRIVATE tinyexr::tinyexr)
target_link_libraries(adaptive_tessellation PUBLIC stb::image)

target_include_directories(adaptive_tessellation PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)


# link the main wildmeshing components library with this library
target_link_libraries(wildmeshing_components PUBLIC adaptive_tessellation)
