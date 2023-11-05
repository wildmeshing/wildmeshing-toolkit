# add this component's recipes
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/recipes/")


include(stb)
include(tinyexr)


set(SRC_FILES
    adaptive_tessellation.cpp
    )


add_library(adaptive_tessellation)
target_sources(adaptive_tessellation PRIVATE ${SRC_FILES})



# Compile definitions
target_compile_definitions(adaptive_tessellation PRIVATE TINYEXR_USE_STB_ZLIB 1)
target_compile_definitions(adaptive_tessellation PRIVATE TINYEXR_USE_MINIZ 0)
target_compile_definitions(adaptive_tessellation PRIVATE TINYEXR_IMPLEMENTATION)


# publicly include the main toolkit, whihc provides compilation features / definitions like using cpp17, NOMINMAX, etc
target_link_libraries(adaptive_tessellation PUBLIC wmtk_toolkit)

# private libraries that will just be used in internal implementation details, not public APIs
target_link_libraries(adaptive_tessellation PRIVATE wmtk::warnings)
target_link_libraries(adaptive_tessellation PRIVATE tinyexr stb)




# link the main wildmeshing components library with this library
target_link_libraries(wildmeshing_components PUBLIC adaptive_tessellation)
