set(SRC_FILES
    adaptive_tessellation.cpp
    )


include(stb)
include(tinyexr)
# Compile definitions
#target_compile_definitions(adaptive_tessellation PRIVATE TINYEXR_USE_STB_ZLIB 1)
#target_compile_definitions(adaptive_tessellation PRIVATE TINYEXR_USE_MINIZ 0)
#target_compile_definitions(adaptive_tessellation PRIVATE TINYEXR_IMPLEMENTATION)


#CURRENT_COMPONENT_LIB_NAME is set from the main cmake
target_sources(${CURRENT_COMPONENT_LIB_NAME} PRIVATE ${SRC_FILES})
target_link_libraries(${CURRENT_COMPONENT_LIB_NAME} PRIVATE tinyexr::tinyexr stb::image)

add_subdirectory(image)
