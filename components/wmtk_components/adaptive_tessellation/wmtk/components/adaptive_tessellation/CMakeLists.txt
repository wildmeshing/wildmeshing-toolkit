
include(stb)
include(tinyexr)


set(SRC_FILES
adaptive_tessellation.hpp
adaptive_tessellation.cpp
)

target_sources(${CURRENT_COMPONENT_LIB_NAME} PRIVATE ${SRC_FILES})

add_subdirectory(image)
add_subdirectory(operations)
add_subdirectory(quadrature)
add_subdirectory(function)
add_subdirectory(multimesh/utils)
add_subdirectory(invariants)
    
#add_subdirectory(tests)

# Compile definitions
#target_compile_definitions(adaptive_tessellation PRIVATE TINYEXR_USE_STB_ZLIB 1)
#target_compile_definitions(adaptive_tessellation PRIVATE TINYEXR_USE_MINIZ 0)
#target_compile_definitions(adaptive_tessellation PRIVATE TINYEXR_IMPLEMENTATION)

# private libraries that will just be used in internal implementation details, not public APIs
target_link_libraries(${CURRENT_COMPONENT_LIB_NAME} PRIVATE tinyexr::tinyexr)
target_link_libraries(${CURRENT_COMPONENT_LIB_NAME} PUBLIC stb::image)

include(FetchContent)
FetchContent_Declare(
    finite-diff
    GIT_REPOSITORY https://github.com/zfergus/finite-diff.git
    GIT_TAG 60f9fee
    GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(finite-diff)
target_link_libraries(${CURRENT_COMPONENT_LIB_NAME} PUBLIC finitediff::finitediff)