set(SRC_FILES
    Image.cpp
    Image.hpp
    Sampling.hpp
)


target_sources(adaptive_tessellation PRIVATE ${SRC_FILES})
add_subdirectory(utils)