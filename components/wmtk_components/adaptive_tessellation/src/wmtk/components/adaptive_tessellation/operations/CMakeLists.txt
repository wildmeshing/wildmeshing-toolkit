set(SRC_FILES
    ATInteriorSplitAtMidpoint.hpp
    ATInteriorSplitAtMidpoint.cpp
)


target_sources(adaptive_tessellation PRIVATE ${SRC_FILES})

add_subdirectory(internal)
