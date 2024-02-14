set(SRC_FILES
    Image.cpp
    Image.hpp
    Sampling.hpp
)


target_sources(${CURRENT_COMPONENT_LIB_NAME} PRIVATE ${SRC_FILES})

add_subdirectory(utils)