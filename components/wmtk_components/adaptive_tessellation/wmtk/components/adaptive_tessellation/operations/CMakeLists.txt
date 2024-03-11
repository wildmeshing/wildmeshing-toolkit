set(SRC_FILES
    RGRefine.cpp
    RGRefine.hpp
    RGBSplit.hpp
    RGBSplit.cpp
    RGBSwap.hpp
    RGBSwap.cpp
)


target_sources(${CURRENT_COMPONENT_LIB_NAME} PRIVATE ${SRC_FILES})

add_subdirectory(internal)