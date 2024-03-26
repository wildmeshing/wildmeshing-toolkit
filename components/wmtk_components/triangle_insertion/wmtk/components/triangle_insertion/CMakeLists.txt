
set(SRC_FILES
    TriInsOptions.hpp
    triangle_insertion.hpp
    triangle_insertion.cpp)


#CURRENT_COMPONENT_LIB_NAME is set from the main cmake
target_sources(${CURRENT_COMPONENT_LIB_NAME} PRIVATE ${SRC_FILES})

# TODO: delete after having the utils
target_link_libraries(${CURRENT_COMPONENT_LIB_NAME} PRIVATE wmtkc_multimesh_from_tag)