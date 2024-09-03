
set(SRC_FILES
    triangle_insertion.hpp
    triangle_insertion.cpp)


add_component("triangle_insertion")

target_sources(wmtk_triangle_insertion PRIVATE ${SRC_FILES})

target_link_libraries(wmtk_triangle_insertion PRIVATE wmtk::multimesh_from_tag)