set(SRC_FILES
    collect_todo_simplices.hpp
    collect_todo_simplices.cpp
    tag_todo_edges.hpp
    tag_todo_edges.cpp
)


target_sources(${CURRENT_COMPONENT_LIB_NAME} PRIVATE ${SRC_FILES})
