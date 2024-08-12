
set(SRC_FILES
    tetwild_simplification.cpp
	tetwild_simplification.hpp)


#CURRENT_COMPONENT_LIB_NAME is set from the main cmake
target_sources(${CURRENT_COMPONENT_LIB_NAME} PRIVATE ${SRC_FILES})
target_link_libraries(${CURRENT_COMPONENT_LIB_NAME} PRIVATE igl::core)

