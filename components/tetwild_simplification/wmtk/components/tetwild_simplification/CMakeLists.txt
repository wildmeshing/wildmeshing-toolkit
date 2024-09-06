set(COMPONENT_NAME tetwild_simplification)
add_component(${COMPONENT_NAME})
if(NOT ${WMTK_ENABLE_COMPONENT_${COMPONENT_NAME}})
    return()
endif()


set(SRC_FILES
    tetwild_simplification.cpp
	tetwild_simplification.hpp)



target_sources(wmtk_${COMPONENT_NAME} PRIVATE ${SRC_FILES})
target_link_libraries(wmtk_${COMPONENT_NAME} PRIVATE igl::core TBB::tbb)
target_compile_definitions(wmtk_${COMPONENT_NAME} PRIVATE -DWMTK_WITH_TBB)
