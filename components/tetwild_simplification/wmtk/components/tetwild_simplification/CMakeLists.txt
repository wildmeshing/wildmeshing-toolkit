
set(SRC_FILES
    tetwild_simplification.cpp
	tetwild_simplification.hpp)


add_component("tetwild_simplification")

target_sources(wmtk_tetwild_simplification PRIVATE ${SRC_FILES})
target_link_libraries(wmtk_tetwild_simplification PRIVATE igl::core TBB::tbb)
target_compile_definitions(wmtk_tetwild_simplification PRIVATE -DWMTK_WITH_TBB)
