# Core library
file(GLOB_RECURSE LIB_SOURCES CONFIGURE_DEPENDS
    "src/*.cpp"
    "src/*.h"
    "src/*.hpp"
)

if(NOT WMTK_APP_SEC)
message(FATAL_ERROR "Remeshing requires shorted edge collapse app; try WMTK_APP_SEC=ON")
endif()


add_library(wmtk_remeshing_lib "${LIB_SOURCES}")
add_library(wmtk::remeshing_lib ALIAS wmtk_remeshing_lib)

# Include headers
target_include_directories(wmtk_remeshing_lib PUBLIC src)

# Direct dependencies
target_link_libraries(wmtk_remeshing_lib PUBLIC
	wmtk::toolkit
	wmtk::data
	igl::predicates
	wmtk::sec_lib
)
target_link_libraries(wmtk_remeshing_lib PRIVATE
    FastEnvelope::FastEnvelope
)

add_subdirectory(app)
if(WMTK_APP_UNIT_TESTS)
    add_subdirectory(tests)
endif()
