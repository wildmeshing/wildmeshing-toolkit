# Core library
file(GLOB_RECURSE LIB_SOURCES CONFIGURE_DEPENDS
    "src/*.cpp"
    "src/*.h"
    "src/*.hpp"
)

add_library(wmtk_sec_lib "${LIB_SOURCES}")
add_library(wmtk::sec_lib ALIAS wmtk_sec_lib)

# Include headers
target_include_directories(wmtk_sec_lib PUBLIC src)

# Direct dependencies
target_link_libraries(wmtk_sec_lib PUBLIC
	wmtk::toolkit
	wmtk::data
	igl::predicates
)

add_subdirectory(app)
if(WMTK_APP_UNIT_TESTS)
    add_subdirectory(tests)
endif()
