# Core library
file(GLOB_RECURSE LIB_SOURCES CONFIGURE_DEPENDS
    "src/*.cpp"
    "src/*.h"
)
add_library(wmtk_qslim_lib "${LIB_SOURCES}")
add_library(wmtk::qslim_lib ALIAS wmtk_qslim_lib)

# Include headers
target_include_directories(wmtk_qslim_lib PUBLIC src)

# Direct dependencies
target_link_libraries(wmtk_qslim_lib PUBLIC
	wmtk::toolkit
	wmtk::data
	igl::predicates
	jse::jse
	FastEnvelope
)

add_subdirectory(app)
if(WMTK_APP_UNIT_TESTS)
    add_subdirectory(tests)
endif()
