# Core library
file(GLOB_RECURSE LIB_SOURCES CONFIGURE_DEPENDS
    "src/*.cpp"
    "src/*.h"
)
add_library(wmtk_remeshing_lib "${LIB_SOURCES}")
add_library(wmtk::remeshing_lib ALIAS wmtk_remeshing_lib)

# Include headers
target_include_directories(wmtk_remeshing_lib PUBLIC src)

# Direct dependencies
target_link_libraries(wmtk_remeshing_lib PUBLIC
    wmtk::toolkit
)

add_subdirectory(app)
add_subdirectory(tests)
