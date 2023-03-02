# Core library
file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS
    "src/*.cpp"
    "src/*.h"
    "src/*.hpp"
)

include(fenvelope)
add_library(wmtk_replayer_lib "${SOURCES}")
add_library(wmtk::replayer_lib ALIAS wmtk_replayer_lib)

# Include headers
include(geogram)
target_include_directories(wmtk_replayer_lib PUBLIC src)

# Direct dependencies
target_link_libraries(wmtk_replayer_lib PUBLIC
	wmtk::toolkit
	wmtk::data
	igl::predicates
	FastEnvelope
	geogram
)

