# Core library
file(GLOB_RECURSE LIB_SOURCES CONFIGURE_DEPENDS
    "src/wmtk/replayer/*.cpp"
    "src/wmtk/replayer/*.h"
    "src/wmtk/replayer/*.hpp"
)
set(LIB_SOURCES
./src/wmtk/replayer/TransportablePoints.cpp
./src/wmtk/replayer/OperationReplayer.cpp
./src/wmtk/replayer/TriMeshOperationLogger.cpp
./src/wmtk/replayer/OperationLogger.cpp
)

include(fenvelope)
add_library(wmtk_replayer_lib "${LIB_SOURCES}")
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

