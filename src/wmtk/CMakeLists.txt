# ###############################################################################
# WildMeshingToolkit library
# ###############################################################################

# Core library
file(GLOB_RECURSE SRC_FILES CONFIGURE_DEPENDS
    "*.cpp"
    "*.h"
    "*.hpp"
)
message(DEBUG "SRC_FILES: ${SRC_FILES}")

target_sources(wildmeshing_toolkit PRIVATE ${SRC_FILES})