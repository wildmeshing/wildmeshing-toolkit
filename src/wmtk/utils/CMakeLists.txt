
set(SRC_FILES
    Logger.cpp
    Logger.hpp
)
message(DEBUG "SRC_FILES: ${SRC_FILES}")
target_sources(wildmeshing_toolkit PRIVATE ${SOURCES})
