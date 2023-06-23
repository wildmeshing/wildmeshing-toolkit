
set(SRC_FILES
    internal/InputOptions.h
    input.h)
message(DEBUG "SRC_FILES: ${SRC_FILES}")
target_sources(wildmeshing_components PRIVATE ${SOURCES})
