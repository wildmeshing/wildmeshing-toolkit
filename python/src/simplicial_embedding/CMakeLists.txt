set(SOURCES
  simplicial_embedding.cpp
)

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}" PREFIX "Source Files" FILES ${SOURCES})
target_sources(pywmtk PRIVATE ${SOURCES})