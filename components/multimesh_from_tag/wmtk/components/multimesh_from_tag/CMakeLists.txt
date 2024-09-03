
set(SRC_FILES
    internal/MultiMeshFromTag.hpp
    internal/MultiMeshFromTag.cpp
	multimesh_from_tag.hpp
    multimesh_from_tag.cpp
    )

add_component("multimesh_from_tag")

target_sources(wmtk_multimesh_from_tag PRIVATE ${SRC_FILES})