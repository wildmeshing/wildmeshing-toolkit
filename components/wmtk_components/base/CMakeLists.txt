
set(SRC_FILES
    ComponentOptionsBase.hpp
    json_utils.hpp
    json_utils.cpp
    )

target_sources(${WMTK_COMPONENT_PREFIX}_base PRIVATE ${SRC_FILES})
