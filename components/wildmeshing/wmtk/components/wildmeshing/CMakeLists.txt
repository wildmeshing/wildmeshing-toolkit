set(COMPONENT_NAME wildmeshing)
add_component(${COMPONENT_NAME})
if(NOT ${WMTK_ENABLE_COMPONENT_${COMPONENT_NAME}})
    return()
endif()


set(SRC_FILES
    wildmeshing.hpp
    wildmeshing.cpp
    internal/wildmeshing2d.hpp
    internal/wildmeshing2d.cpp
    internal/wildmeshing3d.hpp
    internal/wildmeshing3d.cpp
    internal/wildmeshing_utils.hpp
    internal/wildmeshing_utils.cpp
    internal/WildmeshingOptions.hpp)

target_sources(wmtk_${COMPONENT_NAME} PRIVATE ${SRC_FILES})
