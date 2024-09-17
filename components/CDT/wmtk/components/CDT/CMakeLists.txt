set(COMPONENT_NAME CDT)
add_component(${COMPONENT_NAME})
if(NOT ${WMTK_ENABLE_COMPONENT_${COMPONENT_NAME}})
    return()
endif()

set(SRC_FILES
    internal/CDTOptions.hpp
    internal/CDT.hpp
    internal/CDT.cpp
    CDT.hpp
    CDT.cpp
    )

include(CDT)


target_sources(wmtk_${COMPONENT_NAME} PRIVATE ${SRC_FILES})

add_library(wmtk_cdt_get_vf 
    internal/get_vf.hpp
    internal/get_vf.cpp
)

add_library(wmtk_cdt_lib
    internal/cdt_lib.hpp
    internal/cdt_lib.cpp
)

target_link_libraries(wmtk_cdt_get_vf PRIVATE wmtk::toolkit igl::core)

target_link_libraries(wmtk_cdt_lib PRIVATE cdt_lib)

target_link_libraries(wmtk_${COMPONENT_NAME} PRIVATE wmtk_cdt_lib wmtk_cdt_get_vf)

