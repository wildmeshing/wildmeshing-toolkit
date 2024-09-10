set(
    SRC_FILES
    internal/CDTOptions.hpp
    internal/CDT.hpp
    internal/CDT.cpp
    CDT.hpp
    CDT.cpp

)

include(CDT)

target_sources(${CURRENT_COMPONENT_LIB_NAME} PRIVATE ${SRC_FILES})

add_library(wmtk_cdt_get_vf 
    internal/get_vf.hpp
    internal/get_vf.cpp
)

target_link_libraries(wmtk_cdt_get_vf PRIVATE wmtk::toolkit)

target_link_libraries(${CURRENT_COMPONENT_LIB_NAME} PRIVATE
    wmtkc_multimesh_from_tag cdt_lib wmtk_cdt_get_vf)
