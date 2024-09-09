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

target_link_libraries(${CURRENT_COMPONENT_LIB_NAME} PRIVATE wmtkc_multimesh_from_tag cdt_lib)