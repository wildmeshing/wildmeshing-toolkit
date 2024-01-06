
set(SRC_FILES
    ComponentOptionsBase.hpp
    json_utils.hpp
    json_utils.cpp
    get_attributes.hpp
    get_attributes.cpp
    )

target_sources(${WMTK_COMPONENT_PREFIX}_base PRIVATE ${SRC_FILES})
target_link_libraries(${WMTK_COMPONENT_PREFIX}_base PRIVATE wmtk::warnings wmtk::toolkit)
target_include_directories(${WMTK_COMPONENT_PREFIX}_base PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../../..)
