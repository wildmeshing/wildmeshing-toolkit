set(COMPONENT_NAME input)
add_component(${COMPONENT_NAME})
if(NOT ${WMTK_ENABLE_COMPONENT_${COMPONENT_NAME}})
    return()
endif()
set(SRC_FILES
    InputOptions.hpp
    InputOptions.cpp
    mesh_with_tag_from_image.cpp
    mesh_with_tag_from_image.hpp
    input.cpp
	input.hpp
)

include(stb)


target_compile_features(wmtk_${COMPONENT_NAME} PUBLIC cxx_std_20)

target_sources(wmtk_${COMPONENT_NAME} PRIVATE ${SRC_FILES})
target_link_libraries(wmtk_${COMPONENT_NAME} PRIVATE stb::image nlohmann_json)
target_link_libraries(wmtk_${COMPONENT_NAME} PRIVATE stb::image nlohmann_json PUBLIC wmtk::multimesh)

