set(COMPONENT_NAME prismatic_mesh)
add_component(${COMPONENT_NAME})

if(NOT ${WMTK_ENABLE_COMPONENT_${COMPONENT_NAME}})
    return()
endif()

set(SRC_FILES
    prismatic_mesh.hpp
    prismatic_mesh.cpp
    read_vtu.cpp
    write_vtu.cpp
    prism_main.cpp
    prismatic_mesh_spec.json
)

target_sources(wmtk_${COMPONENT_NAME} PRIVATE ${SRC_FILES})

target_link_libraries(wmtk_${COMPONENT_NAME} PUBLIC
    wmtk::toolkit
    PRIVATE tinyxml2
)

if(${WMTK_COMPONENT_TESTS})
    add_subdirectory(tests)
endif()
