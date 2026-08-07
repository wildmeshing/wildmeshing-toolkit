set(COMPONENT_NAME prismatic_mesh)
add_component(${COMPONENT_NAME})

if(NOT ${WMTK_ENABLE_COMPONENT_${COMPONENT_NAME}})
    return()
endif()

target_sources(wmtk_${COMPONENT_NAME} PRIVATE
    Input.cpp
    Meshing.cpp
    OffsetOperations.cpp
    Recovery.cpp
    Validity.cpp
    prismatic_mesh.cpp
)

target_link_libraries(wmtk_${COMPONENT_NAME} PUBLIC
    wmtk::toolkit
    wmtk::topological_offset
    igl::predicates
    jse::jse
)

if(${WMTK_COMPONENT_TESTS})
    add_subdirectory(tests)
endif()
