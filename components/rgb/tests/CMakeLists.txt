
add_component_test(wmtk::${COMPONENT_NAME} 
    rgb.cpp
)

target_link_libraries(${WMTK_COMPONENT_TEST_TARGET} PRIVATE wmtk::input)
