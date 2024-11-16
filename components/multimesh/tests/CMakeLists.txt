
add_component_test(wmtk::${COMPONENT_NAME} named_multimesh.cpp)

target_link_libraries(${WMTK_COMPONENT_TEST_TARGET} PRIVATE wmtk::input)
