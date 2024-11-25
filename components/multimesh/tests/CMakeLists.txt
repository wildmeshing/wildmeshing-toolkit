
add_component_test(wmtk::${COMPONENT_NAME} 
    named_multimesh.cpp 
    get_attributes.cpp
    utils.hpp
    utils.cpp
)

target_link_libraries(${WMTK_COMPONENT_TEST_TARGET} PRIVATE wmtk::input)
