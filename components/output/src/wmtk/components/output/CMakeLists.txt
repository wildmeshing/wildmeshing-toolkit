if(NOT ${WMTK_ENABLE_COMPONENT_${COMPONENT_NAME}})
    return()
endif()
add_component(${COMPONENT_NAME})

set(SRC_FILES
    output.cpp
	output.hpp
    OutputOptions.hpp
    OutputOptions.cpp
    utils/format.hpp
    parse_output.hpp
    parse_output.cpp
    )


target_compile_features(wmtk_${COMPONENT_NAME} PUBLIC cxx_std_20)

target_sources(wmtk_${COMPONENT_NAME} PRIVATE ${SRC_FILES})
target_link_libraries(wmtk_${COMPONENT_NAME} PUBLIC wmtk::multimesh) 
