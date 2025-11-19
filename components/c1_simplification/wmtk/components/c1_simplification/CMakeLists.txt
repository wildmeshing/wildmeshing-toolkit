include(gmp)


set(COMPONENT_NAME c1_simplification)
add_component(${COMPONENT_NAME})

if(NOT ${WMTK_ENABLE_COMPONENT_${COMPONENT_NAME}})
    return()
endif()

set(SRC_FILES
    c1_multimesh.hpp
    c1_multimesh.cpp 
    c1_utils.hpp
    c1_utils.cpp

    clough_tocher/clough_tocher_autogen_matrix_coeffs.hpp
    clough_tocher/clough_tocher_matrices.hpp
    clough_tocher/clough_tocher_matrices.cpp

    c1_simplification.hpp
    c1_simplification.cpp 
    c1_simplification_spec.json
    )

target_sources(wmtk_${COMPONENT_NAME} PRIVATE ${SRC_FILES})

target_link_libraries(wmtk_${COMPONENT_NAME} PUBLIC
	wmtk::toolkit
	wmtk::data
	igl::predicates
	jse::jse
)

######################
## TESTS
######################

if(${WMTK_COMPONENT_TESTS})
    add_subdirectory(tests)
endif()
