set(COMPONENT_NAME isotropic_remeshing)
add_component(${COMPONENT_NAME})

if(NOT ${WMTK_ENABLE_COMPONENT_${COMPONENT_NAME}})
    return()
endif()

set(SRC_FILES
    IsotropicRemeshing.cpp
	IsotropicRemeshing.h

	isotropic_remeshing.hpp
	isotropic_remeshing.cpp
	isotropic_remeshing_spec.hpp
)

target_sources(wmtk_${COMPONENT_NAME} PRIVATE ${SRC_FILES})

target_link_libraries(wmtk_${COMPONENT_NAME} PUBLIC
	wmtk::toolkit
	# igl::predicates
)

######################
## TESTS
######################

if(${WMTK_COMPONENT_TESTS})
    add_subdirectory(tests)
endif()