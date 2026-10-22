set(COMPONENT_NAME tracking)
add_component(${COMPONENT_NAME})

if(NOT ${WMTK_ENABLE_COMPONENT_${COMPONENT_NAME}})
    return()
endif()

set(SRC_FILES
	SurfaceMap.hpp
	SurfaceMap.cpp
	CurveMap.hpp
	CurveMap.cpp

	slim2d.hpp
	slim2d.cpp
	scaffold2d.hpp
	scaffold2d.cpp

	surface_io.hpp
	surface_io.cpp

	transfer.hpp
	transfer.cpp

	tracking.hpp
	tracking.cpp

	app.cpp
)

target_sources(wmtk_${COMPONENT_NAME} PRIVATE ${SRC_FILES})

target_link_libraries(wmtk_${COMPONENT_NAME} PUBLIC
	wmtk::toolkit
)

######################
## TESTS
######################

if(${WMTK_COMPONENT_TESTS})
    add_subdirectory(tests)
endif()
