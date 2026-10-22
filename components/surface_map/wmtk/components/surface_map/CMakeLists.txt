set(COMPONENT_NAME surface_map)
add_component(${COMPONENT_NAME})

if(NOT ${WMTK_ENABLE_COMPONENT_${COMPONENT_NAME}})
    return()
endif()

set(SRC_FILES
	SurfaceMap.hpp
	SurfaceMap.cpp

	slim2d.hpp
	slim2d.cpp
	scaffold2d.hpp
	scaffold2d.cpp

	surface_io.hpp
	surface_io.cpp

	transfer.hpp
	transfer.cpp

	Tracking.hpp
	Tracking.cpp

	surface_map.hpp
	surface_map.cpp
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
