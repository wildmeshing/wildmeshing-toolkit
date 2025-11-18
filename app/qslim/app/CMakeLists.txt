# Include more application dependencies here
include(cli11)

file(GLOB APP_SOURCES CONFIGURE_DEPENDS "*.hpp" "*.h" "*.cpp")
add_executable(qslim_app ${APP_SOURCES})
# Direct dependencies
target_link_libraries(qslim_app PUBLIC
	wmtk::toolkit
	wmtk::data
	igl::predicates
	FastEnvelope
)
target_link_libraries(qslim_app PUBLIC wmtk_qslim_lib)
wmtk_copy_dll(qslim_app)

set_target_properties(qslim_app PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/app")
