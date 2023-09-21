
# Include more application dependencies here
include(cli11)

file(GLOB SOURCES CONFIGURE_DEPENDS "*.h" "*.cpp")
add_executable(replayer_app ${SOURCES})
# Direct dependencies
target_link_libraries(replayer_app PUBLIC
	wmtk::toolkit
	wmtk::data
	wmtk::replayer_lib
	igl::predicates
	FastEnvelope
)
target_link_libraries(replayer_app PUBLIC wmtk_replayer_lib)
wmtk_copy_dll(replayer_app)

set_target_properties(replayer_app PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/app")
