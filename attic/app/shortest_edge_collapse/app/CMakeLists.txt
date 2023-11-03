# Include more application dependencies here
include(cli11)

file(GLOB APP_SOURCES CONFIGURE_DEPENDS "*.h" "*.cpp")
add_executable(sec_app ${APP_SOURCES})
# Direct dependencies
target_link_libraries(sec_app PUBLIC
	wmtk::toolkit
	wmtk::data
	igl::predicates
	FastEnvelope
)
target_link_libraries(sec_app PUBLIC wmtk_sec_lib)
wmtk_copy_dll(sec_app)

set_target_properties(sec_app PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/app")
