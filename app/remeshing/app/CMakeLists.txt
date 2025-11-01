# Include more application dependencies here
include(cli11)

file(GLOB APP_SOURCES CONFIGURE_DEPENDS "*.hpp" "*.h" "*.cpp")
add_executable(remeshing_app ${APP_SOURCES})
# Direct dependencies
target_link_libraries(remeshing_app PUBLIC
	wmtk::toolkit
	wmtk::data
	igl::predicates
	jse::jse
	FastEnvelope
)
target_link_libraries(remeshing_app PUBLIC wmtk_remeshing_lib)
wmtk_copy_dll(remeshing_app)

set_target_properties(remeshing_app PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/app")
