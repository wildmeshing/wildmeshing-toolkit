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
	CLI11::CLI11
)
target_link_libraries(remeshing_app PUBLIC wmtk_remeshing_lib)

# Generate spec from JSON
jse_add_spec(wmtk_remeshing_app_spec
	INPUT "remeshing_spec.json"
	OUTPUT "${WMTK_GENERATED_INCLUDE_DIR}/remeshing_app/remeshing_spec.hpp"
)
target_link_libraries(remeshing_app PRIVATE wmtk_remeshing_app_spec)
wmtk_copy_dll(remeshing_app)

set_target_properties(remeshing_app PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/app")
