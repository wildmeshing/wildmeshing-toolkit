# Include more application dependencies here
include(cli11)

file(GLOB APP_SOURCES CONFIGURE_DEPENDS "*.hpp" "*.h" "*.cpp")
add_executable(cad_remeshing_app ${APP_SOURCES})
# Direct dependencies
target_link_libraries(cad_remeshing_app PUBLIC
	wmtk::toolkit
	wmtk::data
	igl::predicates
	jse::jse
	FastEnvelope
	CLI11::CLI11
)
target_link_libraries(cad_remeshing_app PUBLIC wmtk_cad_remeshing_lib)

# Generate spec from JSON
jse_add_spec(wmtk_cad_remeshing_app_spec
	INPUT "cad_remeshing_spec.json"
	OUTPUT "${WMTK_GENERATED_INCLUDE_DIR}/cad_remeshing_app/cad_remeshing_spec.hpp"
)
target_link_libraries(cad_remeshing_app PRIVATE wmtk_cad_remeshing_app_spec)
wmtk_copy_dll(cad_remeshing_app)

set_target_properties(cad_remeshing_app PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/app")
