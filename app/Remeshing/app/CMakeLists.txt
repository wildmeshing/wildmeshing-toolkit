# Include more application dependencies here
include(cli11)

file(GLOB APP_SOURCES CONFIGURE_DEPENDS "*.h" "*.cpp")
add_executable(remeshing_app ${APP_SOURCES})
target_link_libraries(remeshing_app PUBLIC
    wmtk::remeshing_lib
    CLI11::CLI11
)

set_target_properties(remeshing_app PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/app")
