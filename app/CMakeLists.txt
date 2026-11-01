
include(jse)
# wmtk_app links CLI11::CLI11. The target used to arrive as a side effect
# of fenvelope.cmake pulling cli11 in for fast-envelope's own app, which stopped being true
# when fast-envelope dropped that app -- so declare it where it is actually used.
include(cli11)

# ###############################################################################
# WMTK Application
# ###############################################################################

add_executable(wmtk_app main.cpp)
target_link_libraries(wmtk_app PUBLIC 
    wmtk::toolkit
    nlohmann_json::nlohmann_json
    CLI11::CLI11
)

wmtk_copy_dll(wmtk_app)