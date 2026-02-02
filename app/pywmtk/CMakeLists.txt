pybind11_add_module(pywmtk pywmtk.cpp)

target_link_libraries(pywmtk PUBLIC
    pybind11::module
    pybind11_json
    nlohmann_json::nlohmann_json
)

if(NOT MSVC)
    target_compile_options(pywmtk PUBLIC -fPIC)
endif(NOT MSVC)

set_target_properties(pywmtk PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")