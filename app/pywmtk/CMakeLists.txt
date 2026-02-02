pybind11_add_module(pywmtk pywmtk.cpp)

target_link_libraries(pywmtk PUBLIC
    pybind11::module
    pybind11_json
    wmtk::toolkit
    nlohmann_json::nlohmann_json
)
target_link_libraries(pywmtk PRIVATE
    FastEnvelope::FastEnvelope
)

set_target_properties(pywmtk PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")