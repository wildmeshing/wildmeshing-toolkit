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

include(GNUInstallDirs)
if(DEFINED SKBUILD)
    install(TARGETS pywmtk LIBRARY DESTINATION "${SKBUILD_PLATLIB_DIR}" COMPONENT python_bindings)
else()
    install(TARGETS pywmtk LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}")
endif()