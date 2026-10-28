pybind11_add_module(wildmeshing pywildmeshing.cpp mesh_conversion.cpp)

target_link_libraries(wildmeshing PUBLIC
    pybind11::module
    pybind11_json
    nlohmann_json::nlohmann_json
)

if(NOT MSVC)
    target_compile_options(wildmeshing PUBLIC -fPIC)
endif(NOT MSVC)

# The module is wildmeshing._core. The build tree mirrors the Python package, so that
# PYTHONPATH=<build>/bin imports the same `wildmeshing` package a wheel installs.
set(WMTK_PYTHON_PACKAGE_DIR "${CMAKE_BINARY_DIR}/bin/wildmeshing")
set_target_properties(wildmeshing PROPERTIES
    OUTPUT_NAME _core
    LIBRARY_OUTPUT_DIRECTORY "${WMTK_PYTHON_PACKAGE_DIR}")
add_custom_target(wildmeshing_python_sources
    COMMAND "${CMAKE_COMMAND}" -E copy_directory
        "${CMAKE_CURRENT_SOURCE_DIR}/../../python/wildmeshing" "${WMTK_PYTHON_PACKAGE_DIR}")
add_dependencies(wildmeshing wildmeshing_python_sources)

include(GNUInstallDirs)
if(DEFINED SKBUILD)
    install(TARGETS wildmeshing LIBRARY DESTINATION "${SKBUILD_PLATLIB_DIR}/wildmeshing" COMPONENT python_bindings)

    if(WIN32 AND TARGET gmp::gmp)
        install(
            IMPORTED_RUNTIME_ARTIFACTS gmp::gmp
            RUNTIME DESTINATION "${SKBUILD_PLATLIB_DIR}/wildmeshing"
            COMPONENT python_bindings
        )
    endif()
else()
    install(TARGETS wildmeshing LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}")
endif()
