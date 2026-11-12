# No LTO: pybind11_add_module adds its own unless CMAKE_INTERPROCEDURAL_OPTIMIZATION is defined
# (OFF is CMake's default for every target, so this directory loses nothing). The components the
# module links are static libraries of plain objects, so LTO only ever reached the binding code,
# and mixing the two broke GCC 15.2's link of the offset module, which links the same libraries;
# see components/topological_offset/.../CMakeLists.txt. The Release strip and MSVC's extras stay.
set(CMAKE_INTERPROCEDURAL_OPTIMIZATION OFF)
pybind11_add_module(wildmeshing pywildmeshing.cpp mesh_conversion.cpp)

target_link_libraries(wildmeshing PUBLIC
    pybind11::module
    pybind11_json
    nlohmann_json::nlohmann_json
)
# PolyFEM in process (polyfem_bindings.cpp): with WMTK_WITH_POLYFEM only.
if(WMTK_WITH_POLYFEM)
    target_sources(wildmeshing PRIVATE polyfem_bindings.cpp)
    target_link_libraries(wildmeshing PRIVATE polyfem::polyfem)
    target_compile_definitions(wildmeshing PRIVATE WMTK_WITH_POLYFEM)
endif()
# The component registry (components_*.hpp), which components/CMakeLists.txt generates in app/
# for wmtk_app, the integration tests and this module.
target_include_directories(wildmeshing PRIVATE "${PROJECT_SOURCE_DIR}/app")

if(NOT MSVC)
    target_compile_options(wildmeshing PUBLIC -fPIC)
endif(NOT MSVC)

# The module is wildmeshing._core. The build tree mirrors the Python package, so that
# PYTHONPATH=<build>/bin imports the same `wildmeshing` package a wheel installs. The $<1:...>
# keeps multi-config generators (Visual Studio, which the Windows wheels use) from adding a
# Release/ folder, which would part the module from the package's __init__.py.
set(WMTK_PYTHON_PACKAGE_DIR "${CMAKE_BINARY_DIR}/bin/wildmeshing")
set_target_properties(wildmeshing PROPERTIES
    OUTPUT_NAME _core
    LIBRARY_OUTPUT_DIRECTORY "$<1:${WMTK_PYTHON_PACKAGE_DIR}>")
add_custom_target(wildmeshing_python_sources
    COMMAND "${CMAKE_COMMAND}" -E copy_directory
        "${CMAKE_CURRENT_SOURCE_DIR}/../wildmeshing" "${WMTK_PYTHON_PACKAGE_DIR}")
add_dependencies(wildmeshing wildmeshing_python_sources)

# On Windows the module loads GMP's DLL, which Python (3.8 and later) looks for next to the module,
# not on PATH: the build tree gets it there, as the wheel does (install below).
if(WIN32 AND TARGET gmp::gmp)
    add_custom_command(TARGET wildmeshing POST_BUILD
        COMMAND "${CMAKE_COMMAND}" -E copy_if_different "$<TARGET_FILE:gmp::gmp>"
            "${WMTK_PYTHON_PACKAGE_DIR}"
        VERBATIM)
endif()

# The type stub wildmeshing/_core.pyi, written from the module just built, so it cannot drift from
# the bindings: the hand-regenerated copy git held missed CurveTracking. pybind11-stubgen 3 and
# numpy: 2.5 writes os.PathLike without importing os (7 mypy errors), and without numpy the
# numpy.ndarray annotations have no import. After the GMP copy above, which the import needs.
execute_process(
    COMMAND "${Python_EXECUTABLE}" -c
        "import numpy, importlib.metadata as m; assert int(m.version('pybind11-stubgen').split('.')[0]) >= 3"
    RESULT_VARIABLE WMTK_NO_STUBGEN OUTPUT_QUIET ERROR_QUIET)
if(WMTK_NO_STUBGEN)
    message(STATUS "wildmeshing: no type stub (needs pybind11-stubgen >= 3 and numpy in "
        "${Python_EXECUTABLE})")
else()
    # Run in bin/, which `python -m` puts first on sys.path, so the package just built is the one
    # imported; PYTHONPATH stays as it is: pip's isolated build puts pybind11-stubgen there.
    add_custom_command(TARGET wildmeshing POST_BUILD
        COMMAND "${Python_EXECUTABLE}" -m pybind11_stubgen --exit-code wildmeshing._core
            -o "${CMAKE_BINARY_DIR}/bin"
        WORKING_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
        COMMENT "Writing the type stub wildmeshing/_core.pyi"
        VERBATIM)
    if(DEFINED SKBUILD)
        install(FILES "${WMTK_PYTHON_PACKAGE_DIR}/_core.pyi"
            DESTINATION "${SKBUILD_PLATLIB_DIR}/wildmeshing" COMPONENT python_bindings)
    endif()
endif()

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
