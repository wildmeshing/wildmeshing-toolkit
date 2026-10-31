# Include Catch2 stuff
include(catch2)
FetchContent_GetProperties(catch2)
list(APPEND CMAKE_MODULE_PATH ${catch2_SOURCE_DIR}/extras)
include(Catch)

# Sources
set(TEST_SOURCES
    2d_tests.cpp
    test_delaunay.cpp
    test_embed_triangles.cpp
    test_envelope.cpp
    test_geom.cpp
    test_io.cpp
    test_mesh_data.cpp
    test_knn.cpp
    test_link_condition.cpp
    test_nonmanifold.cpp
    test_operations.cpp
    test_predicates.cpp
    test_raw_simplex_collection.cpp
    test_read_triangle_mesh.cpp
    test_resolve_path.cpp
    test_winding_number.cpp
    test_simplex_navigation.cpp
    test_tuple.cpp
    test_orient.cpp
    test_orient_by_majority.cpp
    test_optimization.cpp
    test_threading.cpp
    test_vertex_coloring.cpp
    test_ring_lock.cpp
    test_slot_allocation.cpp
    test_screening.cpp
    test_history.cpp
    test_tracked_curves.cpp
)

add_executable(wmtk_tests ${TEST_SOURCES})

# Macros
target_compile_definitions(wmtk_tests PUBLIC CATCH_CONFIG_ENABLE_BENCHMARKING)

# Dependencies
include(wmtk_data)
target_link_libraries(wmtk_tests PUBLIC
    wmtk::toolkit
    wmtk::data
    Catch2::Catch2WithMain
)
add_sanitizers(wmtk_tests) # for detecting memory errors in unit tests
wmtk_copy_dll(wmtk_tests)

# Register unit tests
catch_discover_tests(wmtk_tests)

# Eigen's heap across units built with different vectorization (cmake/recipes/eigen.cmake): a unit
# with the build's settings, one with EIGEN_DONT_VECTORIZE as ipc-toolkit exports it, and, with an
# x86 compiler that takes the flags, one built for AVX2. Its own executable, linking Eigen alone:
# the AVX unit's copies of Eigen's inline functions may stand in for the others', and nothing else
# should run them.
add_executable(wmtk_test_eigen_heap
    eigen_heap/test_eigen_heap.cpp
    eigen_heap/eigen_heap_plain.cpp
    eigen_heap/eigen_heap_novec.cpp
)
set_source_files_properties(eigen_heap/eigen_heap_novec.cpp PROPERTIES
    COMPILE_DEFINITIONS EIGEN_DONT_VECTORIZE=1)
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-mavx2 -mfma" WMTK_TEST_EIGEN_HEAP_AVX)
if(WMTK_TEST_EIGEN_HEAP_AVX)
    target_sources(wmtk_test_eigen_heap PRIVATE eigen_heap/eigen_heap_avx.cpp)
    set_source_files_properties(eigen_heap/eigen_heap_avx.cpp PROPERTIES
        COMPILE_OPTIONS "-mavx2;-mfma")
    target_compile_definitions(wmtk_test_eigen_heap PRIVATE WMTK_TEST_EIGEN_HEAP_AVX)
endif()
target_link_libraries(wmtk_test_eigen_heap PRIVATE Eigen3::Eigen Catch2::Catch2WithMain)
wmtk_copy_dll(wmtk_test_eigen_heap)
catch_discover_tests(wmtk_test_eigen_heap)

if(WMTK_BUILD_INTEGRATION_TESTS)
    message(STATUS "Include integration tests.")
    add_executable(wmtk_integration_tests integration_tests.cpp)

    # Macros
    target_compile_definitions(wmtk_integration_tests PUBLIC CATCH_CONFIG_ENABLE_BENCHMARKING)

    # Dependencies
    include(wmtk_data)
    target_link_libraries(wmtk_integration_tests PUBLIC
        wmtk::toolkit
        wmtk::data
        Catch2::Catch2WithMain
    )
    add_sanitizers(wmtk_integration_tests) # for detecting memory errors in integration tests
    wmtk_copy_dll(wmtk_integration_tests)

    # Register unit tests.
    #
    # Well past ctest's 1500s default: the coarsening pass runs at the end of every tetwild and
    # triwild config and roughly triples what those cost, which is the point of it -- it trades
    # time for elements. Set on discovery rather than with set_tests_properties, because
    # catch_discover_tests registers the test at build time, when a configure-time property
    # call has nothing to attach to.
    catch_discover_tests(wmtk_integration_tests PROPERTIES TIMEOUT 7200)

endif()
# wmtk_app ends a failed run with its error and exit code 1, not with an abort: an unknown
# application is the simplest failure.
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/unknown_application.json" "{\"application\": \"none\"}")
add_test(NAME wmtk_app_reports_errors
    COMMAND wmtk_app -j "${CMAKE_CURRENT_BINARY_DIR}/unknown_application.json")
set_tests_properties(wmtk_app_reports_errors PROPERTIES
    PASS_REGULAR_EXPRESSION "wmtk_app stopped: Application none unknown")

# The NumPy tests of the Python module -- the memory interface, and surface tracking through
# it -- registered when the interpreter the module was built for has NumPy (the Linux CI jobs
# install it). The wheel builds run the same files through cibuildwheel.
if(TARGET wildmeshing AND Python_EXECUTABLE)
    execute_process(
        COMMAND "${Python_EXECUTABLE}" -c "import numpy"
        RESULT_VARIABLE WMTK_PYTHON_NUMPY_RESULT
        OUTPUT_QUIET ERROR_QUIET)
    if(WMTK_PYTHON_NUMPY_RESULT EQUAL 0)
        add_test(
            NAME python_memory_io
            COMMAND "${CMAKE_COMMAND}" -E env "PYTHONPATH=$<TARGET_FILE_DIR:wildmeshing>/.."
                    "${Python_EXECUTABLE}" -m unittest discover
                    -s "${PROJECT_SOURCE_DIR}/tests/python" -p "test_memory_io.py")
        add_test(
            NAME python_tracking
            COMMAND "${CMAKE_COMMAND}" -E env "PYTHONPATH=$<TARGET_FILE_DIR:wildmeshing>/.."
                    "${Python_EXECUTABLE}" -m unittest discover
                    -s "${PROJECT_SOURCE_DIR}/tests/python" -p "test_tracking.py")
        # The tracking tutorial's pipeline on the dense bunny, a whole remeshing and tetwild run:
        # Release builds only, as the slow cases of the component tests.
        if(NOT CMAKE_BUILD_TYPE STREQUAL "Debug")
            include(wmtk_data)
            add_test(
                NAME python_tracking_correspondences
                COMMAND "${CMAKE_COMMAND}" -E env "PYTHONPATH=$<TARGET_FILE_DIR:wildmeshing>/.."
                        "WMTK_DATA=${WMTK_DATA_ROOT}/models"
                        "${Python_EXECUTABLE}" -m unittest discover
                        -s "${PROJECT_SOURCE_DIR}/tests/python" -p "test_tracking_correspondences.py")
            set_tests_properties(python_tracking_correspondences PROPERTIES TIMEOUT 1800)
        endif()
    else()
        message(STATUS "NumPy not found for ${Python_EXECUTABLE}: python_memory_io and python_tracking are not registered")
    endif()
endif()
