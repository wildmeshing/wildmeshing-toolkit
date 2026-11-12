# The documentation's examples. The pages include these files (pymdownx.snippets), so what a
# page shows is what is built and run here:
#
#   cpp/<name>.cpp      one program each, linking every component; ctest runs it
#   python/<name>.py    run by tests/python/test_docs_examples.py
#   json/<name>.json    run through wmtk_app by the same test
#   pipeline/<name>.json  a wildmeshing.pipeline file, run by the same test
#
# Each example reads the example meshes from $WMTK_DATA, data/models by default (from the
# repository root), and writes its output to the directory it runs in. The tests run them in the
# build tree, in Release builds only: the examples run whole components, too slow for Debug.

get_property(WMTK_DOC_EXAMPLE_COMPONENTS GLOBAL PROPERTY WMTK_COMPONENTS)
list(TRANSFORM WMTK_DOC_EXAMPLE_COMPONENTS PREPEND "wmtk::")

set(WMTK_DOC_EXAMPLES_RUN_DIR "${CMAKE_CURRENT_BINARY_DIR}/run")
file(MAKE_DIRECTORY "${WMTK_DOC_EXAMPLES_RUN_DIR}")

file(GLOB WMTK_DOC_CPP_EXAMPLES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/cpp/*.cpp")
foreach(source IN LISTS WMTK_DOC_CPP_EXAMPLES)
    get_filename_component(name "${source}" NAME_WE)
    add_executable(wmtk_doc_example_${name} "${source}")
    target_link_libraries(wmtk_doc_example_${name} PRIVATE wmtk::toolkit ${WMTK_DOC_EXAMPLE_COMPONENTS})
    set_target_properties(wmtk_doc_example_${name} PROPERTIES FOLDER wmtk_doc_examples)
    wmtk_copy_dll(wmtk_doc_example_${name})

    if(BUILD_TESTING AND NOT CMAKE_BUILD_TYPE STREQUAL "Debug")
        # A directory per example: ctest runs them in parallel, and two examples writing the
        # same output (first_run and isotropic_remeshing both write bunny_remeshed.msh) in one
        # directory would race.
        file(MAKE_DIRECTORY "${WMTK_DOC_EXAMPLES_RUN_DIR}/cpp_${name}")
        add_test(NAME doc_example_cpp_${name} COMMAND wmtk_doc_example_${name}
            WORKING_DIRECTORY "${WMTK_DOC_EXAMPLES_RUN_DIR}/cpp_${name}")
        set_tests_properties(doc_example_cpp_${name} PROPERTIES
            ENVIRONMENT "WMTK_DATA=${WMTK_DATA_ROOT}/models"
            TIMEOUT 1800
            COST 20)
    endif()
endforeach()

if(BUILD_TESTING AND NOT CMAKE_BUILD_TYPE STREQUAL "Debug" AND TARGET wildmeshing AND Python_EXECUTABLE)
    execute_process(
        COMMAND "${Python_EXECUTABLE}" -c "import numpy"
        RESULT_VARIABLE WMTK_DOC_EXAMPLES_NUMPY_RESULT
        OUTPUT_QUIET ERROR_QUIET)
endif()
if(WMTK_DOC_EXAMPLES_NUMPY_RESULT EQUAL 0)
    add_test(
        NAME doc_examples_python_json
        COMMAND "${CMAKE_COMMAND}" -E env
                "PYTHONPATH=$<TARGET_FILE_DIR:wildmeshing>/.."
                "WMTK_DATA=${WMTK_DATA_ROOT}/models"
                "WMTK_APP=$<TARGET_FILE:wmtk_app>"
                "WMTK_DOC_EXAMPLES_RUN_DIR=${WMTK_DOC_EXAMPLES_RUN_DIR}"
                "${Python_EXECUTABLE}" -m unittest discover
                -s "${PROJECT_SOURCE_DIR}/tests/python" -p "test_docs_examples.py")
    # COST: the longest test of a Release run (about 11 minutes), so a parallel ctest starts it
    # first rather than last.
    set_tests_properties(doc_examples_python_json PROPERTIES TIMEOUT 3600 COST 600)
endif()
