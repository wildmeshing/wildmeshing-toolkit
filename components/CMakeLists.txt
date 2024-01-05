
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}components/cmake/recipes")
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}components/cmake")

add_library(wildmeshing_components)
add_library(wmtk::components ALIAS wildmeshing_components)


include(delaunay_psm)
include(jse)

target_link_libraries(wildmeshing_components PRIVATE wmtk::warnings)
target_link_libraries(wildmeshing_components PUBLIC
    wmtk::toolkit
    geogram::delaunay_psm
    jse::jse
)
# Include headers
target_include_directories(wildmeshing_components PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
add_subdirectory(wmtk_components)

# Group source files for IDEs
file(GLOB_RECURSE COMPONENTS_FILES_FOR_SOURCE_GROUP "*.cpp" "*.h" "*.hpp")
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/wmtk_components" PREFIX "components" FILES ${COMPONENTS_FILES_FOR_SOURCE_GROUP})



if(WILDMESHING_TOOLKIT_TOPLEVEL_PROJECT)
    # Unit tests
    enable_testing()

    add_subdirectory(tests)


    add_executable(wmtk_app main.cpp)
    include(jse)
    include(cli11)

    # ###############################################################################
    # WMTK JSON APP
    # ###############################################################################
    target_compile_options(wmtk_app PRIVATE "-rdynamic")
    target_link_libraries(wmtk_app PUBLIC
        wmtk::components
        CLI11::CLI11
        jse::jse
    )

    # target_compile_definitions(wmtk_app PUBLIC WMTK_APP_INPUT_SPEC="${CMAKE_SOURCE_DIR}/wmtk_spec.json")
endif()






