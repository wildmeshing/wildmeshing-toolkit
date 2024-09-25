

add_library(wmtk_application_utils
    src/wmtk/applications/utils/element_count_report.hpp
    src/wmtk/applications/utils/element_count_report.cpp
)


target_link_libraries(wmtk_application_utils wildmeshing_toolkit nlohmann_json)

target_include_directories(wmtk_application_utils INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/src")

add_library(wmtk::application_utils ALIAS wmtk_application_utils)
