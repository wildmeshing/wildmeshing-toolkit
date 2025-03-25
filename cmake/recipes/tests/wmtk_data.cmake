# data
# License: MIT

if(TARGET wmtk::data)
    return()
endif()

include(wmtk_download_data)
wmtk_download_data()

add_library(wmtk_data INTERFACE)
add_library(wmtk::data ALIAS wmtk_data)
target_compile_definitions(wmtk_data INTERFACE WMTK_DATA_DIR=\"${DATA_DIR}\")
