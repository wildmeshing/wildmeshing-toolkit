# data
# License: MIT

if(TARGET wmtk::data)
    return()
endif()

include(wmtk_download_data)
wmtk_download_data(DATA_DIR 
    https://github.com/wildmeshing/data.git
    152a561697a6e923451ca8535309cbe1e116a9fa
    )

add_library(wmtk_data INTERFACE)
add_library(wmtk::data ALIAS wmtk_data)
target_compile_definitions(wmtk_data INTERFACE WMTK_DATA_DIR=\"${DATA_DIR}\")