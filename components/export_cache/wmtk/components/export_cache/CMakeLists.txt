
set(SRC_FILES
    ExportCacheOptions.hpp
    export_cache.cpp
	export_cache.hpp)

#CURRENT_COMPONENT_LIB_NAME is set from the main cmake
target_sources(${CURRENT_COMPONENT_LIB_NAME} PRIVATE ${SRC_FILES})

