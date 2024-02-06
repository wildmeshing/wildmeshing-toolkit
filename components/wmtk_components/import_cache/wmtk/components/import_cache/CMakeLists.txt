
set(SRC_FILES
    ImportCacheOptions.hpp
    import_cache.cpp
	import_cache.hpp)

#CURRENT_COMPONENT_LIB_NAME is set from the main cmake
target_sources(${CURRENT_COMPONENT_LIB_NAME} PRIVATE ${SRC_FILES})

