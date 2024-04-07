
set(SRC_FILES
    internal/MarchingWindowOptions.hpp
    internal/MarchingWindowOptions.cpp
    internal/MarchingWindow.hpp
    internal/MarchingWindow.cpp
    internal/BackupHelper.hpp
    internal/BackupHelper.cpp
	marching_window.hpp
    marching_window.cpp
    )


#CURRENT_COMPONENT_LIB_NAME is set from the main cmake
target_sources(${CURRENT_COMPONENT_LIB_NAME} PRIVATE ${SRC_FILES})
