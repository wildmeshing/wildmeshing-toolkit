
set(SRC_FILES
	periodic_optimization.hpp
    periodic_optimization.cpp

    internal/PeriodicOptimizationOptions.hpp
    internal/periodic_optimization.hpp
    internal/periodic_optimization.cpp
    )


#CURRENT_COMPONENT_LIB_NAME is set from the main cmake
target_sources(${CURRENT_COMPONENT_LIB_NAME} PRIVATE ${SRC_FILES})

target_link_libraries(${CURRENT_COMPONENT_LIB_NAME} PRIVATE wmtkc_multimesh_from_tag)
