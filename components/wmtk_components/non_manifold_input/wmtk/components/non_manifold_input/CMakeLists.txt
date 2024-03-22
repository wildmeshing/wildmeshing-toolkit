set(SRC_FILES
    NonManifoldInputOptions.hpp
    non_manifold_input.cpp
	non_manifold_input.hpp)

#CURRENT_COMPONENT_LIB_NAME is set from the main cmake
target_sources(${CURRENT_COMPONENT_LIB_NAME} PRIVATE ${SRC_FILES})
target_link_libraries(${CURRENT_COMPONENT_LIB_NAME} PRIVATE wmtkc_input)
