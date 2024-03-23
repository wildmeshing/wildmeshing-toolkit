set(SRC_FILES
    NonManifoldSimplificationOptions.hpp
    non_manifold_simplification.cpp
	non_manifold_simplification.hpp)

#CURRENT_COMPONENT_LIB_NAME is set from the main cmake
target_sources(${CURRENT_COMPONENT_LIB_NAME} PRIVATE ${SRC_FILES})
