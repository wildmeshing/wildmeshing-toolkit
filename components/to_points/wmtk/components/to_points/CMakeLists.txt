
set(SRC_FILES
    to_points.hpp
    to_points.cpp)


add_component("to_points")
#COMPONENT_TARGET_NAME is set from the main cmake
target_sources(wmtk_to_points PRIVATE ${SRC_FILES})