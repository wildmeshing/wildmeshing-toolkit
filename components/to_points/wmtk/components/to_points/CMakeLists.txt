
set(SRC_FILES
    ToPtsOptions.hpp
    to_points.hpp
    to_points.cpp)


#CURRENT_COMPONENT_LIB_NAME is set from the main cmake
target_sources(${CURRENT_COMPONENT_LIB_NAME} PRIVATE ${SRC_FILES})