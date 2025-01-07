set(COMPONENT_NAME "regular_space")
add_component(${COMPONENT_NAME})

if(NOT ${WMTK_ENABLE_COMPONENT_${COMPONENT_NAME}})
    return()
endif()
set(SRC_FILES
    internal/RegularSpaceOptions.cpp
    internal/RegularSpaceOptions.hpp
    internal/RegularSpace.hpp
    internal/RegularSpace.cpp
    #regular_space.hpp
    #regular_space.cpp
    )


#CURRENT_COMPONENT_LIB_NAME is set from the main cmake
target_sources(wmtk_${COMPONENT_NAME} PRIVATE ${SRC_FILES})
