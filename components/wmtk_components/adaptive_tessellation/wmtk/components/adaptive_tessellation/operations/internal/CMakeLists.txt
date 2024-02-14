set(SRC_FILES
    ATData.hpp
    ATData.cpp
    ATOperations.hpp
    ATOperations.cpp
    ATOptions.hpp
    AT_debug.cpp
    
    ATEnergies.cpp
)


target_sources(${CURRENT_COMPONENT_LIB_NAME} PRIVATE ${SRC_FILES})
