set(SRC_FILES
    ATData.hpp
    ATData.cpp
    ATOperations.hpp
    ATOperations.cpp
    AT_debug.cpp
    
    ATSetAttrs.cpp

    ATScheduler.hpp
    ATScheduler.cpp    
)


target_sources(${CURRENT_COMPONENT_LIB_NAME} PRIVATE ${SRC_FILES})
