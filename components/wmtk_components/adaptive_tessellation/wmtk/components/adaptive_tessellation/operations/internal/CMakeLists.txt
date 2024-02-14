set(SRC_FILES
    ATData.hpp
    ATData.cpp
    ATOperations.hpp
    ATOperations.cpp
    ATOptions.hpp
)


target_sources(${CURRENT_COMPONENT_LIB_NAME} PRIVATE ${SRC_FILES})
