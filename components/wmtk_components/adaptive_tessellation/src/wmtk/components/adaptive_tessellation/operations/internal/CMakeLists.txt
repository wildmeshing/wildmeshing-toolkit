set(SRC_FILES
    ATData.hpp
    ATData.cpp
    ATOperations.hpp
    ATOperations.cpp
    ATOptions.hpp
)


target_sources(adaptive_tessellation PRIVATE ${SRC_FILES})