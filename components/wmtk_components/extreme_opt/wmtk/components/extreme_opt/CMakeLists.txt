set(SRC_FILES

    internal/ExtremeOptOptions.hpp

    internal/ExtremeOpt.hpp
    internal/ExtremeOpt.cpp

    internal/ExtremeOptSingle.hpp
    internal/ExtremeOptSingle.cpp

    extreme_opt.hpp
    extreme_opt.cpp

    extreme_opt_single.hpp
    extreme_opt_single.cpp
)

# CURRENT_COMPONENT_LIB_NAME is set from the main cmake
target_sources(${CURRENT_COMPONENT_LIB_NAME} PRIVATE ${SRC_FILES})