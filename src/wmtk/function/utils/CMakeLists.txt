set(SRC_FILES
    AutoDiffUtils.hpp
    autodiff.h
    autodiff.cpp
    DofsToPosition.hpp
    AutoDiffRAII.hpp
    AutoDiffRAII.cpp
    amips.hpp
    amips.cpp
)
target_sources(wildmeshing_toolkit PRIVATE ${SRC_FILES})
