set(SRC_FILES
    AutoDiffUtils.hpp
    autodiff.h
    autodiff.cpp
    DofsToPosition.hpp
)
target_sources(wildmeshing_toolkit PRIVATE ${SRC_FILES})
