set(SRC_FILES
    AutoDiffTypes.hpp
    autodiff.h
    autodiff.cpp
    DofsToPosition.hpp
)
target_sources(wildmeshing_toolkit PRIVATE ${SRC_FILES})
