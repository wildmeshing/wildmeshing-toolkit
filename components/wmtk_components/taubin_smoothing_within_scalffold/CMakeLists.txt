
set(SRC_FILES
    internal/TaubinSmoothingWithinScalffoldOptions.hpp
    internal/TaubinSmoothingWithinScalffold.hpp
    internal/TaubinSmoothingWithinScalffold.cpp
	taubin_smoothing_within_scalffold.hpp
    taubin_smoothing_within_scalffold.cpp
    )
target_sources(wildmeshing_components PRIVATE ${SRC_FILES})
