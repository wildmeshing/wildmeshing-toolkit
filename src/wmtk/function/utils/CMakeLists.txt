set(SRC_FILES
    AutoDiffUtils.hpp
    autodiff.h
    autodiff.cpp
    AutoDiffRAII.hpp
    AutoDiffRAII.cpp
    amips.hpp
    amips.cpp
    SimplexGetter.hpp
    SimplexGetter.cpp
)
target_sources(wildmeshing_toolkit PRIVATE ${SRC_FILES})
target_include_directories(wildmeshing_toolkit PUBLIC ${CMAKE_SOURCE_DIR}/components/wmtk_components/adaptive_tessellation/src)


# # link the main wildmeshing components library with this library
# target_link_libraries(wildmeshing_components PUBLIC adaptive_tessellation)