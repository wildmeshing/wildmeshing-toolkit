

set(SRC_FILES
    AttributeScopeHandle.hpp
    AttributeScopeHandle.cpp
    UseParentScopeRAII.hpp
    UseParentScopeRAII.cpp
    )
target_sources(wildmeshing_toolkit PRIVATE ${SRC_FILES})

