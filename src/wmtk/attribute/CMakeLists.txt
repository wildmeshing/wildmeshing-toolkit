
set(SRC_FILES
    AttributeCache.cpp
    AttributeCache.hpp
    AccessorBase.cpp
    AccessorBase.hpp
    AttributeScope.hpp
    AttributeScope.cpp
    AttributeScopeStack.hpp
    AttributeScopeStack.cpp
    AttributeCacheData.hpp
    AttributeCacheData.cpp
    AttributeManager.hpp
    AttributeManager.cpp
    PerThreadAttributeScopeStacks.hpp
    PerThreadAttributeScopeStacks.cpp
    AttributeScopeHandle.hpp
    AttributeScopeHandle.cpp
)
target_sources(wildmeshing_toolkit PRIVATE ${SRC_FILES})
