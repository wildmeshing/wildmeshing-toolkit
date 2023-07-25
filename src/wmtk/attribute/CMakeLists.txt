
set(SRC_FILES
    Attribute.hpp
    Attribute.cpp
    AttributeCache.cpp
    AttributeCache.hpp
    AttributeHandle.hpp
    AccessorBase.cpp
    AccessorBase.hpp
    AttributeScope.hpp
    AttributeScope.cpp
    MeshAttributes.cpp
    MeshAttributes.hpp
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
