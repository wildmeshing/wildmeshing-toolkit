
set(SRC_FILES
    Attribute.hpp
    Attribute.cpp
    AttributeCache.cpp
    AttributeCache.hpp
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

    AttributeInitializationHandle.hpp
    AttributeInitializationHandle.cpp


    internal/CheckpointScope.hpp
    internal/CheckpointScope.cpp

    AttributeHandle.hpp
    TypedAttributeHandle.hpp
    MeshAttributeHandle.hpp
    MeshAttributeHandle.cpp

    AccessorBase.cpp
    AccessorBase.hpp

    CachingAccessor.cpp
    CachingAccessor.hpp
    
    TupleAccessor.cpp
    TupleAccessor.hpp
    
    ConstAccessor.cpp
    ConstAccessor.hpp

    MutableAccessor.cpp
    MutableAccessor.hpp
)
target_sources(wildmeshing_toolkit PRIVATE ${SRC_FILES})
add_subdirectory(internal)
