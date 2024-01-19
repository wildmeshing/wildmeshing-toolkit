
set(SRC_FILES
    Attribute.hpp
    Attribute.cpp
    AttributeCache.cpp
    AttributeCache.hpp
    AttributeScope.hpp
    AttributeScope.hxx
    MeshAttributes.cpp
    MeshAttributes.hpp
    AttributeScopeStack.hpp
    AttributeScopeStack.hxx
    AttributeCacheData.hpp
    AttributeCacheData.cpp
    AttributeManager.hpp
    AttributeManager.cpp
    PerThreadAttributeScopeStacks.hpp
    PerThreadAttributeScopeStacks.cpp
    AttributeScopeHandle.hpp
    AttributeScopeHandle.cpp


    internal/CheckpointScope.hpp
    internal/CheckpointScope.cpp

    AttributeHandle.hpp
    TypedAttributeHandle.hpp
    TypedAttributeHandle.cpp
    MeshAttributeHandle.hpp
    MeshAttributeHandle.cpp

    AccessorBase.hxx
    AccessorBase.hpp
    AccessorBase.cpp

    CachingAccessor.hxx
    CachingAccessor.hpp
    
    TupleAccessor.hxx
    TupleAccessor.hpp
    
    ConstAccessor.hxx
    ConstAccessor.hpp

    MutableAccessor.hxx
    MutableAccessor.hpp
)
target_sources(wildmeshing_toolkit PRIVATE ${SRC_FILES})
add_subdirectory(internal)
add_subdirectory(utils)
