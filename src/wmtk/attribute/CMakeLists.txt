
set(SRC_FILES
    Attribute.hpp
    Attribute.cpp
    #AttributeCache.hxx
    AttributeCache.hpp
    AttributeScope.hpp
    AttributeScope.hxx
    MeshAttributes.cpp
    MeshAttributes.hpp
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

    CachingAccessor.hxx
    CachingAccessor.hpp
    #
    Accessor.hxx
    Accessor.hpp
    
    AttributeType.hpp
    AttributeType.cpp

)
target_sources(wildmeshing_toolkit PRIVATE ${SRC_FILES})
add_subdirectory(internal)
add_subdirectory(utils)
