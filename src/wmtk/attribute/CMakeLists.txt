
set(SRC_FILES
    Attribute.hpp
    Attribute.cpp
    Attribute.hxx

    TypedAttributeManager.cpp
    TypedAttributeManager.hpp
    AttributeManager.hpp
    AttributeManager.cpp
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
    CachingAccessor.cpp

    CachingAttribute.hxx
    CachingAttribute.hpp
    CachingAttribute.cpp
    #
    Accessor.hxx
    Accessor.hpp
    
    AttributeType.hpp

)
target_sources(wildmeshing_toolkit PRIVATE ${SRC_FILES})
add_subdirectory(internal)
add_subdirectory(utils)
