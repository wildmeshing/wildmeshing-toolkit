set(SRC_FILES
    DerivedReferenceWrapperVariantTraits.hpp
    ReferenceWrappedFunctorReturnCache.hpp
    ReferenceWrappedFunctorReturnType.hpp
    ReferenceWrapperVariant.hpp
    as_variant.hpp
    unwrap_ref.hpp
    )

target_sources(wildmeshing_toolkit PRIVATE ${SRC_FILES})

add_subdirectory(tuple)
