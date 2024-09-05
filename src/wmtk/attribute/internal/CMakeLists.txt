set(SRC_FILES
    hash.hpp
    hash.cpp
    AttributeCacheData.hpp
    AttributeCacheData.hxx
    AttributeTransactionStack.hxx
    AttributeTransactionStack.hpp
    )

target_sources(wildmeshing_toolkit PRIVATE ${SRC_FILES})
