#if defined(WMTK_ENABLED_DEV_MODE)
#include "CachingAccessor.hxx"
#include <wmtk/Mesh.hpp>

namespace wmtk::attribute {


#define CLASS_DEC(TYPE, DIM) template class CachingAccessor<TYPE, DIM>;
#define VECTOR_DEC(TYPE, DIM, D)                                                                   \
    template auto CachingAccessor<TYPE, DIM>::const_vector_attribute<D>(const int64_t index) const \
        -> ConstMapResult<std::max(D, DIM)>;                                                       \
    template auto CachingAccessor<TYPE, DIM>::vector_attribute<D>(const int64_t index)             \
        -> MapResult<std::max(D, DIM)>;

#define MULTI_VECTOR_DEC(TYPE, DIM) \
    VECTOR_DEC(TYPE, DIM, -1)       \
    VECTOR_DEC(TYPE, DIM, DIM)

#define PER_TYPE_PER_DIM_DEC(TYPE, DIM) \
    CLASS_DEC(TYPE, DIM)                \
    MULTI_VECTOR_DEC(TYPE, DIM)

#define M1_DEC(TYPE)         \
    CLASS_DEC(TYPE, -1)      \
    VECTOR_DEC(TYPE, -1, -1) \
    VECTOR_DEC(TYPE, -1, 1)  \
    VECTOR_DEC(TYPE, -1, 2)  \
    VECTOR_DEC(TYPE, -1, 3)  \
    VECTOR_DEC(TYPE, -1, 4)  \
    VECTOR_DEC(TYPE, -1, 5)  \
    VECTOR_DEC(TYPE, -1, 6)


#define PER_TYPE_DEC(TYPE)        \
    M1_DEC(TYPE)                  \
    PER_TYPE_PER_DIM_DEC(TYPE, 1) \
    PER_TYPE_PER_DIM_DEC(TYPE, 2) \
    PER_TYPE_PER_DIM_DEC(TYPE, 3) \
    PER_TYPE_PER_DIM_DEC(TYPE, 4) \
    PER_TYPE_PER_DIM_DEC(TYPE, 5) \
    PER_TYPE_PER_DIM_DEC(TYPE, 6)


PER_TYPE_DEC(double)
PER_TYPE_DEC(int64_t)
PER_TYPE_DEC(Rational)
PER_TYPE_DEC(char)


} // namespace wmtk::attribute
#endif
