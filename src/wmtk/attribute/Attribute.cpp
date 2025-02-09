#include "Attribute.hpp"
#include <numeric>
#include <wmtk/io/MeshWriter.hpp>
#include <wmtk/utils/Rational.hpp>
#include <wmtk/utils/vector_hash.hpp>

namespace wmtk::attribute {


template <typename T>
void Attribute<T>::serialize(int dim, MeshWriter& writer) const
{
    writer.write(m_name, dim, dimension(), m_data, m_default_value);
}


template <typename T>
Attribute<T>::Attribute(const std::string& name, int64_t dimension, T default_value, int64_t size)
    : m_dimension(dimension)
    , m_default_value(default_value)
    , m_name(name)
{
    assert(m_dimension > 0);
    reserve(size);
}

template <typename T>
Attribute<T>::Attribute(Attribute&& o) = default;

template <typename T>
std::map<std::string, size_t> Attribute<T>::child_hashes() const
{
    std::map<std::string, size_t> hashes;
    hashes["dimension"] = m_dimension;
    if constexpr (std::is_same_v<T, Rational>) {
        constexpr static std::hash<std::string> h;
        hashes["default_value"] = h(m_default_value.serialize());
    } else {
        hashes["default_value"] = m_default_value;
    }
    hashes["data"] = wmtk::utils::vector_hash(m_data);
    return hashes;
}


template <typename T>
Attribute<T>::~Attribute() = default;

template <typename T>
Attribute<T>& Attribute<T>::operator=(Attribute&& o) = default;

template <typename T>
bool Attribute<T>::operator==(const Attribute<T>& o) const
{
    return m_name == o.m_name && m_dimension == o.m_dimension && m_data == o.m_data &&
           m_default_value == o.m_default_value;
}


template <typename T>
void Attribute<T>::reserve(int64_t size)
{
    if (size >= 0 && size > reserved_size()) {
        m_data.resize(m_dimension * size, m_default_value);
    }
}
template <typename T>
int64_t Attribute<T>::reserved_size() const
{
    return reserved_size(m_data);
}
template <typename T>
int64_t Attribute<T>::reserved_size(const std::vector<T>& data) const
{
    return data.size() / m_dimension;
}

template <typename T>
void Attribute<T>::set(std::vector<T> val)
{
    assert(!val.empty());
    assert(val.size() % m_dimension == 0);
    m_data = std::move(val);
}
template <typename T>
void Attribute<T>::consolidate(const std::vector<int64_t>& new2old)
{
    for (int64_t i = 0; i < new2old.size(); ++i) {
        vector_attribute(i) = vector_attribute(new2old[i]);
    }

    m_data.resize(new2old.size() * m_dimension);
}

#if defined(__GNUG__) && !defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wsuggest-attribute=noreturn"
#endif
template <typename T>
void Attribute<T>::index_remap(const std::vector<T>& old2new)
{
    std::vector<Eigen::Index> indices(dimension());
    std::iota(indices.begin(), indices.end(), Eigen::Index(0));
    index_remap(old2new, indices);
}

template <typename T>
void Attribute<T>::index_remap(const std::vector<T>& old2new, const std::vector<Eigen::Index>& cols)
{
    if constexpr (std::is_same_v<T, int64_t>) {
        for (int64_t i = 0; i < reserved_size(); ++i) {
            auto vec = vector_attribute(i);
            for (Eigen::Index idx : cols) {
                int64_t& v = vec(idx);
                if (v >= 0) // Negative number are error codes, not indices
                    v = old2new[v];
            }
        }
    } else {
        throw std::runtime_error("Only int64_t attributes can be index remapped.");
    }
}
#if defined(__GNUG__) && !defined(__clang__)
#pragma GCC diagnostic pop
#endif
}

#if defined(WMTK_ENABLED_DEV_MODE)
#include "Attribute.hxx"

namespace wmtk::attribute {
#define VECTOR_DEC(TYPE, D)                                                              \
    template auto Attribute<TYPE>::const_vector_single_value<D>(                  \
        int64_t index,                                                                   \
        int8_t single_index) const -> const TYPE&;                                       \
    template auto Attribute<TYPE>::vector_single_value<D>(                  \
        int64_t index,                                                                   \
        int8_t single_index) -> TYPE&;                                       \
    template auto Attribute<TYPE>::const_vector_attribute<D>(int64_t index) const \
        -> ConstMapResult<D>;                                                            \
    template auto Attribute<TYPE>::vector_attribute<D>(int64_t index) -> MapResult<D>;\
    template auto Attribute<TYPE>::const_vector_attribute<D>(int64_t index, const std::vector<TYPE>&) const \
        -> ConstMapResult<D>;                                                            \
    template auto Attribute<TYPE>::vector_attribute<D>(int64_t index, std::vector<TYPE>&) const -> MapResult<D>;

#define SCALAR_DEC(TYPE)                                                              \
    template auto Attribute<TYPE>::const_scalar_attribute(int64_t index) const \
        -> const TYPE&;                                                            \
    template auto Attribute<TYPE>::scalar_attribute(int64_t index) -> TYPE&;\
    template auto Attribute<TYPE>::const_scalar_attribute(int64_t index, const std::vector<TYPE>& ) const \
        -> const TYPE&;                                                            \
    template auto Attribute<TYPE>::scalar_attribute(int64_t index, std::vector<TYPE>&) const-> TYPE&;


#define DEC(TYPE)        \
    SCALAR_DEC(TYPE) \
    VECTOR_DEC(TYPE, -1) \
    VECTOR_DEC(TYPE, 1)  \
    VECTOR_DEC(TYPE, 2)  \
    VECTOR_DEC(TYPE, 3)  \
    VECTOR_DEC(TYPE, 4)  \
    VECTOR_DEC(TYPE, 5)  \
    VECTOR_DEC(TYPE, 6)


DEC(double)
DEC(int64_t)
DEC(Rational)
DEC(char)
#endif


template class Attribute<char>;
template class Attribute<int64_t>;
template class Attribute<double>;
template class Attribute<Rational>;
} // namespace wmtk::attribute
