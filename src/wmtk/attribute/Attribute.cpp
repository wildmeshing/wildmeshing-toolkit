#include "Attribute.hpp"
#include <wmtk/attribute/PerThreadAttributeScopeStacks.hpp>
#include <wmtk/io/MeshWriter.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/Rational.hpp>
#include <wmtk/utils/vector_hash.hpp>

namespace wmtk::attribute {


template <typename T>
void Attribute<T>::serialize(const std::string& name, const int dim, MeshWriter& writer) const
{
    auto ptr = get_local_scope_stack_ptr();
    if (ptr == nullptr) {
        writer.write(name, dim, dimension(), m_data);
    } else {
        std::vector<T> data = m_data;

        ptr->flush_changes_to_vector(*this, data);
        writer.write(name, dim, dimension(), data);
    }
}


template <typename T>
Attribute<T>::Attribute(int64_t dimension, T default_value, int64_t size)
    : m_scope_stacks(new PerThreadAttributeScopeStacks<T>())
    , m_dimension(dimension)
    , m_default_value(default_value)
{
    assert(m_dimension > 0);
    if (size > 0) {
        m_data = std::vector<T>(size * dimension, m_default_value);
    }
}

template <typename T>
Attribute<T>::Attribute(const Attribute& o)
    : Attribute(o.m_dimension, o.m_default_value)
{
    m_data = o.m_data;
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
        hashes["default_numerator"] = h(m_default_value.numerator());
        hashes["default_denominator"] = h(m_default_value.denominator());
    } else {
        hashes["default_value"] = m_default_value;
    }
    hashes["data"] = wmtk::utils::vector_hash(m_data);
    return hashes;
}


template <typename T>
Attribute<T>::~Attribute() = default;

template <typename T>
Attribute<T>& Attribute<T>::operator=(const Attribute& o)
{
    m_data = o.m_data;
    m_dimension = o.m_dimension;
    m_default_value = o.m_default_value;
    return *this;
}
template <typename T>
Attribute<T>& Attribute<T>::operator=(Attribute&& o) = default;

template <typename T>
bool Attribute<T>::operator==(const Attribute<T>& o) const
{
    return m_dimension == o.m_dimension && m_data == o.m_data &&
           m_default_value == o.m_default_value;
}


template <typename T>
void Attribute<T>::reserve(const int64_t size)
{
    if (size > (m_data.size() / m_dimension)) {
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
int64_t Attribute<T>::dimension() const
{
    return m_dimension;
}

template <typename T>
void Attribute<T>::set(std::vector<T> val)
{
    assert(val.size() % m_dimension == 0);
    m_data = std::move(val);
}
template <typename T>
auto Attribute<T>::const_vector_attribute(const int64_t index) const -> ConstMapResult
{
    return const_vector_attribute(index, m_data);
}
template <typename T>
auto Attribute<T>::const_vector_attribute(const int64_t index, const std::vector<T>& data) const
    -> ConstMapResult
{
    assert(index < reserved_size(data));
    assert(data.size() % m_dimension == 0);
    assert(m_dimension > 0);
    const int64_t start = index * m_dimension;
    ConstMapResult R(data.data() + start, m_dimension);

    assert(R.size() == m_dimension);

    return R;
}


template <typename T>
auto Attribute<T>::vector_attribute(const int64_t index) -> MapResult
{
    return vector_attribute(index, m_data);
}
template <typename T>
auto Attribute<T>::vector_attribute(const int64_t index, std::vector<T>& data) const -> MapResult
{
    assert(index < reserved_size(data));
    assert(data.size() % m_dimension == 0);
    assert(m_dimension > 0);
    const int64_t start = index * m_dimension;
    MapResult R(data.data() + start, m_dimension);
    assert(R.size() == m_dimension);
    return R;
}

template <typename T>
T Attribute<T>::const_scalar_attribute(const int64_t index) const
{
    return const_scalar_attribute(index, m_data);
}
template <typename T>
T Attribute<T>::const_scalar_attribute(const int64_t index, const std::vector<T>& data) const
{
    assert(index < reserved_size(data));
    assert(m_dimension == 1);
    return data[index];
}

template <typename T>
T& Attribute<T>::scalar_attribute(const int64_t index)
{
    return scalar_attribute(index, m_data);
}
template <typename T>
T& Attribute<T>::scalar_attribute(const int64_t index, std::vector<T>& data) const
{
    assert(index < reserved_size(data));
    assert(m_dimension == 1);
    return data[index];
}
template <typename T>
AttributeScopeStack<T>* Attribute<T>::get_local_scope_stack_ptr() const
{
    if (bool(m_scope_stacks)) {
        return &m_scope_stacks->local();
    }
    return nullptr;
}

template <typename T>
void Attribute<T>::push_scope()
{
    if (m_scope_stacks) {
        m_scope_stacks->local().emplace();
    }
}
template <typename T>
void Attribute<T>::pop_scope(bool apply_updates)
{
    if (m_scope_stacks) {
        m_scope_stacks->local().pop(*this, apply_updates);
    }
}

template <typename T>
void Attribute<T>::clear_current_scope()
{
    if (m_scope_stacks) {
        m_scope_stacks->local().clear_current_scope();
    }
}

template <typename T>
void Attribute<T>::consolidate(const std::vector<int64_t>& new2old)
{
    for (int64_t i = 0; i < new2old.size(); ++i) vector_attribute(i) = vector_attribute(new2old[i]);

    m_data.resize(new2old.size() * m_dimension);
}

#if defined(__GNUG__) && !defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wsuggest-attribute=noreturn"
#endif
template <typename T>
void Attribute<T>::index_remap(const std::vector<T>& old2new)
{
    if constexpr (std::is_same_v<T, int64_t>) {
        for (int64_t i = 0; i < m_data.size(); ++i)
            if (m_data[i] >= 0) // Negative number are error codes, not indices
                m_data[i] = old2new[m_data[i]];
    } else {
        throw std::runtime_error("Only int64_t attributes can be index remapped.");
    }
}
#if defined(__GNUG__) && !defined(__clang__)
#pragma GCC diagnostic pop
#endif


template class Attribute<char>;
template class Attribute<int64_t>;
template class Attribute<double>;
template class Attribute<Rational>;
} // namespace wmtk::attribute
