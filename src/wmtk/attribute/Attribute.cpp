#include "Attribute.hpp"
#include <wmtk/attribute/PerThreadAttributeScopeStacks.hpp>
#include <wmtk/io/MeshWriter.hpp>
#include <wmtk/utils/Rational.hpp>

namespace wmtk::attribute {


template <typename T>
void Attribute<T>::serialize(const std::string& name, const int dim, MeshWriter& writer) const
{
    writer.write(name, dim, dimension(), m_data);
}

template <typename T>
Attribute<T>::Attribute(long dimension)
    : m_scope_stacks(new PerThreadAttributeScopeStacks<T>())
    , m_dimension(dimension)
{
    assert(m_dimension > 0);
}

template <typename T>
Attribute<T>::Attribute(long dimension, long size)
    : Attribute(dimension)
{
    if (size > 0) {
        m_data = std::vector<T>(size * dimension, T(0));
    }
}

template <typename T>
Attribute<T>::Attribute(const Attribute& o)
    : Attribute(o.m_dimension)
{
    m_data = o.m_data;
}
template <typename T>
Attribute<T>::Attribute(Attribute&& o)
    : Attribute(o.m_dimension)
{
    m_data = std::move(o.m_data);
}
template <typename T>
Attribute<T>& Attribute<T>::operator=(const Attribute& o)
{
    m_data = o.m_data;
    m_dimension = o.m_dimension;
    return *this;
}
template <typename T>
Attribute<T>& Attribute<T>::operator=(Attribute&& o)
{
    m_data = std::move(o.m_data);
    m_dimension = o.m_dimension;
    return *this;
}

template <typename T>
bool Attribute<T>::operator==(const Attribute<T>& o) const
{
    return m_dimension == o.m_dimension && m_data == o.m_data;
}

template <typename T>
void Attribute<T>::reserve(const long size)
{
    if (size > (m_data.size() / m_dimension)) {
        m_data.resize(m_dimension * size, T(0));
    }
}
template <typename T>
long Attribute<T>::reserved_size() const
{
    return m_data.size() / m_dimension;
}
template <typename T>
long Attribute<T>::dimension() const
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
auto Attribute<T>::const_vector_attribute(const long index) const -> ConstMapResult
{
    assert(index < reserved_size());
    assert(m_dimension > 0);
    const long start = index * m_dimension;
    ConstMapResult R(m_data.data() + start, m_dimension);

    assert(R.size() == m_dimension);

    return R;
}


template <typename T>
typename Attribute<T>::MapResult Attribute<T>::vector_attribute(const long index)
{
    assert(index < reserved_size());
    assert(m_dimension > 0);
    const long start = index * m_dimension;
    MapResult R(m_data.data() + start, m_dimension);
    assert(R.size() == m_dimension);
    return R;
}

template <typename T>
T Attribute<T>::const_scalar_attribute(const long index) const
{
    assert(index < reserved_size());
    assert(m_dimension == 1);
    return m_data[index];
}

template <typename T>
T& Attribute<T>::scalar_attribute(const long index)
{
    assert(index < reserved_size());
    assert(m_dimension == 1);
    return m_data[index];
}
template <typename T>
AttributeScopeStack<T>* Attribute<T>::get_local_scope_stack_ptr()
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

template class Attribute<char>;
template class Attribute<long>;
template class Attribute<double>;
template class Attribute<Rational>;
} // namespace wmtk::attribute
