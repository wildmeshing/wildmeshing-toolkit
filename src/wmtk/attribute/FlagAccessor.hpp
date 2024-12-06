#pragma once

#include "Accessor.hpp"
namespace wmtk::attribute {


template <typename MeshType>
class FlagAccessor
{
public:
    enum class FlagBit : char { Active = 1 };
    using BaseAccessor = Accessor<char, MeshType, 1>;


    FlagAccessor(BaseAccessor acc)
        : m_base_accessor{std::move(acc)}
    {}
    template <typename MeshType2>
    FlagAccessor(const FlagAccessor<MeshType2>& o)
        : m_base_accessor{o.m_base_accessor}
    {}

    template <typename T>
    bool is_active(const T& t) const
    {
        return _is_active(m_base_accessor.const_scalar_attribute(t));
    }
    template <typename T>
    void set_active(const T& t)
    {
        return _set_active(m_base_accessor.const_scalar_attribute(t));
    }


    static bool _is_active(char value) { return value & FlagBit::Active; }
    static void _set_active(char& value) { return value |= FlagBit::Active; }

private:
    BaseAccessor m_base_accessor;

public:
    // utility class to enable index_access
    class IndexFlagAccessor
    {
    public:
        IndexFlagAccessor(const FlagAccessor& accessor)
            : m_accessor(accessor)
        {}
        template <typename T>
        bool is_active(const T& t) const
        {
            return _is_active(m_accessor.m_base_accessor.index_access().const_scalar_attribute(t));
        }
        template <typename T>
        void set_active(const T& t)
        {
            return _set_active(m_accessor.m_base_accessor.index_access().const_scalar_attribute(t));
        }

    private:
        const FlagAccessor& m_accessor;
    };


    IndexFlagAccessor index_access() const { return IndexAccessor(*this); }
};


} // namespace wmtk::attribute
