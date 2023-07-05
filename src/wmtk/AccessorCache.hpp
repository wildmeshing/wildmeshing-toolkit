#pragma once
#include <Eigen/Core>
#include <map>


namespace wmtk {
template <typename T, bool IsConst>
class AccessorBase;
template <typename T>
class AccessorCache
{
public:
    struct Data
    {
        template <typename Derived>
        Data(const Eigen::MatrixBase<Derived>& a, bool d = false)
            : data(a)
            , dirty(d)
        {}
        Eigen::Matrix<T, Eigen::Dynamic, 1> data;
        bool dirty = false;
    };
    using DataStorage = std::map<long, Data>;


    template <bool IsConst>
    typename DataStorage::iterator load(const AccessorBase<T, IsConst>& accessor, long index) const
    {
        if (auto it = m_data.find(index); it != m_data.end()) {
            return it;
        } else {
        }
        auto [it, was_inserted] =
            m_data.try_emplace(index, accessor.vector_attribute(index), false);
        return it;
    }


    template <bool IsConst>
    void read(const AccessorBase<T, IsConst>& accessor, long index) const;


    template <bool IsConst>
    typename AccessorBase<T, IsConst>::ConstMapResult const_vector_attribute(
        const AccessorBase<T, IsConst>& accessor,
        AccessorAccessMode mode,
        long index) const
    {
        switch (mode) {
        case AccessorAccessMode::Buffered: {
            auto it = load(accessor, index);
            const auto& dat = it->second.data;
            return typename AccessorBase<T, IsConst>::ConstMapResult(dat.data(), dat.size());
        }
        default: {
        }
        }
        return accessor.const_vector_attribute(index);
    }

    template <bool IsConst>
    typename AccessorBase<T, IsConst>::MapResultT
    vector_attribute(AccessorBase<T, IsConst>& accessor, AccessorAccessMode mode, long index) const
    {
        switch (mode) {
        case AccessorAccessMode::Buffered: {
            auto it = load(accessor, index);
            auto& dat = it->second.data;
            return typename AccessorBase<T, IsConst>::MapResultT(dat.data(), dat.size());
        }
        default: {
        }
        }
        return accessor.vector_attribute(index);
    }

    template <bool IsConst>
    typename AccessorBase<T, IsConst>::TT
    scalar_attribute(AccessorBase<T, IsConst>& accessor, AccessorAccessMode mode, long index) const
    {
        switch (mode) {
        case AccessorAccessMode::Buffered: {
            auto it = load(accessor, index);
            return it->second.data[0];
        }
        default: {
        }
        }
        return accessor.scalar_attribute(index);
    }

    template <bool IsConst>
    typename AccessorBase<T, IsConst>::T const_scalar_attribute(
        const AccessorBase<T, IsConst>& accessor,
        AccessorAccessMode mode,
        long index) const
    {
        switch (mode) {
        case AccessorAccessMode::Buffered: {
            auto it = load(accessor, index);
            return it->second.data[0];
        }
        default: {
        }
        }
        return accessor.const_scalar_attribute(index);
    }

    void clear() { m_data.clear(); }

    template <bool IsConst>
    void flush(AccessorBase<T, false>& accessor) const
    {
        for (const auto& [index, data] : m_data) {
            if (data.dirty) {
                accessor.vector_attribute(index) = data.data;
            }
        }
    }

private:
    mutable DataStorage m_data;
};
} // namespace wmtk
