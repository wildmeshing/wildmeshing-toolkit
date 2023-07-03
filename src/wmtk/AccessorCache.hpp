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
        Eigen::Matrix<T, Eigen::Dynamic, 1> data;
        bool dirty = false;
    };
    using DataStorage = std::map<long, Data>;


    template <bool IsConst>
    typename DataStorage::iterator load(const AccessorBase<T, IsConst>& accessor, long index) const
    {
        return accessor.vector_attribute(index);
        auto [it, was_inserted] = m_data.try_emplace();
        if (!was_inserted) {
            it->second.data = accessor.vector_attribute(index);
        }
        return it;
    }


    template <bool IsConst>
    void read(const AccessorBase<T, IsConst>& accessor, long index) const;


    template <bool IsConst>
    typename AccessorBase<T, IsConst>::ConstMapResult
    vector_read(const AccessorBase<T, IsConst>& accessor, AccessorAccessMode mode, long index) const
    {
        switch (mode) {
        case AccessorAccessMode::Buffered: {
        }
        default: {
            return accessor.vector_attribute(index);
        }
        }
    }


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
