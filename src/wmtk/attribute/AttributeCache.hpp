#pragma once
#include <Eigen/Core>
#include <unordered_map>


namespace wmtk {
template <typename T>
class AccessorBase;
template <typename T>
class AttributeCache
{
public:
    struct Data;
    using DataStorage = std::unordered_map<long, Data>;

    using MapResultT = typename AccessorBase<T>::MapResultT;
    using ConstMapResult = typename AccessorBase<T>::ConstMapResult;


    AttributeCache();
    ~AttributeCache();

    // returns an iterator and if the value was inserted
    // the returned value may have undetermined state if new oen was inserted
    std::pair<typename DataStorage::iterator, bool> load_it(long index) const;



    // returns an iterator and makes sure a value is set
    std::pair<typename DataStorage::iterator, bool> load_it(
        const AccessorBase<T>& accessor,
        long index) const;


    MapResult
    vector_attribute(AccessorBase<T>& accessor, AccessorAccessMode mode, long index);

    ConstMapResult const_vector_attribute(
        const AccessorBase<T>& accessor,
        AccessorAccessMode mode,
        long index) const;


        T&
    scalar_attribute(AccessorBase<T>& accessor, AccessorAccessMode mode, long index);

    T const_scalar_attribute(
        const AccessorBase<T>& accessor,
        AccessorAccessMode mode,
        long index) const;

    void clear();

    // flushes cache to parent scope unless there is no parent scope, in which
    // case it flushes data to the underlying attribute storage
    void flush(AccessorBase<T>& accessor);

private:

    MapResult load_cached_vector_value(AccessorBase<T>& accessor, long index);
    ConstMapResult load_const_cached_vector_value(const AccessorBase<T>& accessor, long index) const;
private:
    mutable DataStorage m_data;
    std::shared_ptr<AttributeCache<T>> m_parent_cache;
};
} // namespace wmtk
