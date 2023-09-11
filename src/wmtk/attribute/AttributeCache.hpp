#pragma once
#include <Eigen/Core>
#include <map>
#include <memory>
#include "AccessorBase.hpp"
#include "AttributeCacheData.hpp"


namespace wmtk::attribute {
template <typename T>
class AttributeCache
{
public:
    using Data = AttributeCacheData<T>;
    using DataStorage = std::map<long, Data>;

    using MapResult = typename AccessorBase<T>::MapResult;
    using ConstMapResult = typename AccessorBase<T>::ConstMapResult;


    AttributeCache();
    ~AttributeCache();

    // returns an iterator and if the value was inserted
    // the returned value may have undetermined state if new oen was inserted
    std::pair<typename DataStorage::iterator, bool> load_it(long index) const;


    void clear();

    void flush_to(Attribute<T>& attribute);
    void flush_to(AttributeCache<T>& other);


protected:
    mutable DataStorage m_data;
};
} // namespace wmtk
