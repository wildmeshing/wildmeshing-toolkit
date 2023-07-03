#pragma once

namespace wmtk {
template <typename T>
class AccessorCache
{
public:
    template <typename IsConst>
    void read(const AccessorBase<T, IsConst>& accessor, long index) const;


    template <typename IsConst>
    ConstMapResult
    read(const AccessorBase<T, IsConst>& accessor, AccessorAccessMode mode, long index) const
    {}

private:
    struct Data
    {
        std::vector<T> data;
        bool dirty = false;
    };
    std::map<long, Data> m_write_cache;
};
} // namespace wmtk
