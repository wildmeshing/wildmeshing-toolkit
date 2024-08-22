#pragma once


#include <string_view>
namespace wmtk::io {
class CacheStack;
class SubCacheHandle
{
public:
    // SubCacheHandle(std::string_view cache_name);
    SubCacheHandle(std::string_view cache_name, CacheStack& stack);
    SubCacheHandle(SubCacheHandle&&);
    SubCacheHandle& operator=(SubCacheHandle&&);

    ~SubCacheHandle();

private:
    CacheStack* m_stack = nullptr;
};
} // namespace wmtk::io
