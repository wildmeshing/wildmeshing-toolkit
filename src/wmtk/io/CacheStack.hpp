#pragma once

#include <stack>
#include <string_view>
#include "Cache.hpp"


namespace wmtk::io {
class SubCacheHandle;
class CacheStack
{
public:
    friend class SubCacheHandle;
    // a small singleton stack that anyone can request at any time
    // static CacheStack& default_stack();
    // reset the default cache stack to be something else.
    // static void set_default_stack(CacheStack&& stack);
    // default creates a stack with the current folder as the cache
    CacheStack();
    CacheStack(Cache&& cache);
    const Cache& get_current_cache() const;


    bool has_sub_caches() const;

private:
    const Cache& create_sub_cache(const std::string_view subcache_name);
    std::stack<Cache> m_caches;
};
} // namespace wmtk::io
