#include "SubCacheHandle.hpp"
#include "CacheStack.hpp"

namespace wmtk::io {

// SubCacheHandle::SubCacheHandle(std::string_view cache_name)
//     : SubCacheHandle(cache_name, CacheStack::default_stack())
//{}
SubCacheHandle::SubCacheHandle(std::string_view cache_name, CacheStack& stack)
{
    stack.create_sub_cache(cache_name);
    m_stack = &stack;
}

SubCacheHandle::SubCacheHandle(SubCacheHandle&& o)
    : m_stack(o.m_stack)
{
    o.m_stack = nullptr;
}
SubCacheHandle& SubCacheHandle::operator=(SubCacheHandle&& o)
{
    m_stack = o.m_stack;
    o.m_stack = nullptr;
    return *this;
}


SubCacheHandle::~SubCacheHandle()
{
    if (m_stack != nullptr) {
        m_stack->m_caches.pop();
    }
}
} // namespace wmtk::io
