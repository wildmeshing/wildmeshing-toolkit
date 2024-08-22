#include "CacheStack.hpp"


namespace wmtk::io {
// namespace {
//  CacheStack _default_stack;
//  }
//  CacheStack& CacheStack::default_stack()
//{
//      return _default_stack;
//  }
//  void CacheStack::set_default_stack(CacheStack&& stack)
//{
//     if (_default_stack.has_sub_caches()) {
//         throw std::runtime_error("Cannot change the default stack when it still has subcaches");
//     }
//     if (stack.has_sub_caches()) {
//         throw std::runtime_error("Cannot change the default stack to a stack with sub_caches");
//     }
//     if (!stack.has_sub_caches()) {
//         _default_stack = std::move(stack);
//     }
// }
CacheStack::CacheStack()
    : CacheStack(Cache("wmtk_cache"))
{}

CacheStack::CacheStack(Cache&& cache)
{
    m_caches.push(std::move(cache));
}

const Cache& CacheStack::get_current_cache() const
{
    return m_caches.top();
}

const Cache& CacheStack::create_sub_cache(const std::string_view subcache_name)
{
    std::filesystem::path parent_path = get_current_cache().get_cache_path();

    m_caches.push(Cache(std::string(subcache_name), parent_path, false));
    return m_caches.top();
}

bool CacheStack::has_sub_caches() const
{
    return m_caches.size() == 1;
}
} // namespace wmtk::io
