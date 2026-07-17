#pragma once

#include <cstddef>

namespace wmtk::threading {

// ---------------------------------------------------------------------------
// blocked_range: replaces tbb::blocked_range.
// ---------------------------------------------------------------------------
template <typename Index>
class blocked_range
{
    Index m_begin, m_end;

public:
    blocked_range(Index begin, Index end, std::size_t /*grainsize*/ = 1)
        : m_begin(begin)
        , m_end(end)
    {}
    Index begin() const { return m_begin; }
    Index end() const { return m_end; }
    bool empty() const { return !(m_begin < m_end); }
    std::size_t size() const { return static_cast<std::size_t>(m_end - m_begin); }
};

} // namespace wmtk::threading