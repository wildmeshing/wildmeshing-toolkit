#pragma once

#include <cstddef>

namespace wmtk::threading {

// ---------------------------------------------------------------------------
// range: replaces tbb::range.
// ---------------------------------------------------------------------------
class range
{
    size_t m_begin, m_end;

public:
    range(size_t begin, size_t end, std::size_t /*grainsize*/ = 1)
        : m_begin(begin)
        , m_end(end)
    {}
    size_t begin() const { return m_begin; }
    size_t end() const { return m_end; }
    bool empty() const { return !(m_begin < m_end); }
    std::size_t size() const { return static_cast<std::size_t>(m_end - m_begin); }
};

} // namespace wmtk::threading