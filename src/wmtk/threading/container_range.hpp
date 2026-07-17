#pragma once

namespace wmtk::threading {
// ---------------------------------------------------------------------------
// A trivial "whole container" range object, returned by concurrent_map::range()
// and consumed by parallel_for below (iterated serially).
// ---------------------------------------------------------------------------
template <typename It>
class container_range
{
    It m_begin, m_end;

public:
    container_range(It b, It e)
        : m_begin(b)
        , m_end(e)
    {}
    It begin() const { return m_begin; }
    It end() const { return m_end; }
    bool empty() const { return m_begin == m_end; }
};

} // namespace wmtk::threading