#pragma once

#include <wmtk/utils/TupleInspector.hpp>


namespace wmtk::tests {

struct DEBUG_Tuple
{
public:
    DEBUG_Tuple(const Tuple& t)
        : m_data(t)
    {}

    int64_t local_vid() const { return m_data.local_vid(); }
    int64_t local_eid() const { return m_data.local_eid(); }
    int64_t local_fid() const { return m_data.local_fid(); }
    int64_t global_cid() const { return m_data.global_cid(); }


    operator std::string() const;
    operator Tuple() const { return m_data; }

private:
    Tuple m_data;
};
} // namespace wmtk::tests
