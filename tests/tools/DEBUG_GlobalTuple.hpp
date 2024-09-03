#pragma once

#include <wmtk/utils/TupleInspector.hpp>


namespace wmtk::tests {

struct DEBUG_GlobalTuple
{
public:
    DEBUG_GlobalTuple(const Mesh& m, const Tuple& t)
        : m_data(t)
    {}

    int64_t vid() const { return wmtk::utils::TupleInspector::local_vid(m_data); }
    int64_t eid() const { return wmtk::utils::TupleInspector::local_eid(m_data); }
    int64_t fid() const { return wmtk::utils::TupleInspector::local_fid(m_data); }
    int64_t cid() const { return wmtk::utils::TupleInspector::global_cid(m_data); }


    operator std::string() const;
    operator Tuple() const { return m_data; }

private:
    Tuple m_data;
};
} // namespace wmtk::tests
