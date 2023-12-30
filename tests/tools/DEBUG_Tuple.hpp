#pragma once

#include <wmtk/utils/TupleInspector.hpp>


namespace wmtk::tests {

struct DEBUG_Tuple
{
public:
    DEBUG_Tuple(const Tuple& t)
        : m_data(t)
    {}

    int64_t local_vid() const { return wmtk::utils::TupleInspector::local_vid(m_data); }
    int64_t local_eid() const { return wmtk::utils::TupleInspector::local_eid(m_data); }
    int64_t local_fid() const { return wmtk::utils::TupleInspector::local_fid(m_data); }
    int64_t global_cid() const { return wmtk::utils::TupleInspector::global_cid(m_data); }
    int64_t hash() const { return wmtk::utils::TupleInspector::hash(m_data); }


    operator std::string() const;
    operator Tuple() const { return m_data; }

private:
    Tuple m_data;
};
} // namespace wmtk::tests
