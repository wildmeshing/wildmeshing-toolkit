#pragma once

#include <wmtk/utils/TupleInspector.hpp>


namespace wmtk::tests {

struct DEBUG_Tuple
{
public:
    DEBUG_Tuple(const Tuple& t)
        : m_data(t)
    {}

    long local_vid() const { return wmtk::utils::TupleInspector::local_vid(m_data); }
    long local_eid() const { return wmtk::utils::TupleInspector::local_eid(m_data); }
    long local_fid() const { return wmtk::utils::TupleInspector::local_fid(m_data); }
    long global_cid() const { return wmtk::utils::TupleInspector::global_cid(m_data); }
    long hash() const { return wmtk::utils::TupleInspector::hash(m_data); }


    operator std::string() const;
    operator Tuple() const { return m_data; }

private:
    Tuple m_data;
};
} // namespace wmtk::tests
