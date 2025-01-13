#pragma once
#include <wmtk/TriMesh.hpp>

namespace wmtk::utils {
class TriMeshInspector
{
    static auto& vf_accessor(TriMesh& m) { return *m.m_vf_accessor; }
    static auto& ef_accessor(TriMesh& m) { return *m.m_ef_accessor; }
    static auto& fv_accessor(TriMesh& m) { return *m.m_fv_accessor; }
    static auto& fe_accessor(TriMesh& m) { return *m.m_fe_accessor; }
    static auto& ff_accessor(TriMesh& m) { return *m.m_ff_accessor; }
};
} // namespace wmtk::utils
