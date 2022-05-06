#pragma once

#include <Eigen/Core>
#include <memory>

namespace GEO {
class MeshFacetsAABBWithEps;
class Mesh;
} // namespace GEO

namespace sample_envelope {
class SampleEnvelope
{
public:
    SampleEnvelope() {};
	double eps2 = 1e-3;
    void init(
        const std::vector<Eigen::Vector3d>& m_ver,
        const std::vector<Eigen::Vector3i>& m_faces,
        const double);
    bool is_outside(const std::array<Eigen::Vector3d, 3>& tris);
    bool is_outside(const Eigen::Vector3d& pts);

private:
    std::shared_ptr<GEO::MeshFacetsAABBWithEps> geo_tree_ptr_;
    std::shared_ptr<GEO::Mesh> geo_polyhedron_ptr_;
    std::vector<int> geo_vertex_ind;
    std::vector<int> geo_face_ind;
};
} // namespace sample_envelope