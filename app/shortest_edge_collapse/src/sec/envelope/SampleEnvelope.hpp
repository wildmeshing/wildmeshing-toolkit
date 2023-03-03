#pragma once

#include <Eigen/Core>
#include <memory>

// clang-format off
#include <wmtk/utils/DisableWarnings.hpp>
#include "fastenvelope/FastEnvelope.h"
#include <wmtk/utils/EnableWarnings.hpp>
// clang-format on

namespace GEO {
class MeshFacetsAABBWithEps;
class Mesh;
} // namespace GEO

namespace wmtk {
class Envelope
{
public:
    virtual ~Envelope() = default;
    virtual void init(
        const std::vector<Eigen::Vector3d>& m_ver,
        const std::vector<Eigen::Vector3i>& m_faces,
        const double){};
    virtual bool is_outside(const std::array<Eigen::Vector3d, 3>& tris) { return false; };
    virtual bool is_outside(const Eigen::Vector3d& pts) { return false; };
};

class ExactEnvelope : public Envelope, public fastEnvelope::FastEnvelope
{
public:
    ExactEnvelope()
        : fastEnvelope::FastEnvelope::FastEnvelope(){};
    ~ExactEnvelope(){};
    void init(
        const std::vector<Eigen::Vector3d>& m_ver,
        const std::vector<Eigen::Vector3i>& m_faces,
        const double eps)
    {
        fastEnvelope::FastEnvelope::init(m_ver, m_faces, eps);
    }
    bool is_outside(const std::array<Eigen::Vector3d, 3>& tris)
    {
        return fastEnvelope::FastEnvelope::is_outside(tris);
    }
    bool is_outside(const Eigen::Vector3d& pts)
    {
        return fastEnvelope::FastEnvelope::is_outside(pts);
    }
};
} // namespace wmtk
namespace sample_envelope {
class SampleEnvelope : public wmtk::Envelope
{
public:
    SampleEnvelope(bool exact = false)
        : use_exact(exact){};
    double eps2 = 1e-6;
    double sampling_dist = 1e-3;
    bool use_exact = false;
    void init(
        const std::vector<Eigen::Vector3d>& m_ver,
        const std::vector<Eigen::Vector3i>& m_faces,
        const double);
    bool is_outside(const std::array<Eigen::Vector3d, 3>& tris);
    bool is_outside(const Eigen::Vector3d& pts);
    double nearest_point(const Eigen::Vector3d& pts, Eigen::Vector3d& result) {
        auto dist = 0.;
        geo_tree_ptr_->nearest_facet(pts, result, dist);
        return dist;
    }

private:
    std::shared_ptr<GEO::MeshFacetsAABBWithEps> geo_tree_ptr_;
    std::shared_ptr<GEO::Mesh> geo_polyhedron_ptr_;
    std::vector<int> geo_vertex_ind;
    std::vector<int> geo_face_ind;

private:
    fastEnvelope::FastEnvelope exact_envelope;
};
} // namespace sample_envelope