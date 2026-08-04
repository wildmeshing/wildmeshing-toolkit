#pragma once

#include <Eigen/Core>
#include <memory>

// clang-format off
#include <wmtk/utils/DisableWarnings.hpp>
#include <fastenvelope/FastEnvelope.h>
#include <SimpleBVH/BVH.hpp>
#include <wmtk/utils/EnableWarnings.hpp>
// clang-format on

namespace wmtk {
class Envelope
{
public:
    virtual ~Envelope() = default;
    virtual void init(
        const std::vector<Eigen::Vector3d>& m_ver,
        const std::vector<Eigen::Vector3i>& m_faces,
        const double) {};
    virtual bool is_outside(const std::array<Eigen::Vector3d, 3>& tris) const { return false; };
    virtual bool is_outside(const Eigen::Vector3d& pts) const { return false; };
};

class ExactEnvelope : public Envelope, public fastEnvelope::FastEnvelope
{
public:
    ExactEnvelope()
        : fastEnvelope::FastEnvelope::FastEnvelope() {};
    ~ExactEnvelope() {};
    void init(
        const std::vector<Eigen::Vector3d>& m_ver,
        const std::vector<Eigen::Vector3i>& m_faces,
        const double eps)
    {
        fastEnvelope::FastEnvelope::init(m_ver, m_faces, eps);
    }
    bool is_outside(const std::array<Eigen::Vector3d, 3>& tris) const
    {
        return fastEnvelope::FastEnvelope::is_outside(tris);
    }
    bool is_outside(const Eigen::Vector3d& pts) const
    {
        return fastEnvelope::FastEnvelope::is_outside(pts);
    }
};


class SampleEnvelope : public wmtk::Envelope
{
public:
    SampleEnvelope(bool exact = false)
        : use_exact(exact) {};
    double eps2 = 1e-6;
    double sampling_dist = 1e-3;
    bool use_exact = false;

    /**
     * Debug escape hatch: when set, every is_outside() answers "inside".
     *
     * The envelope is the only thing that can reject an operation for geometric rather
     * than combinatorial reasons, so turning it off tells you whether a stalled
     * optimization is blocked by the envelope or by the mesh itself. It removes the
     * containment guarantee entirely -- the output is not usable, only diagnostic.
     *
     * Note nearest_point() is unaffected, so EnvelopeEnergy still pulls surface vertices
     * toward the input while smoothing. Only the veto goes away, which is the point: it
     * separates "the optimizer cannot move because the envelope forbids it" from "the
     * optimizer cannot move at all".
     */
    bool disabled = false;
    void init(
        const std::vector<Eigen::Vector3d>& m_ver,
        const std::vector<Eigen::Vector3i>& m_faces,
        const double);
    void init(
        const std::vector<Eigen::Vector3d>& m_ver,
        const std::vector<Eigen::Vector2i>& m_edges,
        const double);
    void init(
        const std::vector<Eigen::Vector2d>& m_ver,
        const std::vector<Eigen::Vector2i>& m_edges,
        const double);
    bool is_outside(const std::array<Eigen::Vector3d, 3>& tris) const;
    bool is_outside(const std::array<Eigen::Vector3d, 2>& edge) const;
    bool is_outside(const std::array<Eigen::Vector2d, 2>& edge) const;
    bool is_outside(const Eigen::Vector3d& pts) const;
    bool is_outside(const Eigen::Vector2d& pts) const;
    double nearest_point(const Eigen::Vector3d& pts, Eigen::Vector3d& result) const;
    double nearest_point(const Eigen::Vector2d& pts, Eigen::Vector2d& result) const;
    bool initialized() { return m_bvh != nullptr; };

    double squared_distance(const Eigen::Vector3d& p) const;
    double squared_distance(const Eigen::Vector2d& p) const;

private:
    std::vector<int> geo_vertex_ind;
    std::vector<int> geo_face_ind;
    std::shared_ptr<SimpleBVH::BVH> m_bvh;

private:
    fastEnvelope::FastEnvelope exact_envelope;
};
} // namespace wmtk