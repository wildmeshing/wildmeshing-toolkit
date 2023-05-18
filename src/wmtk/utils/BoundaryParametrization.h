#pragma once

#include "Logger.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/LU>

#include <array>
#include <cmath>

namespace wmtk {
class Boundary
{
public:
    struct ParameterizedCurves
    {
        // List of 2d positions for each curve
        std::vector<std::vector<Eigen::Vector2d>> positions;

        // Whether a curve is a closed periodic curve, or an open polyline
        std::vector<bool> periodic;

        // Mapping from curve_id to its parent curve_id.
        // - For non-seam curves, this is the identity function.
        // - For a pair of seam curves (i, j), we map one to the other. I.e. parent[i] = i and
        //   parent[j] = i.
        std::vector<int> parent_curve;

        // Cumulative arclength along each curve. The size of the arclength vector is v+1, where v
        // is the number of vertices along the curve.
        std::vector<std::vector<double>> arclengths;
    };

    struct ParameterizedSegment
    {
        // First endpoint
        Eigen::Vector2d A;

        // Second endpoint
        Eigen::Vector2d B;

        // Arclength at the start of the segment (A)
        double t0;

        // Normalization factor for the parameter t
        double tlen;
    };

private:
    ParameterizedCurves m_curves;

public:
    ///
    /// Computes a list of parameterized curves from the boundary loops of a mesh. Optional seam
    /// edges can be provided, causing loops to be split at T-junctions between seams. Curves across
    /// the same seam will have a "parent curve" defined between the two of them.
    ///
    /// @note       **IMPORTANT** Vertices on the left column of E0 must match vertices on the left
    ///             column of E1. I.e. the orientation matters for E0 and E1.
    ///
    /// @param[in]  V     #V x 2 matrix of mesh vertices.
    /// @param[in]  F     #F x 3 matrix of mesh faces.
    /// @param[in]  E0    #E x 2 matrix of seam edges (left side).
    /// @param[in]  E1    #E x 2 matrix of seam edges (right side).
    ///
    void construct_boundaries(
        const Eigen::MatrixXd& V,
        const Eigen::MatrixXi& F,
        const Eigen::MatrixXi& E0,
        const Eigen::MatrixXi& E1);

    ///
    /// Computes information required to evaluate a parameterized point along a curve.
    ///
    /// @param[in]  curve_id  Curve index that the point belongs to.
    /// @param[in]  t         DoF of the point along the curve.
    ///
    /// @return     Parameterized segment info.
    ///
    ParameterizedSegment t_to_segment(int curve_id, double t) const;

    Eigen::Vector2d t_to_uv(int curve_id, double t) const
    {
        auto s = t_to_segment(curve_id, t);
        return s.A + (s.B - s.A) * (t - s.t0) / s.tlen;
    }

    /// @return A pair (curve_id, t) such that uv = t_to_uv(curve_id, t)
    std::pair<int, double> uv_to_t(const Eigen::Vector2d& v) const;

    std::pair<int, int> uv_to_ij(const Eigen::Vector2d& v, double& t) const;

    double upper_bound(int curve_id) const
    {
        return m_curves.arclengths[parent_curve(curve_id)].back();
    }

    bool is_periodic(int curve_id) const { return m_curves.periodic[curve_id]; }

    size_t num_curves() const { return m_curves.arclengths.size(); }

    // ==== debug unit tests ====
    double get_t_at_x(int curve_id, int x) const { return m_curves.arclengths[curve_id][x]; }
    Eigen::Vector2d get_position_at_x(int curve_id, int x) const
    {
        return m_curves.positions[curve_id][x];
    }
    double curve_size(int curve_id) const { return m_curves.arclengths[curve_id].size(); }

protected:
    const std::vector<Eigen::Vector2d>& positions(int curve_id) const
    {
        return m_curves.positions[curve_id];
    }

    const std::vector<double>& arclengths(int curve_id) const
    {
        return m_curves.arclengths[curve_id];
    }

    int parent_curve(int curve_id) const { return m_curves.parent_curve[curve_id]; }
};

} // namespace wmtk
