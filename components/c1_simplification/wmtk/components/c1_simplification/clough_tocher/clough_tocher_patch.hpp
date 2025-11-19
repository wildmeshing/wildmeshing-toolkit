#pragma once

#include "common.h"
#include "convex_polygon.h"

#include "evaluate_surface_normal.h"
#include "optimize_spline_surface.h"

class CloughTocherPatch {

public:
  static const std::array<Eigen::Matrix<double, 3, 3>, 3>
      m_CTtri_bounds; // constant ct sub tri boundaries
  static const std::array<Eigen::Matrix<double, 10, 12>, 3>
      m_CT_matrices; // constant ct matrices for 3 sub tris

public:
  CloughTocherPatch();
  CloughTocherPatch(Eigen::Matrix<double, 12, 3> &boundary_data);

  int triangle_ind(const double &u, const double &v, const double &w) const;

  Eigen::Matrix<double, 10, 1>
  monomial_basis_eval(const double &u, const double &v, const double &w) const;

  Eigen::Matrix<double, 10, 10>
  lagrange_to_monomial_basis() const;

  Eigen::Matrix<double, 3, 1> CT_eval(const double &u, const double &v) const;

  std::array<Eigen::Matrix<double, 10, 3>, 3> get_coeffs() const;

  void set_lagrange_nodes(
    const std::array<Eigen::Vector2d, 19>& planar_nodes,
    const std::array<Eigen::Vector3d, 19>& lagrange_nodes);

  // test usage
  double external_boundary_data_eval(
      const double &u, const double &v,
      Eigen::Matrix<double, 12, 1> &external_boundary_data) const;

  /**
   * @brief Triangulate the patch.
   * 
   * @param num_subdivisions: number of subdivisions for the mesh
   * @param V: patch vertices
   * @param F: patch faces
   */
  void
  triangulate(size_t num_refinements,
              std::array<Eigen::MatrixXd, 3>& V,
              std::array<Eigen::MatrixXi, 3>& F) const;
  void
  triangulate_normalized(size_t num_refinements,
              std::array<Eigen::MatrixXd, 3>& V,
              std::array<Eigen::MatrixXi, 3>& F) const;

  /**
   * @brief Get the boundaries of the three subtriangle patches
   * 
   * @param patch_boundaries: list of 3 triangle line segments per subtriangle
   */
  void parametrize_patch_boundaries(std::array<std::array<LineSegment, 3>, 3>& patch_boundaries) const;

  Eigen::Matrix<double, 3, 1> CT_eval_normalized(int idx, const double &u,
                                                 const double &v) const;
  void view() const;

public:
  //   std::array<Eigen::Matrix<double, 12, 1>, 3> m_boundary_data;

  std::array<TriangleCornerFunctionData, 3> m_corner_data;
  std::array<TriangleMidpointFunctionData, 3> m_midpoint_data;
  Eigen::Matrix<double, 12, 3>
      m_boundary_data; // p0, p1, p2, G01, G10, G12, G21, G20, G02, N01,
                       // N12,N20, for x, y, z
  std::array<Eigen::Matrix<double, 10, 3>, 3>
      m_CT_coeffs; // constant ct matrices for 3 sub tris

  int64_t m_patch_id;
};