#include "clough_tocher_patch.hpp"
#include "clough_tocher_matrices.hpp"

#include <fstream>
#include <igl/per_face_normals.h>
#include <igl/per_vertex_normals.h>
#include <polyscope/point_cloud.h>
#include <polyscope/surface_mesh.h>

const std::array<Eigen::Matrix<double, 3, 3>, 3>
  CloughTocherPatch::m_CTtri_bounds = CT_subtri_bound_matrices();

const std::array<Eigen::Matrix<double, 10, 12>, 3>
  CloughTocherPatch::m_CT_matrices = CT_subtri_matrices();

CloughTocherPatch::CloughTocherPatch(
  Eigen::Matrix<double, 12, 3>& boundary_data)
  : m_boundary_data(boundary_data)
{

  // TODO: old deprecated code, hij are quadratic, wrong , just here for
  // reference
  // m_boundary_data.row(0) = m_corner_data[0].function_value; // p0
  // m_boundary_data.row(1) = m_corner_data[1].function_value; // p1
  // m_boundary_data.row(2) = m_corner_data[2].function_value; // p2

  // m_boundary_data.row(3) = m_corner_data[0].first_edge_derivative;  // d01
  // m_boundary_data.row(4) = m_corner_data[1].second_edge_derivative; // d10
  // m_boundary_data.row(5) = m_corner_data[1].first_edge_derivative;  // d12
  // m_boundary_data.row(6) = m_corner_data[2].second_edge_derivative; // d21
  // m_boundary_data.row(7) = m_corner_data[2].first_edge_derivative;  // d20
  // m_boundary_data.row(8) = m_corner_data[0].second_edge_derivative; // d02

  // m_boundary_data.row(9) = m_midpoint_data[2].normal_derivative;  // h01
  // m_boundary_data.row(10) = m_midpoint_data[0].normal_derivative; // h12
  // m_boundary_data.row(11) = m_midpoint_data[1].normal_derivative; // h20

  // compute coeff matrices
  for (int i = 0; i < 3; ++i) {
    m_CT_coeffs[i] = m_CT_matrices[i] * m_boundary_data;
  }
}

void
CloughTocherPatch::set_lagrange_nodes(
  const std::array<Eigen::Vector2d, 19>& planar_nodes,
  const std::array<Eigen::Vector3d, 19>& lagrange_nodes)
{
  // TODO: Obtained from affine_manifold.cpp. Make standalone function
  const std::array<PlanarPoint, 19> CT_nodes = { {
    PlanarPoint(1., 0.),           // b0    0
    PlanarPoint(0., 1.),           // b1    1
    PlanarPoint(0., 0.),           // b2    2
    PlanarPoint(2. / 3., 1. / 3.), // b01   3
    PlanarPoint(1. / 3., 2. / 3.), // b10   4
    PlanarPoint(0., 2. / 3.),      // b12   5
    PlanarPoint(0., 1. / 3.),      // b21   6
    PlanarPoint(1. / 3., 0.),      // b20   7
    PlanarPoint(2. / 3., 0.),      // b02   8
    PlanarPoint(4. / 9., 4. / 9.), // b01^c 9
    PlanarPoint(1. / 9., 4. / 9.), // b12^c 10
    PlanarPoint(4. / 9., 1. / 9.), // b20^c 11
    PlanarPoint(7. / 9., 1. / 9.), // b0c   12
    PlanarPoint(5. / 9., 2. / 9.), // bc0   13
    PlanarPoint(1. / 9., 7. / 9.), // b1c   14
    PlanarPoint(2. / 9., 5. / 9.), // bc1   15
    PlanarPoint(1. / 9., 1. / 9.), // b2c   16
    PlanarPoint(2. / 9., 2. / 9.), // bc2   17
    PlanarPoint(1. / 3., 1. / 3.), // bc    18
  } };

  // TODO: make global
  std::array<std::array<int64_t, 10>, 3> local_nodes = {
    { { { 0, 1, 18, 3, 4, 14, 15, 13, 12, 9 } },
      { { 1, 2, 18, 5, 6, 16, 17, 15, 14, 10 } },
      { { 2, 0, 18, 7, 8, 12, 13, 17, 16, 11 } } }
  };
  // std::array<int64_t, 10> perm = {0, 9, 3, 4, 7, 8, 6, 2, 1, 5};

  Eigen::Matrix<double, 10, 10> l2m;
  Eigen::Matrix<double, 10, 1> l;

  for (int i = 0; i < 3; ++i) {
    for (int d = 0; d < 3; ++d) {
      for (int j = 0; j < 10; ++j) {
        int n = local_nodes[i][j];
        double u = planar_nodes[n][0];
        double v = planar_nodes[n][1];
        double w = 1. - u - v;
        u = CT_nodes[n][0];
        v = CT_nodes[n][1];
        w = 1. - u - v;
        // l2m.col(j) = monomial_basis_eval(u, v, w);
        l2m.row(j) = monomial_basis_eval(u, v, w);
        l[j] = lagrange_nodes[n][d];
      }
      // Eigen::SparseLU<Eigen::Matrix<double, 10, 10>> solver;
      // solver.compute(l2m);
      m_CT_coeffs[i].col(d) = l2m.fullPivLu().solve(l);
      // m_CT_coeffs[i].col(d) = l2m * l;

      /*
      for (int j = 0; j < 10; ++j)
      {
        //int n = local_nodes[i][perm[j]];
        //m_CT_coeffs[i](j, d) = lagrange_nodes[n][d];
        int n = local_nodes[i][j];
        m_CT_coeffs[i](perm[j], d) = lagrange_nodes[n][d];
      }
      */
    }
  }
}

int
CloughTocherPatch::triangle_ind(const double& u,
                                const double& v,
                                const double& w) const
{
  int idx = -1;
  for (int i = 0; i < 3; ++i) {
    if (m_CTtri_bounds[i](0, 0) * u + m_CTtri_bounds[i](0, 1) * v +
            m_CTtri_bounds[i](0, 2) * w >=
          -1e-7 &&
        m_CTtri_bounds[i](1, 0) * u + m_CTtri_bounds[i](1, 1) * v +
            m_CTtri_bounds[i](1, 2) * w >=
          -1e-7 &&
        m_CTtri_bounds[i](2, 0) * u + m_CTtri_bounds[i](2, 1) * v +
            m_CTtri_bounds[i](2, 2) * w >=
          -1e-7) {
      idx = i;
      break;
    }
  }

  assert(idx > -1);
  return idx;
}

Eigen::Matrix<double, 10, 1>
CloughTocherPatch::monomial_basis_eval(const double& u,
                                       const double& v,
                                       const double& w) const
{
  Eigen::Matrix<double, 10, 1> monomial_basis_values;
  monomial_basis_values(0, 0) = w * w * w; // w3
  monomial_basis_values(1, 0) = v * w * w; // vw2
  monomial_basis_values(2, 0) = v * v * w; // v2w
  monomial_basis_values(3, 0) = v * v * v; // v3
  monomial_basis_values(4, 0) = u * w * w; // uw2
  monomial_basis_values(5, 0) = u * v * w; // uvw
  monomial_basis_values(6, 0) = u * v * v; // uv2
  monomial_basis_values(7, 0) = u * u * w; // u2w
  monomial_basis_values(8, 0) = u * u * v; // u2v
  monomial_basis_values(9, 0) = u * u * u; // u3

  return monomial_basis_values;
}

PlanarPoint
normalize_domain_point(const ConvexPolygon& domain,
                       const PlanarPoint& domain_point)
{
  // Get domain triangle vertices
  MatrixXr domain_vertices = domain.get_vertices();
  PlanarPoint v0 = domain_vertices.row(0);
  PlanarPoint v1 = domain_vertices.row(1);
  PlanarPoint v2 = domain_vertices.row(2);

  // Generate affine transformation mapping the standard triangle to the domain
  // triangle
  Eigen::Matrix<double, 2, 2> linear_transformation;
  PlanarPoint translation;
  linear_transformation.row(0) = v1 - v0;
  linear_transformation.row(1) = v2 - v0;
  translation = v0;

  // Denormalize the domain point
  return (domain_point - translation) * linear_transformation.inverse();
}

Eigen::Matrix<double, 3, 1>
CloughTocherPatch::CT_eval_normalized(int idx,
                                      const double& u,
                                      const double& v) const
{
  const double w = 1.0 - u - v;
  // std::cout << "subtri_idx: " << idx << std::endl;
  Eigen::Matrix<double, 10, 1> bb_vector =
    CloughTocherPatch::monomial_basis_eval(u, v, w);

  // std::cout << "monomial: " << bb_vector << std::endl;

  Eigen::Matrix<double, 3, 1> val;
  val = m_CT_coeffs[idx].transpose() * bb_vector;
  return val;
}

Eigen::Matrix<double, 3, 1>
CloughTocherPatch::CT_eval(const double& u, const double& v) const
{
  const double w = 1.0 - u - v;
  int idx = CloughTocherPatch::triangle_ind(u, v, w);

  // std::cout << "subtri_idx: " << idx << std::endl;
  Eigen::Matrix<double, 10, 1> bb_vector =
    CloughTocherPatch::monomial_basis_eval(u, v, w);

  // std::cout << "monomial: " << bb_vector << std::endl;

  Eigen::Matrix<double, 3, 1> val;
  val = m_CT_coeffs[idx].transpose() * bb_vector;
  return val;
}

std::array<Eigen::Matrix<double, 10, 3>, 3>
CloughTocherPatch::get_coeffs() const
{
  return m_CT_coeffs;
}

double
CloughTocherPatch::external_boundary_data_eval(
  const double& u,
  const double& v,
  Eigen::Matrix<double, 12, 1>& external_boundary_data) const
{
  const double w = 1.0 - u - v;
  int idx = CloughTocherPatch::triangle_ind(u, v, w);

  Eigen::Matrix<double, 10, 1> bb_vector =
    CloughTocherPatch::monomial_basis_eval(u, v, w);

  double value =
    (m_CT_matrices[idx] * external_boundary_data).transpose() * bb_vector;

  return value;
}

void
CloughTocherPatch::triangulate(size_t num_refinements,
                               std::array<Eigen::MatrixXd, 3>& V,
                               std::array<Eigen::MatrixXi, 3>& F) const
{
  for (int n = 0; n < 3; ++n) {
    // Triangulate the domain
    Eigen::MatrixXd V_domain;
    std::array<Eigen::Matrix<double, 3, 1>, 3> boundary_segments_coeffs;
    for (int i = 0; i < 3; ++i) {
      boundary_segments_coeffs[i][0] = m_CTtri_bounds[n](i, 2);
      boundary_segments_coeffs[i][1] =
        m_CTtri_bounds[n](i, 0) - m_CTtri_bounds[n](i, 2);
      boundary_segments_coeffs[i][2] =
        m_CTtri_bounds[n](i, 1) - m_CTtri_bounds[n](i, 2);
    }

    ConvexPolygon domain(boundary_segments_coeffs);
    domain.triangulate(num_refinements, V_domain, F[n]);

    // Lift the domain vertices to the surface and also compute the normals
    V[n].resize(V_domain.rows(), 3);
    for (Eigen::Index i = 0; i < V_domain.rows(); ++i) {
      SpatialVector surface_point;
      SpatialVector surface_normal;
      // surface_point = CT_eval_normalized(V_domain(i, 0), V_domain(i, 1));
      surface_point = CT_eval(V_domain(i, 0), V_domain(i, 1));
      V[n].row(i) = surface_point;
    }
  }
}

void
CloughTocherPatch::triangulate_normalized(
  size_t num_refinements,
  std::array<Eigen::MatrixXd, 3>& V,
  std::array<Eigen::MatrixXi, 3>& F) const
{
  for (int n = 0; n < 3; ++n) {
    Eigen::MatrixXd V_domain;
    Eigen::Matrix<double, 3, 2> vertices;
    vertices << 0., 0., 0., 1., 1., 0.;
    ConvexPolygon domain(vertices);
    domain.triangulate(num_refinements, V_domain, F[n]);

    // Lift the domain vertices to the surface and also compute the normals
    V[n].resize(V_domain.rows(), 3);
    for (Eigen::Index i = 0; i < V_domain.rows(); ++i) {
      SpatialVector surface_point;
      SpatialVector surface_normal;
      surface_point = CT_eval_normalized(n, V_domain(i, 0), V_domain(i, 1));
      V[n].row(i) = surface_point;
    }
  }
}

void
CloughTocherPatch::parametrize_patch_boundaries(
  std::array<std::array<LineSegment, 3>, 3>& patch_boundaries) const
{
  for (int n = 0; n < 3; ++n) {
    // Triangulate the domain
    Eigen::MatrixXd V_domain;
    std::array<Eigen::Matrix<double, 3, 1>, 3> boundary_segments_coeffs;
    for (int i = 0; i < 3; ++i) {
      boundary_segments_coeffs[i][0] = m_CTtri_bounds[n](i, 2);
      boundary_segments_coeffs[i][1] =
        m_CTtri_bounds[n](i, 0) - m_CTtri_bounds[n](i, 2);
      boundary_segments_coeffs[i][2] =
        m_CTtri_bounds[n](i, 1) - m_CTtri_bounds[n](i, 2);
    }
    ConvexPolygon domain(boundary_segments_coeffs);
    domain.parametrize_patch_boundaries(patch_boundaries[n]);
  }
}

void
CloughTocherPatch::view() const
{
  // Generate mesh discretization
  std::array<Eigen::MatrixXd, 3> V;
  std::array<Eigen::MatrixXi, 3> F;
  triangulate(3, V, F);

  // Add surface mesh
  polyscope::init();
  for (int i = 0; i < 3; ++i) {
    polyscope::registerSurfaceMesh("patch " + std::to_string(i), V[i], F[i])
      ->setEdgeWidth(0);
    polyscope::registerPointCloud("control points" + std::to_string(i),
                                  m_CT_coeffs[i]);
  }

  polyscope::show();
}