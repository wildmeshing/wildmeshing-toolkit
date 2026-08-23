// Python bindings for the 2D offset potentials.
//
// WHY THIS EXISTS. visualize_error_field.py draws the field E = (Phi - c)^2 that the offset
// optimization descends. For `offset_field: "euclidean"` Phi is the distance to the input
// complex and Python can compute it in a few lines of numpy. For `offset_field: "smooth"` --
// which is the DEFAULT -- Phi is the ipc-toolkit high_order_contact (OGC) potential, and there
// is no reimplementing that in Python: it is a sum over primitives selected by feasible-region
// tests, with its own smoothing parameters. Anything Python computed instead would be a
// different field wearing the same name, which for a diagnostic is worse than nothing.
//
// So the viewer calls the SAME C++ object the run does. Both fields are exposed, so a script
// can switch between them and compare, and the euclidean one doubles as a check that the
// binding agrees with the numpy path it replaces.
//
// THE BATCH ENTRY POINTS ARE THE POINT. A grid is 10^5 samples; crossing the language boundary
// per sample costs more than evaluating the potential. values()/gradients() take an (n, 2)
// array and return (n,) / (n, 2), evaluating in C++ with the GIL released.

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "../OffsetPotential.hpp"
#include "../SimplicialComplexBVH.hpp"

#include <Eigen/Core>
#include <memory>
#include <vector>

namespace py = pybind11;
using namespace wmtk::components::topological_offset;

namespace {

/// Shared batch evaluation, so the two potentials expose an identical surface.
template <typename Potential>
Eigen::VectorXd values_of(const Potential& p, const Eigen::MatrixXd& Q)
{
    if (Q.cols() != 2) {
        throw std::invalid_argument("query points must be an (n, 2) array");
    }
    Eigen::VectorXd out(Q.rows());
    {
        py::gil_scoped_release nogil;
        for (Eigen::Index i = 0; i < Q.rows(); ++i) {
            out(i) = p.value(Eigen::Vector2d(Q(i, 0), Q(i, 1)));
        }
    }
    return out;
}

template <typename Potential>
Eigen::MatrixXd gradients_of(const Potential& p, const Eigen::MatrixXd& Q)
{
    if (Q.cols() != 2) {
        throw std::invalid_argument("query points must be an (n, 2) array");
    }
    Eigen::MatrixXd out(Q.rows(), 2);
    {
        py::gil_scoped_release nogil;
        for (Eigen::Index i = 0; i < Q.rows(); ++i) {
            out.row(i) = p.gradient(Eigen::Vector2d(Q(i, 0), Q(i, 1))).transpose();
        }
    }
    return out;
}

/// The input-complex BVH EuclideanOffsetPotential queries -- the SAME structure the mesh
/// retains as its one description of the input complex (isolated points enter as the pseudo-
/// edge (i, i), which SimplicialComplexBVH::init builds from P itself).
std::shared_ptr<SimplicialComplexBVH>
make_query_bvh(const Eigen::MatrixXd& V, const Eigen::MatrixXi& E, const std::vector<int>& P)
{
    Eigen::MatrixXi Pm(P.size(), 1);
    for (size_t i = 0; i < P.size(); ++i) {
        Pm(Eigen::Index(i), 0) = P[i];
    }
    auto bvh = std::make_shared<SimplicialComplexBVH>();
    bvh->init(V, Eigen::MatrixXi(0, 4), Eigen::MatrixXi(0, 3), E, Pm);
    return bvh;
}

} // namespace

PYBIND11_MODULE(wmtk_offset_field, m)
{
    m.doc() = "The 2D offset potentials, as the topological_offset component computes them.\n"
              "Phi and its gradient for both offset_field settings; the level set Phi = c is the\n"
              "offset boundary and E = (Phi - c)^2 is what the optimization minimises.";

    py::class_<SmoothOffsetPotential2D, std::shared_ptr<SmoothOffsetPotential2D>>(
        m,
        "SmoothOffsetPotential2D",
        "offset_field: \"smooth\" -- the ipc-toolkit high_order_contact (OGC) potential.")
        .def(
            py::init([](const Eigen::MatrixXd& V,
                        const Eigen::MatrixXi& E,
                        const std::vector<int>& P,
                        double delta,
                        double dhat_factor) {
                // F is empty in 2D by the potential's own contract.
                return std::make_shared<SmoothOffsetPotential2D>(
                    V,
                    E,
                    Eigen::MatrixXi(0, 3),
                    P,
                    delta,
                    dhat_factor);
            }),
            py::arg("V"),
            py::arg("E"),
            py::arg("P") = std::vector<int>{},
            py::arg("delta"),
            py::arg("dhat_factor") = 2.0,
            "V: (nv, 2) complex vertices. E: (ne, 2) segments. P: indices of isolated vertices.\n"
            "delta: the offset distance. dhat_factor: support radius as a multiple of delta,\n"
            "must be > 1 -- Phi is IDENTICALLY ZERO beyond it, so a grid wider than\n"
            "dhat_factor * delta is flat by construction, not by accident.")
        .def("value", &SmoothOffsetPotential2D::value, py::arg("p"))
        .def("gradient", &SmoothOffsetPotential2D::gradient, py::arg("p"))
        .def(
            "values",
            &values_of<SmoothOffsetPotential2D>,
            py::arg("Q"),
            "Phi at each row of an (n, 2) array -> (n,). Evaluated in C++ with the GIL released.")
        .def("gradients", &gradients_of<SmoothOffsetPotential2D>, py::arg("Q"))
        .def("residual_length", &SmoothOffsetPotential2D::residual_length, py::arg("p"))
        .def("within_support", &SmoothOffsetPotential2D::within_support, py::arg("p"))
        .def("target_level", &SmoothOffsetPotential2D::target_level)
        .def("dhat", &SmoothOffsetPotential2D::dhat)
        .def("level_set_slope", &SmoothOffsetPotential2D::level_set_slope);

    py::class_<EuclideanOffsetPotential2D, std::shared_ptr<EuclideanOffsetPotential2D>>(
        m,
        "EuclideanOffsetPotential2D",
        "offset_field: \"euclidean\" -- exact distance to the input complex.")
        .def(
            py::init([](const Eigen::MatrixXd& V,
                        const Eigen::MatrixXi& E,
                        const std::vector<int>& P,
                        double delta) {
                return std::make_shared<EuclideanOffsetPotential2D>(make_query_bvh(V, E, P), delta);
            }),
            py::arg("V"),
            py::arg("E"),
            py::arg("P") = std::vector<int>{},
            py::arg("delta"),
            "Same arguments as the smooth potential minus dhat_factor: d has no compact\n"
            "support, so there is no radius to set.")
        .def("value", &EuclideanOffsetPotential2D::value, py::arg("p"))
        .def("gradient", &EuclideanOffsetPotential2D::gradient, py::arg("p"))
        .def("values", &values_of<EuclideanOffsetPotential2D>, py::arg("Q"))
        .def("gradients", &gradients_of<EuclideanOffsetPotential2D>, py::arg("Q"))
        .def("residual_length", &EuclideanOffsetPotential2D::residual_length, py::arg("p"))
        .def("target_level", &EuclideanOffsetPotential2D::target_level)
        .def("level_set_slope", &EuclideanOffsetPotential2D::level_set_slope);
}
