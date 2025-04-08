#include <common.hpp>

#include <pybind11/pybind11.h>
#include <pybind11_json/pybind11_json.hpp>
#include <wmtk/components/simplicial_embedding/simplicial_embedding.hpp>
#include <wmtk/utils/Logger.hpp>

namespace py = pybind11;

void foo()
{
    wmtk::logger().info("Hello");
}

void boo(const nlohmann::json& j)
{
    wmtk::components::simplicial_embedding::simplicial_embedding(j);
}

void define_simplicial_embedding(py::module_& m)
{
    // m.def(
    //     "point_edge_aabb_cd", &point_edge_aabb_cd, py::arg("p"), py::arg("e0"),
    //     py::arg("e1"), py::arg("dist"));

    // m.def(
    //     "simplicial_embedding",
    //     &wmtk::components::simplicial_embedding::simplicial_embedding,
    //     py::arg("j"));

    m.def("simplicial_embedding", &boo, py::arg("j"));
}