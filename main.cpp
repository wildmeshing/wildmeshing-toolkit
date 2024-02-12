#include <CLI/CLI.hpp>
#include <chrono>
#include <filesystem>
#include <nlohmann/json.hpp>
#include <wmtk/PointMesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/components/run_components.hpp>
#include <wmtk/io/MeshReader.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/mesh_utils.hpp>

#include <polysolve/Utils.hpp>

using json = nlohmann::json;
using namespace wmtk;

const std::filesystem::path data_dir = "C:/Projects/wmtk-daniel-zint/data";

int main(int argc, char** argv)
{
    const std::filesystem::path meshfile = data_dir / "armadillo.msh";

    logger().set_level(spdlog::level::trace);

    auto mesh_in = wmtk::read_mesh(meshfile);
    Mesh& m = *mesh_in;

    auto pos_handle = m.get_attribute_handle<double>("vertices", PrimitiveType::Vertex);
    auto pos_acc = m.create_accessor<double>(pos_handle);

    const size_t n_repetitions = 50000;

    const auto vertices = m.get_all_simplices(PrimitiveType::Vertex);

    // create matrix of positions
    Eigen::MatrixXd positions;
    positions.resize(vertices.size(), 3);
    for (size_t i = 0; i < vertices.size(); ++i) {
        positions.row(i) = pos_acc.const_vector_attribute(vertices[i]);
    }

    PointMesh pm(vertices.size());
    auto pph = mesh_utils::set_matrix_attribute(positions, "vertices", PrimitiveType::Vertex, pm);
    auto pp_acc = pm.create_accessor<double>(pph);

    {
        const auto vv = pm.get_all_simplices(PrimitiveType::Vertex);
        POLYSOLVE_SCOPED_STOPWATCH("Direct", logger());
        double sum = 0;
        // for (size_t i = 0; i < n_repetitions; ++i) {
        for (size_t i = 0; i < vertices.size(); ++i) {
            for (size_t n = 0; n < n_repetitions; ++n) {
                sum += positions(i, 0);
            }
        }
        for (size_t i = 0; i < vertices.size(); ++i) {
            for (size_t n = 0; n < n_repetitions; ++n) {
                sum += positions(i, 1);
            }
        }
        for (size_t i = 0; i < vertices.size(); ++i) {
            for (size_t n = 0; n < n_repetitions; ++n) {
                sum += positions(i, 2);
            }
        }

        // for (const simplex::Simplex& t : vv) {
        //     int64_t id = t.id();
        //     sum += positions(id, 0);
        // }
        // for (const simplex::Simplex& t : vv) {
        //     int64_t id = t.id();
        //     sum += positions(id, 1);
        // }
        // for (const simplex::Simplex& t : vv) {
        //     int64_t id = t.id();
        //     sum += positions(id, 2);
        // }
        //}
        std::cout << "sum = " << sum << std::endl;
    }
    {
        const auto vv = pm.get_all_simplices(PrimitiveType::Vertex);
        POLYSOLVE_SCOPED_STOPWATCH("PointMesh Accessors", logger());
        double sum = 0;
        // for (size_t i = 0; i < n_repetitions; ++i) {
        for (const simplex::Simplex& t : vv) {
            for (size_t n = 0; n < n_repetitions; ++n) {
                sum += pp_acc.const_vector_attribute(t)[0];
            }
        }
        for (const simplex::Simplex& t : vv) {
            for (size_t n = 0; n < n_repetitions; ++n) {
                sum += pp_acc.const_vector_attribute(t)[1];
            }
        }
        for (const simplex::Simplex& t : vv) {
            for (size_t n = 0; n < n_repetitions; ++n) {
                sum += pp_acc.const_vector_attribute(t)[2];
            }
        }
        //}
        std::cout << "sum = " << sum << std::endl;
    }

    {
        POLYSOLVE_SCOPED_STOPWATCH("TriMesh Accessors", logger());
        double sum = 0;
        // for (size_t i = 0; i < n_repetitions; ++i) {
        for (const simplex::Simplex& t : vertices) {
            for (size_t n = 0; n < n_repetitions; ++n) {
                sum += pos_acc.const_vector_attribute(t)[0];
            }
        }
        for (const simplex::Simplex& t : vertices) {
            for (size_t n = 0; n < n_repetitions; ++n) {
                sum += pos_acc.const_vector_attribute(t)[1];
            }
        }
        for (const simplex::Simplex& t : vertices) {
            for (size_t n = 0; n < n_repetitions; ++n) {
                sum += pos_acc.const_vector_attribute(t)[2];
            }
        }
        //}
        std::cout << "sum = " << sum << std::endl;
    }
}
