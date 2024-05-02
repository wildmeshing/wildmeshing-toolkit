
#include <catch2/catch_test_macros.hpp>
#include <polysolve/Utils.hpp>
#include <random>
#include <wmtk/PointMesh.hpp>
#include <wmtk/io/MeshReader.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/mesh_utils.hpp>
namespace {

const std::filesystem::path data_dir = WMTK_DATA_DIR;


auto setup()
{
    const std::filesystem::path meshfile = data_dir / "armadillo.msh";

    auto mesh_in = wmtk::read_mesh(meshfile);
    wmtk::Mesh& m = *mesh_in;

    auto pos_handle = m.get_attribute_handle<double>("vertices", wmtk::PrimitiveType::Vertex);
    auto pos_acc = m.create_accessor<double, 3>(pos_handle);

    const auto vertices = m.get_all(wmtk::PrimitiveType::Vertex);

    // create matrix of positions
    Eigen::MatrixXd positions;
    positions.resize(vertices.size(), 3);
    for (size_t i = 0; i < vertices.size(); ++i) {
        positions.row(i) = pos_acc.const_vector_attribute(vertices[i]);
    }


    auto pm_ptr = std::make_shared<wmtk::PointMesh>(vertices.size());
    auto& pm = *pm_ptr;

    auto pph = wmtk::mesh_utils::set_matrix_attribute(
        positions,
        "vertices",
        wmtk::PrimitiveType::Vertex,
        pm);

    return std::make_tuple(positions, pm_ptr, pph);
}
} // namespace

TEST_CASE("accessor_read_performance", "[attributes][.]")
{
    wmtk::logger().set_level(spdlog::level::trace);

    const size_t n_repetitions = 5000000;
    auto [positions, pm_ptr, pph] = setup();
    auto& pm = *pm_ptr;
    auto pp_acc = pm.create_accessor<double>(pph);
    std::vector<double> data(positions.size());
    Eigen::Map<Eigen::MatrixXd>(data.data(), positions.rows(), positions.cols()) = positions;

    auto vertex_tuples = pm.get_all(wmtk::PrimitiveType::Vertex);
    std::mt19937 g(25);

    std::shuffle(vertex_tuples.begin(), vertex_tuples.end(), g);

    std::vector<wmtk::Tuple> vv = vertex_tuples;
    vv.resize(20);

    {
        POLYSOLVE_SCOPED_STOPWATCH("Vector Direct Read (vec[3*i+j])", wmtk::logger());
        double sum = 0;
        for (size_t i = 0; i < n_repetitions; ++i) {
            for (size_t i = 0; i < 20; ++i) {
                for (int j = 0; j < 3; ++j) {
                    sum += data[3 * i + j];
                }
            }
        }
        std::cout << "sum = " << sum << std::endl;
    }
    {
        POLYSOLVE_SCOPED_STOPWATCH("Eigen Array sum: (A.sum())", wmtk::logger());
        double sum = 0;
        for (size_t i = 0; i < n_repetitions; ++i) {
            sum += positions.topRows<20>().array().sum();
        }
        std::cout << "sum = " << sum << std::endl;
    }
    {
        POLYSOLVE_SCOPED_STOPWATCH("Eigen Direct Read (A(i,j))", wmtk::logger());
        double sum = 0;
        for (size_t i = 0; i < n_repetitions; ++i) {
            for (size_t i = 0; i < 20; ++i) {
                for (int j = 0; j < 3; ++j) {
                    sum += positions(i, j);
                }
            }
        }
        std::cout << "sum = " << sum << std::endl;
    }
    {
        POLYSOLVE_SCOPED_STOPWATCH("Eigen Direct Block Read (A.row(i)(j))", wmtk::logger());
        double sum = 0;
        for (size_t i = 0; i < n_repetitions; ++i) {
            for (size_t i = 0; i < 20; ++i) {
                auto r = positions.row(i);
                for (int j = 0; j < 3; ++j) {
                    sum += r(j);
                }
            }
        }
        std::cout << "sum = " << sum << std::endl;
    }
    {
        const auto& attr = pp_acc.attribute();
        POLYSOLVE_SCOPED_STOPWATCH(
            "Attribute Read (attr.const_vector_attribute(t)[j])",
            wmtk::logger());
        double sum = 0;
        for (size_t i = 0; i < n_repetitions; ++i) {
            for (size_t i = 0; i < 20; ++i) {
                auto v = attr.const_vector_attribute(i);
                for (int j = 0; j < 3; ++j) {
                    sum += v(j);
                }
            }
        }
        std::cout << "sum = " << sum << std::endl;
    }
    {
        POLYSOLVE_SCOPED_STOPWATCH(
            "ConstAccessor no Scope (acc.const_vector_attribute(t)[j])",
            wmtk::logger());
        double sum = 0;
        for (size_t i = 0; i < n_repetitions; ++i) {
            for (const wmtk::Tuple& t : vv) {
                auto v = pp_acc.const_vector_attribute(t);
                for (int j = 0; j < 3; ++j) {
                    sum += v(j);
                }
            }
        }
        std::cout << "sum = " << sum << std::endl;
    }
    {
        POLYSOLVE_SCOPED_STOPWATCH(
            "ConstAccessor with Scope (create_scope for(t,j)(acc.const_vector_attribute(t)[j]))",
            wmtk::logger());
        double sum = 0;
        for (size_t i = 0; i < n_repetitions; ++i) {
            auto scope = pm.create_scope();

            for (const wmtk::Tuple& t : vv) {
                auto v = pp_acc.const_vector_attribute(t);
                for (int j = 0; j < 3; ++j) {
                    sum += v(j);
                }
            }
        }
        std::cout << "sum = " << sum << std::endl;
    }
    {
        auto scope = pm.create_scope();
        POLYSOLVE_SCOPED_STOPWATCH(
            "ConstAccessor with Scope Already there (create_scope "
            "for(iter,t,j)(acc.const_vector_attribute(t)[j]))",
            wmtk::logger());
        double sum = 0;
        for (size_t i = 0; i < n_repetitions; ++i) {
            for (const wmtk::Tuple& t : vv) {
                auto v = pp_acc.const_vector_attribute(t);
                for (int j = 0; j < 3; ++j) {
                    sum += v(j);
                }
            }
        }
        std::cout << "sum = " << sum << std::endl;
    }
}

TEST_CASE("accessor_write_performance", "[attributes][.]")
{
    wmtk::logger().set_level(spdlog::level::trace);

    const size_t n_repetitions = 5000000;
    auto [positions, pm_ptr, pph] = setup();
    auto& pm = *pm_ptr;
    auto pp_acc = pm.create_accessor<double>(pph);

    auto vertex_tuples = pm.get_all(wmtk::PrimitiveType::Vertex);

    std::mt19937 g(25);

    std::shuffle(vertex_tuples.begin(), vertex_tuples.end(), g);
    std::vector<wmtk::Tuple> vv = vertex_tuples;
    vv.resize(20);

    std::vector<double> data(positions.size());
    Eigen::Map<Eigen::MatrixXd>(data.data(), positions.rows(), positions.cols()) = positions;
    {
        POLYSOLVE_SCOPED_STOPWATCH("Vector Direct Write (vec[3*i+j])", wmtk::logger());
        double sum = 0;
        for (size_t i = 0; i < n_repetitions; ++i) {
            for (size_t i = 0; i < 20; ++i) {
                for (int j = 0; j < 3; ++j) {
                    data[3 * i + j] += 1;
                }
            }
        }
        std::cout << "sum = " << sum << std::endl;
    }
    {
        POLYSOLVE_SCOPED_STOPWATCH("Eigen Direct Write (A(i,j))", wmtk::logger());
        double sum = 0;
        for (size_t i = 0; i < n_repetitions; ++i) {
            for (size_t i = 0; i < 20; ++i) {
                for (size_t j = 0; j < 3; ++j) {
                    positions(i, j) += 1;
                }
            }
        }
    }
    {
        POLYSOLVE_SCOPED_STOPWATCH("Vec map write", wmtk::logger());
        wmtk::attribute::Attribute<double>& attr = pp_acc.attribute();
        const size_t dim = attr.dimension();
        for (size_t i = 0; i < n_repetitions; ++i) {
            for (size_t i = 0; i < 20; ++i) {
                Eigen::Map<Eigen::Vector3d> d(data.data() + dim * i, dim);
                for (int j = 0; j < 3; ++j) {
                    d(j) += 1;
                }
            }
        }
    }
    {
        std::vector<double> data(3 * 20);
        POLYSOLVE_SCOPED_STOPWATCH("Vec map write template size", wmtk::logger());
        wmtk::attribute::Attribute<double>& attr = pp_acc.attribute();
        double sum = 0;
        const size_t dim = attr.dimension();
        for (size_t i = 0; i < n_repetitions; ++i) {
            for (size_t i = 0; i < 20; ++i) {
                Eigen::Map<Eigen::Vector3d> d(data.data() + dim * i, 3);
                for (int j = 0; j < 3; ++j) {
                    d(j) += 1;
                }
            }
        }
    }
    {
        POLYSOLVE_SCOPED_STOPWATCH(
            "Attribute Write (attr.const_vector_attribute(t)[j])",
            wmtk::logger());
        wmtk::attribute::Attribute<double>& attr = pp_acc.attribute();
        double sum = 0;
        for (size_t i = 0; i < n_repetitions; ++i) {
            for (size_t i = 0; i < 20; ++i) {
                auto v = attr.vector_attribute(i);
                for (int j = 0; j < 3; ++j) {
                    v(j) += 1;
                }
            }
        }
    }
    {
        POLYSOLVE_SCOPED_STOPWATCH(
            "Accessor no Scope (acc.vector_attribute(t)[j])",
            wmtk::logger());
        double sum = 0;
        for (size_t i = 0; i < n_repetitions; ++i) {
            for (const wmtk::Tuple& t : vv) {
                auto v = pp_acc.vector_attribute(t);
                for (int j = 0; j < 3; ++j) {
                    v(j) += 1;
                }
            }
        }
    }
    {
        POLYSOLVE_SCOPED_STOPWATCH(
            "Accessor with Scope (create_scope for(t,j)(acc._vector_attribute(t)[j]))",
            wmtk::logger());
        double sum = 0;
        for (size_t i = 0; i < n_repetitions; ++i) {
            auto scope = pm.create_scope();

            for (const wmtk::Tuple& t : vv) {
                auto v = pp_acc.vector_attribute(t);
                for (int j = 0; j < 3; ++j) {
                    v(j) += 1;
                }
            }
        }
    }
    {
        POLYSOLVE_SCOPED_STOPWATCH(
            "Accessor with Scope already there"
            "(create_scope "
            "for(iter,t,j)(acc.vector_attribute(t)[j]))",
            wmtk::logger());
        auto scope = pm.create_scope();
        double sum = 0;
        for (size_t i = 0; i < n_repetitions; ++i) {
            for (const wmtk::Tuple& t : vv) {
                auto v = pp_acc.vector_attribute(t);
                for (int j = 0; j < 3; ++j) {
                    v(j) += 1;
                }
            }
        }
    }
}
