
#include <catch2/catch_test_macros.hpp>
#include <polysolve/Utils.hpp>
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
    {
        POLYSOLVE_SCOPED_STOPWATCH("Direct Read", wmtk::logger());
        double sum = 0;
        for (size_t i = 0; i < n_repetitions; ++i) {
            for (size_t i = 0; i < 20; ++i) {
                sum += positions(i, 0);
            }
            for (size_t i = 0; i < 20; ++i) {
                sum += positions(i, 1);
            }
            for (size_t i = 0; i < 20; ++i) {
                sum += positions(i, 2);
            }
        }
        std::cout << "sum = " << sum << std::endl;
    }
    {
        auto vv = pm.get_all(wmtk::PrimitiveType::Vertex);
        vv.resize(20);
        const auto& attr = pp_acc.attribute();
        POLYSOLVE_SCOPED_STOPWATCH("Attribute Read", wmtk::logger());
        double sum = 0;
        for (size_t i = 0; i < n_repetitions; ++i) {
            for (size_t i = 0; i < 20; ++i) {
                sum += attr.const_vector_attribute(i)[0];
            }
            for (size_t i = 0; i < 20; ++i) {
                sum += attr.const_vector_attribute(i)[1];
            }
            for (size_t i = 0; i < 20; ++i) {
                sum += attr.const_vector_attribute(i)[2];
            }
        }
        std::cout << "sum = " << sum << std::endl;
    }
    {
        auto vv = pm.get_all(wmtk::PrimitiveType::Vertex);
        vv.resize(20);
        POLYSOLVE_SCOPED_STOPWATCH("ConstAccessor no Scope", wmtk::logger());
        double sum = 0;
        for (size_t i = 0; i < n_repetitions; ++i) {
            for (const wmtk::Tuple& t : vv) {
                sum += pp_acc.const_vector_attribute(t)[0];
            }
            for (const wmtk::Tuple& t : vv) {
                sum += pp_acc.const_vector_attribute(t)[1];
            }
            for (const wmtk::Tuple& t : vv) {
                sum += pp_acc.const_vector_attribute(t)[2];
            }
        }
        std::cout << "sum = " << sum << std::endl;
    }
    {
        auto vv = pm.get_all(wmtk::PrimitiveType::Vertex);
        vv.resize(20);
        POLYSOLVE_SCOPED_STOPWATCH("ConstAccessor with Scope", wmtk::logger());
        double sum = 0;
        for (size_t i = 0; i < n_repetitions; ++i) {
            auto scope = pm.create_scope();

            for (const wmtk::Tuple& t : vv) {
                sum += pp_acc.const_vector_attribute(t)[0];
            }
            for (const wmtk::Tuple& t : vv) {
                sum += pp_acc.const_vector_attribute(t)[1];
            }
            for (const wmtk::Tuple& t : vv) {
                sum += pp_acc.const_vector_attribute(t)[2];
            }
        }
        std::cout << "sum = " << sum << std::endl;
    }
    {
        auto vv = pm.get_all(wmtk::PrimitiveType::Vertex);
        vv.resize(20);
        auto scope = pm.create_scope();
        POLYSOLVE_SCOPED_STOPWATCH("ConstAccessor with Scope Already there", wmtk::logger());
        double sum = 0;
        for (size_t i = 0; i < n_repetitions; ++i) {
            for (const wmtk::Tuple& t : vv) {
                sum += pp_acc.const_vector_attribute(t)[0];
            }
            for (const wmtk::Tuple& t : vv) {
                sum += pp_acc.const_vector_attribute(t)[1];
            }
            for (const wmtk::Tuple& t : vv) {
                sum += pp_acc.const_vector_attribute(t)[2];
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
    {
        POLYSOLVE_SCOPED_STOPWATCH("Direct Write", wmtk::logger());
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
        std::vector<double> data(3 * 20);
        POLYSOLVE_SCOPED_STOPWATCH("Vec map write", wmtk::logger());
        wmtk::attribute::Attribute<double>& attr = pp_acc.attribute();
        const size_t dim = attr.dimension();
        for (size_t i = 0; i < n_repetitions; ++i) {
            for (size_t i = 0; i < 20; ++i) {
                for (int j = 0; j < 3; ++j) {
                    Eigen::Map<Eigen::Vector3d> d(data.data() + dim * i, dim);
                    d(j) += 1;
                }
            }
        }
    }
    {
        auto vv = pm.get_all(wmtk::PrimitiveType::Vertex);
        vv.resize(20);
        std::vector<double> data(3 * 20);
        POLYSOLVE_SCOPED_STOPWATCH("Vec map read", wmtk::logger());
        wmtk::attribute::Attribute<double>& attr = pp_acc.attribute();
        double sum = 0;
        for (size_t i = 0; i < n_repetitions; ++i) {
            for (int j = 0; j < 3; ++j) {
                for (size_t i = 0; i < 20; ++i) {
                    Eigen::Map<const Eigen::Vector3d> d(data.data(), 3);
                    sum += d(j);
                }
            }
        }
    }
    {
        auto vv = pm.get_all(wmtk::PrimitiveType::Vertex);
        vv.resize(20);
        POLYSOLVE_SCOPED_STOPWATCH("Attribute Write", wmtk::logger());
        wmtk::attribute::Attribute<double>& attr = pp_acc.attribute();
        double sum = 0;
        for (size_t i = 0; i < n_repetitions; ++i) {
            for (int j = 0; j < 3; ++j) {
                for (size_t i = 0; i < 20; ++i) {
                    attr.vector_attribute(i)(j) += 1;
                }
            }
        }
    }
    {
        auto vv = pm.get_all(wmtk::PrimitiveType::Vertex);
        vv.resize(20);
        POLYSOLVE_SCOPED_STOPWATCH("Accessor no Scope", wmtk::logger());
        double sum = 0;
        for (size_t i = 0; i < n_repetitions; ++i) {
            for (const wmtk::Tuple& t : vv) {
                pp_acc.vector_attribute(t)[0] += 1;
            }
            for (const wmtk::Tuple& t : vv) {
                pp_acc.vector_attribute(t)[1] += 1;
            }
            for (const wmtk::Tuple& t : vv) {
                pp_acc.vector_attribute(t)[2] += 1;
            }
        }
    }
    {
        auto vv = pm.get_all(wmtk::PrimitiveType::Vertex);
        vv.resize(20);
        POLYSOLVE_SCOPED_STOPWATCH("Accessor with Scope", wmtk::logger());
        double sum = 0;
        for (size_t i = 0; i < n_repetitions; ++i) {
            auto scope = pm.create_scope();

            for (const wmtk::Tuple& t : vv) {
                pp_acc.vector_attribute(t)[0] += 1;
            }
            for (const wmtk::Tuple& t : vv) {
                pp_acc.vector_attribute(t)[1] += 1;
            }
            for (const wmtk::Tuple& t : vv) {
                pp_acc.vector_attribute(t)[2] += 1;
            }
        }
    }
    {
        auto vv = pm.get_all(wmtk::PrimitiveType::Vertex);
        vv.resize(20);
        POLYSOLVE_SCOPED_STOPWATCH("Accessor with Scope already there", wmtk::logger());
        auto scope = pm.create_scope();
        double sum = 0;
        for (size_t i = 0; i < n_repetitions; ++i) {
            for (const wmtk::Tuple& t : vv) {
                pp_acc.vector_attribute(t)[0] += 1;
            }
            for (const wmtk::Tuple& t : vv) {
                pp_acc.vector_attribute(t)[1] += 1;
            }
            for (const wmtk::Tuple& t : vv) {
                pp_acc.vector_attribute(t)[2] += 1;
            }
        }
    }
}
