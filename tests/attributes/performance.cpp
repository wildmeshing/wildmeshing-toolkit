#include <catch2/catch_test_macros.hpp>
#include <polysolve/Utils.hpp>
#include <random>
#include <wmtk/PointMesh.hpp>
#include <wmtk/io/MeshReader.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/mesh_utils.hpp>

#include <igl/Timer.h>
#include <catch2/catch_test_macros.hpp>
#include <iostream>
#include <map>

namespace wmtk {
/**
 * @brief serving as buffers for attributes data that can be modified by operations
 *
 */
class AbstractAttributeContainer
{
public:
    virtual ~AbstractAttributeContainer() = default;
    virtual void move(size_t from, size_t to) {};
    virtual void resize(size_t) {};
    virtual void rollback() {};
    virtual void begin_protect() {};
    virtual void end_protect() {};
};


template <typename T>
struct AttributeCollection : public AbstractAttributeContainer
{
    void move(size_t from, size_t to) override
    {
        if (from == to) return;
        m_attributes[to] = std::move(m_attributes[from]);
    }
    void resize(size_t s) override
    {
        // m_attributes.grow_to_at_least(s);
        m_attributes.resize(s);
        // if (m_attributes.size() > s) {
        //     m_attributes.resize(s);
        //     m_attributes.shrink_to_fit();
        // }
        // TODO: in Concurrent, vertex partition id, vertex mutex should be part of attribute
    }

    bool assign(size_t to, T&& val) // always use this in OP_after
    {
        m_attributes[to] = val;
        if (recording) m_rollback_list[to] = val;
        // TODO: are locks necessary? not now.
        return true;
    }
    /**
     * @brief retrieve the protected attribute data on operation-fail
     *
     */
    void rollback() override
    {
        for (auto& [i, v] : m_rollback_list) {
            m_attributes[i] = std::move(v);
        }
        end_protect();
    }
    /**
     * @brief clean local buffers for attribute, and start recording
     *
     */
    void begin_protect() override
    {
        m_rollback_list.clear();
        recording = true;
    };
    /**
     * @brief clear local buffers and finish recording
     *
     */
    void end_protect() override
    {
        m_rollback_list.clear();
        recording = false;
    }

    const T& operator[](size_t i) const { return m_attributes[i]; }

    T& operator[](size_t i)
    {
        if (recording) {
            m_rollback_list.emplace(i, m_attributes[i]);
            // m_rollback_list_pair.emplace_back(i, m_attributes[i]);
        }
        return m_attributes[i];
    }

    const T& at(size_t i) const { return m_attributes[i]; }

    size_t size() const { return m_attributes.size(); }
    std::map<size_t, T> m_rollback_list;
    // experimenting with tbb, could be templated as well.
    std::vector<T> m_attributes;
    bool recording = false;
    std::vector<std::pair<size_t, T>> m_rollback_list_pair;
};

template <typename T>
struct AttributeCollectionWithVectorPair : public AbstractAttributeContainer
{
    AttributeCollectionWithVectorPair() { m_rollback_list_pair.reserve(1000000); }


    void move(size_t from, size_t to) override
    {
        if (from == to) return;
        m_attributes[to] = std::move(m_attributes[from]);
    }
    void resize(size_t s) override
    {
        // m_attributes.grow_to_at_least(s);
        m_attributes.resize(s);
        // if (m_attributes.size() > s) {
        //     m_attributes.resize(s);
        //     m_attributes.shrink_to_fit();
        // }
        // TODO: in Concurrent, vertex partition id, vertex mutex should be part of attribute
    }

    bool assign(size_t to, T&& val) // always use this in OP_after
    {
        m_attributes[to] = val;
        if (recording) m_rollback_list[to] = val;
        // TODO: are locks necessary? not now.
        return true;
    }
    /**
     * @brief retrieve the protected attribute data on operation-fail
     *
     */
    void rollback() override
    {
        for (auto& [i, v] : m_rollback_list) {
            m_attributes[i] = std::move(v);
        }
        end_protect();
    }
    /**
     * @brief clean local buffers for attribute, and start recording
     *
     */
    void begin_protect() override
    {
        m_rollback_list_pair.clear();
        recording = true;
    };
    /**
     * @brief clear local buffers and finish recording
     *
     */
    void end_protect() override
    {
        m_rollback_list_pair.clear();
        recording = false;
    }

    const T& operator[](size_t i) const { return m_attributes[i]; }

    T& operator[](size_t i)
    {
        if (recording) {
            // m_rollback_list.emplace(i, m_attributes[i]);
            m_rollback_list_pair.emplace_back(i, m_attributes[i]);
        }
        return m_attributes[i];
    }

    const T& at(size_t i) const { return m_attributes[i]; }

    size_t size() const { return m_attributes.size(); }
    std::map<size_t, T> m_rollback_list;
    // experimenting with tbb, could be templated as well.
    std::vector<T> m_attributes;
    bool recording = false;
    std::vector<std::pair<size_t, T>> m_rollback_list_pair;
};

template <typename T>
struct AttributeCollectionWithVectorPairStatic : public AbstractAttributeContainer
{
    void move(size_t from, size_t to) override
    {
        if (from == to) return;
        m_attributes[to] = std::move(m_attributes[from]);
    }
    void resize(size_t s) override
    {
        // m_attributes.grow_to_at_least(s);
        m_attributes.resize(s);
        // if (m_attributes.size() > s) {
        //     m_attributes.resize(s);
        //     m_attributes.shrink_to_fit();
        // }
        // TODO: in Concurrent, vertex partition id, vertex mutex should be part of attribute
    }

    bool assign(size_t to, T&& val) // always use this in OP_after
    {
        m_attributes[to] = val;
        if (recording) m_rollback_list[to] = val;
        // TODO: are locks necessary? not now.
        return true;
    }
    /**
     * @brief retrieve the protected attribute data on operation-fail
     *
     */
    void rollback() override
    {
        for (auto& [i, v] : m_rollback_list) {
            m_attributes[i] = std::move(v);
        }
        end_protect();
    }
    /**
     * @brief clean local buffers for attribute, and start recording
     *
     */
    void begin_protect() override
    {
        m_rollback_list.clear();
        recording = true;
    };
    /**
     * @brief clear local buffers and finish recording
     *
     */
    void end_protect() override
    {
        m_rollback_list.clear();
        recording = false;
    }

    const T& operator[](size_t i) const { return m_attributes[i]; }

    T& operator[](size_t i)
    {
        if (recording) {
            // m_rollback_list.emplace(i, m_attributes[i]);
            // m_rollback_list_pair.emplace_back(i, m_attributes[i]);
            // m_rollback_list_pair.push_back(std::make_pair(i, m_attributes[i]));
            push_back(std::make_pair(i, m_attributes[i]));
        }
        return m_attributes[i];
    }

    static void push_back(std::pair<size_t, T> value) { m_rollback_list_pair.push_back(value); }

    const T& at(size_t i) const { return m_attributes[i]; }

    size_t size() const { return m_attributes.size(); }
    std::map<size_t, T> m_rollback_list;
    std::vector<T> m_attributes;
    bool recording = false;
    static std::vector<std::pair<size_t, T>> m_rollback_list_pair;
};


template <typename T>
struct AttributeCollectionWithVectorPairIterators : public AbstractAttributeContainer
{
    AttributeCollectionWithVectorPairIterators() { m_rollback_list_pair.resize(100); }

    void move(size_t from, size_t to) override
    {
        if (from == to) return;
        m_attributes[to] = std::move(m_attributes[from]);
    }
    void resize(size_t s) override { m_attributes.resize(s); }

    bool assign(size_t to, T&& val) // always use this in OP_after
    {
        m_attributes[to] = val;
        if (recording) m_rollback_list[to] = val;
        return true;
    }
    /**
     * @brief retrieve the protected attribute data on operation-fail
     *
     */
    void rollback() override
    {
        for (auto& [i, v] : m_rollback_list) {
            m_attributes[i] = std::move(v);
        }
        end_protect();
    }
    /**
     * @brief clean local buffers for attribute, and start recording
     *
     */
    void begin_protect() override
    {
        m_rollback_list.clear();
        m_record_end = m_record_begin;
        recording = true;
    };
    /**
     * @brief clear local buffers and finish recording
     *
     */
    void end_protect() override
    {
        m_rollback_list.clear();
        recording = false;
    }

    const T& operator[](size_t i) const { return m_attributes[i]; }

    T& operator[](size_t i)
    {
        if (recording) {
            if (m_record_end + 1 == m_rollback_list_pair.size()) {
                m_rollback_list_pair.resize(1.5 * m_rollback_list_pair.size());
            }
            m_rollback_list_pair[m_record_end++] = std::pair<size_t, T>(i, m_attributes[i]);
        }
        return m_attributes[i];
    }

    const T& at(size_t i) const { return m_attributes[i]; }

    size_t size() const { return m_attributes.size(); }
    std::map<size_t, T> m_rollback_list;
    std::vector<T> m_attributes;
    bool recording = false;
    std::vector<std::pair<size_t, T>> m_rollback_list_pair;
    size_t m_record_begin = 0;
    size_t m_record_end = 0;
};
} // namespace wmtk

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
        POLYSOLVE_SCOPED_STOPWATCH(
            "Attribute Read (attr.const_vector_attribute(t)[j])",
            wmtk::logger());
        double sum = 0;
        for (size_t i = 0; i < n_repetitions; ++i) {
            for (size_t i = 0; i < 20; ++i) {
                auto v = pp_acc.const_vector_attribute(i);
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

    const size_t n_repetitions = 1000000;
    auto [positions, pm_ptr, pph] = setup();
    auto& pm = *pm_ptr;
    auto pp_acc = pm.create_accessor<double>(pph);

    auto vertex_tuples = pm.get_all(wmtk::PrimitiveType::Vertex);

    std::mt19937 g(25);

    // std::shuffle(vertex_tuples.begin(), vertex_tuples.end(), g);
    std::vector<wmtk::Tuple> vv = vertex_tuples;
    // vv.resize(20);

    std::vector<std::array<int64_t, 20>> rand_entries;
    rand_entries.resize(n_repetitions);
    for (int64_t i = 0; i < n_repetitions; ++i) {
        for (int64_t j = 0; j < 20; ++j) {
            rand_entries[i][j] = rand() % positions.rows();
        }
    }


    std::vector<double> data(positions.size());
    Eigen::Map<Eigen::MatrixXd>(data.data(), positions.rows(), positions.cols()) = positions;
    {
        POLYSOLVE_SCOPED_STOPWATCH("Vector Direct Write (vec[3*i+j])", wmtk::logger());
        double sum = 0;
        for (size_t k = 0; k < n_repetitions; ++k) {
            for (size_t i = 0; i < 20; ++i) {
                for (int j = 0; j < 3; ++j) {
                    data[3 * rand_entries[k][i] + j] += 1;
                }
            }
        }
        std::cout << "sum = " << sum << std::endl;
    }
    {
        POLYSOLVE_SCOPED_STOPWATCH("Eigen Direct Write (A(i,j))", wmtk::logger());
        double sum = 0;
        for (size_t k = 0; k < n_repetitions; ++k) {
            for (size_t i = 0; i < 20; ++i) {
                for (size_t j = 0; j < 3; ++j) {
                    positions(rand_entries[k][i], j) += 1;
                }
            }
        }
    }
    {
        POLYSOLVE_SCOPED_STOPWATCH("Vec map write", wmtk::logger());
        const size_t dim = pp_acc.dimension();
        for (size_t k = 0; k < n_repetitions; ++k) {
            for (size_t i = 0; i < 20; ++i) {
                Eigen::Map<Eigen::VectorXd> d(data.data() + dim * rand_entries[k][i], dim);
                for (int j = 0; j < 3; ++j) {
                    d(j) += 1;
                }
            }
        }
    }
    {
        // std::vector<double> data(3 * 20);
        POLYSOLVE_SCOPED_STOPWATCH("Vec map write template size", wmtk::logger());
        double sum = 0;
        const size_t dim = pp_acc.dimension();
        for (size_t k = 0; k < n_repetitions; ++k) {
            for (size_t i = 0; i < 20; ++i) {
                Eigen::Map<Eigen::Vector3d> d(data.data() + 3 * rand_entries[k][i], 3);
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
        double sum = 0;
        for (size_t k = 0; k < n_repetitions; ++k) {
            for (size_t i = 0; i < 20; ++i) {
                auto v = pp_acc.vector_attribute(rand_entries[k][i]);
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
        for (size_t k = 0; k < n_repetitions; ++k) {
            for (auto i : rand_entries[k]) {
                auto v = pp_acc.vector_attribute(vv[i]);
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
        for (size_t k = 0; k < n_repetitions; ++k) {
            auto scope = pm.create_scope();

            for (auto i : rand_entries[k]) {
                auto v = pp_acc.vector_attribute(vv[i]);
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
        for (size_t k = 0; k < n_repetitions; ++k) {
            for (auto i : rand_entries[k]) {
                auto v = pp_acc.vector_attribute(vv[i]);
                for (int j = 0; j < 3; ++j) {
                    v(j) += 1;
                }
            }
        }
    }
    {
        wmtk::logger().info(
            "---------------------------- old wmtk test ---------------------------");
        using namespace wmtk;
        std::vector<Vector3d> raw_vector;
        std::map<size_t, Vector3d> raw_map;
        AttributeCollection<Vector3d> attribute_collection;
        AttributeCollectionWithVectorPair<Vector3d> ac_vectorpair;
        AttributeCollectionWithVectorPairIterators<Vector3d> ac_vecpair_iters;

        double time = 0;
        igl::Timer timer;

        size_t N = positions.rows();
        size_t iter = n_repetitions;

        raw_vector.resize(N);
        attribute_collection.resize(N);
        ac_vectorpair.resize(N);
        ac_vecpair_iters.resize(N);

        for (int64_t i = 0; i < N; ++i) {
            raw_vector[i] = positions.row(i);
            attribute_collection[i] = positions.row(i);
            ac_vectorpair[i] = positions.row(i);
            ac_vecpair_iters[i] = positions.row(i);
        }

        timer.start();
        for (size_t k = 0; k < iter; ++k) {
            for (size_t i = 0; i < 20; ++i) {
                raw_vector[rand_entries[k][i]][0] = rand_entries[k][i];
                raw_vector[rand_entries[k][i]][1] = rand_entries[k][i];
                raw_vector[rand_entries[k][i]][2] = rand_entries[k][i];
            }
        }
        wmtk::logger().info("raw vector write: {}", timer.getElapsedTime());

        timer.start();
        for (size_t k = 0; k < iter; ++k) {
            raw_map.clear();
            for (size_t i = 0; i < 20; ++i) {
                raw_map[rand_entries[k][i]][0] = rand_entries[k][i];
                raw_map[rand_entries[k][i]][1] = rand_entries[k][i];
                raw_map[rand_entries[k][i]][2] = rand_entries[k][i];
            }
        }
        wmtk::logger().info("raw map write: {}", timer.getElapsedTime());

        timer.start();
        for (size_t k = 0; k < iter; ++k) {
            attribute_collection.begin_protect();
            for (size_t i = 0; i < 20; ++i) {
                attribute_collection[rand_entries[k][i]][0] = rand_entries[k][i];
                attribute_collection[rand_entries[k][i]][1] = rand_entries[k][i];
                attribute_collection[rand_entries[k][i]][2] = rand_entries[k][i];
            }
            attribute_collection.end_protect();
        }
        wmtk::logger().info("attribute collection with map write: {}", timer.getElapsedTime());

        timer.start();
        for (size_t k = 0; k < iter; ++k) {
            ac_vectorpair.begin_protect();
            for (size_t i = 0; i < 20; ++i) {
                ac_vectorpair[rand_entries[k][i]][0] = rand_entries[k][i];
                ac_vectorpair[rand_entries[k][i]][1] = rand_entries[k][i];
                ac_vectorpair[rand_entries[k][i]][2] = rand_entries[k][i];
            }
            ac_vectorpair.end_protect();
        }
        wmtk::logger().info(
            "attribute collection with vector pair write: {}",
            timer.getElapsedTime());

        timer.start();
        for (size_t k = 0; k < iter; ++k) {
            ac_vecpair_iters.begin_protect();
            for (size_t i = 0; i < 20; ++i) {
                ac_vecpair_iters[rand_entries[k][i]][0] = rand_entries[k][i];
                ac_vecpair_iters[rand_entries[k][i]][1] = rand_entries[k][i];
                ac_vecpair_iters[rand_entries[k][i]][2] = rand_entries[k][i];
            }
            ac_vecpair_iters.end_protect();
        }
        wmtk::logger().info(
            "attribute collection with vector pair iterator write: {}",
            timer.getElapsedTime());
    }
}
