#include "ProjectOperation.hpp"

#include <wmtk/Mesh.hpp>
#include <wmtk/simplex/faces_single_dimension.hpp>
#include <wmtk/submesh/Embedding.hpp>
#include <wmtk/submesh/SubMesh.hpp>
#include <wmtk/utils/Logger.hpp>

#include <SimpleBVH/BVH.hpp>

namespace wmtk::operations::composite {

ProjectOperation::ProjectOperation(
    std::shared_ptr<Operation> main_op,
    const attribute::MeshAttributeHandle& project_to_mesh,
    attribute::MeshAttributeHandle& child_mesh_coordinates)
    : ProjectOperation(main_op, {std::make_pair(project_to_mesh, child_mesh_coordinates)})
{}


ProjectOperation::ProjectOperation(
    std::shared_ptr<Operation> main_op,
    const std::vector<MeshConstrainPair>& mesh_constaint_pairs)
    : ProjectOperation(main_op->mesh(), mesh_constaint_pairs)
{
    m_main_op = main_op;
}

ProjectOperation::ProjectOperation(
    Mesh& mesh,
    const std::vector<MeshConstrainPair>& mesh_constaint_pairs)
    : AttributesUpdate(mesh)
{
    for (auto& pair : mesh_constaint_pairs) {
        int64_t count = 0;
        int64_t index = 0;

        const std::vector<Tuple>& facest =
            pair.first.mesh().get_all(pair.first.mesh().top_simplex_type());

        const int64_t dim = int64_t(pair.first.mesh().top_simplex_type()) + 1;

        Eigen::MatrixXd vertices(dim * facest.size(), pair.first.dimension());
        Eigen::MatrixXi faces(facest.size(), dim);

        // hugly copy paste
        if (pair.first.holds<double>()) {
            const attribute::Accessor<double> accessor =
                pair.first.mesh().create_const_accessor(pair.first.as<double>());

            for (const auto& f : facest) {
                auto tmp = faces_single_dimension_tuples(
                    pair.first.mesh(),
                    simplex::Simplex(pair.first.mesh(), pair.first.mesh().top_simplex_type(), f),
                    PrimitiveType::Vertex);

                assert(tmp.size() == dim);
                for (int64_t j = 0; j < tmp.size(); ++j) {
                    auto p = accessor.const_vector_attribute(tmp[j]);
                    faces(index, j) = count;
                    vertices.row(dim * index + j) = p;

                    ++count;
                }
                ++index;
            }
        } else {
            const attribute::Accessor<Rational> accessor =
                pair.first.mesh().create_const_accessor(pair.first.as<Rational>());


            for (const auto& f : facest) {
                auto tmp = faces_single_dimension_tuples(
                    pair.first.mesh(),
                    simplex::Simplex(pair.first.mesh(), pair.first.mesh().top_simplex_type(), f),
                    PrimitiveType::Vertex);

                assert(tmp.size() == dim);
                for (int64_t j = 0; j < tmp.size(); ++j) {
                    auto p = accessor.const_vector_attribute(tmp[j]).cast<double>();
                    faces(index, j) = count;
                    vertices.row(dim * index + j) = p;

                    ++count;
                }
                ++index;
            }
        }

        auto bvh = std::make_shared<SimpleBVH::BVH>();
        bvh->init(vertices, faces, 1e-10);

        m_bvh.emplace_back(pair.second, bvh);
    }
}

ProjectOperation::ProjectOperation(
    std::shared_ptr<Operation> main_op,
    submesh::Embedding& emb,
    const attribute::MeshAttributeHandle& pos_handle)
    : ProjectOperation(emb, pos_handle)
{
    m_main_op = main_op;
}

ProjectOperation::ProjectOperation(
    submesh::Embedding& emb,
    const attribute::MeshAttributeHandle& pos_handle)
    : AttributesUpdate(emb.mesh())
    , m_embedding_pos_handle(pos_handle)
{
    const Mesh& m = emb.mesh();
    assert(&m == &pos_handle.mesh());
    // Wrapper for the position accessor that works for double and Rational. Probably not the most
    // efficient code but good enough for what is required here
    auto get_pos = [&pos_handle, &m](const Tuple& _t) -> Eigen::VectorXd {
        return std::visit(
            [&m, &_t](auto&& tah) noexcept -> Eigen::VectorXd {
                using HandleType = typename std::decay_t<decltype(tah)>;
                using AttributeType = typename HandleType::Type;

                const auto accessor = m.create_const_accessor(tah);
                if constexpr (std::is_same_v<AttributeType, double>) {
                    return accessor.const_vector_attribute(_t);
                }
                if constexpr (std::is_same_v<AttributeType, Rational>) {
                    return accessor.const_vector_attribute(_t).template cast<double>();
                }
                log_and_throw_error("Position attribute must be double or rational");
            },
            pos_handle.handle());
    };

    for (const auto& sub_ptr : emb.get_child_meshes()) {
        const submesh::SubMesh& sub = *sub_ptr;
        const PrimitiveType pt = sub.top_simplex_type();

        if (pt == PrimitiveType::Vertex) {
            logger().info("Ignoring vertex submeshes in ProjectOperation");
            continue;
        }

        int64_t count = 0;
        int64_t index = 0;

        const std::vector<simplex::IdSimplex> facest = sub.get_all_id_simplex(pt);

        const int64_t dim = int64_t(pt) + 1;

        Eigen::MatrixXd vertices(dim * facest.size(), pos_handle.dimension());
        Eigen::MatrixXi faces(facest.size(), dim);

        for (const simplex::IdSimplex& cell : facest) {
            const auto tmp =
                faces_single_dimension_tuples(m, m.get_simplex(cell), PrimitiveType::Vertex);

            assert(tmp.size() == dim);
            for (int64_t j = 0; j < tmp.size(); ++j) {
                Eigen::VectorXd p = get_pos(tmp[j]);
                faces(index, j) = count;
                vertices.row(dim * index + j) = p;

                ++count;
            }
            ++index;
        }

        auto bvh = std::make_shared<SimpleBVH::BVH>();
        bvh->init(vertices, faces, 1e-10);

        m_submesh_bvh.emplace_back(sub_ptr, bvh);
    }
}

std::vector<simplex::Simplex> ProjectOperation::execute(const simplex::Simplex& simplex)
{
    std::vector<simplex::Simplex> main_simplices;
    if (m_main_op) {
        // mesh has to be the same as the main_op mesh
        assert(&m_main_op->mesh() == &mesh());
        main_simplices = (*m_main_op)(simplex);
        if (main_simplices.empty()) return {};
        assert(main_simplices.size() == 1);
    } else {
        main_simplices.emplace_back(simplex);
    }
    const auto main_tup = main_simplices.front().tuple();


    for (auto& [pos_attribute, bvh] : m_bvh) {
        // const std::vector<Tuple> mapped_tuples_after =
        //     mesh().map_tuples(pos_attribute.mesh(), primitive_type(), {main_tup});
        const std::vector<Tuple> mapped_tuples_after = {main_tup};

        if (mapped_tuples_after.empty()) continue;

        if (pos_attribute.holds<double>()) {
            wmtk::attribute::Accessor<double> accessor =
                pos_attribute.mesh().create_accessor(pos_attribute.as<double>());

            for (const auto& t : mapped_tuples_after) {
                auto p = accessor.vector_attribute(t);
                SimpleBVH::VectorMax3d nearest_point;
                double sq_dist;
                bvh->nearest_facet(p, nearest_point, sq_dist);
                p = nearest_point;
            }
        } else {
            assert((pos_attribute.holds<Rational>()));
            wmtk::attribute::Accessor<Rational> accessor =
                pos_attribute.mesh().create_accessor(pos_attribute.as<Rational>());

            for (const auto& t : mapped_tuples_after) {
                auto p_map = accessor.vector_attribute(t);

                if (p_map.rows() == 3) {
                    const Eigen::Vector3d p = p_map.cast<double>();
                    SimpleBVH::VectorMax3d nearest_point;
                    double sq_dist;
                    bvh->nearest_facet(p, nearest_point, sq_dist);
                    for (int64_t d = 0; d < pos_attribute.dimension(); ++d) {
                        p_map(d) = Rational(nearest_point[d], true);
                    }
                } else if (p_map.rows() == 2) {
                    const Eigen::Vector2d p = p_map.cast<double>();
                    SimpleBVH::VectorMax3d nearest_point;
                    double sq_dist;
                    bvh->nearest_facet(p, nearest_point, sq_dist);
                    for (int64_t d = 0; d < pos_attribute.dimension(); ++d) {
                        p_map(d) = Rational(nearest_point[d], true);
                    }
                } else {
                    throw std::runtime_error("wrong vector dimension");
                }
            }
        }
    }

    for (auto& [sub_ptr, bvh] : m_submesh_bvh) {
        const submesh::SubMesh& sub = *sub_ptr;
        if (!sub.contains(main_tup, primitive_type())) {
            continue;
        }

        if (m_embedding_pos_handle.holds<double>()) {
            wmtk::attribute::Accessor<double> accessor =
                mesh().create_accessor(m_embedding_pos_handle.as<double>());

            auto p = accessor.vector_attribute(main_tup);
            SimpleBVH::VectorMax3d nearest_point;
            double sq_dist;
            bvh->nearest_facet(p, nearest_point, sq_dist);
            p = nearest_point;
        } else {
            assert((m_embedding_pos_handle.holds<Rational>()));
            wmtk::attribute::Accessor<Rational> accessor =
                mesh().create_accessor(m_embedding_pos_handle.as<Rational>());

            auto p_map = accessor.vector_attribute(main_tup);

            if (p_map.rows() == 3) {
                const Eigen::Vector3d p = p_map.cast<double>();
                SimpleBVH::VectorMax3d nearest_point;
                double sq_dist;
                bvh->nearest_facet(p, nearest_point, sq_dist);
                for (int64_t d = 0; d < m_embedding_pos_handle.dimension(); ++d) {
                    p_map(d) = Rational(nearest_point[d], true);
                }
            } else if (p_map.rows() == 2) {
                const Eigen::Vector2d p = p_map.cast<double>();
                SimpleBVH::VectorMax3d nearest_point;
                double sq_dist;
                bvh->nearest_facet(p, nearest_point, sq_dist);
                for (int64_t d = 0; d < m_embedding_pos_handle.dimension(); ++d) {
                    p_map(d) = Rational(nearest_point[d], true);
                }
            } else {
                throw std::runtime_error("wrong vector dimension");
            }
        }
    }

    return main_simplices;
}

} // namespace wmtk::operations::composite
