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
    const submesh::Embedding& emb,
    const attribute::MeshAttributeHandle& pos_handle)
    : AttributesUpdate(main_op->mesh())
    , m_main_op(main_op)
{
    const Mesh& m = emb.mesh();
    assert(&m == &pos_handle.mesh());

    log_and_throw_error("incomplete implementation");

    // Wrapper for the position accessor that works for double and Rational. Probably not the most
    // efficient code but good enough for what is required here
    auto get_pos = [&pos_handle, &m](const simplex::IdSimplex& s) -> Eigen::VectorXd {
        return std::visit(
            [&m, &s](auto&& tah) noexcept -> Eigen::VectorXd {
                using HandleType = typename std::decay_t<decltype(tah)>;
                using AttributeType = typename HandleType::Type;

                const auto accessor = m.create_const_accessor(tah);
                if constexpr (std::is_same_v<AttributeType, double>) {
                    return accessor.const_vector_attribute(s);
                }
                if constexpr (std::is_same_v<AttributeType, Rational>) {
                    return accessor.const_vector_attribute(s).cast<double>();
                }
                log_and_throw_error("Position attribute must be double or rational");
            },
            pos_handle.handle());
    };

    for (const auto& sub_ptr : emb.get_child_meshes()) {
        const submesh::SubMesh& sub = *sub_ptr;
        const PrimitiveType pt = sub.top_simplex_type();

        for (const simplex::IdSimplex& cell : sub.get_all_id_simplex(pt)) {
            //
        }
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


    for (auto& pair : m_bvh) {
        const std::vector<Tuple> mapped_tuples_after =
            mesh().map_tuples(pair.first.mesh(), primitive_type(), {main_tup});

        if (mapped_tuples_after.empty()) continue;

        if (pair.first.holds<double>()) {
            wmtk::attribute::Accessor<double> accessor =
                pair.first.mesh().create_accessor(pair.first.as<double>());

            for (const auto& t : mapped_tuples_after) {
                auto p = accessor.vector_attribute(t);
                SimpleBVH::VectorMax3d nearest_point;
                double sq_dist;
                pair.second->nearest_facet(p, nearest_point, sq_dist);
                p = nearest_point;
            }
        } else {
            assert((pair.first.holds<Rational>()));
            wmtk::attribute::Accessor<Rational> accessor =
                pair.first.mesh().create_accessor(pair.first.as<Rational>());

            for (const auto& t : mapped_tuples_after) {
                auto p_map = accessor.vector_attribute(t);

                if (p_map.rows() == 3) {
                    const Eigen::Vector3d p = p_map.cast<double>();
                    SimpleBVH::VectorMax3d nearest_point;
                    double sq_dist;
                    pair.second->nearest_facet(p, nearest_point, sq_dist);
                    for (int64_t d = 0; d < pair.first.dimension(); ++d) {
                        p_map(d) = Rational(nearest_point[d], true);
                    }
                } else if (p_map.rows() == 2) {
                    const Eigen::Vector2d p = p_map.cast<double>();
                    SimpleBVH::VectorMax3d nearest_point;
                    double sq_dist;
                    pair.second->nearest_facet(p, nearest_point, sq_dist);
                    for (int64_t d = 0; d < pair.first.dimension(); ++d) {
                        p_map(d) = Rational(nearest_point[d], true);
                    }
                } else {
                    throw std::runtime_error("wrong vector dimension");
                }
            }
        }
    }

    return main_simplices;
}

} // namespace wmtk::operations::composite
