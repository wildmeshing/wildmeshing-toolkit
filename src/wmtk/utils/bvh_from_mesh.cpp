#include "bvh_from_mesh.hpp"

#include <wmtk/simplex/faces_single_dimension.hpp>
#include <wmtk/simplex/top_dimension_cofaces.hpp>
#include <wmtk/utils/Logger.hpp>

namespace wmtk::components::internal::utils {


std::shared_ptr<SimpleBVH::BVH> bvh_from_mesh(const attribute::MeshAttributeHandle& position_handle)
{
    const Mesh& mesh = position_handle.mesh();

    constexpr PrimitiveType PV = PrimitiveType::Vertex;
    constexpr PrimitiveType PE = PrimitiveType::Edge;

    const attribute::Accessor<double> accessor =
        mesh.create_const_accessor(position_handle.as<double>());

    Eigen::MatrixXd V;
    Eigen::MatrixXi F;

    if (mesh.top_simplex_type() == PrimitiveType::Triangle) {
        int64_t count = 0;
        assert(accessor.dimension() <= 3);

        const std::vector<Tuple>& face_tuples = mesh.get_all(PrimitiveType::Triangle);

        V.resize(3 * face_tuples.size(), accessor.dimension());
        V.setZero();
        F.resize(face_tuples.size(), 3);

        for (const Tuple& f : face_tuples) {
            const int64_t fid = f.global_cid();

            auto p0 = accessor.const_vector_attribute(f);
            auto p1 = accessor.const_vector_attribute(mesh.switch_tuple(f, PV));
            auto p2 = accessor.const_vector_attribute(mesh.switch_tuples(f, {PE, PV}));

            F.row(fid) = Eigen::Vector3i(count, count + 1, count + 2);
            V.row(3 * fid) = p0;
            V.row(3 * fid + 1) = p1;
            V.row(3 * fid + 2) = p2;

            count += 3;
        }


    } else if (mesh.top_simplex_type() == PrimitiveType::Edge) {
        int64_t count = 0;

        const std::vector<Tuple>& edge_tuples = mesh.get_all(PrimitiveType::Edge);

        V.resize(2 * edge_tuples.size(), accessor.dimension());
        F.resize(edge_tuples.size(), 2);

        for (const Tuple& e : edge_tuples) {
            const int64_t eid = e.global_cid();

            auto p0 = accessor.const_vector_attribute(e);
            auto p1 = accessor.const_vector_attribute(mesh.switch_tuple(e, PV));

            F.row(eid) = Eigen::Vector2i(count, count + 1);
            V.row(2 * eid) = p0;
            V.row(2 * eid + 1) = p1;

            count += 2;
        }

    } else {
        log_and_throw_error("bvh_from_mesh works only for tri/edges meshes");
    }


    std::shared_ptr<SimpleBVH::BVH> bvh = std::make_shared<SimpleBVH::BVH>();
    bvh->init(V, F, 1e-10);
    return bvh;
}

std::shared_ptr<SimpleBVH::BVH> bvh_from_mesh(
    const attribute::MeshAttributeHandle& position_handle,
    const PrimitiveType pt)

{
    const Mesh& mesh = position_handle.mesh();

    constexpr PrimitiveType PV = PrimitiveType::Vertex;
    constexpr PrimitiveType PE = PrimitiveType::Edge;

    const attribute::Accessor<double> accessor =
        mesh.create_const_accessor(position_handle.as<double>());

    Eigen::MatrixXd V;
    Eigen::MatrixXi F;

    if (pt == PrimitiveType::Triangle) {
        int64_t count = 0;
        assert(accessor.dimension() <= 3);

        const std::vector<Tuple>& face_tuples = mesh.get_all(PrimitiveType::Triangle);

        V.resize(3 * face_tuples.size(), accessor.dimension());
        V.setZero();
        F.resize(face_tuples.size(), 3);

        for (int64_t fid = 0; fid < face_tuples.size(); ++fid) {
            const Tuple& f = face_tuples[fid];

            auto p0 = accessor.const_vector_attribute(f);
            auto p1 = accessor.const_vector_attribute(mesh.switch_tuple(f, PV));
            auto p2 = accessor.const_vector_attribute(mesh.switch_tuples(f, {PE, PV}));

            F.row(fid) = Eigen::Vector3i(count, count + 1, count + 2);
            V.row(3 * fid) = p0;
            V.row(3 * fid + 1) = p1;
            V.row(3 * fid + 2) = p2;

            count += 3;
        }


    } else if (pt == PrimitiveType::Edge) {
        int64_t count = 0;

        const std::vector<Tuple>& edge_tuples = mesh.get_all(PrimitiveType::Edge);

        V.resize(2 * edge_tuples.size(), accessor.dimension());
        F.resize(edge_tuples.size(), 2);

        for (int64_t eid = 0; eid < edge_tuples.size(); ++eid) {
            const Tuple& e = edge_tuples[eid];

            auto p0 = accessor.const_vector_attribute(e);
            auto p1 = accessor.const_vector_attribute(mesh.switch_tuple(e, PV));

            F.row(eid) = Eigen::Vector2i(count, count + 1);
            V.row(2 * eid) = p0;
            V.row(2 * eid + 1) = p1;

            count += 2;
        }

    } else {
        log_and_throw_error("bvh_from_mesh works only for tri/edges meshes");
    }


    std::shared_ptr<SimpleBVH::BVH> bvh = std::make_shared<SimpleBVH::BVH>();
    bvh->init(V, F, 1e-10);
    return bvh;
}

std::shared_ptr<SimpleBVH::BVH> bvh_from_mesh(
    attribute::MeshAttributeHandle& position_handle,
    attribute::MeshAttributeHandle& label_handle,
    const int64_t label_value)
{
    const Mesh& mesh = position_handle.mesh();

    std::shared_ptr<SimpleBVH::BVH> bvh;

    const auto p_acc = mesh.create_const_accessor<double>(position_handle);
    const auto tag_acc = mesh.create_const_accessor<int64_t>(label_handle);

    std::vector<Eigen::Vector3d> pts;

    for (const Tuple& t : mesh.get_all(PrimitiveType::Triangle)) {
        const simplex::Simplex tri(mesh, PrimitiveType::Triangle, t);

        if (tag_acc.primitive_type() == PrimitiveType::Tetrahedron) {
            const auto tets = simplex::top_dimension_cofaces(mesh, tri);
            if (tets.size() != 2) {
                continue;
            }

            // find tets that are on the boundary of the tagged region
            if ((tag_acc.const_scalar_attribute(tets.simplex_vector()[0]) == label_value) ==
                (tag_acc.const_scalar_attribute(tets.simplex_vector()[1]) == label_value)) {
                continue;
            }
        } else {
            // find tagged triangles
            if (tag_acc.const_scalar_attribute(tri) != label_value) {
                continue;
            }
        }

        const auto vertices = simplex::faces_single_dimension(mesh, tri, PrimitiveType::Vertex);
        pts.emplace_back(p_acc.const_vector_attribute(vertices.simplex_vector()[0]));
        pts.emplace_back(p_acc.const_vector_attribute(vertices.simplex_vector()[1]));
        pts.emplace_back(p_acc.const_vector_attribute(vertices.simplex_vector()[2]));
    }

    if (pts.empty()) {
        log_and_throw_error("Seems like the input mesh does not contain any tagged elements.");
    }

    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    V.resize(pts.size(), 3);
    F.resize(pts.size() / 3, 3);
    for (size_t i = 0; i < F.rows(); ++i) {
        const size_t v0 = 3 * i + 0;
        const size_t v1 = 3 * i + 1;
        const size_t v2 = 3 * i + 2;
        V.row(v0) = pts[v0];
        V.row(v1) = pts[v1];
        V.row(v2) = pts[v2];
        F.row(i) = Eigen::Vector3i(v0, v1, v2);
    }
    bvh = std::make_shared<SimpleBVH::BVH>();
    bvh->init(V, F, 1e-10);

    return bvh;
}

} // namespace wmtk::components::internal::utils