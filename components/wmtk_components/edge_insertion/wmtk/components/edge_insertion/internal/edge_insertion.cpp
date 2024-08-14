#include "edge_insertion.hpp"

#include <wmtk/utils/EigenMatrixWriter.hpp>
#include <wmtk/utils/Logger.hpp>

#include <wmtk/operations/EdgeSplit.hpp>
#include <wmtk/operations/composite/TriFaceSplit.hpp>

#include <wmtk/attribute/>

namespace wmtk::components::internal {

wmtk::Rational det(const wmtk::Vector2r& a, const wmtk::Vector2r& b)
{
    return a[0] * b[1] - a[1] * b[0];
}

int is_point_inside_triangle(
    const wmtk::Vector2r& P,
    const wmtk::Vector2r& A,
    const wmtk::Vector2r& B,
    const wmtk::Vector2r& C)
{
    Vector2r AP = P - A;
    Vector2r BP = P - B;
    Vector2r CP = P - C;
    Vector2r AB = B - A;
    Vector2r BC = C - B;
    Vector2r CA = A - C;

    auto S_pab = abs(det(AP, AB)) / 2;
    auto S_pbc = abs(det(BP, BC)) / 2;
    auto S_pca = abs(det(CP, CA)) / 2;
    auto S_abc = abs(det(AB, -CA)) / 2;

    if (S_pab + S_pbc + S_pca != S_abc) {
        // outside
        return -1;
    }

    if (S_pab * S_pbc * S_pca > 0) {
        // inside
        return 0;
    }

    if (S_pab == 0) {
        if (S_pbc * S_pca > 0) {
            // on AB
            return 1;
        } else if (S_pca == 0) {
            // is A
            return 4;
        } else {
            // is B
            return 5;
        }
    }

    if (S_pbc == 0) {
        if (S_pca * S_pab > 0) {
            // on BC
            return 2;
        } else if (S_pab == 0) {
            // is B
            return 5;
        } else {
            // is C
            return 6;
        }
    }

    if (S_pca == 0) {
        if (S_pab * S_pbc > 0) {
            // on CA
            return 3;
        } else if (S_pbc == 0) {
            // is C
            return 6;
        } else {
            // is A
            return 4;
        }
    }

    return -1;
}

bool segment_segment_inter(
    const Vector2r& s0,
    const Vector2r& e0,
    const Vector2r& s1,
    const Vector2r& e1,
    Vector2r& res)
{
    Rational dd = e0[0] * e1[1] - e0[0] * s1[1] - e0[1] * e1[0] + e0[1] * s1[0] + e1[0] * s0[1] -
                  e1[1] * s0[0] + s0[0] * s1[1] - s0[1] * s1[0];

    if (dd.get_sign() == 0) {
        return false;
    }

    const Rational t0 = (e1[0] * s0[1] - e1[0] * s1[1] - e1[1] * s0[0] + e1[1] * s1[0] +
                         s0[0] * s1[1] - s0[1] * s1[0]) /
                        dd;
    const Rational t1 = (e0[0] * s0[1] - e0[0] * s1[1] - e0[1] * s0[0] + e0[1] * s1[0] +
                         s0[0] * s1[1] - s0[1] * s1[0]) /
                        dd;

    if (t0 < 0 || t0 > 1 || t1 < 0 || t1 > 1) {
        return false;
    }

    res = (1 - t0) * s0 + t0 * e0;
#ifndef NDEBUG
    const Vector2r p1 = (1 - t1) * s1 + t1 * e1;

    assert(res[0] == p1[0] && res[1] == p1[1] && res[2] == p1[2]);
#endif
    return true;
}

bool is_point_in_bbox(const Vector2r& point, const Vector2r& bbox_min, const Vector2r& bbox_max)
{
    if (point[0] >= bbox_min[0] && point[0] <= bbox_max[0] && point[1] >= bbox_min[1] &&
        point[1] <= bbox_max[1]) {
        return true;
    }
    return false;
}

bool is_bbox_intersect(const bbox& b0, const bbox& b1)
{
    // can intersect at boundary
    if (b1.x_min >= b0.x_min && b1.x_min <= b0.x_max && b1.y_min >= b0.y_min &&
        b1.y_max <= b0.y_max) {
        return true;
    }
    return false;
}

bool is_bbox_intersect(
    const Vector2r& bbox_min_0,
    const Vector2r& bbox_max_0,
    const Vector2r& bbox_min_1,
    const Vector2r& bbox_max_1)
{
    // can intersect at boundary
    if (bbox_min_1[0] >= bbox_min_0[0] && bbox_min_1[0] <= bbox_max_0[0] &&
        bbox_min_1[1] >= bbox_min_0[1] && bbox_min_1[1] <= bbox_max_0[1]) {
        return true;
    }
    return false;
}

std::array<wmtk::Vector2r, 2>
compute_bbox(const wmtk::Vector2r& p0, const wmtk::Vector2r& p1, const wmtk::Vector2r& p2)
{
    wmtk::Rational x_min, x_max, y_min, y_max;
    if (p0[0] < p1[0]) {
        x_min = p0[0];
        x_max = p1[0];
    } else {
        x_min = p1[0];
        x_max = p0[0];
    }

    if (p0[1] < p1[1]) {
        y_min = p0[1];
        y_max = p1[1];
    } else {
        y_min = p1[1];
        y_max = p0[1];
    }

    x_min = (x_min > p2[0]) ? p2[0] : x_min;
    x_max = (x_max < p2[0]) ? p2[0] : x_max;
    y_min = (y_min > p2[1]) ? p2[1] : y_min;
    y_max = (y_max < p2[1]) ? p2[1] : y_max;

    return {{wmtk::Vector2r(x_min, y_min), wmtk::Vector2r(x_max, y_max)}};
}

void edge_insertion(const TriMesh& _trimesh, const EdgeMesh& edgemesh)
{
    TriMesh trimesh = _trimesh; // make a full copy here. we may not want to change the input

    // code is assuming edgemesh doesn't have duplicated vertices

    wmtk::utils::EigenMatrixWriter writer_edge;
    edgemesh.serialize(writer_edge);

    MatrixX<int64_t> EV;
    MatrixX<Rational> V_edge;

    writer_edge.get_EV_matrix(EV);
    writer_edge.get_position_matrix(V_edge);

    // add segment vertices into tris and split the tris

    // add bbox attribute
    auto bbox_min_handle =
        trimesh.register_attribute<Rational>("bbox_min", PrimitiveType::Triangle, 2);
    auto bbox_min_ccessor = create_accessor<Rational>(bbox_min_handle);
    auto bbox_max_handle =
        trimesh.register_attribute<Rational>("bbox_max", PrimitiveType::Triangle, 2);
    auto bbox_max_ccessor = create_accessor<Rational>(bbox_max_handle);

    auto position_handle =
        trimesh.get_attribute_handle<Rational>("vertices", PrimitiveType::Vertex);
    auto position_accessor = trimesh.create_accessor<Rational>(position_handle);

    auto segment_index_handle =
        trimesh.register_attribute<int64_t>("segment_index", PrimitiveType::Vertex, 1, -1);
    auto segment_index_accessor = create_accessor<int64_t>(segment_index_handle);

    for (const auto& f : trimesh.get_all(PrimitiveType::Triangle)) {
        // compute bounding box
        const auto v0 = f;
        const auto v1 = trimesh.switch_tuple(f, PrimitiveType::Vertex);
        const auto v2 = trimesh.switch_tuples(f, {PrimitiveType::Edge, PrimitiveType::Vertex});

        const Vector2r& p0 = position_accessor.const_vector_attribute(v0);
        const Vector2r& p1 = position_accessor.const_vector_attribute(v1);
        const Vector2r& p2 = position_accessor.const_vector_attribute(v2);

        const auto bbox = compute_bbox(p0, p1, p2);

        bbox_min_accessor.vector_attribute(f) = bbox[0];
        bbox_max_accessor.vector_attribute(f) = bbox[1];
    }

    // add segment vertices in to trimesh
    for (int64_t i = 0; i < V_edge.rows(); ++i) {
        const Vector2r p = V_edge.row(i);

        for (const auto& f : trimesh.get_all(PrimitiveType::Triangle)) {
            // TODO: add hash
            if (!is_point_in_bbox(
                    p,
                    bbox_min_accessor.const_vector_attribute(f),
                    bbox_max_accessor.const_vector_attribute(f))) {
                // check bbox
                continue;
            }

            const auto v0 = f;
            const auto v1 = trimesh.switch_tuple(f, PrimitiveType::Vertex);
            const auto v2 = trimesh.switch_tuples(f, {PrimitiveType::Edge, PrimitiveType::Vertex});

            const Vector2r& p0 = position_accessor.const_vector_attribute(v0);
            const Vector2r& p1 = position_accessor.const_vector_attribute(v1);
            const Vector2r& p2 = position_accessor.const_vector_attribute(v2);

            const std::array<Vector2r, 3> ps = {{p0, p1, p2}};

            int in_tri_case = is_point_inside_triangle(p, p0, p1, p2);

            if (in_tri_case == -1) {
                continue;
            } else {
                switch (in_tri_case) {
                case 0: {
                    // inside
                    wmtk::operations::composite::TriFaceSplit facesplit(trimesh);
                    const auto new_v = facesplit(wmtk::simplex::Simplex::face(trimesh, f));

                    // assign position
                    position_accessor.vector_attribute(new_v) = p;

                    // compute bbox for new tris
                    const Tuple new_f0 = new_v;
                    const Tuple new_f1 = trimesh.switch_tuple(new_f0, PrimitiveType::Triangle);
                    const Tuple new_f2 = trimesh.switch_tuples(
                        new_f0,
                        {PrimitiveType::Edge, PrimitiveType::Triangle});

                    const auto bbox_0 = compute_bbox(p0, p1, p);
                    const auto bbox_1 = compute_bbox(p0, p2, p);
                    const auto bbox_2 = compute_bbox(p1, p2, p);

                    bbox_min_accessor.vector_attribute(new_f0) = bbox_0[0];
                    bbox_max_accessor.vector_attribute(new_f0) = bbox_0[1];
                    bbox_min_accessor.vector_attribute(new_f1) = bbox_1[0];
                    bbox_max_accessor.vector_attribute(new_f1) = bbox_1[1];
                    bbox_min_accessor.vector_attribute(new_f2) = bbox_2[0];
                    bbox_max_accessor.vector_attribute(new_f2) = bbox_2[1];

                    segment_index_accessor.scalar_attribute(new_v) = i;

                    break;
                }
                case 1: {
                    // on 01
                    wmtk::operations::EdgeSplit split(trimesh);
                    const auto new_v = spit(wmtk::simplex::Simplex::edge(trimesh, f));

                    // assign position
                    position_accessor.vector_attribute(new_v) = p;

                    // compute bbox for new tris
                    const Tuple new_f0 = new_v;
                    const Tuple new_f1 = trimesh.switch_tuples(
                        new_f0,
                        {PrimitiveType::Edge, PrimitiveType::Triangle});

                    const auto bbox_0 = compute_bbox(p1, p2, p);
                    const auto bbox_1 = compute_bbox(p0, p2, p);

                    bbox_min_accessor.vector_attribute(new_f0) = bbox_0[0];
                    bbox_max_accessor.vector_attribute(new_f0) = bbox_0[1];
                    bbox_min_accessor.vector_attribute(new_f1) = bbox_1[0];
                    bbox_max_accessor.vector_attribute(new_f1) = bbox_1[1];

                    if (!trimesh.is_boundary(PrimtiveType::Edge, new_v)) {
                        const Tuple new_f2 = trimesh.switch_tuple(new_v, PrimitiveType::Triangle);
                        const Tuple new_f3 = trimesh.switch_tuples(
                            new_f2,
                            {PrimitiveType::Edge, PrimitiveType::Triangle});

                        const auto& p4 =
                            position_accessor.const_vector_attribute(trimesh.swtich_tuples(
                                new_f2,
                                {PrimtiveType::Edge, PrimtiveType::Vertex}));

                        const auto bbox_2 = compute_bbox(p1, p4, p);
                        const auto bbox_3 = compute_bbox(p0, p4, p);

                        bbox_min_accessor.vector_attribute(new_f2) = bbox_2[0];
                        bbox_max_accessor.vector_attribute(new_f2) = bbox_2[1];
                        bbox_min_accessor.vector_attribute(new_f3) = bbox_3[0];
                        bbox_max_accessor.vector_attribute(new_f3) = bbox_3[1];
                    }

                    segment_index_accessor.scalar_attribute(new_v) = i;

                    break;
                }
                case 2: {
                    // on 12

                    wmtk::operations::EdgeSplit split(trimesh);
                    const auto new_v = spit(wmtk::simplex::Simplex::edge(
                        trimesh,
                        trimesh.switch_tuples(f, {PrimitiveType::Vertex, PrimtiveType::Edge})));

                    // assign position
                    position_accessor.vector_attribute(new_v) = p;

                    // compute bbox for new tris
                    const Tuple new_f0 = new_v;
                    const Tuple new_f1 = trimesh.switch_tuples(
                        new_f0,
                        {PrimitiveType::Edge, PrimitiveType::Triangle});

                    const auto bbox_0 = compute_bbox(p0, p2, p);
                    const auto bbox_1 = compute_bbox(p0, p1, p);

                    bbox_min_accessor.vector_attribute(new_f0) = bbox_0[0];
                    bbox_max_accessor.vector_attribute(new_f0) = bbox_0[1];
                    bbox_min_accessor.vector_attribute(new_f1) = bbox_1[0];
                    bbox_max_accessor.vector_attribute(new_f1) = bbox_1[1];

                    if (!trimesh.is_boundary(PrimtiveType::Edge, new_v)) {
                        const Tuple new_f2 = trimesh.switch_tuple(new_v, PrimitiveType::Triangle);
                        const Tuple new_f3 = trimesh.switch_tuples(
                            new_f2,
                            {PrimitiveType::Edge, PrimitiveType::Triangle});

                        const auto& p4 =
                            position_accessor.const_vector_attribute(trimesh.swtich_tuples(
                                new_f2,
                                {PrimtiveType::Edge, PrimtiveType::Vertex}));

                        const auto bbox_2 = compute_bbox(p2, p4, p);
                        const auto bbox_3 = compute_bbox(p1, p4, p);

                        bbox_min_accessor.vector_attribute(new_f2) = bbox_2[0];
                        bbox_max_accessor.vector_attribute(new_f2) = bbox_2[1];
                        bbox_min_accessor.vector_attribute(new_f3) = bbox_3[0];
                        bbox_max_accessor.vector_attribute(new_f3) = bbox_3[1];
                    }

                    segment_index_accessor.scalar_attribute(new_v) = i;

                    break;
                }
                case 3: {
                    // on 23
                    wmtk::operations::EdgeSplit split(trimesh);
                    const auto new_v = spit(wmtk::simplex::Simplex::edge(
                        trimesh,
                        trimesh.switch_tuple(f, PrimtiveType::Edge)));

                    // assign position
                    position_accessor.vector_attribute(new_v) = p;

                    // compute bbox for new tris
                    const Tuple new_f0 = new_v;
                    const Tuple new_f1 = trimesh.switch_tuples(
                        new_f0,
                        {PrimitiveType::Edge, PrimitiveType::Triangle});

                    const auto bbox_0 = compute_bbox(p1, p2, p);
                    const auto bbox_1 = compute_bbox(p0, p1, p);

                    bbox_min_accessor.vector_attribute(new_f0) = bbox_0[0];
                    bbox_max_accessor.vector_attribute(new_f0) = bbox_0[1];
                    bbox_min_accessor.vector_attribute(new_f1) = bbox_1[0];
                    bbox_max_accessor.vector_attribute(new_f1) = bbox_1[1];

                    if (!trimesh.is_boundary(PrimtiveType::Edge, new_v)) {
                        const Tuple new_f2 = trimesh.switch_tuple(new_v, PrimitiveType::Triangle);
                        const Tuple new_f3 = trimesh.switch_tuples(
                            new_f2,
                            {PrimitiveType::Edge, PrimitiveType::Triangle});

                        const auto& p4 =
                            position_accessor.const_vector_attribute(trimesh.swtich_tuples(
                                new_f2,
                                {PrimtiveType::Edge, PrimtiveType::Vertex}));

                        const auto bbox_2 = compute_bbox(p2, p4, p);
                        const auto bbox_3 = compute_bbox(p0, p4, p);

                        bbox_min_accessor.vector_attribute(new_f2) = bbox_2[0];
                        bbox_max_accessor.vector_attribute(new_f2) = bbox_2[1];
                        bbox_min_accessor.vector_attribute(new_f3) = bbox_3[0];
                        bbox_max_accessor.vector_attribute(new_f3) = bbox_3[1];
                    }

                    segment_index_accessor.scalar_attribute(new_v) = i;

                    break;
                }
                case 4: {
                    // is 0
                    segment_index_accessor.scalar_attribute(v0) = i;
                    break;
                }
                case 5: {
                    // is 1
                    segment_index_accessor.scalar_attribute(v1) = i;
                    break;
                }
                case 6: {
                    // is 2
                    segment_index_accessor.scalar_attribute(v2) = i;
                    break;
                }
                default: {
                    throw std::runtime_error("wrong inside triangle case");
                }
                }

                // found, break, go add next vertex
                break;
            }
        }
    }


    // remove duplicated vertices
}

} // namespace wmtk::components::internal