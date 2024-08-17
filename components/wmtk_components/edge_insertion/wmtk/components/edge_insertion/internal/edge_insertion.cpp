#include "edge_insertion.hpp"

#include <wmtk/utils/EigenMatrixWriter.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/orient.hpp>

#include <wmtk/operations/EdgeSplit.hpp>
#include <wmtk/operations/composite/TriFaceSplit.hpp>

#include <igl/edges.h>

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
    Vector2r& res,
    Rational& t0,
    Rational& t1)
{
    Rational dd = e0[0] * e1[1] - e0[0] * s1[1] - e0[1] * e1[0] + e0[1] * s1[0] + e1[0] * s0[1] -
                  e1[1] * s0[0] + s0[0] * s1[1] - s0[1] * s1[0];

    if (dd.get_sign() == 0) {
        return false;
    }

    t0 = (e1[0] * s0[1] - e1[0] * s1[1] - e1[1] * s0[0] + e1[1] * s1[0] + s0[0] * s1[1] -
          s0[1] * s1[0]) /
         dd;
    t1 = (e0[0] * s0[1] - e0[0] * s1[1] - e0[1] * s0[0] + e0[1] * s1[0] + s0[0] * s1[1] -
          s0[1] * s1[0]) /
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

bool is_bbox_intersect(
    const Vector2r& bbox_min_0,
    const Vector2r& bbox_max_0,
    const Vector2r& bbox_min_1,
    const Vector2r& bbox_max_1)
{
    if (bbox_min_0[0] > bbox_max_1[0] || bbox_max_0[0] < bbox_min_1[0]) {
        return false;
    }
    if (bbox_min_0[1] > bbox_max_1[1] || bbox_max_0[1] < bbox_min_1[1]) {
        return false;
    }
    return true;
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

std::array<wmtk::Vector2r, 2> compute_bbox(const wmtk::Vector2r& p0, const wmtk::Vector2r& p1)
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

    return {{wmtk::Vector2r(x_min, y_min), wmtk::Vector2r(x_max, y_max)}};
}

bool is_colinear(const Vector2r& p0, const Vector2r& p1, const Vector2r& q0, const Vector2r& q1)
{
    if (wmtk::orient2d(p0, p1, q0) == 0 && wmtk::orient2d(p0, p1, q1) == 0) {
        return true;
    }
    return false;
}

void dfs_triangulate(
    std::vector<std::vector<std::tuple<int64_t, bool, bool>>>& VV,
    std::vector<Vector2r>& V_pos,
    std::vector<std::array<int64_t, 3>>& FV,
    std::vector<std::array<bool, 3>>& local_e_on_input)
{
    std::vector<bool> in_stack(VV.size(), false);
    std::vector<int64_t> in_stack_idx(VV.size(), -1);

    std::vector<int64_t> stack;

    Vector2r prev_vector(1, 0);
    int64_t v_cur = -1;
    int64_t v_prev = -1;

    // stack.push_back(0);
    // in_stack[0] = true;
    // in_stack_idx[0] = 0;

    // while (stack.size() != 0) {
    //     stack.pop_back()
    // }
}

void dfs_triangulate_aux(
    int64_t v,
    std::vector<std::vector<std::tuple<int64_t, bool, bool>>>& VV,
    std::vector<Vector2r>& V_pos,
    std::vector<int64_t>& stack,
    std::vector<bool>& in_stack,
    std::vector<bool>& is_on_input,
    std::vector<int64_t>& in_stack_idx,
    const Vector2r& prev_vector,
    const int64_t v_prev,
    std::vector<std::array<int64_t, 3>>& FV,
    std::vector<std::array<bool, 3>>& local_e_on_input)
{
    if (in_stack[v]) {
        // got a polygon, triangulate it
        const int64_t begin_idx = in_stack_idx[v];
        const int64_t end_idx = stack.size() - 1;

        assert(end_idx - begin_idx >= 2);

        if (end_idx - begin_idx == 2) {
            // triangle case
            FV.push_back({{stack[begin_idx], stack[begin_idx + 1], stack[end_idx]}});
            local_e_on_input.push_back(
                {{is_on_input[begin_idx + 1], is_on_input[end_idx], is_on_input[begin_idx]}});
        } else {
            // polygon case
            // add vertex in the center
            Vector2r centroid(0, 0);
            int64_t centroid_idx = V_pos.size();
            for (int64_t i = begin_idx; i < end_idx + 1; ++i) {
                centroid = centroid + V_pos[stack[i]];
                FV.push_back({{stack[i], stack[begin_idx + (i + 1) % stack.size()], centroid_idx}});
                local_e_on_input.push_back({{false, false, is_on_input[i]}});
            }

            V_pos.push_back(centroid / (end_idx - begin_idx + 1));
        }
        return;
    }

    stack.push_back(v);
    in_stack[v] = true;
    in_stack_idx[v] = stack.size() - 1;

    std::vector<std::pair<int64_t, Rational>> idx_angle;

    for (int64_t i = 0; i < VV[v].size(); ++i) {
        Vector2r cur_vector = V_pos[VV[v].get(0)] - V_pos[v];
    }

    stack.pop_back();
    in_stack[v] = false;
    in_stack_idx[v] = -1;
}

void edge_insertion(const TriMesh& _trimesh, const EdgeMesh& edgemesh)
{
    TriMesh trimesh = _trimesh; // make a full copy here. we may not want to change the input

    // code is assuming edgemesh doesn't have duplicated vertices

    wmtk::utils::EigenMatrixWriter writer_edge;
    edgemesh.serialize(writer_edge);

    Eigen::MatrixX<int64_t> EV_tmp;
    Eigen::MatrixX<Rational> V_edge;

    writer_edge.get_EV_matrix(EV_tmp);
    writer_edge.get_position_matrix(V_edge);

    // TODO: merge colinear and overlapping segments
    std::vector<std::array<int64_t, 2>> EV;
    for (int64_t i = 0; i < EV_tmp.rows(); ++i) {
        const auto& p0 = V_edge.row(EV_tmp(i, 0));
        const auto& p1 = V_edge.row(EV_tmp(i, 1));
        auto bbox_p = compute_bbox(p0, p1);

        bool overlapped = false;
        for (auto& e : EV) {
            const auto& q0 = V_edge.row(e[0]);
            const auto& q1 = V_edge.row(e[1]);
            auto bbox_q = compute_bbox(q0, q1);

            if (is_colinear(p0, p1, q0, q1) &&
                is_bbox_intersect(bbox_p[0], bbox_p[1], bbox_q[0], bbox_q[1])) {
                overlapped = true;

                // merge two segments
                int64_t endpoint0 = EV_tmp(i, 0);
                int64_t endpoint1 = EV_tmp(i, 1);

                const Vector2r p0p1 = p1 - p0;
                const Vector2r p0q0 = q0 - p0;
                const Vector2r p0q1 = q1 - p0;
                const Vector2r p1q0 = q0 - p1;
                const Vector2r p1q1 = q1 - p1;

                if (p0p1[0] * p0q0[0] < 0 || p0p1[1] * p0q0[1] < 0) {
                    endpoint0 = e[0];
                } else if (p0p1[0] * p0q1[0] < 0 || p0p1[1] * p0q1[1] < 0) {
                    endpoint0 = e[1];
                }

                if (-p0p1[0] * p1q0[0] < 0 || -p0p1[1] * p1q0[1] < 0) {
                    endpoint1 = e[0];
                } else if (-p0p1[0] * p1q1[0] < 0 || -p0p1[1] * p1q1[1] < 0) {
                    endpoint1 = e[1];
                }

                e[0] = endpoint[0];
                e[1] = endpoint[1];

                break;
            }
        }
        if (!overlapped) {
            EV.push_back({{EV_tmp(i, 0), EV_tmp(i, 1)}});
        }
    }


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

    // get vertex map segment -> trimesh
    std::vector<int64_t> v_map_seg_to_tri(V_edge.rows());

    const auto& vs = trimesh.get_all(PrimitiveType::Vertex);
    for (int64_t i = 0; i < vs.size(); ++i) {
        int64_t seg_idx = segment_index_accessor.const_scalar_attribute(vs[i]);

        if (seg_idx < 0) {
            continue;
        }

        assert(seg_idx < V_edge.rows());
        v_map_seg_to_tri[seg_idx] = i;
    }

    // get all edges from trimesh
    wmtk::utils::EigenMatrixWriter writer_tri;
    trimesh.serialize(writer_tri);

    Eigen::MatrixX<int64_t> FV;
    Eigen::MatrixX<Rational> V_tris;

    writer_tri.get_FV_matrix(FV);
    writer_tri.get_position_matrix(V_tris);

    Eigen::MatrixX<int64_t> edges;
    edges = igl::edges(FV);

    // compute intersections, store in each segment
    std::vector<Vector2r> v_final(V_tris.rows());

    for (int64_t i = 0; i < V_tris.rows(); ++i) {
        v_final[i] = V_tris.row(i);
    }

    std::vector<Segment> segments;
    for (int64_t i = 0; i < edges.rows(); ++i) {
        segments.emplace_back(v_final[edges(i, 0)], v_final[edges(i, 1)], edges(i, 0), edges(i, 1));
    }

    const int64_t not_on_input_cnt = segments.size();

    for (int64_t i = 0; i < EV.size(); ++i) {
        const int64_t idx0 = v_map_seg_to_tri[EV[i][0]];
        const int64_t idx1 = v_map_seg_to_tri[EV[i][1]];
        const Vector2r p0 = v_final[idx0];
        const Vector2r p1 = v_final[idx1];

        Segment s(p0, p1, idx0, idx1);

        for (int64_t j = 0; j < segments.size(); ++j) {
            auto& seg = segments[j];
            // check bbox intersection
            if (!is_bbox_intersect(s.bbox_min, s.bbox_max, seg.bbox_min, seg.bbox_max)) {
                continue;
            }

            // get real intersection
            Vector2r res;
            Rational t0, t1;
            if (!segment_segment_inter(p0, p1, seg.p0, seg.p1, res, t0, t1)) {
                continue;
            }

            assert(t0 >= 0 && t0 <= 1);
            assert(t1 >= 0 && t1 <= 1);

            // add point to segment
            bool v_exist_on_0 = false;
            bool v_exist_on_1 = false;
            int64_t new_v_idx = -1;

            for (int64_t k = 0; k < s.points_on_segment.size(); ++k) {
                if (t0 == s.points_on_segment[k].second) {
                    v_exist_on_0 = true;
                    new_v_idx = s.points_on_segment[k].first;
                    break;
                }
            }

            for (int64_t k = 0; k < seg.points_on_segment.size(); ++k) {
                if (t1 == seg.points_on_segment[k].second) {
                    v_exist_on_1 = true;
                    new_v_idx = seg.points_on_segment[k].first;
                    break;
                }
            }

            if (!v_exist_on_0 && !v_exist_on_1) {
                v_final.push_back(res);
                new_v_idx = v_final.size();
            }

            assert(new_v_idx > -1);

            if (!v_exist_on_0) {
                for (int64_t k = 0; k < s.points_on_segment.size() - 1; ++k) {
                    // insert by order of t
                    if (t0 > s.points_on_segment[k].second &&
                        t0 < s.points_on_segment[k + 1].second) {
                        s.points_on_segment.insert(
                            std::next(s.points_on_segment.begin(), k + 1),
                            std::make_pair(new_v_idx, t0));
                        break;
                    }
                }
            }

            if (!v_exist_on_1) {
                for (int64_t k = 0; k < seg.points_on_segment.size() - 1; ++k) {
                    // insert by order of t
                    if (t1 > seg.points_on_segment[k].second &&
                        t1 < seg.points_on_segment[k + 1].second) {
                        seg.points_on_segment.insert(
                            std::next(seg.points_on_segment.begin(), k + 1),
                            std::make_pair(new_v_idx, t1));
                        break;
                    }
                }
            }
        }

        segments.push_back(s);
    }

    // get all v-v connectivity in ccw order
    // bool 1: is on input, bool 2: visited
    std::vector<std::vector<std::tuple<int64_t, bool, bool>>> VV(v_final.size());

    for (int64_t i = 0; i < not_on_input_cnt; ++i) {
        for (int64_t j = 0; j < segments[i].points_on_segment.size() - 1; ++j) {
            VV[segments[i].points_on_segment[j].first].push_back(
                std::tie(segments[i].points_on_segment[j + 1].first, false, false));
            VV[segments[i].points_on_segment[j + 1].first].push_back(
                std::tie(segments[i].points_on_segment[j].first, false, false));
        }
    }

    for (int64_t i = not_on_input_cnt; i < segments.size(); ++i) {
        for (int64_t j = 0; j < segments[i].points_on_segment.size() - 1; ++j) {
            VV[segments[i].points_on_segment[j].first].push_back(
                std::tie(segments[i].points_on_segment[j + 1].first, true, false));
            VV[segments[i].points_on_segment[j + 1].first].push_back(
                std::tie(segments[i].points_on_segment[j].first, true, false));
        }
    }

    std::vector<std::array<int64_t, 3>> triangles;
}

} // namespace wmtk::components::internal