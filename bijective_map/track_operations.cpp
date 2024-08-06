#include <fstream>
#include <iostream>
#include <sstream>

#include "track_operations.hpp"


// TODO: Rational Version of this code
Eigen::Vector3d ComputeBarycentricCoordinates3D(
    const Eigen::Vector3d& p,
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b,
    const Eigen::Vector3d& c,
    double eps = 1e-3)
{
    Eigen::Vector3d v0 = b - a, v1 = c - a, v2 = p - a;
    Eigen::Vector3d n = v0.cross(v1);
    n.normalized();
    if (std::abs(n.dot(p - a)) > eps) {
        // std::cout << "not on the plane, " << std::abs(n.dot(p - a)) << std::endl;
        // not on this face
        return Eigen::Vector3d(-1, -1, -1);
    }
    double d00 = v0.dot(v0);
    double d01 = v0.dot(v1);
    double d11 = v1.dot(v1);
    double d20 = v2.dot(v0);
    double d21 = v2.dot(v1);
    double denom = d00 * d11 - d01 * d01;
    double v = (d11 * d20 - d01 * d21) / denom;
    double w = (d00 * d21 - d01 * d20) / denom;
    double u = 1.0 - v - w;
    return Eigen::Vector3d(u, v, w);
}

Eigen::Vector3d ComputeBarycentricCoordinates2D(
    const Eigen::Vector2d& p,
    const Eigen::Vector2d& a,
    const Eigen::Vector2d& b,
    const Eigen::Vector2d& c)
{
    Eigen::Vector2d v0 = b - a, v1 = c - a, v2 = p - a;
    double d00 = v0.dot(v0);
    double d01 = v0.dot(v1);
    double d11 = v1.dot(v1);
    double d20 = v2.dot(v0);
    double d21 = v2.dot(v1);
    double denom = d00 * d11 - d01 * d01;
    double v = (d11 * d20 - d01 * d21) / denom;
    double w = (d00 * d21 - d01 * d20) / denom;
    double u = 1.0 - v - w;
    return Eigen::Vector3d(u, v, w);
}

// Rational version of ComputeBarycentricCoordinates2D
Eigen::Vector3<wmtk::Rational> ComputeBarycentricCoordinates2D_r(
    const Eigen::Vector2<wmtk::Rational>& p,
    const Eigen::Vector2<wmtk::Rational>& a,
    const Eigen::Vector2<wmtk::Rational>& b,
    const Eigen::Vector2<wmtk::Rational>& c)
{
    Eigen::Vector2<wmtk::Rational> v0 = b - a, v1 = c - a, v2 = p - a;
    wmtk::Rational d00 = v0.dot(v0);
    wmtk::Rational d01 = v0.dot(v1);
    wmtk::Rational d11 = v1.dot(v1);
    wmtk::Rational d20 = v2.dot(v0);
    wmtk::Rational d21 = v2.dot(v1);
    wmtk::Rational denom = d00 * d11 - d01 * d01;
    wmtk::Rational v = (d11 * d20 - d01 * d21) / denom;
    wmtk::Rational w = (d00 * d21 - d01 * d20) / denom;
    wmtk::Rational u = 1.0 - v - w;
    return Eigen::Vector3<wmtk::Rational>(u, v, w);
}


void handle_collapse_edge(
    const Eigen::MatrixXd& UV_joint,
    const Eigen::MatrixXi& F_before,
    const Eigen::MatrixXi& F_after,
    const std::vector<int64_t>& v_id_map_joint,
    const std::vector<int64_t>& id_map_before,
    const std::vector<int64_t>& id_map_after,
    std::vector<query_point>& query_points,
    bool use_rational)
{
    if (use_rational) {
        handle_collapse_edge_r(
            UV_joint,
            F_before,
            F_after,
            v_id_map_joint,
            id_map_before,
            id_map_after,
            query_points);
        return;
    }
    std::cout << "Handling EdgeCollapse" << std::endl;
    for (int id = 0; id < query_points.size(); id++) {
        query_point& qp = query_points[id];
        if (qp.f_id < 0) continue;

        // find if qp is in the id_map_after
        auto it = std::find(id_map_after.begin(), id_map_after.end(), qp.f_id);
        if (it != id_map_after.end()) {
            // std::cout << "find qp: " << qp.f_id << std::endl;
            // std::cout << "qp.bc: (" << qp.bc(0) << ", " << qp.bc(1) << ", " << qp.bc(2) << ")"
            //           << std::endl;

            // find the index of qp in id_map_after
            int local_index_in_f_after = std::distance(id_map_after.begin(), it);
            // offset of the qp.fv_ids
            int offset_in_f_after = -1;
            for (int i = 0; i < 3; i++) {
                if (v_id_map_joint[F_after(local_index_in_f_after, i)] == qp.fv_ids[0]) {
                    offset_in_f_after = i;
                    break;
                }
            }
            if (offset_in_f_after == -1) {
                std::cout << "something is wrong!" << std::endl;
                continue;
                // return;
            }

            // compute the location of the qp
            Eigen::Vector2d p(0, 0);
            for (int i = 0; i < 3; i++) {
                p += UV_joint.row(F_after(local_index_in_f_after, (i + offset_in_f_after) % 3)) *
                     qp.bc(i);
            }

            // std::cout << "p:\n" << p << std::endl;

            // compute bc of the p in (V, F)_before
            int local_index_in_f_before = -1;
            double bc_min_coef = 1;
            bool bc_updated = false;
            for (int i = 0; i < F_before.rows(); i++) {
                Eigen::Vector3d bc;

                bc = ComputeBarycentricCoordinates2D(
                    p,
                    UV_joint.row(F_before(i, 0)),
                    UV_joint.row(F_before(i, 1)),
                    UV_joint.row(F_before(i, 2)));

                // std::cout << "bc candidate:" << bc << std::endl;
                // std::cout << "check with p:\n"
                //           << bc(0) * UV_joint.row(F_before(i, 0)) +
                //                  bc(1) * UV_joint.row(F_before(i, 1)) +
                //                  bc(2) * UV_joint.row(F_before(i, 2))
                //           << std::endl;
                if (-bc.minCoeff() < bc_min_coef) {
                    // std::cout << "good!" << std::endl;
                    bc_min_coef = -bc.minCoeff();
                    local_index_in_f_before = i;
                    qp.bc = bc;
                    bc_updated = true;
                }
            }

            if (!bc_updated) {
                std::cout << "bc not updated\n" << std::endl;
                continue;
                // return;
            }
            // else {
            //     std::cout << "bc updated\n" << std::endl;
            // }

            // update qp
            qp.f_id = id_map_before[local_index_in_f_before];
            for (int i = 0; i < 3; i++) {
                qp.fv_ids[i] = v_id_map_joint[F_before(local_index_in_f_before, i)];
            }
            // avoid numerical issue
            qp.bc[0] = std::max(0.0, std::min(1.0, qp.bc[0]));
            qp.bc[1] = std::max(0.0, std::min(1.0, qp.bc[1]));
            qp.bc[2] = std::max(0.0, std::min(1.0, qp.bc[2]));
            qp.bc /= qp.bc.sum();

            // std::cout << "qp.bc: (" << qp.bc(0) << ", " << qp.bc(1) << ", " << qp.bc(2) << ")"
            //           << std::endl;
        }
    }
}

// Rational version of handle_collapse_edge
void handle_collapse_edge_r(
    const Eigen::MatrixXd& UV_joint,
    const Eigen::MatrixXi& F_before,
    const Eigen::MatrixXi& F_after,
    const std::vector<int64_t>& v_id_map_joint,
    const std::vector<int64_t>& id_map_before,
    const std::vector<int64_t>& id_map_after,
    std::vector<query_point>& query_points)
{
    std::cout << "Handling EdgeCollapse Rational" << std::endl;
    // first convert the UV_joint to Rational
    Eigen::MatrixX<wmtk::Rational> UV_joint_r(UV_joint.rows(), UV_joint.cols());
    for (int i = 0; i < UV_joint.rows(); i++) {
        for (int j = 0; j < UV_joint.cols(); j++) {
            UV_joint_r(i, j) = wmtk::Rational(UV_joint(i, j));
        }
    }


    for (int id = 0; id < query_points.size(); id++) {
        query_point& qp = query_points[id];
        if (qp.f_id < 0) continue;

        // find if qp is in the id_map_after
        auto it = std::find(id_map_after.begin(), id_map_after.end(), qp.f_id);
        if (it != id_map_after.end()) {
            // find the index of qp in id_map_after
            int local_index_in_f_after = std::distance(id_map_after.begin(), it);
            // offset of the qp.fv_ids
            int offset_in_f_after = -1;
            for (int i = 0; i < 3; i++) {
                if (v_id_map_joint[F_after(local_index_in_f_after, i)] == qp.fv_ids[0]) {
                    offset_in_f_after = i;
                    break;
                }
            }
            if (offset_in_f_after == -1) {
                std::cout << "find qp: " << qp.f_id << std::endl;
                std::cout << "qp.bc: (" << qp.bc(0) << ", " << qp.bc(1) << ", " << qp.bc(2) << ")"
                          << std::endl;
                std::cout << "qp.fv_id: " << qp.fv_ids[0] << ", " << qp.fv_ids[1] << ", "
                          << qp.fv_ids[2] << std::endl;
                std::cout << "local_index_in_f_after: " << local_index_in_f_after << std::endl;
                std::cout << "something is wrong!" << std::endl;
                continue;
                // return;
            }

            // compute the location of the qp
            Eigen::Vector2<wmtk::Rational> p(0, 0);
            for (int i = 0; i < 3; i++) {
                p = p + UV_joint_r.row(F_after(local_index_in_f_after, (i + offset_in_f_after) % 3))
                                .transpose() *
                            wmtk::Rational(qp.bc(i));
            }


            // compute bc of the p in (V, F)_before
            int local_index_in_f_before = -1;

            // if computation is exact we don't need this anymore
            // double bc_min_coef = 1;
            bool bc_updated = false;
            for (int i = 0; i < F_before.rows(); i++) {
                Eigen::Vector3<wmtk::Rational> bc_rational;

                bc_rational = ComputeBarycentricCoordinates2D_r(
                    p,
                    UV_joint_r.row(F_before(i, 0)),
                    UV_joint_r.row(F_before(i, 1)),
                    UV_joint_r.row(F_before(i, 2)));


                if (bc_rational.minCoeff() >= 0 && bc_rational.maxCoeff() <= 1) {
                    local_index_in_f_before = i;
                    qp.bc(0) = bc_rational(0).to_double();
                    qp.bc(1) = bc_rational(1).to_double();
                    qp.bc(2) = bc_rational(2).to_double();
                    bc_updated = true;
                }
            }

            if (!bc_updated) {
                std::cout << "bc not updated in rational\n" << std::endl;
                continue;
                // return;
            }

            // update qp
            qp.f_id = id_map_before[local_index_in_f_before];
            for (int i = 0; i < 3; i++) {
                qp.fv_ids[i] = v_id_map_joint[F_before(local_index_in_f_before, i)];
            }
        }
    }
}

bool intersectSegmentEdge(
    const Eigen::Vector2d& a,
    const Eigen::Vector2d& b,
    const Eigen::Vector2d& c,
    const Eigen::Vector2d& d,
    Eigen::Vector2d& barycentric)
{
    Eigen::Vector2d ab = b - a;
    Eigen::Vector2d cd = d - c;
    Eigen::Vector2d ac = c - a;

    double denominator = (ab.x() * cd.y() - ab.y() * cd.x());
    if (std::abs(denominator) < 1e-8) return false; // parallel

    double t = (ac.x() * cd.y() - ac.y() * cd.x()) / denominator;
    double u = -(ab.x() * ac.y() - ab.y() * ac.x()) / denominator;

    if (t >= 0 && t <= 1 && u >= 0 && u <= 1) {
        // Calculate barycentric coordinates for intersection
        double alpha = 1 - u;
        double beta = u;
        barycentric = Eigen::Vector2d(alpha, beta);
        return true;
    }
    return false;
}

bool intersectSegmentEdge_r(
    const Eigen::Vector2<wmtk::Rational>& a,
    const Eigen::Vector2<wmtk::Rational>& b,
    const Eigen::Vector2<wmtk::Rational>& c,
    const Eigen::Vector2<wmtk::Rational>& d,
    Eigen::Vector2<wmtk::Rational>& barycentric)
{
    Eigen::Vector2<wmtk::Rational> ab = b - a;
    Eigen::Vector2<wmtk::Rational> cd = d - c;
    Eigen::Vector2<wmtk::Rational> ac = c - a;

    wmtk::Rational denominator = (ab.x() * cd.y() - ab.y() * cd.x());
    if (denominator == wmtk::Rational(0)) return false; // parallel

    wmtk::Rational t = (ac.x() * cd.y() - ac.y() * cd.x()) / denominator;
    wmtk::Rational u = -(ab.x() * ac.y() - ab.y() * ac.x()) / denominator;

    if (t >= 0 && t <= 1 && u >= 0 && u <= 1) {
        // Calculate barycentric coordinates for intersection
        wmtk::Rational alpha = 1 - u;
        wmtk::Rational beta = u;
        barycentric = Eigen::Vector2<wmtk::Rational>(alpha, beta);
        return true;
    }
    return false;
}

void handle_collapse_edge_curve(
    const Eigen::MatrixXd& UV_joint,
    const Eigen::MatrixXi& F_before,
    const Eigen::MatrixXi& F_after,
    const std::vector<int64_t>& v_id_map_joint,
    const std::vector<int64_t>& id_map_before,
    const std::vector<int64_t>& id_map_after,
    query_curve& curve,
    bool use_rational)
{
    if (use_rational) {
        handle_collapse_edge_curve_r(
            UV_joint,
            F_before,
            F_after,
            v_id_map_joint,
            id_map_before,
            id_map_after,
            curve);
        return;
    }
    std::cout << "Handling EdgeCollapse curve" << std::endl;
    int curve_length = curve.segments.size();
    for (int id = 0; id < curve_length; id++) {
        query_segment& qs = curve.segments[id];
        query_point qp0 = {qs.f_id, qs.bcs[0], qs.fv_ids};
        query_point qp1 = {qs.f_id, qs.bcs[1], qs.fv_ids};
        std::vector<query_point> query_points = {qp0, qp1};
        // std::cout << "qp0.f_id: " << qp0.f_id << std::endl;
        // std::cout << "qp0.bc:\n" << qp0.bc << std::endl;
        // std::cout << "qp1.f_id: " << qp1.f_id << std::endl;
        // std::cout << "qp1.bc:\n" << qp1.bc << std::endl;
        handle_collapse_edge(
            UV_joint,
            F_before,
            F_after,
            v_id_map_joint,
            id_map_before,
            id_map_after,
            query_points);

        // std::cout << "query_points[0].f_id: " << query_points[0].f_id << std::endl;
        // std::cout << "query_points[0].bc:\n" << query_points[0].bc << std::endl;
        // std::cout << "query_points[1].f_id: " << query_points[1].f_id << std::endl;
        // std::cout << "query_points[1].bc:\n" << query_points[1].bc << std::endl;

        if (query_points[0].f_id == query_points[1].f_id) {
            std::cout << "no intersections" << std::endl;
            // no intersections
            qs.f_id = query_points[0].f_id;
            qs.bcs[0] = query_points[0].bc;
            qs.bcs[1] = query_points[1].bc;
            qs.fv_ids = query_points[0].fv_ids;
        } else {
            double eps = 1e-8;
            std::cout << "need compute intersections" << std::endl;
            // compute intersections
            int old_next_seg = curve.next_segment_ids[id];

            Eigen::MatrixXi TT, TTi;
            igl::triangle_triangle_adjacency(F_before, TT, TTi);

            // check vertex_on_edge case
            for (int i = 0; i < 3; i++) {
                if (query_points[0].bc(i) < eps) {
                    if (TT(query_points[0].f_id, (i + 1) % 3) == query_points[1].f_id) {
                        // TODO: implement this
                    }
                }
            }
            // compute the two end points
            auto it = std::find(id_map_before.begin(), id_map_before.end(), query_points[0].f_id);
            int current_fid = std::distance(id_map_before.begin(), it);
            Eigen::Vector2d p0(0, 0);
            for (int i = 0; i < 3; i++) {
                p0 += UV_joint.row(F_before(current_fid, i)) * query_points[0].bc(i);
            }
            it = std::find(id_map_before.begin(), id_map_before.end(), query_points[1].f_id);
            int target_fid = std::distance(id_map_before.begin(), it);
            Eigen::Vector2d p1(0, 0);
            for (int i = 0; i < 3; i++) {
                p1 += UV_joint.row(F_before(target_fid, i)) * query_points[1].bc(i);
            }

            // std::cout << "current fid: " << current_fid << std::endl;
            // std::cout << "target fid: " << target_fid << std::endl;
            // std::cout << "TT(current_fid): " << TT.row(current_fid) << std::endl;
            // std::cout << "TTi(current_fid): " << TTi.row(current_fid) << std::endl;
            int current_edge_id = -1;
            Eigen::Vector2d last_edge_bc;
            {
                int next_edge_id0 = -1;
                // find the first intersection
                for (int edge_id = 0; edge_id < 3; edge_id++) {
                    Eigen::VectorXd a = UV_joint.row(F_before(current_fid, edge_id));
                    Eigen::VectorXd b = UV_joint.row(F_before(current_fid, (edge_id + 1) % 3));
                    Eigen::Vector2d edge_bc;
                    if (intersectSegmentEdge(p0, p1, a, b, edge_bc)) {
                        next_edge_id0 = edge_id;
                        qs.f_id = query_points[0].f_id;
                        qs.bcs[0] = query_points[0].bc;
                        qs.bcs[1] << 0, 0, 0;
                        qs.bcs[1](edge_id) = edge_bc(0);
                        qs.bcs[1]((edge_id + 1) % 3) = edge_bc(1);
                        qs.fv_ids = query_points[0].fv_ids;
                        last_edge_bc = edge_bc;
                        curve.next_segment_ids[id] =
                            curve.segments.size(); // next segment will add to the end
                        break;
                    }
                }

                if (next_edge_id0 == -1) {
                    std::cout << "no intersection found" << std::endl;
                    throw std::runtime_error("no intersection found");
                    continue;
                } else {
                    // std::cout << "next_edge_id0: " << next_edge_id0 << std::endl;
                }
                current_edge_id = TTi(current_fid, next_edge_id0);
                current_fid = TT(current_fid, next_edge_id0);
                // std::cout << "last_edge_bc:\n" << last_edge_bc << std::endl;
            }
            // find the rest intersections
            while (current_fid != target_fid) {
                // std::cout << "current fid: " << current_fid << std::endl;
                // std::cout << "target fid: " << target_fid << std::endl;
                // std::cout << "current edge id: " << current_edge_id << std::endl;
                // std::cout << "last_edge_bc: " << last_edge_bc << std::endl;

                int next_edge_id = -1;
                for (int edge_id = 0; edge_id < 3; edge_id++) {
                    if (edge_id == current_edge_id) continue;
                    Eigen::VectorXd a = UV_joint.row(F_before(current_fid, edge_id));
                    Eigen::VectorXd b = UV_joint.row(F_before(current_fid, (edge_id + 1) % 3));
                    Eigen::Vector2d edge_bc;
                    query_segment qs_new;
                    if (intersectSegmentEdge(p0, p1, a, b, edge_bc)) {
                        next_edge_id = edge_id;
                        qs_new.f_id = id_map_before[current_fid];
                        qs_new.bcs[0] = Eigen::Vector3d(0, 0, 0);
                        qs_new.bcs[1] = Eigen::Vector3d(0, 0, 0);

                        qs_new.bcs[0](current_edge_id) = last_edge_bc(1);
                        qs_new.bcs[0]((current_edge_id + 1) % 3) = last_edge_bc(0);
                        qs_new.bcs[1](next_edge_id) = edge_bc(0);
                        qs_new.bcs[1]((next_edge_id + 1) % 3) = edge_bc(1);

                        qs_new.fv_ids << v_id_map_joint[F_before(current_fid, 0)],
                            v_id_map_joint[F_before(current_fid, 1)],
                            v_id_map_joint[F_before(current_fid, 2)];

                        // push the new segment
                        curve.segments.push_back(qs_new);
                        curve.next_segment_ids.push_back(curve.next_segment_ids.size());

                        // record the last edge
                        last_edge_bc = edge_bc;
                        break;
                    }
                }

                if (next_edge_id == -1) {
                    std::cout << "no intersection found" << std::endl;
                    throw std::runtime_error("no intersection found");
                    continue;
                } else {
                    // std::cout << "next_edge_id: " << next_edge_id << std::endl;
                    // std::cout << "TT(current_fid): " << TT.row(current_fid) << std::endl;
                    // std::cout << "TTi(current_fid): " << TTi.row(current_fid) << std::endl;
                    // std::cout << std::endl;
                }
                current_edge_id = TTi(current_fid, next_edge_id);
                current_fid = TT(current_fid, next_edge_id);
            }

            // TODO: last segment
            {
                query_segment qs_new;
                qs_new.f_id = query_points[1].f_id;
                qs_new.bcs[0] = Eigen::Vector3d(0, 0, 0);
                qs_new.bcs[1] = query_points[1].bc;
                qs_new.bcs[0](current_edge_id) = last_edge_bc(1);
                qs_new.bcs[0]((current_edge_id + 1) % 3) = last_edge_bc(0);
                qs_new.fv_ids = query_points[1].fv_ids;

                // push the new segment
                curve.segments.push_back(qs_new);
                curve.next_segment_ids.push_back(old_next_seg);
            }
        }
    }
}

void handle_collapse_edge_curve_r(
    const Eigen::MatrixXd& UV_joint,
    const Eigen::MatrixXi& F_before,
    const Eigen::MatrixXi& F_after,
    const std::vector<int64_t>& v_id_map_joint,
    const std::vector<int64_t>& id_map_before,
    const std::vector<int64_t>& id_map_after,
    query_curve& curve)
{}

void handle_split_edge(
    const Eigen::MatrixXd& V_before,
    const Eigen::MatrixXi& F_before,
    const std::vector<int64_t>& id_map_before,
    const std::vector<int64_t>& v_id_map_before,
    const Eigen::MatrixXd& V_after,
    const Eigen::MatrixXi& F_after,
    const std::vector<int64_t>& id_map_after,
    const std::vector<int64_t>& v_id_map_after,
    std::vector<query_point>& query_points)
{
    std::cout << "Handling EdgeSplit" << std::endl;
    // igl::parallel_for(query_points.size(), [&](int id) {
    for (int id = 0; id < query_points.size(); id++) {
        query_point& qp = query_points[id];
        if (qp.f_id < 0) continue;
        // find if qp is in the id_map_after
        auto it = std::find(id_map_after.begin(), id_map_after.end(), qp.f_id);
        if (it != id_map_after.end()) {
            // std::cout << "find qp: " << qp.f_id << std::endl;
            // std::cout << "qp.bc: (" << qp.bc(0) << ", " << qp.bc(1) << ", " << qp.bc(2) << ")"
            //           << std::endl;

            // find the index of qp in id_map_after
            int local_index_in_f_after = std::distance(id_map_after.begin(), it);
            // offset of the qp.fv_ids
            int offset_in_f_after = -1;
            for (int i = 0; i < 3; i++) {
                if (v_id_map_after[F_after(local_index_in_f_after, i)] == qp.fv_ids[0]) {
                    offset_in_f_after = i;
                    break;
                }
            }
            if (offset_in_f_after == -1) {
                std::cout << "something is wrong!" << std::endl;
                continue;
                // return;
            }

            // compute the location of the qp
            int V_cols = V_after.cols();
            Eigen::VectorXd p = Eigen::VectorXd::Zero(V_cols);
            for (int i = 0; i < 3; i++) {
                p += V_after.row(F_after(local_index_in_f_after, (i + offset_in_f_after) % 3)) *
                     qp.bc(i);
            }


            // compute bc of the p in (V, F)_before
            int local_index_in_f_before = -1;
            double bc_min_coef = 1;
            bool bc_updated = false;
            for (int i = 0; i < F_before.rows(); i++) {
                Eigen::Vector3d bc;
                if (V_cols == 2) {
                    bc = ComputeBarycentricCoordinates2D(
                        p,
                        V_before.row(F_before(i, 0)),
                        V_before.row(F_before(i, 1)),
                        V_before.row(F_before(i, 2)));
                } else // V_cols == 3
                {
                    bc = ComputeBarycentricCoordinates3D(
                        p,
                        V_before.row(F_before(i, 0)),
                        V_before.row(F_before(i, 1)),
                        V_before.row(F_before(i, 2)));
                }
                if (-bc.minCoeff() < bc_min_coef) {
                    bc_min_coef = -bc.minCoeff();
                    local_index_in_f_before = i;
                    qp.bc = bc;
                    bc_updated = true;
                }
            }

            if (!bc_updated) {
                std::cout << "bc not updated\n" << std::endl;
                continue;
                // return;
            }

            // update qp
            qp.f_id = id_map_before[local_index_in_f_before];
            for (int i = 0; i < 3; i++) {
                qp.fv_ids[i] = v_id_map_before[F_before(local_index_in_f_before, i)];
            }
            // avoid numerical issue
            qp.bc[0] = std::max(0.0, std::min(1.0, qp.bc[0]));
            qp.bc[1] = std::max(0.0, std::min(1.0, qp.bc[1]));
            qp.bc[2] = std::max(0.0, std::min(1.0, qp.bc[2]));
            qp.bc /= qp.bc.sum();

            // std::cout << "qp-> " << qp.f_id << std::endl;
            // std::cout << "qp.bc: (" << qp.bc(0) << ", " << qp.bc(1) << ", " << qp.bc(2) << ")"
            //           << std::endl
            //           << std::endl;
        }
    }
    // });
}

void handle_swap_edge(
    const Eigen::MatrixXd& V_before,
    const Eigen::MatrixXi& F_before,
    const std::vector<int64_t>& id_map_before,
    const std::vector<int64_t>& v_id_map_before,
    const Eigen::MatrixXd& V_after,
    const Eigen::MatrixXi& F_after,
    const std::vector<int64_t>& id_map_after,
    const std::vector<int64_t>& v_id_map_after,
    std::vector<query_point>& query_points)
{
    std::cout << "Handling EdgeSwap" << std::endl;
    // igl::parallel_for(query_points.size(), [&](int id) {
    for (int id = 0; id < query_points.size(); id++) {
        query_point& qp = query_points[id];
        if (qp.f_id < 0) continue;
        // find if qp is in the id_map_after
        auto it = std::find(id_map_after.begin(), id_map_after.end(), qp.f_id);
        if (it != id_map_after.end()) {
            // std::cout << "find qp: " << qp.f_id << std::endl;
            // std::cout << "qp.bc: (" << qp.bc(0) << ", " << qp.bc(1) << ", " << qp.bc(2) << ")"
            //           << std::endl;

            // find the index of qp in id_map_after
            int local_index_in_f_after = std::distance(id_map_after.begin(), it);
            // offset of the qp.fv_ids
            int offset_in_f_after = -1;
            for (int i = 0; i < 3; i++) {
                if (v_id_map_after[F_after(local_index_in_f_after, i)] == qp.fv_ids[0]) {
                    offset_in_f_after = i;
                    break;
                }
            }
            if (offset_in_f_after == -1) {
                std::cout << "something is wrong!" << std::endl;
                continue;
                // return;
            }

            // compute the location of the qp
            int V_cols = V_after.cols();
            Eigen::VectorXd p = Eigen::VectorXd::Zero(V_cols);
            for (int i = 0; i < 3; i++) {
                p += V_after.row(F_after(local_index_in_f_after, (i + offset_in_f_after) % 3)) *
                     qp.bc(i);
            }


            // compute bc of the p in (V, F)_before
            int local_index_in_f_before = -1;
            double bc_min_coef = 1;
            bool bc_updated = false;
            for (int i = 0; i < F_before.rows(); i++) {
                Eigen::Vector3d bc;
                if (V_cols == 2) {
                    bc = ComputeBarycentricCoordinates2D(
                        p,
                        V_before.row(F_before(i, 0)),
                        V_before.row(F_before(i, 1)),
                        V_before.row(F_before(i, 2)));
                } else // V_cols == 3
                {
                    bc = ComputeBarycentricCoordinates3D(
                        p,
                        V_before.row(F_before(i, 0)),
                        V_before.row(F_before(i, 1)),
                        V_before.row(F_before(i, 2)),
                        100000);
                }
                if (-bc.minCoeff() < bc_min_coef) {
                    bc_min_coef = -bc.minCoeff();
                    local_index_in_f_before = i;
                    qp.bc = bc;
                    bc_updated = true;
                }
            }

            if (!bc_updated) {
                std::cout << "bc not updated\n" << std::endl;
                continue;
                // return;
            }

            // update qp
            qp.f_id = id_map_before[local_index_in_f_before];
            for (int i = 0; i < 3; i++) {
                qp.fv_ids[i] = v_id_map_before[F_before(local_index_in_f_before, i)];
            }
            // avoid numerical issue
            qp.bc[0] = std::max(0.0, std::min(1.0, qp.bc[0]));
            qp.bc[1] = std::max(0.0, std::min(1.0, qp.bc[1]));
            qp.bc[2] = std::max(0.0, std::min(1.0, qp.bc[2]));
            qp.bc /= qp.bc.sum();

            // std::cout << "qp-> " << qp.f_id << std::endl;
            // std::cout << "qp.bc: (" << qp.bc(0) << ", " << qp.bc(1) << ", " << qp.bc(2) << ")"
            //           << std::endl
            //           << std::endl;
        }
    }
    // });
}

void parse_consolidate_file(
    const json& operation_log,
    std::vector<int64_t>& face_ids_maps,
    std::vector<int64_t>& vertex_ids_maps)
{
    face_ids_maps = operation_log["new2old"][2].get<std::vector<int64_t>>();
    vertex_ids_maps = operation_log["new2old"][0].get<std::vector<int64_t>>();
}

template <typename Matrix>
Matrix json_to_matrix(const json& js)
{
    int rows = js["rows"];
    int cols = js["values"][0].size();

    Matrix mat(rows, cols);
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            mat(i, j) = js["values"][i][j];
        }
    }

    return mat;
}

void parse_edge_split_file(
    const json& operation_log,
    Eigen::MatrixXd& V_before,
    Eigen::MatrixXi& F_before,
    std::vector<int64_t>& id_map_before,
    std::vector<int64_t>& v_id_map_before,
    Eigen::MatrixXd& V_after,
    Eigen::MatrixXi& F_after,
    std::vector<int64_t>& id_map_after,
    std::vector<int64_t>& v_id_map_after)
{
    F_before = json_to_matrix<Eigen::MatrixXi>(operation_log["F_before"]);
    V_before = json_to_matrix<Eigen::MatrixXd>(operation_log["V_before"]);
    id_map_before = operation_log["F_id_map_before"].get<std::vector<int64_t>>();
    v_id_map_before = operation_log["V_id_map_before"].get<std::vector<int64_t>>();

    F_after = json_to_matrix<Eigen::MatrixXi>(operation_log["F_after"]);
    V_after = json_to_matrix<Eigen::MatrixXd>(operation_log["V_after"]);
    id_map_after = operation_log["F_id_map_after"].get<std::vector<int64_t>>();
    v_id_map_after = operation_log["V_id_map_after"].get<std::vector<int64_t>>();
}

void parse_edge_collapse_file(
    const json& operation_log,
    Eigen::MatrixXd& UV_joint,
    Eigen::MatrixXi& F_before,
    Eigen::MatrixXi& F_after,
    std::vector<int64_t>& v_id_map_joint,
    std::vector<int64_t>& id_map_before,
    std::vector<int64_t>& id_map_after)
{
    UV_joint = json_to_matrix<Eigen::MatrixXd>(operation_log["UV_joint"]);
    F_before = json_to_matrix<Eigen::MatrixXi>(operation_log["F_before"]);
    F_after = json_to_matrix<Eigen::MatrixXi>(operation_log["F_after"]);

    v_id_map_joint = operation_log["v_id_map_joint"].get<std::vector<int64_t>>();
    id_map_before = operation_log["F_id_map_before"].get<std::vector<int64_t>>();
    id_map_after = operation_log["F_id_map_after"].get<std::vector<int64_t>>();
}