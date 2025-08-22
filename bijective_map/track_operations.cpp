#include <chrono>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>

#include "track_operations.hpp"
#include "track_operations_curve.hpp"
// Function to save a vector<query_curve> to a file
void save_query_curves(const std::vector<query_curve>& curves, const std::string& filename)
{
    std::ofstream ofs(filename, std::ios::binary);

    // Write number of curves
    size_t num_curves = curves.size();
    ofs.write(reinterpret_cast<const char*>(&num_curves), sizeof(num_curves));

    // Save each query_curve
    for (const auto& curve : curves) {
        // Write number of segments
        size_t num_segments = curve.segments.size();
        ofs.write(reinterpret_cast<const char*>(&num_segments), sizeof(num_segments));

        // Write each segment
        for (const auto& segment : curve.segments) {
            ofs.write(reinterpret_cast<const char*>(&segment.f_id), sizeof(segment.f_id));
            ofs.write(reinterpret_cast<const char*>(segment.bcs[0].data()), sizeof(segment.bcs[0]));
            ofs.write(reinterpret_cast<const char*>(segment.bcs[1].data()), sizeof(segment.bcs[1]));
            ofs.write(reinterpret_cast<const char*>(segment.fv_ids.data()), sizeof(segment.fv_ids));
        }

        // Write next_segment_ids
        size_t num_next_ids = curve.next_segment_ids.size();
        ofs.write(reinterpret_cast<const char*>(&num_next_ids), sizeof(num_next_ids));
        ofs.write(
            reinterpret_cast<const char*>(curve.next_segment_ids.data()),
            num_next_ids * sizeof(int));
    }

    ofs.close();
}

// Function to load a vector<query_curve> from a file
std::vector<query_curve> load_query_curves(const std::string& filename)
{
    std::ifstream ifs(filename, std::ios::binary);
    std::vector<query_curve> curves;

    // Read number of curves
    size_t num_curves;
    ifs.read(reinterpret_cast<char*>(&num_curves), sizeof(num_curves));
    curves.resize(num_curves);

    // Load each query_curve
    for (auto& curve : curves) {
        // Read number of segments
        size_t num_segments;
        ifs.read(reinterpret_cast<char*>(&num_segments), sizeof(num_segments));
        curve.segments.resize(num_segments);

        // Read each segment
        for (auto& segment : curve.segments) {
            ifs.read(reinterpret_cast<char*>(&segment.f_id), sizeof(segment.f_id));
            ifs.read(reinterpret_cast<char*>(segment.bcs[0].data()), sizeof(segment.bcs[0]));
            ifs.read(reinterpret_cast<char*>(segment.bcs[1].data()), sizeof(segment.bcs[1]));
            ifs.read(reinterpret_cast<char*>(segment.fv_ids.data()), sizeof(segment.fv_ids));
        }

        // Read next_segment_ids
        size_t num_next_ids;
        ifs.read(reinterpret_cast<char*>(&num_next_ids), sizeof(num_next_ids));
        curve.next_segment_ids.resize(num_next_ids);
        ifs.read(
            reinterpret_cast<char*>(curve.next_segment_ids.data()),
            num_next_ids * sizeof(int));
    }

    ifs.close();
    return curves;
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

    // Check for degenerate triangle (collinear points)
    if (denom == wmtk::Rational(0)) {
        throw std::runtime_error("Degenerate triangle: points are collinear");
    }

    wmtk::Rational v = (d11 * d20 - d01 * d21) / denom;
    wmtk::Rational w = (d00 * d21 - d01 * d20) / denom;
    wmtk::Rational u = wmtk::Rational(1) - v - w;

    wmtk::Rational sum = u + v + w;
    if (sum == wmtk::Rational(0)) {
        throw std::runtime_error("Invalid barycentric coordinates: sum is zero");
    }

    return Eigen::Vector3<wmtk::Rational>(u, v, w) / sum;
}


// Precomputed barycentric helper (2D)
// defined in header

static std::vector<BarycentricPrecompute2D> build_barycentric_cache_2d(
    const Eigen::MatrixX<wmtk::Rational>& V2,
    const Eigen::MatrixXi& F)
{
    std::vector<BarycentricPrecompute2D> cache;
    cache.resize(F.rows());
    for (int i = 0; i < F.rows(); ++i) {
        Eigen::Matrix<wmtk::Rational, 2, 1> A = V2.row(F(i, 0)).head<2>().transpose();
        Eigen::Matrix<wmtk::Rational, 2, 1> B = V2.row(F(i, 1)).head<2>().transpose();
        Eigen::Matrix<wmtk::Rational, 2, 1> C = V2.row(F(i, 2)).head<2>().transpose();

        Eigen::Matrix<wmtk::Rational, 2, 2> M;
        M.col(0) = B - A;
        M.col(1) = C - A;

        wmtk::Rational det = M(0, 0) * M(1, 1) - M(0, 1) * M(1, 0);
        if (det == wmtk::Rational(0)) {
            throw std::runtime_error("Degenerate triangle in barycentric precompute (det==0)");
        }
        Eigen::Matrix<wmtk::Rational, 2, 2> Minv;
        Minv(0, 0) = M(1, 1) / det;
        Minv(0, 1) = -M(0, 1) / det;
        Minv(1, 0) = -M(1, 0) / det;
        Minv(1, 1) = M(0, 0) / det;

        cache[i].Minv = Minv;
        cache[i].a = A;
    }
    return cache;
}

static inline Eigen::Matrix<wmtk::Rational, 3, 1> barycentric_from_cache(
    const BarycentricPrecompute2D& bc,
    const Eigen::Matrix<wmtk::Rational, 2, 1>& p)
{
    Eigen::Matrix<wmtk::Rational, 2, 1> v2 = p - bc.a;
    Eigen::Matrix<wmtk::Rational, 2, 1> x = bc.Minv * v2; // [v; w]
    wmtk::Rational v = x(0);
    wmtk::Rational w = x(1);
    wmtk::Rational u = wmtk::Rational(1) - v - w;
    return Eigen::Matrix<wmtk::Rational, 3, 1>(u, v, w);
}

std::vector<BarycentricPrecompute2D> build_barycentric_cache_2d_from_double(
    const Eigen::MatrixXd& V2,
    const Eigen::MatrixXi& F)
{
    Eigen::MatrixX<wmtk::Rational> V2_r(V2.rows(), V2.cols());
    for (int i = 0; i < V2.rows(); ++i) {
        for (int j = 0; j < V2.cols(); ++j) V2_r(i, j) = wmtk::Rational(V2(i, j));
    }
    return build_barycentric_cache_2d(V2_r, F);
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
    // std::cout << "Handling EdgeCollapse" << std::endl;
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
                std::stringstream error_msg;
                error_msg
                    << "FATAL ERROR in handle_collapse_edge: Failed to find vertex ID mapping.\n";
                error_msg << "Query point vertex ID " << qp.fv_ids[0]
                          << " not found in F_after triangle.\n";
                error_msg << "F_after triangle vertices: ";
                for (int i = 0; i < 3; i++) {
                    error_msg << v_id_map_joint[F_after(local_index_in_f_after, i)];
                    if (i < 2) error_msg << ", ";
                }
                error_msg << "\nQuery point vertex IDs: ";
                for (int i = 0; i < 3; i++) {
                    error_msg << qp.fv_ids[i];
                    if (i < 2) error_msg << ", ";
                }
                error_msg << "\nThis indicates a serious numerical or topological error.";
                throw std::runtime_error(error_msg.str());
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
                std::stringstream error_msg;
                error_msg << "FATAL ERROR in handle_collapse_edge: Failed to update barycentric "
                             "coordinates.\n";
                error_msg
                    << "Query point could not be located in any triangle after mesh operation.\n";
                error_msg << "This indicates a serious numerical or topological error.";
                throw std::runtime_error(error_msg.str());
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
    std::vector<query_point>& query_points,
    const std::vector<BarycentricPrecompute2D>* barycentric_cache)
{
    struct SimpleTimer
    {
        std::chrono::high_resolution_clock::time_point t;
        void start() { t = std::chrono::high_resolution_clock::now(); }
        void stop() {}
        double getElapsedTime() const
        {
            return std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - t)
                .count();
        }
    };
    SimpleTimer total_timer;
    total_timer.start();
    // first convert the UV_joint to Rational
    Eigen::MatrixX<wmtk::Rational> UV_joint_r(UV_joint.rows(), UV_joint.cols());
    SimpleTimer convert_timer;
    convert_timer.start();
    for (int i = 0; i < UV_joint.rows(); i++) {
        for (int j = 0; j < UV_joint.cols(); j++) {
            UV_joint_r(i, j) = wmtk::Rational(UV_joint(i, j));
        }
    }
    double t_convert_ms = convert_timer.getElapsedTime() * 1000.0;

    // Precompute barycentric cache for F_before using UV_joint_r (2D) if not provided
    std::vector<BarycentricPrecompute2D> local_cache;
    const std::vector<BarycentricPrecompute2D>* bc_cache_ptr = barycentric_cache;
    if (bc_cache_ptr == nullptr) {
        std::cout << "???Building barycentric cache for F_before" << std::endl;
        local_cache = build_barycentric_cache_2d(UV_joint_r, F_before);
        bc_cache_ptr = &local_cache;
    }

    // Profiling accumulators
    double t_find_in_after_ms = 0.0;
    double t_find_offset_ms = 0.0;
    double t_compute_point_ms = 0.0;
    double t_bc_search_ms = 0.0;
    double t_bc_call_ms_total = 0.0;
    long long bc_call_count = 0;
    int num_processed_points = 0;

    for (int id = 0; id < query_points.size(); id++) {
        query_point& qp = query_points[id];
        if (qp.f_id < 0) continue;

        // find if qp is in the id_map_after
        SimpleTimer t_find_after;
        t_find_after.start();
        auto it = std::find(id_map_after.begin(), id_map_after.end(), qp.f_id);
        t_find_in_after_ms += t_find_after.getElapsedTime() * 1000.0;
        if (it != id_map_after.end()) {
            num_processed_points++;
            // std::cout << "find qp: " << qp.f_id << std::endl;
            // std::cout << "qp.bc: (" << qp.bc(0) << ", " << qp.bc(1) << ", " << qp.bc(2) << ")"
            //           << std::endl;
            // find the index of qp in id_map_after
            int local_index_in_f_after = std::distance(id_map_after.begin(), it);
            // offset of the qp.fv_ids
            int offset_in_f_after = -1;
            {
                SimpleTimer t_offset;
                t_offset.start();
                for (int i = 0; i < 3; i++) {
                    if (v_id_map_joint[F_after(local_index_in_f_after, i)] == qp.fv_ids[0]) {
                        offset_in_f_after = i;
                        break;
                    }
                }
                t_find_offset_ms += t_offset.getElapsedTime() * 1000.0;
            }
            if (offset_in_f_after == -1) {
                std::stringstream error_msg;
                error_msg
                    << "FATAL ERROR in handle_collapse_edge_r: Failed to find vertex ID mapping.\n";
                error_msg << "Query point vertex IDs: " << qp.fv_ids[0] << ", " << qp.fv_ids[1]
                          << ", " << qp.fv_ids[2] << "\n";
                error_msg << "Local index in F_after: " << local_index_in_f_after << "\n";
                error_msg << "This indicates a serious numerical or topological error in rational "
                             "computation.";
                throw std::runtime_error(error_msg.str());
            }

            // First convert barycentric coordinates to rational and normalize
            Eigen::Vector3<wmtk::Rational> bc_rational(
                wmtk::Rational(qp.bc(0)),
                wmtk::Rational(qp.bc(1)),
                wmtk::Rational(qp.bc(2)));
            bc_rational /= bc_rational.sum();
            // compute the location of the qp
            Eigen::Vector2<wmtk::Rational> p(0, 0);
            {
                SimpleTimer t_compute_p;
                t_compute_p.start();
                for (int i = 0; i < 3; i++) {
                    p = p +
                        UV_joint_r.row(F_after(local_index_in_f_after, (i + offset_in_f_after) % 3))
                                .transpose() *
                            bc_rational(i);
                }
                t_compute_point_ms += t_compute_p.getElapsedTime() * 1000.0;
            }


            // compute bc of the p in (V, F)_before
            int local_index_in_f_before = -1;

            // if computation is exact we don't need this anymore
            // double bc_min_coef = 1;
            bool bc_updated = false;
            {
                SimpleTimer t_bc_search;
                t_bc_search.start();
                for (int i = 0; i < F_before.rows(); i++) {
                    Eigen::Matrix<wmtk::Rational, 3, 1> bc_rational;

                    SimpleTimer t_bc_call;
                    t_bc_call.start();
                    bc_rational = barycentric_from_cache((*bc_cache_ptr)[i], p);
                    t_bc_call_ms_total += t_bc_call.getElapsedTime() * 1000.0;
                    bc_call_count++;


                    if (bc_rational.minCoeff() >= 0 && bc_rational.maxCoeff() <= 1) {
                        local_index_in_f_before = i;
                        qp.bc(0) = bc_rational(0).to_double();
                        qp.bc(1) = bc_rational(1).to_double();
                        qp.bc(2) = bc_rational(2).to_double();
                        bc_updated = true;
                        break;
                    }
                }
                t_bc_search_ms += t_bc_search.getElapsedTime() * 1000.0;
            }

            if (!bc_updated) {
                // INSERT_YOUR_CODE
                std::cout << "Input barycentric coordinates (bc): ";
                std::cout << std::setprecision(16) << qp.bc(0) << ", " << qp.bc(1) << ", "
                          << qp.bc(2) << std::endl;
                std::stringstream error_msg;
                error_msg << "FATAL ERROR in rational computation: Failed to update barycentric "
                             "coordinates.\n";
                error_msg
                    << "Query point could not be located in any triangle after mesh operation.\n";
                error_msg << "This indicates a serious numerical or topological error in exact "
                             "arithmetic.";
                throw std::runtime_error(error_msg.str());
            }

            // update qp
            qp.f_id = id_map_before[local_index_in_f_before];
            for (int i = 0; i < 3; i++) {
                qp.fv_ids[i] = v_id_map_joint[F_before(local_index_in_f_before, i)];
            }
        }
    }

    // Suppress detailed timing output to reduce noise
    // std::cout << "F_before.rows(): " << F_before.rows() << std::endl;
    // std::cout << "handle_collapse_edge_r barycentric call time: " << t_bc_call_ms_total << " ms"
    //           << std::endl;
    // std::cout << "handle_collapse_edge_r barycentric calls: " << bc_call_count << std::endl;
    // std::cout << "handle_collapse_edge_r barycentric search time: " << t_bc_search_ms << " ms"
    //           << std::endl;
}

void handle_non_collapse_operation(
    const Eigen::MatrixXd& V_before,
    const Eigen::MatrixXi& F_before,
    const std::vector<int64_t>& id_map_before,
    const std::vector<int64_t>& v_id_map_before,
    const Eigen::MatrixXd& V_after,
    const Eigen::MatrixXi& F_after,
    const std::vector<int64_t>& id_map_after,
    const std::vector<int64_t>& v_id_map_after,
    std::vector<query_point>& query_points,
    const std::string& operation_name,
    bool use_rational)
{
    if (use_rational) {
        handle_non_collapse_operation_r(
            V_before,
            F_before,
            id_map_before,
            v_id_map_before,
            V_after,
            F_after,
            id_map_after,
            v_id_map_after,
            query_points,
            operation_name);
        return;
    }
    // std::cout << "Handling " << operation_name << std::endl;
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
                std::stringstream error_msg;
                error_msg << "FATAL ERROR in " << operation_name
                          << ": Failed to find vertex ID mapping.\n";
                error_msg << "Query point face ID: " << qp.f_id << "\n";
                error_msg << "Expected vertex ID: " << qp.fv_ids[0] << "\n";
                error_msg << "This indicates a serious numerical or topological error.";
                throw std::runtime_error(error_msg.str());
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
                    throw std::runtime_error("FATAL ERROR: V_cols == 3 after local flattening");
                }
                if (-bc.minCoeff() < bc_min_coef) {
                    bc_min_coef = -bc.minCoeff();
                    local_index_in_f_before = i;
                    qp.bc = bc;
                    bc_updated = true;
                }
            }

            if (!bc_updated) {
                std::stringstream error_msg;
                error_msg << "FATAL ERROR in " << operation_name
                          << ": Failed to update barycentric coordinates.\n";
                error_msg
                    << "Query point could not be located in any triangle after mesh operation.\n";
                error_msg << "This indicates a serious numerical or topological error.";
                throw std::runtime_error(error_msg.str());
            }

            // update qp
            qp.f_id = id_map_before[local_index_in_f_before];
            for (int i = 0; i < 3; i++) {
                qp.fv_ids[i] = v_id_map_before[F_before(local_index_in_f_before, i)];
            }
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


// Rational version of handle_non_collapse_operation
void handle_non_collapse_operation_r(
    const Eigen::MatrixXd& V_before,
    const Eigen::MatrixXi& F_before,
    const std::vector<int64_t>& id_map_before,
    const std::vector<int64_t>& v_id_map_before,
    const Eigen::MatrixXd& V_after,
    const Eigen::MatrixXi& F_after,
    const std::vector<int64_t>& id_map_after,
    const std::vector<int64_t>& v_id_map_after,
    std::vector<query_point>& query_points,
    const std::string& operation_name,
    const std::vector<BarycentricPrecompute2D>* barycentric_cache)
{
    // std::cout << "Handling " << operation_name << " Rational" << std::endl;

    // Convert V_before and V_after to rational
    Eigen::MatrixX<wmtk::Rational> V_before_r(V_before.rows(), V_before.cols());
    Eigen::MatrixX<wmtk::Rational> V_after_r(V_after.rows(), V_after.cols());

    for (int i = 0; i < V_before.rows(); i++) {
        for (int j = 0; j < V_before.cols(); j++) {
            V_before_r(i, j) = wmtk::Rational(V_before(i, j));
        }
    }

    for (int i = 0; i < V_after.rows(); i++) {
        for (int j = 0; j < V_after.cols(); j++) {
            V_after_r(i, j) = wmtk::Rational(V_after(i, j));
        }
    }

    // Precompute barycentric cache for F_before (2D) if not provided
    std::vector<BarycentricPrecompute2D> local_cache;
    const std::vector<BarycentricPrecompute2D>* bc_cache_ptr = barycentric_cache;
    if (bc_cache_ptr == nullptr) {
        local_cache = build_barycentric_cache_2d(V_before_r, F_before);
        bc_cache_ptr = &local_cache;
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
                if (v_id_map_after[F_after(local_index_in_f_after, i)] == qp.fv_ids[0]) {
                    offset_in_f_after = i;
                    break;
                }
            }
            if (offset_in_f_after == -1) {
                std::stringstream error_msg;
                error_msg << "FATAL ERROR in rational " << operation_name
                          << ": Failed to find vertex ID mapping.\n";
                error_msg << "Query point face ID: " << qp.f_id << "\n";
                error_msg << "Expected vertex ID: " << qp.fv_ids[0] << "\n";
                error_msg << "This indicates a serious numerical or topological error in rational "
                             "computation.";
                throw std::runtime_error(error_msg.str());
            }

            // Convert barycentric coordinates to rational and normalize
            Eigen::Vector3<wmtk::Rational> bc_rational(
                wmtk::Rational(qp.bc(0)),
                wmtk::Rational(qp.bc(1)),
                wmtk::Rational(qp.bc(2)));
            bc_rational /= bc_rational.sum();

            // compute the location of the qp using rational arithmetic
            int V_cols = V_after.cols();
            Eigen::VectorX<wmtk::Rational> p = Eigen::VectorX<wmtk::Rational>::Zero(V_cols);
            for (int i = 0; i < 3; i++) {
                p = p + V_after_r.row(F_after(local_index_in_f_after, (i + offset_in_f_after) % 3))
                                .transpose() *
                            bc_rational(i);
            }

            // compute bc of the p in (V, F)_before using rational arithmetic
            int local_index_in_f_before = -1;
            bool bc_updated = false;
            for (int i = 0; i < F_before.rows(); i++) {
                Eigen::Matrix<wmtk::Rational, 3, 1> bc_rational_result;
                if (V_cols == 2) {
                    bc_rational_result = barycentric_from_cache((*bc_cache_ptr)[i], p.head<2>());
                } else {
                    throw std::runtime_error(
                        "FATAL ERROR: V_cols == 3 after local flattening in rational computation");
                }

                // Check if point is inside triangle (all barycentric coordinates >= 0)
                if (bc_rational_result.minCoeff() >= 0 && bc_rational_result.maxCoeff() <= 1) {
                    local_index_in_f_before = i;
                    qp.bc(0) = bc_rational_result(0).to_double();
                    qp.bc(1) = bc_rational_result(1).to_double();
                    qp.bc(2) = bc_rational_result(2).to_double();
                    bc_updated = true;
                    break;
                }
            }

            if (!bc_updated) {
                std::stringstream error_msg;
                error_msg << "FATAL ERROR in rational " << operation_name
                          << ": Failed to update barycentric coordinates.\n";
                error_msg
                    << "Query point could not be located in any triangle after mesh operation.\n";
                error_msg << "This indicates a serious numerical or topological error in exact "
                             "arithmetic.";
                throw std::runtime_error(error_msg.str());
            }

            // update qp
            qp.f_id = id_map_before[local_index_in_f_before];
            for (int i = 0; i < 3; i++) {
                qp.fv_ids[i] = v_id_map_before[F_before(local_index_in_f_before, i)];
            }

            // Normalize barycentric coordinates (they should already be normalized from rational
            // computation)
            qp.bc /= qp.bc.sum();
        }
    }
}

// Note: intersectSegmentEdge_r and other curve-related functions moved to
// track_operations_curve.cpp

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

void parse_consolidate_file(
    const json& operation_log,
    std::vector<int64_t>& face_ids_maps,
    std::vector<int64_t>& vertex_ids_maps)
{
    face_ids_maps = operation_log["new2old"][2].get<std::vector<int64_t>>();
    vertex_ids_maps = operation_log["new2old"][0].get<std::vector<int64_t>>();
}

void parse_non_collapse_file(
    const json& operation_log,
    bool& is_skipped,
    Eigen::MatrixXd& V_before,
    Eigen::MatrixXi& F_before,
    std::vector<int64_t>& id_map_before,
    std::vector<int64_t>& v_id_map_before,
    Eigen::MatrixXd& V_after,
    Eigen::MatrixXi& F_after,
    std::vector<int64_t>& id_map_after,
    std::vector<int64_t>& v_id_map_after)
{
    is_skipped = operation_log["is_skipped"].get<bool>();
    if (is_skipped) {
        return;
    }

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

// Template implementations

// Template version of handle_non_collapse_operation
template <typename CoordType>
void handle_non_collapse_operation_t(
    const Eigen::MatrixXd& V_before,
    const Eigen::MatrixXi& F_before,
    const std::vector<int64_t>& id_map_before,
    const std::vector<int64_t>& v_id_map_before,
    const Eigen::MatrixXd& V_after,
    const Eigen::MatrixXi& F_after,
    const std::vector<int64_t>& id_map_after,
    const std::vector<int64_t>& v_id_map_after,
    std::vector<query_point_t<CoordType>>& query_points,
    const std::string& operation_name,
    bool use_rational,
    const std::vector<BarycentricPrecompute2D>* barycentric_cache)
{
    if constexpr (std::is_same_v<CoordType, double>) {
        // For double query points, use use_rational to decide between regular and rational
        // processing
        if (use_rational) {
            handle_non_collapse_operation_r(
                V_before,
                F_before,
                id_map_before,
                v_id_map_before,
                V_after,
                F_after,
                id_map_after,
                v_id_map_after,
                query_points,
                operation_name,
                barycentric_cache);
        } else {
            handle_non_collapse_operation(
                V_before,
                F_before,
                id_map_before,
                v_id_map_before,
                V_after,
                F_after,
                id_map_after,
                v_id_map_after,
                query_points,
                operation_name,
                false);
        }
    } else if constexpr (std::is_same_v<CoordType, wmtk::Rational>) {
        // For rational query points, always use rational processing (ignore use_rational parameter)

        // Convert V_before and V_after to rational
        Eigen::MatrixX<wmtk::Rational> V_before_r(V_before.rows(), V_before.cols());
        Eigen::MatrixX<wmtk::Rational> V_after_r(V_after.rows(), V_after.cols());

        for (int i = 0; i < V_before.rows(); i++) {
            for (int j = 0; j < V_before.cols(); j++) {
                V_before_r(i, j) = wmtk::Rational(V_before(i, j));
            }
        }

        for (int i = 0; i < V_after.rows(); i++) {
            for (int j = 0; j < V_after.cols(); j++) {
                V_after_r(i, j) = wmtk::Rational(V_after(i, j));
            }
        }

        // Precompute barycentric cache for F_before (2D) if not provided
        std::vector<BarycentricPrecompute2D> local_cache;
        const std::vector<BarycentricPrecompute2D>* bc_cache_ptr = barycentric_cache;
        if (bc_cache_ptr == nullptr) {
            local_cache = build_barycentric_cache_2d(V_before_r, F_before);
            bc_cache_ptr = &local_cache;
        }

        for (int id = 0; id < query_points.size(); id++) {
            query_point_t<CoordType>& qp = query_points[id];
            if (qp.f_id < 0) continue;

            // find if qp is in the id_map_after
            auto it = std::find(id_map_after.begin(), id_map_after.end(), qp.f_id);
            if (it != id_map_after.end()) {
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
                    std::stringstream error_msg;
                    error_msg << "FATAL ERROR in rational template " << operation_name
                              << ": Failed to find vertex ID mapping.\n";
                    error_msg << "Query point face ID: " << qp.f_id << "\n";
                    error_msg << "Expected vertex ID: " << qp.fv_ids[0] << "\n";
                    error_msg
                        << "This indicates a serious numerical or topological error in rational "
                           "template computation.";
                    throw std::runtime_error(error_msg.str());
                }

                // Normalize barycentric coordinates
                wmtk::Rational bc_sum = qp.bc.sum();
                if (bc_sum != wmtk::Rational(0)) {
                    qp.bc /= bc_sum;
                }

                // compute the location of the qp using rational arithmetic
                int V_cols = V_after.cols();
                Eigen::VectorX<wmtk::Rational> p = Eigen::VectorX<wmtk::Rational>::Zero(V_cols);
                for (int i = 0; i < 3; i++) {
                    p = p +
                        V_after_r.row(F_after(local_index_in_f_after, (i + offset_in_f_after) % 3))
                                .transpose() *
                            qp.bc(i);
                }

                // compute bc of the p in (V, F)_before using rational arithmetic
                int local_index_in_f_before = -1;
                bool bc_updated = false;
                for (int i = 0; i < F_before.rows(); i++) {
                    Eigen::Matrix<wmtk::Rational, 3, 1> bc_rational_result;
                    if (V_cols == 2) {
                        bc_rational_result =
                            barycentric_from_cache((*bc_cache_ptr)[i], p.head<2>());
                    } else {
                        throw std::runtime_error("FATAL ERROR: V_cols == 3 after local flattening "
                                                 "in rational template computation");
                    }

                    // Check if point is inside triangle (all barycentric coordinates >= 0)
                    if (bc_rational_result.minCoeff() >= 0 && bc_rational_result.maxCoeff() <= 1) {
                        local_index_in_f_before = i;
                        qp.bc = bc_rational_result;
                        bc_updated = true;
                        break;
                    }
                }

                if (!bc_updated) {
                    std::stringstream error_msg;
                    error_msg << "FATAL ERROR in rational template " << operation_name
                              << ": Failed to update barycentric coordinates.\n";
                    error_msg << "Query point could not be located in any triangle after mesh "
                                 "operation.\n";
                    error_msg << "This indicates a serious numerical or topological error in exact "
                                 "template arithmetic.";
                    throw std::runtime_error(error_msg.str());
                }

                // update qp
                qp.f_id = id_map_before[local_index_in_f_before];
                for (int i = 0; i < 3; i++) {
                    qp.fv_ids[i] = v_id_map_before[F_before(local_index_in_f_before, i)];
                }

                // Normalize barycentric coordinates (they should already be normalized from
                // rational computation)
                wmtk::Rational bc_sum_final = qp.bc.sum();
                if (bc_sum_final != wmtk::Rational(0)) {
                    qp.bc /= bc_sum_final;
                }
            }
        }
    } else {
        static_assert(
            std::is_same_v<CoordType, double> || std::is_same_v<CoordType, wmtk::Rational>,
            "CoordType must be either double or wmtk::Rational");
    }
}

// Template version of handle_collapse_edge
template <typename CoordType>
void handle_collapse_edge_t(
    const Eigen::MatrixXd& UV_joint,
    const Eigen::MatrixXi& F_before,
    const Eigen::MatrixXi& F_after,
    const std::vector<int64_t>& v_id_map_joint,
    const std::vector<int64_t>& id_map_before,
    const std::vector<int64_t>& id_map_after,
    std::vector<query_point_t<CoordType>>& query_points,
    bool use_rational,
    const std::vector<BarycentricPrecompute2D>* barycentric_cache)
{
    if constexpr (std::is_same_v<CoordType, double>) {
        if (use_rational) {
            handle_collapse_edge_r(
                UV_joint,
                F_before,
                F_after,
                v_id_map_joint,
                id_map_before,
                id_map_after,
                query_points,
                barycentric_cache);
        } else {
            handle_collapse_edge(
                UV_joint,
                F_before,
                F_after,
                v_id_map_joint,
                id_map_before,
                id_map_after,
                query_points,
                false);
        }
    } else if constexpr (std::is_same_v<CoordType, wmtk::Rational>) {
        // For rational query points, always use rational processing (ignore use_rational parameter)

        // Convert UV_joint to rational
        Eigen::MatrixX<wmtk::Rational> UV_joint_r(UV_joint.rows(), UV_joint.cols());
        for (int i = 0; i < UV_joint.rows(); i++) {
            for (int j = 0; j < UV_joint.cols(); j++) {
                UV_joint_r(i, j) = wmtk::Rational(UV_joint(i, j));
            }
        }

        // Precompute barycentric cache for F_before (2D) if not provided
        std::vector<BarycentricPrecompute2D> local_cache;
        const std::vector<BarycentricPrecompute2D>* bc_cache_ptr = barycentric_cache;
        if (bc_cache_ptr == nullptr) {
            local_cache = build_barycentric_cache_2d(UV_joint_r, F_before);
            bc_cache_ptr = &local_cache;
        }

        for (int id = 0; id < query_points.size(); id++) {
            query_point_t<CoordType>& qp = query_points[id];
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
                    std::stringstream error_msg;
                    error_msg << "FATAL ERROR in rational template edge collapse"
                              << ": Failed to find vertex ID mapping.\n";
                    error_msg << "Query point face ID: " << qp.f_id << "\n";
                    error_msg << "Expected vertex ID: " << qp.fv_ids[0] << "\n";
                    error_msg
                        << "This indicates a serious numerical or topological error in rational "
                           "template edge collapse computation.";
                    throw std::runtime_error(error_msg.str());
                }

                // Normalize barycentric coordinates
                wmtk::Rational bc_sum = qp.bc.sum();
                if (bc_sum != wmtk::Rational(0)) {
                    qp.bc /= bc_sum;
                }

                // compute the location of the qp using rational arithmetic
                Eigen::Vector2<wmtk::Rational> p = Eigen::Vector2<wmtk::Rational>::Zero();
                for (int i = 0; i < 3; i++) {
                    p = p +
                        UV_joint_r.row(F_after(local_index_in_f_after, (i + offset_in_f_after) % 3))
                                .head<2>()
                                .transpose() *
                            qp.bc(i);
                }

                // compute bc of the p in F_before using rational arithmetic
                int local_index_in_f_before = -1;
                bool bc_updated = false;

                for (int i = 0; i < F_before.rows(); i++) {
                    Eigen::Matrix<wmtk::Rational, 3, 1> bc_rational_result =
                        barycentric_from_cache((*bc_cache_ptr)[i], p);

                    // Check if point is inside triangle (all barycentric coordinates >= 0)
                    // TODO: this could take time
                    if (bc_rational_result.minCoeff() >= 0 && bc_rational_result.maxCoeff() <= 1) {
                        local_index_in_f_before = i;
                        qp.bc = bc_rational_result;
                        bc_updated = true;
                        break;
                    }
                }

                if (!bc_updated) {
                    std::stringstream error_msg;
                    error_msg << "FATAL ERROR in rational template edge collapse"
                              << ": Failed to update barycentric coordinates.\n";
                    error_msg << "Query point could not be located in any triangle after edge "
                                 "collapse.\n";
                    error_msg << "This indicates a serious numerical or topological error in exact "
                                 "template edge collapse arithmetic.";
                    throw std::runtime_error(error_msg.str());
                }

                // update qp
                qp.f_id = id_map_before[local_index_in_f_before];
                for (int i = 0; i < 3; i++) {
                    qp.fv_ids[i] = v_id_map_joint[F_before(local_index_in_f_before, i)];
                }

                // Normalize barycentric coordinates
                wmtk::Rational bc_sum_final = qp.bc.sum();
                if (bc_sum_final != wmtk::Rational(0)) {
                    qp.bc /= bc_sum_final;
                }
            }
        }
    } else {
        static_assert(
            std::is_same_v<CoordType, double> || std::is_same_v<CoordType, wmtk::Rational>,
            "CoordType must be either double or wmtk::Rational");
    }
}

// Explicit template instantiations
template void handle_non_collapse_operation_t<double>(
    const Eigen::MatrixXd& V_before,
    const Eigen::MatrixXi& F_before,
    const std::vector<int64_t>& id_map_before,
    const std::vector<int64_t>& v_id_map_before,
    const Eigen::MatrixXd& V_after,
    const Eigen::MatrixXi& F_after,
    const std::vector<int64_t>& id_map_after,
    const std::vector<int64_t>& v_id_map_after,
    std::vector<query_point_t<double>>& query_points,
    const std::string& operation_name,
    bool use_rational,
    const std::vector<BarycentricPrecompute2D>* barycentric_cache);

template void handle_non_collapse_operation_t<wmtk::Rational>(
    const Eigen::MatrixXd& V_before,
    const Eigen::MatrixXi& F_before,
    const std::vector<int64_t>& id_map_before,
    const std::vector<int64_t>& v_id_map_before,
    const Eigen::MatrixXd& V_after,
    const Eigen::MatrixXi& F_after,
    const std::vector<int64_t>& id_map_after,
    const std::vector<int64_t>& v_id_map_after,
    std::vector<query_point_t<wmtk::Rational>>& query_points,
    const std::string& operation_name,
    bool use_rational,
    const std::vector<BarycentricPrecompute2D>* barycentric_cache);

template void handle_collapse_edge_t<double>(
    const Eigen::MatrixXd& UV_joint,
    const Eigen::MatrixXi& F_before,
    const Eigen::MatrixXi& F_after,
    const std::vector<int64_t>& v_id_map_joint,
    const std::vector<int64_t>& id_map_before,
    const std::vector<int64_t>& id_map_after,
    std::vector<query_point_t<double>>& query_points,
    bool use_rational,
    const std::vector<BarycentricPrecompute2D>* barycentric_cache);

template void handle_collapse_edge_t<wmtk::Rational>(
    const Eigen::MatrixXd& UV_joint,
    const Eigen::MatrixXi& F_before,
    const Eigen::MatrixXi& F_after,
    const std::vector<int64_t>& v_id_map_joint,
    const std::vector<int64_t>& id_map_before,
    const std::vector<int64_t>& id_map_after,
    std::vector<query_point_t<wmtk::Rational>>& query_points,
    bool use_rational,
    const std::vector<BarycentricPrecompute2D>* barycentric_cache);
