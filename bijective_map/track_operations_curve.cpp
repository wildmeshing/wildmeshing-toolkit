
#include "track_operations_curve.hpp"
#include <igl/Timer.h>
#include <igl/doublearea.h>
#include <set>
#ifdef USE_IGL_VIEWER
#include <igl/opengl/glfw/Viewer.h>
#endif

// Template version of clean_up_curve
template <typename CoordType>
void clean_up_curve_t(query_curve_t<CoordType>& curve)
{
    // TODO: make this function work for loops
    if (curve.segments.empty()) {
        return;
    }

    std::vector<query_segment_t<CoordType>> new_segments;
    std::vector<int> new_next_segment_ids;
    std::vector<int> old_to_new_mapping(curve.segments.size(), -1);


    // Process segments in chain order starting from 0
    // int current_id = 0;
    // while (current_id != -1 && current_id < curve.segments.size()) {
    for (int current_id = 0; current_id < curve.segments.size(); current_id++) {
        if (old_to_new_mapping[current_id] != -1) {
            // This segment was already processed, skip
            // current_id = curve.next_segment_ids[current_id];
            continue;
        }

        query_segment_t<CoordType>& current_segment = curve.segments[current_id];

        // Convert to rational arithmetic for exact collinearity check (only first 2 components)
        Eigen::Vector2<wmtk::Rational> start_bc_r(
            wmtk::Rational(current_segment.bcs[0](0)),
            wmtk::Rational(current_segment.bcs[0](1)));

        Eigen::Vector2<wmtk::Rational> current_slope_r =
            Eigen::Vector2<wmtk::Rational>(
                wmtk::Rational(current_segment.bcs[1](0)),
                wmtk::Rational(current_segment.bcs[1](1))) -
            start_bc_r;

        // Find consecutive segments that can be merged
        std::vector<int> segments_to_merge = {current_id};
        int next_id = curve.next_segment_ids[current_id];

        while (next_id != -1 && next_id < curve.segments.size()) {
            if (next_id == current_id) {
                break; // loop detected
            }
            query_segment_t<CoordType>& next_segment = curve.segments[next_id];

            // Check if they can be merged (same origin_segment_id and f_id)
            if (current_segment.origin_segment_id != next_segment.origin_segment_id ||
                current_segment.f_id != next_segment.f_id) {
                break; // Different segments or faces, cannot merge
            }

            // Convert next segment to rational arithmetic (only first 2 components)
            Eigen::Vector2<wmtk::Rational> next_slope_r =
                Eigen::Vector2<wmtk::Rational>(
                    wmtk::Rational(next_segment.bcs[1](0)),
                    wmtk::Rational(next_segment.bcs[1](1))) -
                start_bc_r;

            // Check collinearity using cross product in 2D (exact test)
            // Two 2D vectors are collinear if their cross product is zero
            wmtk::Rational cross_product_2d =
                current_slope_r(0) * next_slope_r(1) - current_slope_r(1) * next_slope_r(0);


            bool is_collinear = (cross_product_2d == wmtk::Rational(0));


            // handle zero slope case
            if (current_slope_r.norm() == 0) {
                current_slope_r = next_slope_r;
            }

            if (!is_collinear) {
                // std::cout << "Collinearity check failed for segment " << current_id << " and "
                //           << next_id << std::endl;
                // std::cout << "  current_slope_r: [" << current_slope_r(0).to_double() << ", "
                //           << current_slope_r(1).to_double() << "]" << std::endl;
                // std::cout << "  next_slope_r: [" << next_slope_r(0).to_double() << ", "
                //           << next_slope_r(1).to_double() << "]" << std::endl;
                // std::cout << "  cross_product_2d: " << std::fixed << std::setprecision(16)
                //           << cross_product_2d.to_double() << std::endl;

                break; // not collinear, stop merging
            }

            segments_to_merge.push_back(next_id);
            next_id = curve.next_segment_ids[next_id];
        }

        // for debug
        {
            if (segments_to_merge.size() > 1) {
                std::cout << "Merging segments: ";
                for (int idx : segments_to_merge) {
                    const auto& seg = curve.segments[idx];
                    std::cout << "\n  seg id: " << idx << ", f_id: " << seg.f_id
                              << ", origin_segment_id: " << seg.origin_segment_id << ", bcs[0]: [";
                    if constexpr (std::is_same_v<CoordType, double>) {
                        std::cout << std::fixed << std::setprecision(16) << seg.bcs[0].transpose();
                    } else if constexpr (std::is_same_v<CoordType, wmtk::Rational>) {
                        std::cout << std::fixed << std::setprecision(16)
                                  << seg.bcs[0](0).to_double() << ", " << seg.bcs[0](1).to_double()
                                  << ", " << seg.bcs[0](2).to_double();
                    }
                    std::cout << "]" << ", bcs[1]: [";
                    if constexpr (std::is_same_v<CoordType, double>) {
                        std::cout << std::fixed << std::setprecision(16) << seg.bcs[1].transpose();
                    } else if constexpr (std::is_same_v<CoordType, wmtk::Rational>) {
                        std::cout << std::fixed << std::setprecision(16)
                                  << seg.bcs[1](0).to_double() << ", " << seg.bcs[1](1).to_double()
                                  << ", " << seg.bcs[1](2).to_double();
                    }
                    std::cout << "]" << ", fv_ids: [";
                    for (int k = 0; k < seg.fv_ids.size(); ++k) {
                        std::cout << seg.fv_ids[k];
                        if (k + 1 < seg.fv_ids.size()) std::cout << ", ";
                    }
                    std::cout << "]";
                }
                std::cout << std::endl;
            }
        }
        // Create merged segment
        query_segment_t<CoordType> merged_segment;
        merged_segment.f_id = current_segment.f_id;
        merged_segment.origin_segment_id = current_segment.origin_segment_id;
        merged_segment.bcs[0] = current_segment.bcs[0]; // First segment's bc0
        merged_segment.bcs[1] =
            curve.segments[segments_to_merge.back()].bcs[1]; // Last segment's bc1
        merged_segment.fv_ids = current_segment.fv_ids;

        // Add to new segments
        int new_segment_id = new_segments.size();
        new_segments.push_back(merged_segment);

        // Map all merged segments to the new segment
        for (int old_id : segments_to_merge) {
            old_to_new_mapping[old_id] = new_segment_id;
        }

        // Set next segment ID for the merged segment (this will be updated later)
        int final_next = curve.next_segment_ids[segments_to_merge.back()];
        new_next_segment_ids.push_back(final_next);

        // Move to the next unprocessed segment
        // current_id = final_next;
    }

    // Update next_segment_ids to point to new indices
    for (int i = 0; i < new_next_segment_ids.size(); i++) {
        if (new_next_segment_ids[i] != -1) {
            int old_next = new_next_segment_ids[i];
            if (old_next < old_to_new_mapping.size() && old_to_new_mapping[old_next] != -1) {
                new_next_segment_ids[i] = old_to_new_mapping[old_next];
            } else {
                // If the old_next was not mapped, it means it was merged into another segment
                // We need to find which segment it was merged into
                std::cout << "Warning: old_next " << old_next << " was not mapped, setting to -1"
                          << std::endl;
                new_next_segment_ids[i] = -1;
            }
        }
    }
    std::cout << "clean up curve, segments size: " << curve.segments.size() << " -> "
              << new_segments.size() << std::endl;
    // Replace the curve's segments and next_segment_ids
    curve.segments = std::move(new_segments);
    curve.next_segment_ids = std::move(new_next_segment_ids);
}
// Template version of is_curve_valid
template <typename CoordType>
bool is_curve_valid_t(const query_curve_t<CoordType>& curve)
{
    // TODO: make this function work for loops
    if (curve.segments.empty()) {
        std::cout << "Warning:curve is empty" << std::endl;
        return true;
    }

    // int cur_seg_id = 0;
    bool is_valid = true;
    // while (cur_seg_id != -1 && cur_seg_id < curve.segments.size()) {
    for (int cur_seg_id = 0; cur_seg_id < curve.segments.size(); cur_seg_id++) {
        int next_seg_id = curve.next_segment_ids[cur_seg_id];
        // std::cout << "cur_seg: " << cur_seg_id << " next_seg: " << next_seg_id << std::endl;
        if (next_seg_id == -1) {
            continue;
        }
        const auto& cur_seg = curve.segments[cur_seg_id];
        const auto& next_seg = curve.segments[next_seg_id];

        // std::cout << "cur_seg.f_id: " << cur_seg.f_id << " next_seg.f_id: " << next_seg.f_id
        //           << std::endl;
        if (cur_seg.f_id == next_seg.f_id) {
            auto bc_diff = next_seg.bcs[0] - cur_seg.bcs[1];
            bool is_invalid = false;

            if constexpr (std::is_same_v<CoordType, double>) {
                is_invalid = (bc_diff.norm() > 1e-8);
            } else if constexpr (std::is_same_v<CoordType, wmtk::Rational>) {
                // For rational, check if all components are exactly zero
                is_invalid =
                    (bc_diff(0) != wmtk::Rational(0) || bc_diff(1) != wmtk::Rational(0) ||
                     bc_diff(2) != wmtk::Rational(0));
            }

            if (is_invalid) {
                std::cout << "Error: bc_diff is too large" << std::endl;
                if constexpr (std::is_same_v<CoordType, double>) {
                    std::cout << "cur_seg.bcs[1]: " << cur_seg.bcs[1].transpose() << std::endl;
                    std::cout << "next_seg.bcs[0]: " << next_seg.bcs[0].transpose() << std::endl;
                    std::cout << "bc_diff: " << bc_diff.transpose() << std::endl;
                } else if constexpr (std::is_same_v<CoordType, wmtk::Rational>) {
                    std::cout << "cur_seg.bcs[1]: [";
                    for (int k = 0; k < cur_seg.bcs[1].size(); k++) {
                        std::cout << cur_seg.bcs[1](k).to_double();
                        if (k < cur_seg.bcs[1].size() - 1) std::cout << ", ";
                    }
                    std::cout << "]" << std::endl;
                    std::cout << "next_seg.bcs[0]: [";
                    for (int k = 0; k < next_seg.bcs[0].size(); k++) {
                        std::cout << next_seg.bcs[0](k).to_double();
                        if (k < next_seg.bcs[0].size() - 1) std::cout << ", ";
                    }
                    std::cout << "]" << std::endl;
                    std::cout << "bc_diff: [";
                    for (int k = 0; k < bc_diff.size(); k++) {
                        std::cout << bc_diff(k).to_double();
                        if (k < bc_diff.size() - 1) std::cout << ", ";
                    }
                    std::cout << "]" << std::endl;
                }
                is_valid = false;
                return is_valid;
            }
        } else {
            for (int i = 0; i < 3; i++) {
                bool is_nonzero;
                if constexpr (std::is_same_v<CoordType, double>) {
                    is_nonzero = (cur_seg.bcs[1](i) != 0.0);
                } else if constexpr (std::is_same_v<CoordType, wmtk::Rational>) {
                    is_nonzero = (cur_seg.bcs[1](i) != wmtk::Rational(0));
                }

                if (is_nonzero) {
                    int vid = cur_seg.fv_ids[i];

                    // find vid in next_seg.fv_ids
                    auto it = std::find(next_seg.fv_ids.begin(), next_seg.fv_ids.end(), vid);
                    if (it == next_seg.fv_ids.end()) {
                        std::cout << "Error: vid not found in next_seg.fv_ids" << std::endl;
                        std::cout << "cur_seg.f_id: " << cur_seg.f_id << std::endl;
                        std::cout << "cur_seg.fv_ids: [";
                        for (int k = 0; k < cur_seg.fv_ids.size(); k++) {
                            std::cout << cur_seg.fv_ids[k];
                            if (k < cur_seg.fv_ids.size() - 1) std::cout << ", ";
                        }
                        std::cout << "]" << std::endl;
                        if constexpr (std::is_same_v<CoordType, double>) {
                            std::cout << "cur_seg.bcs[1]: " << cur_seg.bcs[1].transpose()
                                      << std::endl;
                        } else if constexpr (std::is_same_v<CoordType, wmtk::Rational>) {
                            std::cout << "cur_seg.bcs[1]: [";
                            for (int k = 0; k < cur_seg.bcs[1].size(); k++) {
                                std::cout << cur_seg.bcs[1](k).to_double();
                                if (k < cur_seg.bcs[1].size() - 1) std::cout << ", ";
                            }
                            std::cout << "]" << std::endl;
                        }
                        std::cout << "next_seg.f_id: " << next_seg.f_id << std::endl;
                        std::cout << "next_seg.fv_ids: [";
                        for (int k = 0; k < next_seg.fv_ids.size(); k++) {
                            std::cout << next_seg.fv_ids[k];
                            if (k < next_seg.fv_ids.size() - 1) std::cout << ", ";
                        }
                        std::cout << "]" << std::endl;
                        if constexpr (std::is_same_v<CoordType, double>) {
                            std::cout << "next_seg.bcs[0]: " << next_seg.bcs[0].transpose()
                                      << std::endl;
                        } else if constexpr (std::is_same_v<CoordType, wmtk::Rational>) {
                            std::cout << "next_seg.bcs[0]: [";
                            for (int k = 0; k < next_seg.bcs[0].size(); k++) {
                                std::cout << next_seg.bcs[0](k).to_double();
                                if (k < next_seg.bcs[0].size() - 1) std::cout << ", ";
                            }
                            std::cout << "]" << std::endl;
                        }
                        is_valid = false;
                    } else {
                        int next_seg_vid_id = std::distance(next_seg.fv_ids.begin(), it);
                        auto diff = next_seg.bcs[0](next_seg_vid_id) - cur_seg.bcs[1](i);
                        bool is_diff_too_large = false;

                        if constexpr (std::is_same_v<CoordType, double>) {
                            is_diff_too_large = (abs(diff) > 1e-8);
                        } else if constexpr (std::is_same_v<CoordType, wmtk::Rational>) {
                            // For rational, check if exactly zero
                            is_diff_too_large = (diff != wmtk::Rational(0));
                        }

                        if (is_diff_too_large) {
                            std::cout << "Error: bc_diff is too large" << std::endl;

                            std::cout << "cur_seg.fv_ids: [";
                            for (int k = 0; k < cur_seg.fv_ids.size(); k++) {
                                std::cout << cur_seg.fv_ids[k];
                                if (k < cur_seg.fv_ids.size() - 1) std::cout << ", ";
                            }
                            std::cout << "]" << std::endl;
                            if constexpr (std::is_same_v<CoordType, double>) {
                                std::cout << "diff:" << diff << std::endl;
                                std::cout << "cur_seg.bcs[1]: " << cur_seg.bcs[1].transpose()
                                          << std::endl;
                            } else if constexpr (std::is_same_v<CoordType, wmtk::Rational>) {
                                std::cout << "diff:" << diff.to_double() << std::endl;
                                std::cout << "cur_seg.bcs[1]: [";
                                for (int k = 0; k < cur_seg.bcs[1].size(); k++) {
                                    std::cout << cur_seg.bcs[1](k).to_double();
                                    if (k < cur_seg.bcs[1].size() - 1) std::cout << ", ";
                                }
                                std::cout << "]" << std::endl;
                            }
                            std::cout << "next_seg.fv_ids: [";
                            for (int k = 0; k < next_seg.fv_ids.size(); k++) {
                                std::cout << next_seg.fv_ids[k];
                                if (k < next_seg.fv_ids.size() - 1) std::cout << ", ";
                            }
                            std::cout << "]" << std::endl;
                            if constexpr (std::is_same_v<CoordType, double>) {
                                std::cout << "next_seg.bcs[0]: " << next_seg.bcs[0].transpose()
                                          << std::endl;
                            } else if constexpr (std::is_same_v<CoordType, wmtk::Rational>) {
                                std::cout << "next_seg.bcs[0]: [";
                                for (int k = 0; k < next_seg.bcs[0].size(); k++) {
                                    std::cout << next_seg.bcs[0](k).to_double();
                                    if (k < next_seg.bcs[0].size() - 1) std::cout << ", ";
                                }
                                std::cout << "]" << std::endl;
                            }
                            is_valid = false;
                        }
                    }
                }
            }
        }
        // cur_seg_id = next_seg_id;
    }
    return is_valid;
}


// Explicit template instantiation for compilation
template bool is_curve_valid_t<double>(const query_curve_t<double>& curve);
template bool is_curve_valid_t<wmtk::Rational>(const query_curve_t<wmtk::Rational>& curve);

template void clean_up_curve_t<double>(query_curve_t<double>& curve);
template void clean_up_curve_t<wmtk::Rational>(query_curve_t<wmtk::Rational>& curve);


////////////////////////////////////////////////////////////
// Utils functions for all rational curves version
////////////////////////////////////////////////////////////
void get_all_query_points_for_one_curve_rational(
    const std::vector<int64_t>& id_map_after,
    const query_curve_t<wmtk::Rational>& curve,
    std::vector<query_point_r>& all_query_points,
    std::vector<int>& all_query_seg_ids,
    std::vector<int>& bc0_places)
{
    int cur_id = 0;
    const int start_id = 0;
    bool pre_segment_included = false;

    int cnt = 0;
    while (cur_id != -1 && cur_id < curve.segments.size()) {
        cnt++;
        query_segment_r qs = curve.segments[cur_id];

        auto it = std::find(id_map_after.begin(), id_map_after.end(), qs.f_id);
        if (it == id_map_after.end()) { // if current segment is not in the local patch
            cur_id = curve.next_segment_ids[cur_id];
            pre_segment_included = false;
            if (cur_id == start_id) {
                break;
            }
            continue;
        }

        // current segment is in the local patch
        all_query_seg_ids.push_back(cur_id);
        bc0_places.push_back(all_query_points.size() - 1);
        if (!pre_segment_included) {
            // in this case, we need add bc0 of the current segment
            bc0_places.back()++;
            all_query_points.push_back(query_point_r{qs.f_id, qs.bcs[0], qs.fv_ids});
        }
        all_query_points.push_back(query_point_r{qs.f_id, qs.bcs[1], qs.fv_ids});
        pre_segment_included = true;
        cur_id = curve.next_segment_ids[cur_id];

        if (cur_id == start_id) { // meet the loop
            break;
        }

        if (cnt > curve.segments.size()) {
            throw std::runtime_error(
                "Error: get_all_query_points_for_one_curve_rational: meet the partial loop");
        }
    } // end while
}

void classify_boundary_and_interior_query_points(
    const std::vector<query_point_r>& all_query_points,
    const std::vector<int>& all_query_seg_ids,
    const std::vector<int>& bc0_places,
    const query_curve_t<wmtk::Rational>& curve,
    std::vector<query_point_r>& non_bd_qps,
    std::vector<int>& non_bd_qps_ids,
    std::vector<int>& bd_qps_ids,
    std::vector<std::vector<int>>& all_curve_parts)
{
    // check first point and last point
    if (curve.next_segment_ids[all_query_seg_ids.back()] != all_query_seg_ids.front()) {
        // this means the first point and the last point are not connected
        // check 0s explicitly
        const auto& first_qp = all_query_points.front();
        if (first_qp.bc(0) == 0 || first_qp.bc(1) == 0 || first_qp.bc(2) == 0) {
            bd_qps_ids.push_back(0);
        } else {
            non_bd_qps.push_back(first_qp);
            non_bd_qps_ids.push_back(0);
        }
        const auto& last_qp = all_query_points.back();
        if (last_qp.bc(0) == 0 || last_qp.bc(1) == 0 || last_qp.bc(2) == 0) {
            bd_qps_ids.push_back(all_query_points.size() - 1);
        } else {
            non_bd_qps.push_back(last_qp);
            non_bd_qps_ids.push_back(all_query_points.size() - 1);
        }
    } else {
        non_bd_qps.push_back(all_query_points[0]);
        non_bd_qps_ids.push_back(0);
        non_bd_qps.push_back(all_query_points.back());
        non_bd_qps_ids.push_back(all_query_points.size() - 1);
    }

    // get every other qps that are on the boundary of the local patch
    for (int i = 1; i < all_query_seg_ids.size(); i++) {
        if (bc0_places[i] != bc0_places[i - 1] + 1) {
            // it indicates a break point in the curve by the local patch's boundary
            bd_qps_ids.push_back(bc0_places[i]); // current bc0 place
            bd_qps_ids.push_back(bc0_places[i] - 1); // previous bc1 place
        } else {
            non_bd_qps.push_back(all_query_points[bc0_places[i]]); // current bc0 place(also is
                                                                   // previous bc1 place)
            non_bd_qps_ids.push_back(bc0_places[i]);
        }
    }

    // Generate all_curve_parts: segments of curve between boundary points
    all_curve_parts.clear();

    // Check if it's a closed loop
    bool is_closed_loop =
        (curve.next_segment_ids[all_query_seg_ids.back()] == all_query_seg_ids.front());

    // case 1 whole loop is in this local patch
    if (bd_qps_ids.empty()) {
        // No boundary points - the entire curve is one part
        all_curve_parts.push_back(all_query_seg_ids);
    } else {
        std::vector<int> current_part;
        for (int i = 0; i < all_query_seg_ids.size(); i++) {
            current_part.push_back(all_query_seg_ids[i]);
            if (i < all_query_seg_ids.size() - 1 && bc0_places[i] != bc0_places[i + 1] - 1) {
                all_curve_parts.push_back(current_part);
                current_part.clear();
            }
        }
        if (current_part.size() > 0) {
            all_curve_parts.push_back(current_part);
        }

        if (is_closed_loop) {
            // For closed loops, we need to merge the last and first parts
            // if they are both connected to the boundary
            if (all_curve_parts.size() > 1) {
                // Merge last part into first part
                std::vector<int> merged_part = all_curve_parts.back();
                merged_part.insert(
                    merged_part.end(),
                    all_curve_parts.front().begin(),
                    all_curve_parts.front().end());

                // Replace first part with merged part and remove last part
                all_curve_parts[0] = merged_part;
                all_curve_parts.pop_back();
            }
        }
    }

    // DEBUG:
    bool debug_output = false;
    if (debug_output) {
        std::cout << "bd_qps_ids: ";
        for (int id : bd_qps_ids) {
            std::cout << id << " ";
        }
        std::cout << std::endl;

        std::cout << "non_bd_qps_ids: ";
        for (int id : non_bd_qps_ids) {
            std::cout << id << " ";
        }
        std::cout << std::endl;

        std::cout << "all_curve_parts (" << all_curve_parts.size() << " parts):" << std::endl;
        for (int i = 0; i < all_curve_parts.size(); i++) {
            std::cout << "  part " << i << ": ";
            for (int seg_id : all_curve_parts[i]) {
                std::cout << seg_id << " ";
            }
            std::cout << std::endl;
        }

        if (bd_qps_ids.size() + non_bd_qps_ids.size() != all_query_points.size()) {
            throw std::runtime_error("Error: map_all_query_points_rational: the number of "
                                     "boundary and non-boundary query points is not correct");
        }
    }
}

void map_local_boundary_qps(
    const Eigen::MatrixXi& F_before,
    const std::vector<int64_t>& v_id_map_before,
    const std::vector<int64_t>& id_map_before,
    std::vector<query_point_r>& all_query_points,
    const std::vector<int>& bd_qps_ids)
{
    // 2 cases: point case and edge case
    for (int id : bd_qps_ids) {
        auto& qp = all_query_points[id];

        // case1: check for point case
        {
            bool bc_updated = false;
            for (int j = 0; j < 3; j++) {
                if (qp.bc(j) == 1) {
                    // this means the qp in landed on a boundary vertex
                    int v0 = qp.fv_ids(j);
                    // look for v0 in v_id_map_before(F_before)

                    for (int f_id = 0; f_id < F_before.rows(); f_id++) {
                        for (int k = 0; k < 3; k++) {
                            if (v_id_map_before[F_before(f_id, k)] == v0) {
                                qp.bc(k) = 1;
                                qp.bc((k + 1) % 3) = 0;
                                qp.bc((k + 2) % 3) = 0;
                                qp.f_id = id_map_before[f_id];
                                qp.fv_ids << v_id_map_before[F_before(f_id, 0)],
                                    v_id_map_before[F_before(f_id, 1)],
                                    v_id_map_before[F_before(f_id, 2)];
                                bc_updated = true;
                                break;
                            }
                        } // end for k
                        if (bc_updated) {
                            break;
                        }
                    } // end for f_id

                    if (!bc_updated) {
                        std::cout << "qp: " << qp << std::endl;
                        throw std::runtime_error("Error: map_local_boundary_qps: "
                                                 "boundary point can't map");
                    }
                }
            } // end for j

            if (bc_updated) {
                continue; // next boundary qp
            }
        }
        // case2: check for edge case
        {
            int v0 = -1, v1 = -1;
            int v0_index = -1;

            for (int j = 0; j < 3; j++) {
                if (qp.bc(j) == 0) {
                    v0 = qp.fv_ids((j + 1) % 3);
                    v1 = qp.fv_ids((j + 2) % 3);
                    v0_index = (j + 1) % 3;
                    break;
                }
            }

            if (v0 == -1) {
                throw std::runtime_error("Error: map_local_boundary_qps: not a boundary point");
            }

            // first handle edge cases
            bool bc_updated = false;

            for (int f_id = 0; f_id < F_before.rows(); f_id++) {
                for (int j = 0; j < 3; j++) {
                    if (v_id_map_before[F_before(f_id, j)] == v0 &&
                        v_id_map_before[F_before(f_id, (j + 1) % 3)] == v1) {
                        auto qp_bc_copy = qp.bc;
                        qp.bc(j) = qp_bc_copy(v0_index);
                        qp.bc((j + 1) % 3) = qp_bc_copy((v0_index + 1) % 3);
                        qp.bc((j + 2) % 3) = 0;
                        qp.f_id = id_map_before[f_id];
                        qp.fv_ids << v_id_map_before[F_before(f_id, 0)],
                            v_id_map_before[F_before(f_id, 1)], v_id_map_before[F_before(f_id, 2)];
                        bc_updated = true;
                        break;
                    }
                }
                if (bc_updated) {
                    break;
                }
            }

            if (!bc_updated) {
                std::cout << "qp: " << qp << std::endl;
                throw std::runtime_error("Error: map_local_boundary_qps: "
                                         "not a boundary edgepoint");
            }
        }
    }
}


// Rounding segments to double
void rounding_segments_to_double(
    const std::vector<std::vector<std::vector<int>>>& all_curve_parts_after_mapping,
    std::vector<query_curve_t<wmtk::Rational>>& curves)
{
    for (int curve_id = 0; curve_id < all_curve_parts_after_mapping[curve_id].size(); curve_id++) {
        for (int part_id = 0; part_id < all_curve_parts_after_mapping[curve_id].size(); part_id++) {
            const auto& curve_part = all_curve_parts_after_mapping[curve_id][part_id];
            for (int i = 0; i < curve_part.size() - 1; i++) {
                int seg_id = curve_part[i];
                int next_seg_id = curve_part[i + 1];
                for (int j = 0; j < 3; j++) {
                    curves[curve_id].segments[seg_id].bcs[1](j) =
                        wmtk::Rational(curves[curve_id].segments[seg_id].bcs[1](j).to_double());
                    curves[curve_id].segments[next_seg_id].bcs[0](j) = wmtk::Rational(
                        curves[curve_id].segments[next_seg_id].bcs[0](j).to_double());
                }

                // TODO: Check if the segment intersects with other segments
            }
        }
    }
}
