
#include "track_operations_curve.hpp"
#include <igl/Timer.h>
#include <igl/doublearea.h>
#include <cstddef>
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
                std::cout << "v_id_map_before[F_before]:" << std::endl;
                for (int row = 0; row < F_before.rows(); ++row) {
                    for (int col = 0; col < F_before.cols(); ++col) {
                        int fid_entry = F_before(row, col);
                        int vid = v_id_map_before[fid_entry];
                        std::cout << vid << " ";
                    }
                    std::cout << std::endl;
                }
                throw std::runtime_error("Error: map_local_boundary_qps: "
                                         "not a boundary edgepoint");
            }
        }
    }
}


// Group segments by triangle f_id
// Returns a map from f_id to list of (curve_id, seg_id) pairs
std::map<int64_t, std::vector<std::pair<int, int>>> group_segments_by_triangle(
    const std::vector<std::vector<std::vector<int>>>& all_curve_parts_after_mapping,
    const std::vector<query_curve_t<wmtk::Rational>>& curves)
{
    std::map<int64_t, std::vector<std::pair<int, int>>> triangle_to_segments;

    // Iterate through all curves
    for (int curve_id = 0; curve_id < all_curve_parts_after_mapping.size(); curve_id++) {
        // Iterate through all parts of this curve
        for (int part_id = 0; part_id < all_curve_parts_after_mapping[curve_id].size(); part_id++) {
            const auto& curve_part = all_curve_parts_after_mapping[curve_id][part_id];

            // Iterate through all segments in this part
            for (int i = 0; i < curve_part.size(); i++) {
                int seg_id = curve_part[i];

                // Get the triangle f_id for this segment
                int64_t f_id = curves[curve_id].segments[seg_id].f_id;

                // Add (curve_id, seg_id) pair to the list for this triangle
                triangle_to_segments[f_id].push_back(std::make_pair(curve_id, seg_id));
            }
        }
    }

    return triangle_to_segments;
}

// Rounding segments to double
void rounding_segments_to_double(
    const std::vector<std::vector<std::vector<int>>>& all_curve_parts_after_mapping,
    std::vector<query_curve_t<wmtk::Rational>>& curves,
    bool do_check_intersection)
{
    // Control verbose output
    const bool verbose = false;

    auto triangle_to_segments = group_segments_by_triangle(all_curve_parts_after_mapping, curves);
    // Helper struct to store intersection with position
    struct IntersectionInfo
    {
        int curve_id;
        int seg_id;
        wmtk::Rational t;
        int from_seg; // 0 for seg1, 1 for seg2

        bool operator<(const IntersectionInfo& other) const
        {
            if (from_seg != other.from_seg) return from_seg < other.from_seg;
            return t < other.t;
        }
    };

    // Helper lambda to get combined sorted intersections for two segments
    auto get_combined_intersections = [&](int cid, int sid1, int sid2) -> std::vector<int> {
        std::vector<IntersectionInfo> all_intersections;

        for (int idx = 0; idx < 2; idx++) {
            int sid = (idx == 0) ? sid1 : sid2;
            const auto& seg = curves[cid].segments[sid];
            int64_t f_id = seg.f_id;

            if (triangle_to_segments.count(f_id) == 0) continue;

            const auto& segs_in_tri = triangle_to_segments[f_id];
            for (const auto& [other_cid, other_sid] : segs_in_tri) {
                if (other_cid == cid && (other_sid == sid1 || other_sid == sid2)) continue;

                // Skip if other_sid is adjacent to sid1 or sid2
                if (other_cid == cid) {
                    // Check if other_sid is the next segment of sid
                    if (curves[cid].next_segment_ids[sid] == other_sid) continue;
                    // Check if sid is the next segment of other_sid
                    if (curves[cid].next_segment_ids[other_sid] == sid) continue;
                }

                const auto& other_seg = curves[other_cid].segments[other_sid];
                auto inter_info = seg_seg_intersect_with_params_rational(seg, other_seg);

                if (inter_info.intersects) {
                    all_intersections.push_back({other_cid, other_sid, inter_info.t, idx});
                }
            }
        }

        // Sort by from_seg first, then by parameter t
        std::sort(all_intersections.begin(), all_intersections.end());

        // Extract curve_ids in order
        std::vector<int> result;
        for (const auto& inter : all_intersections) {
            result.push_back(inter.curve_id);
        }
        return result;
    };

    if (do_check_intersection) {
        for (int curve_id = 0; curve_id < all_curve_parts_after_mapping.size(); curve_id++) {
            for (int part_id = 0; part_id < all_curve_parts_after_mapping[curve_id].size();
                 part_id++) {
                const auto& curve_part = all_curve_parts_after_mapping[curve_id][part_id];
                for (int i = 0; i < curve_part.size() - 1; i++) {
                    int seg_id = curve_part[i];
                    int next_seg_id = curve_part[i + 1];

                    // Get combined intersections BEFORE rounding
                    auto inter_before = get_combined_intersections(curve_id, seg_id, next_seg_id);

                    if (verbose) {
                        std::cout << "[DEBUG] Curve " << curve_id << " Part " << part_id
                                  << " Seg pair [" << seg_id << "," << next_seg_id << "]"
                                  << std::endl;
                        std::cout << "  BEFORE rounding - intersections found: "
                                  << inter_before.size() << std::endl;
                        if (!inter_before.empty()) {
                            std::cout << "  BEFORE intersection curve_ids: [";
                            for (int k = 0; k < inter_before.size(); k++) {
                                std::cout << inter_before[k];
                                if (k + 1 < inter_before.size()) std::cout << ", ";
                            }
                            std::cout << "]" << std::endl;
                        }
                    }

                    // Backup original coordinates
                    auto backup_seg_bc1 = curves[curve_id].segments[seg_id].bcs[1];
                    auto backup_next_bc0 = curves[curve_id].segments[next_seg_id].bcs[0];

                    if (verbose) {
                        std::cout << "  Original seg[" << seg_id << "].bcs[1]: ["
                                  << backup_seg_bc1(0).to_double() << ", "
                                  << backup_seg_bc1(1).to_double() << ", "
                                  << backup_seg_bc1(2).to_double() << "]" << std::endl;
                        std::cout << "  Original seg[" << next_seg_id << "].bcs[0]: ["
                                  << backup_next_bc0(0).to_double() << ", "
                                  << backup_next_bc0(1).to_double() << ", "
                                  << backup_next_bc0(2).to_double() << "]" << std::endl;
                    }

                    // Apply rounding
                    for (int j = 0; j < 3; j++) {
                        curves[curve_id].segments[seg_id].bcs[1](j) =
                            wmtk::Rational(curves[curve_id].segments[seg_id].bcs[1](j).to_double());
                        curves[curve_id].segments[next_seg_id].bcs[0](j) = wmtk::Rational(
                            curves[curve_id].segments[next_seg_id].bcs[0](j).to_double());
                    }

                    if (verbose) {
                        std::cout << "  Rounded seg[" << seg_id << "].bcs[1]: ["
                                  << curves[curve_id].segments[seg_id].bcs[1](0).to_double() << ", "
                                  << curves[curve_id].segments[seg_id].bcs[1](1).to_double() << ", "
                                  << curves[curve_id].segments[seg_id].bcs[1](2).to_double() << "]"
                                  << std::endl;
                        std::cout << "  Rounded seg[" << next_seg_id << "].bcs[0]: ["
                                  << curves[curve_id].segments[next_seg_id].bcs[0](0).to_double()
                                  << ", "
                                  << curves[curve_id].segments[next_seg_id].bcs[0](1).to_double()
                                  << ", "
                                  << curves[curve_id].segments[next_seg_id].bcs[0](2).to_double()
                                  << "]" << std::endl;
                    }

                    // Get combined intersections AFTER rounding
                    auto inter_after = get_combined_intersections(curve_id, seg_id, next_seg_id);

                    if (verbose) {
                        std::cout << "  AFTER rounding - intersections found: "
                                  << inter_after.size() << std::endl;
                        if (!inter_after.empty()) {
                            std::cout << "  AFTER intersection curve_ids: [";
                            for (int k = 0; k < inter_after.size(); k++) {
                                std::cout << inter_after[k];
                                if (k + 1 < inter_after.size()) std::cout << ", ";
                            }
                            std::cout << "]" << std::endl;
                        }
                    }

                    // Rollback if intersection order changed
                    if (inter_before != inter_after) {
                        if (verbose) {
                            std::cout << "  *** ROLLBACK: Intersection order changed! ***"
                                      << std::endl;
                        }
                        curves[curve_id].segments[seg_id].bcs[1] = backup_seg_bc1;
                        curves[curve_id].segments[next_seg_id].bcs[0] = backup_next_bc0;
                    } else if (verbose && (!inter_before.empty() || !inter_after.empty())) {
                        std::cout << "  OK: Intersection order preserved" << std::endl;
                    }
                    if (verbose) {
                        std::cout << std::endl;
                    }
                }
            }
        }
    } else {
        for (int curve_id = 0; curve_id < all_curve_parts_after_mapping.size(); curve_id++) {
            for (int part_id = 0; part_id < all_curve_parts_after_mapping[curve_id].size();
                 part_id++) {
                const auto& curve_part = all_curve_parts_after_mapping[curve_id][part_id];
                for (int i = 0; i < curve_part.size() - 1; i++) {
                    int seg_id = curve_part[i];
                    int next_seg_id = curve_part[i + 1];

                    // Apply rounding
                    for (int j = 0; j < 3; j++) {
                        curves[curve_id].segments[seg_id].bcs[1](j) =
                            wmtk::Rational(curves[curve_id].segments[seg_id].bcs[1](j).to_double());
                        curves[curve_id].segments[next_seg_id].bcs[0](j) = wmtk::Rational(
                            curves[curve_id].segments[next_seg_id].bcs[0](j).to_double());
                    }
                }
            }
        }
    }
}


void merge_segments(
    const std::vector<std::vector<std::vector<int>>>& all_curve_parts_after_mapping,
    std::vector<query_curve_t<wmtk::Rational>>& curves)
{
    // Control verbose output
    const bool verbose = false;
    std::vector<std::set<int>> removed_segments(curves.size());
    std::size_t merge_attempt_count = 0;
    std::size_t merge_step5_fail_count = 0;

    // Iterate through all curves
    for (int curve_id = 0; curve_id < curves.size(); curve_id++) {
        for (int part_id = 0; part_id < all_curve_parts_after_mapping[curve_id].size(); part_id++) {
            const auto& curve_part = all_curve_parts_after_mapping[curve_id][part_id];

            for (int i = 0; i < curve_part.size(); i++) {
                int seg_id = curve_part[i];
                if (removed_segments[curve_id].count(seg_id) > 0) {
                    continue;
                }

                while (true) {
                    int next_idx = i + 1;
                    while (next_idx < curve_part.size() &&
                           removed_segments[curve_id].count(curve_part[next_idx]) > 0) {
                        next_idx++;
                    }
                    if (next_idx >= curve_part.size()) {
                        break;
                    }

                    int next_seg_id = curve_part[next_idx];
                    if (removed_segments[curve_id].count(next_seg_id) > 0) {
                        break;
                    }

                    auto& seg = curves[curve_id].segments[seg_id];
                    auto& next_seg = curves[curve_id].segments[next_seg_id];

                    // Step 1: Check if they belong to different faces
                    if (seg.f_id != next_seg.f_id) {
                        break;
                    }

                    // Step 2: Get combined intersections for the two separate segments
                    std::vector<int> segment_ids = {seg_id, next_seg_id};
                    std::vector<query_segment_r> original_segments = {seg, next_seg};
                    auto inter_before = get_intersections_seq(
                        curve_id,
                        segment_ids,
                        original_segments,
                        all_curve_parts_after_mapping,
                        removed_segments,
                        curves);

                    if (verbose) {
                        std::cout << "[DEBUG] Merge attempt - Curve " << curve_id << " Part "
                                  << part_id << " Seg pair [" << seg_id << "," << next_seg_id << "]"
                                  << std::endl;
                        std::cout << "  BEFORE merge - intersections:" << std::endl;
                        for (const auto& intersection : inter_before) {
                            std::cout << "    " << intersection << std::endl;
                        }
                    }

                    // Step 3: Create merged segment
                    query_segment_r merged_seg;
                    merged_seg.f_id = seg.f_id;
                    merged_seg.origin_segment_id = seg.origin_segment_id;
                    merged_seg.bcs[0] = seg.bcs[0]; // seg's bc[0]
                    merged_seg.bcs[1] = next_seg.bcs[1]; // next_seg's bc[1]
                    merged_seg.fv_ids = seg.fv_ids;

                    // Step 4: Get intersection sequence for the merged segment
                    // (computing intersections without seg and next_seg)
                    std::vector<query_segment_r> merged_segment_vec = {merged_seg};
                    auto inter_after = get_intersections_seq(
                        curve_id,
                        segment_ids,
                        merged_segment_vec,
                        all_curve_parts_after_mapping,
                        removed_segments,
                        curves);

                    if (verbose) {
                        std::cout << "  AFTER merge - intersections:" << std::endl;
                        for (const auto& intersection : inter_after) {
                            std::cout << "    " << intersection << std::endl;
                        }
                    }

                    // Step 5: Check if intersection sequences are the same
                    merge_attempt_count++;
                    if (inter_before.size() != inter_after.size()) {
                        merge_step5_fail_count++;
                        if (verbose) {
                            std::cout << std::endl;
                        }
                        break;
                    }

                    bool same = true;
                    for (size_t k = 0; k < inter_before.size(); ++k) {
                        if (inter_before[k].other_curve_id != inter_after[k].other_curve_id) {
                            same = false;
                            break;
                        }
                    }

                    if (!same) {
                        merge_step5_fail_count++;
                        if (verbose) {
                            std::cout << "  Cannot merge: Intersection sequences differ"
                                      << std::endl;
                            std::cout << std::endl;
                        }
                        break;
                    }

                    if (verbose) {
                        std::cout << "  *** MERGE SUCCESS: Intersection sequences match ***"
                                  << std::endl;
                    }

                    // Update the segment at seg_id with merged segment
                    curves[curve_id].segments[seg_id] = merged_seg;

                    // Update next_segment_id: seg now points to what next_seg was pointing to
                    curves[curve_id].next_segment_ids[seg_id] =
                        curves[curve_id].next_segment_ids[next_seg_id];

                    // Mark next_seg for removal
                    removed_segments[curve_id].insert(next_seg_id);

                    // Redirect any segments that were pointing to next_seg_id
                    for (int j = 0; j < curves[curve_id].next_segment_ids.size(); j++) {
                        if (j != seg_id && curves[curve_id].next_segment_ids[j] == next_seg_id) {
                            curves[curve_id].next_segment_ids[j] = seg_id;
                        }
                    }

                    if (verbose) {
                        std::cout << std::endl;
                    }
                }
            }

            if (verbose) {
                std::cout << "Curve " << curve_id << ": marked "
                          << removed_segments[curve_id].size()
                          << " segments for removal after merging" << std::endl;
            }
        }
    }
    // Final pass: Actually remove the marked segments
    if (verbose) {
        std::cout << "\n=== Final cleanup pass: removing marked segments ===" << std::endl;
    }
    for (int curve_id = 0; curve_id < curves.size(); curve_id++) {
        if (removed_segments[curve_id].empty()) continue;

        if (verbose) {
            std::cout << "Curve " << curve_id << ": Removing " << removed_segments[curve_id].size()
                      << " segments" << std::endl;
        }

        // Create mapping from old indices to new indices
        std::vector<int> old_to_new(curves[curve_id].segments.size(), -1);
        int new_idx = 0;
        for (int old_idx = 0; old_idx < curves[curve_id].segments.size(); old_idx++) {
            if (removed_segments[curve_id].count(old_idx) == 0) {
                old_to_new[old_idx] = new_idx;
                new_idx++;
            }
        }

        // Create new segments vector without removed segments
        std::vector<query_segment_r> new_segments;
        std::vector<int> new_next_segment_ids;
        for (int old_idx = 0; old_idx < curves[curve_id].segments.size(); old_idx++) {
            if (removed_segments[curve_id].count(old_idx) == 0) {
                new_segments.push_back(curves[curve_id].segments[old_idx]);
                // Remap next_segment_id
                int old_next = curves[curve_id].next_segment_ids[old_idx];
                int new_next = (old_next == -1) ? -1 : old_to_new[old_next];
                new_next_segment_ids.push_back(new_next);
            }
        }

        // Replace with new vectors
        curves[curve_id].segments = std::move(new_segments);
        curves[curve_id].next_segment_ids = std::move(new_next_segment_ids);

        if (verbose) {
            std::cout << "  New segment count: " << curves[curve_id].segments.size() << std::endl;
        }
    }
    if (verbose) {
        std::cout << "=== Cleanup pass complete ===" << std::endl;
    }

    std::cout << "[merge_segments] attempts reaching step5: " << merge_attempt_count
              << ", rejected due to mismatch: " << merge_step5_fail_count;
    if (merge_attempt_count > 0) {
        double failure_ratio =
            static_cast<double>(merge_step5_fail_count) / static_cast<double>(merge_attempt_count);
        std::cout << " (ratio " << failure_ratio << ")";
    }
    std::cout << std::endl;
}
