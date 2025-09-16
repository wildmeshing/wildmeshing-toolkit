
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


// function that computes all the intersections between two curves
template <typename CoordType>
int compute_intersections_between_two_curves_t(
    const query_curve_t<CoordType>& curve1,
    const query_curve_t<CoordType>& curve2,
    bool verbose)
{
    int intersections = 0;

    std::set<std::pair<int, int>> skip_seg_pairs;

    for (int seg1_id = 0; seg1_id < curve1.segments.size(); seg1_id++) {
        for (int seg2_id = 0; seg2_id < curve2.segments.size(); seg2_id++) {
            auto seg1 = curve1.segments[seg1_id];
            auto seg2 = curve2.segments[seg2_id];

            if (skip_seg_pairs.find(std::make_pair(seg1_id, seg2_id)) != skip_seg_pairs.end()) {
                continue;
            }

            if (seg1.f_id != seg2.f_id) {
                continue;
            }

            Eigen::Vector2<wmtk::Rational> s1_a, s1_b, s2_a, s2_b;
            s1_a << wmtk::Rational(seg1.bcs[0](0)), wmtk::Rational(seg1.bcs[0](1));
            s1_b << wmtk::Rational(seg1.bcs[1](0)), wmtk::Rational(seg1.bcs[1](1));
            s2_a << wmtk::Rational(seg2.bcs[0](0)), wmtk::Rational(seg2.bcs[0](1));
            s2_b << wmtk::Rational(seg2.bcs[1](0)), wmtk::Rational(seg2.bcs[1](1));

            Eigen::Vector2<wmtk::Rational> bc_on_seg2;
            Eigen::Vector2<wmtk::Rational> bc_on_seg1;
            if (intersectSegmentEdge_r(s1_a, s1_b, s2_a, s2_b, bc_on_seg2, bc_on_seg1, false)) {
                // handle intersection on endpoints
                if (bc_on_seg2(0) == wmtk::Rational(0) || bc_on_seg2(0) == wmtk::Rational(1) ||
                    bc_on_seg1(0) == wmtk::Rational(0) || bc_on_seg1(0) == wmtk::Rational(1)) {
                    int seg1_nei = -1;
                    int seg2_nei = -1;
                    if (bc_on_seg2(0) == wmtk::Rational(1)) {
                        seg2_nei = std::find(
                                       curve2.next_segment_ids.begin(),
                                       curve2.next_segment_ids.end(),
                                       seg2_id) -
                                   curve2.next_segment_ids.begin();
                    } else if (bc_on_seg2(0) == wmtk::Rational(0)) {
                        seg2_nei = curve2.next_segment_ids[seg2_id];
                    }


                    if (bc_on_seg1(0) == wmtk::Rational(1)) {
                        seg1_nei = std::find(
                                       curve1.next_segment_ids.begin(),
                                       curve1.next_segment_ids.end(),
                                       seg1_id) -
                                   curve1.next_segment_ids.begin();
                    } else if (bc_on_seg1(0) == wmtk::Rational(0)) {
                        seg1_nei = curve1.next_segment_ids[seg1_id];
                    }
                    if (seg2_nei != -1) {
                        skip_seg_pairs.insert(std::make_pair(seg1_id, seg2_nei));
                        std::cout << "add pair: " << seg1_id << ", " << seg2_nei << std::endl;
                    }
                    if (seg1_nei != -1) {
                        skip_seg_pairs.insert(std::make_pair(seg1_nei, seg2_id));
                        skip_seg_pairs.insert(std::make_pair(seg1_nei, seg2_nei));
                        std::cout << "add pair: " << seg1_nei << ", " << seg2_nei << std::endl;
                        std::cout << "add pair: " << seg1_nei << ", " << seg2_id << std::endl;
                    }
                }

                intersections++;
                if (verbose) {
                    if constexpr (std::is_same_v<CoordType, double>) {
                        // std::cout << "Intersection found in face " << seg1.f_id << std::endl;
                        std::cout << "seg1: f_id=" << seg1.f_id
                                  << ", bcs[0]=" << seg1.bcs[0].transpose()
                                  << ", bcs[1]=" << seg1.bcs[1].transpose() << std::endl;
                        std::cout << "seg2: f_id=" << seg2.f_id
                                  << ", bcs[0]=" << seg2.bcs[0].transpose()
                                  << ", bcs[1]=" << seg2.bcs[1].transpose() << std::endl;
                        std::cout << "intersection position at t = " << bc_on_seg2(0).to_double()
                                  << std::endl;
                    } else if constexpr (std::is_same_v<CoordType, wmtk::Rational>) {
                        std::cout << "seg1_id: " << seg1_id << ", seg2_id: " << seg2_id
                                  << std::endl;
                        std::cout << "seg1: f_id=" << seg1.f_id << ", bcs[0]=[";
                        for (int k = 0; k < seg1.bcs[0].size(); k++) {
                            std::cout << seg1.bcs[0](k).to_double();
                            if (k < seg1.bcs[0].size() - 1) std::cout << ", ";
                        }
                        std::cout << "], bcs[1]=[";
                        for (int k = 0; k < seg1.bcs[1].size(); k++) {
                            std::cout << seg1.bcs[1](k).to_double();
                            if (k < seg1.bcs[1].size() - 1) std::cout << ", ";
                        }
                        std::cout << "]" << std::endl;
                        std::cout << "seg2: f_id=" << seg2.f_id << ", bcs[0]=[";
                        for (int k = 0; k < seg2.bcs[0].size(); k++) {
                            std::cout << seg2.bcs[0](k).to_double();
                            if (k < seg2.bcs[0].size() - 1) std::cout << ", ";
                        }
                        std::cout << "], bcs[1]=[";
                        for (int k = 0; k < seg2.bcs[1].size(); k++) {
                            std::cout << seg2.bcs[1](k).to_double();
                            if (k < seg2.bcs[1].size() - 1) std::cout << ", ";
                        }
                        std::cout << "]" << std::endl;
                        std::cout << "intersection position at t = " << bc_on_seg2(0).to_double()
                                  << std::endl;
                        std::cout << std::endl;
                    }
                }
            }
        }
    }
    return intersections;
}
int compute_intersections_between_two_curves(
    const query_curve& curve1,
    const query_curve& curve2,
    bool verbose)
{
    return compute_intersections_between_two_curves_t(curve1, curve2, verbose);
}

// Template version of compute_curve_self_intersections
template <typename CoordType>
int compute_curve_self_intersections_t(const query_curve_t<CoordType>& curve, bool verbose)
{
    int intersections = 0;

    for (int i = 0; i < curve.segments.size(); i++) {
        for (int j = i + 1; j < curve.segments.size(); j++) {
            if (curve.next_segment_ids[i] == j || curve.next_segment_ids[j] == i) {
                continue;
            }

            const auto& seg1 = curve.segments[i];
            const auto& seg2 = curve.segments[j];
            if (seg1.f_id != seg2.f_id) {
                continue;
            }

            Eigen::Vector2<wmtk::Rational> s1_a, s1_b, s2_a, s2_b;
            s1_a << wmtk::Rational(seg1.bcs[0](0)), wmtk::Rational(seg1.bcs[0](1));
            s1_b << wmtk::Rational(seg1.bcs[1](0)), wmtk::Rational(seg1.bcs[1](1));
            s2_a << wmtk::Rational(seg2.bcs[0](0)), wmtk::Rational(seg2.bcs[0](1));
            s2_b << wmtk::Rational(seg2.bcs[1](0)), wmtk::Rational(seg2.bcs[1](1));

            Eigen::Vector2<wmtk::Rational> bc_tmp, bc_tmp_seg1;
            if (intersectSegmentEdge_r(s1_a, s1_b, s2_a, s2_b, bc_tmp, bc_tmp_seg1, false)) {
                // intersectSegmentEdge_r(s1_a, s1_b, s2_a, s2_b, bc_tmp, bc_tmp_seg1, true);
                intersections++;
                if (verbose) {
                    if constexpr (std::is_same_v<CoordType, double>) {
                        std::cout << "i=" << i << ", j=" << j << std::endl;
                        std::cout << "seg1: f_id=" << seg1.f_id
                                  << ", bcs[0]=" << seg1.bcs[0].transpose()
                                  << ", bcs[1]=" << seg1.bcs[1].transpose() << std::endl;
                        std::cout << "seg1: origin_segment_id=" << seg1.origin_segment_id
                                  << std::endl;
                        std::cout << "seg1: next_segment_id=" << curve.next_segment_ids[i]
                                  << std::endl;

                        if (curve.next_segment_ids[i] != -1) {
                            const auto& next_seg1 = curve.segments[curve.next_segment_ids[i]];
                            std::cout << "seg1 next: f_id=" << next_seg1.f_id << ", bcs[0]=[";
                            for (int k = 0; k < next_seg1.bcs[0].size(); k++) {
                                std::cout << next_seg1.bcs[0](k);
                                if (k < next_seg1.bcs[0].size() - 1) std::cout << ", ";
                            }
                            std::cout << "], bcs[1]=[";
                            for (int k = 0; k < next_seg1.bcs[1].size(); k++) {
                                std::cout << next_seg1.bcs[1](k);
                                if (k < next_seg1.bcs[1].size() - 1) std::cout << ", ";
                            }
                            std::cout << "]" << std::endl;
                            std::cout
                                << "seg1 next: origin_segment_id=" << next_seg1.origin_segment_id
                                << std::endl;
                        }
                        std::cout << "seg2: f_id=" << seg2.f_id << ", bcs[0]=[";
                        for (int k = 0; k < seg2.bcs[0].size(); k++) {
                            std::cout << seg2.bcs[0](k);
                            if (k < seg2.bcs[0].size() - 1) std::cout << ", ";
                        }
                        std::cout << "], bcs[1]=[";
                        for (int k = 0; k < seg2.bcs[1].size(); k++) {
                            std::cout << seg2.bcs[1](k);
                            if (k < seg2.bcs[1].size() - 1) std::cout << ", ";
                        }
                        std::cout << "]" << std::endl;
                        std::cout << "seg2: origin_segment_id=" << seg2.origin_segment_id
                                  << std::endl;
                        std::cout << "seg2: next_segment_id=" << curve.next_segment_ids[j]
                                  << std::endl;

                        if (curve.next_segment_ids[j] != -1) {
                            const auto& next_seg2 = curve.segments[curve.next_segment_ids[j]];
                            std::cout << "seg2 next: f_id=" << next_seg2.f_id << ", bcs[0]=[";
                            for (int k = 0; k < next_seg2.bcs[0].size(); k++) {
                                std::cout << next_seg2.bcs[0](k);
                                if (k < next_seg2.bcs[0].size() - 1) std::cout << ", ";
                            }
                            std::cout << "], bcs[1]=[";
                            for (int k = 0; k < next_seg2.bcs[1].size(); k++) {
                                std::cout << next_seg2.bcs[1](k);
                                if (k < next_seg2.bcs[1].size() - 1) std::cout << ", ";
                            }
                            std::cout << "]" << std::endl;
                            std::cout
                                << "seg2 next: origin_segment_id=" << next_seg2.origin_segment_id
                                << std::endl;
                        }

                        std::cout << "intersection position at t = " << bc_tmp(0).to_double()
                                  << std::endl;
                    } else if constexpr (std::is_same_v<CoordType, wmtk::Rational>) {
                        std::cout << "i=" << i << ", j=" << j << std::endl;
                        std::cout << "seg1: f_id=" << seg1.f_id << ", bcs[0]=[";
                        for (int k = 0; k < seg1.bcs[0].size(); k++) {
                            std::cout << seg1.bcs[0](k).to_double();
                            if (k < seg1.bcs[0].size() - 1) std::cout << ", ";
                        }
                        std::cout << "], bcs[1]=[";
                        for (int k = 0; k < seg1.bcs[1].size(); k++) {
                            std::cout << seg1.bcs[1](k).to_double();
                            if (k < seg1.bcs[1].size() - 1) std::cout << ", ";
                        }
                        std::cout << "]" << std::endl;
                        std::cout << "seg1: origin_segment_id=" << seg1.origin_segment_id
                                  << std::endl;
                        std::cout << "seg1: next_segment_id=" << curve.next_segment_ids[i]
                                  << std::endl;

                        if (curve.next_segment_ids[i] != -1) {
                            const auto& next_seg1 = curve.segments[curve.next_segment_ids[i]];
                            std::cout << "seg1 next: f_id=" << next_seg1.f_id << ", bcs[0]=[";
                            for (int k = 0; k < next_seg1.bcs[0].size(); k++) {
                                std::cout << next_seg1.bcs[0](k).to_double();
                                if (k < next_seg1.bcs[0].size() - 1) std::cout << ", ";
                            }
                            std::cout << "], bcs[1]=[";
                            for (int k = 0; k < next_seg1.bcs[1].size(); k++) {
                                std::cout << next_seg1.bcs[1](k).to_double();
                                if (k < next_seg1.bcs[1].size() - 1) std::cout << ", ";
                            }
                            std::cout << "]" << std::endl;
                            std::cout
                                << "seg1 next: origin_segment_id=" << next_seg1.origin_segment_id
                                << std::endl;
                        }
                        std::cout << "seg2: f_id=" << seg2.f_id << ", bcs[0]=[";
                        for (int k = 0; k < seg2.bcs[0].size(); k++) {
                            std::cout << seg2.bcs[0](k).to_double();
                            if (k < seg2.bcs[0].size() - 1) std::cout << ", ";
                        }
                        std::cout << "], bcs[1]=[";
                        for (int k = 0; k < seg2.bcs[1].size(); k++) {
                            std::cout << seg2.bcs[1](k).to_double();
                            if (k < seg2.bcs[1].size() - 1) std::cout << ", ";
                        }
                        std::cout << "]" << std::endl;
                        std::cout << "seg2: origin_segment_id=" << seg2.origin_segment_id
                                  << std::endl;
                        std::cout << "seg2: next_segment_id=" << curve.next_segment_ids[j]
                                  << std::endl;

                        if (curve.next_segment_ids[j] != -1) {
                            const auto& next_seg2 = curve.segments[curve.next_segment_ids[j]];
                            std::cout << "seg2 next: f_id=" << next_seg2.f_id << ", bcs[0]=[";
                            for (int k = 0; k < next_seg2.bcs[0].size(); k++) {
                                std::cout << next_seg2.bcs[0](k).to_double();
                                if (k < next_seg2.bcs[0].size() - 1) std::cout << ", ";
                            }
                            std::cout << "], bcs[1]=[";
                            for (int k = 0; k < next_seg2.bcs[1].size(); k++) {
                                std::cout << next_seg2.bcs[1](k).to_double();
                                if (k < next_seg2.bcs[1].size() - 1) std::cout << ", ";
                            }
                            std::cout << "]" << std::endl;
                            std::cout
                                << "seg2 next: origin_segment_id=" << next_seg2.origin_segment_id
                                << std::endl;
                        }

                        std::cout << "intersection position at t = " << bc_tmp(0).to_double()
                                  << std::endl;
                    }
                }
            }
        }
    }

    return intersections;
}
int compute_curve_self_intersections(const query_curve& curve, bool verbose)
{
    return compute_curve_self_intersections_t(curve, verbose);
}

// Explicit template instantiation for compilation
template bool is_curve_valid_t<double>(const query_curve_t<double>& curve);
template bool is_curve_valid_t<wmtk::Rational>(const query_curve_t<wmtk::Rational>& curve);

template void clean_up_curve_t<double>(query_curve_t<double>& curve);
template void clean_up_curve_t<wmtk::Rational>(query_curve_t<wmtk::Rational>& curve);

template int compute_intersections_between_two_curves_t<double>(
    const query_curve_t<double>& curve1,
    const query_curve_t<double>& curve2,
    bool verbose);
template int compute_intersections_between_two_curves_t<wmtk::Rational>(
    const query_curve_t<wmtk::Rational>& curve1,
    const query_curve_t<wmtk::Rational>& curve2,
    bool verbose);

template int compute_curve_self_intersections_t<double>(
    const query_curve_t<double>& curve,
    bool verbose);
template int compute_curve_self_intersections_t<wmtk::Rational>(
    const query_curve_t<wmtk::Rational>& curve,
    bool verbose);
