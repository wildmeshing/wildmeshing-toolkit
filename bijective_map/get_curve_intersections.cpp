#include <set>
#include "track_operations_curve.hpp"

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