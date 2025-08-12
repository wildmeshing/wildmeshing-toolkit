#include "generate_plane_curves.hpp"
#include <algorithm>
#include <iostream>
#include <map>
#include <queue>

std::map<std::pair<int, int>, EdgeInfo> extractMeshEdges(const MatrixXi& F)
{
    std::map<std::pair<int, int>, EdgeInfo> edges;

    // Process each triangle
    for (int fid = 0; fid < F.rows(); ++fid) {
        // Process each edge of the triangle
        for (int i = 0; i < 3; ++i) {
            int v0 = F(fid, i);
            int v1 = F(fid, (i + 1) % 3);

            // Create directed edge (v0 -> v1)
            std::pair<int, int> edge_key = {v0, v1};
            std::pair<int, int> reverse_edge_key = {v1, v0};

            if (edges.find(edge_key) != edges.end()) {
                std::cerr << "Warning: Directed edge (" << v0 << "," << v1 << ") appears twice!"
                          << std::endl;
                throw std::runtime_error(
                    "Directed edge (" + std::to_string(v0) + "," + std::to_string(v1) +
                    ") appears twice!");
            }

            if (edges.find(reverse_edge_key) != edges.end()) {
                edges[reverse_edge_key].fr = fid;
            } else {
                EdgeInfo info;
                info.v0 = v0;
                info.v1 = v1;
                info.fl = fid;
                info.fr = -1;
                edges[edge_key] = info;
            }
        }
    }

    std::cout << "Extracted " << edges.size() << " directed edges from mesh" << std::endl;
    return edges;
}

std::vector<PlaneIntersection> computePlaneIntersections(
    const MatrixXd& V,
    const MatrixXi& F,
    const std::map<std::pair<int, int>, EdgeInfo>& edges,
    double axis_value,
    int axis_index)
{
    double turb_eps = 1e-6;

    // Check for degeneracy first - if any vertex coordinate exactly equals axis_value
    bool needs_perturbation = false;
    for (int i = 0; i < V.rows(); ++i) {
        if (V(i, axis_index) == axis_value) {
            needs_perturbation = true;
            break;
        }
    }

    // If degeneracy detected, perturb axis_value and recursively call
    if (needs_perturbation) {
        std::cout << "Degeneracy detected at axis_value=" << axis_value << ", perturbing by "
                  << turb_eps << std::endl;
        return computePlaneIntersections(V, F, edges, axis_value + turb_eps, axis_index);
    }

    std::vector<PlaneIntersection> intersections;

    // Check each edge for intersection with the plane
    for (const auto& edge_pair : edges) {
        const EdgeInfo& edge = edge_pair.second;
        int v0 = edge.v0;
        int v1 = edge.v1;

        Vector3d p0 = V.row(v0);
        Vector3d p1 = V.row(v1);

        double coord0 = p0(axis_index);
        double coord1 = p1(axis_index);

        // Check if edge crosses the plane
        if ((coord0 <= axis_value && coord1 >= axis_value) ||
            (coord0 >= axis_value && coord1 <= axis_value)) {
            double t = (axis_value - coord0) / (coord1 - coord0);

            int left_intersection_id = -1;
            int right_intersection_id = -1;

            // Create intersection for left triangle (if exists)
            if (edge.fl != -1) {
                PlaneIntersection intersection;
                intersection.triangle_id = edge.fl;
                intersection.edge_v0 = v0;
                intersection.edge_v1 = v1;
                intersection.connected_intersection_id = -1; // Will be set later

                // Find which edge of the triangle this corresponds to
                int edge_idx = -1;
                for (int i = 0; i < 3; ++i) {
                    if ((F(edge.fl, i) == v0 && F(edge.fl, (i + 1) % 3) == v1) ||
                        (F(edge.fl, i) == v1 && F(edge.fl, (i + 1) % 3) == v0)) {
                        edge_idx = i;
                        break;
                    }
                }

                if (edge_idx != -1) {
                    intersection.barycentric = Vector3d::Zero();
                    if (F(edge.fl, edge_idx) == v0) {
                        intersection.barycentric(edge_idx) = 1 - t;
                        intersection.barycentric((edge_idx + 1) % 3) = t;
                    } else {
                        intersection.barycentric(edge_idx) = t;
                        intersection.barycentric((edge_idx + 1) % 3) = 1 - t;
                    }
                    left_intersection_id = intersections.size();
                    intersections.push_back(intersection);
                }
            }

            // Create intersection for right triangle (if exists)
            if (edge.fr != -1) {
                PlaneIntersection intersection;
                intersection.triangle_id = edge.fr;
                intersection.edge_v0 = v0;
                intersection.edge_v1 = v1;
                intersection.connected_intersection_id = -1; // Will be set later

                // Find which edge of the triangle this corresponds to
                int edge_idx = -1;
                for (int i = 0; i < 3; ++i) {
                    if ((F(edge.fr, i) == v0 && F(edge.fr, (i + 1) % 3) == v1) ||
                        (F(edge.fr, i) == v1 && F(edge.fr, (i + 1) % 3) == v0)) {
                        edge_idx = i;
                        break;
                    }
                }

                if (edge_idx != -1) {
                    intersection.barycentric = Vector3d::Zero();
                    if (F(edge.fr, edge_idx) == v0) {
                        intersection.barycentric(edge_idx) = 1 - t;
                        intersection.barycentric((edge_idx + 1) % 3) = t;
                    } else {
                        intersection.barycentric(edge_idx) = t;
                        intersection.barycentric((edge_idx + 1) % 3) = 1 - t;
                    }
                    right_intersection_id = intersections.size();
                    intersections.push_back(intersection);
                }
            }

            // Connect the left and right intersections if both exist
            if (left_intersection_id != -1 && right_intersection_id != -1) {
                intersections[left_intersection_id].connected_intersection_id =
                    right_intersection_id;
                intersections[right_intersection_id].connected_intersection_id =
                    left_intersection_id;
            }
        }
    }

    std::cout << "Found " << intersections.size() << " plane intersections with connectivity"
              << std::endl;
    return intersections;
}


std::vector<query_curve> generatePlaneCurves(const MatrixXd& V, const MatrixXi& F, int N)
{
    std::vector<query_curve> all_curves;

    // Extract edges once
    auto edges = extractMeshEdges(F);

    // Compute bounding box
    Vector3d min_coords = V.colwise().minCoeff();
    Vector3d max_coords = V.colwise().maxCoeff();

    // Generate curves for each axis
    for (int axis = 0; axis < 3; ++axis) {
        double min_val = min_coords(axis);
        double max_val = max_coords(axis);

        for (int i = 0; i < N; ++i) {
            double axis_value = min_val + (max_val - min_val) * (i + 1) / (N + 1);

            auto intersections = computePlaneIntersections(V, F, edges, axis_value, axis);
            auto query_curves = buildQueryCurvesFromIntersections(intersections, V, F);

            all_curves.insert(all_curves.end(), query_curves.begin(), query_curves.end());
        }
    }

    std::cout << "Generated " << all_curves.size() << " total plane curves" << std::endl;
    return all_curves;
}

std::vector<query_curve> buildQueryCurvesFromIntersections(
    const std::vector<PlaneIntersection>& intersections,
    const MatrixXd& V,
    const MatrixXi& F)
{
    std::vector<query_curve> query_curves;

    // Step 1: Group intersections by triangle ID
    std::map<int, std::vector<int>> triangle_intersections;
    for (size_t i = 0; i < intersections.size(); ++i) {
        triangle_intersections[intersections[i].triangle_id].push_back(i);
    }

    std::cout << "=== Building Query Curves from Intersections ===" << std::endl;
    std::cout << "Found intersections in " << triangle_intersections.size() << " triangles"
              << std::endl;

    // Step 2: Create segments within each triangle
    struct SegmentInfo
    {
        query_segment segment;
        int intersection_idx0; // Intersection at bc[0]
        int intersection_idx1; // Intersection at bc[1]
    };

    std::vector<SegmentInfo> all_segments;
    std::map<int, int> intersection_to_segment;

    for (const auto& triangle_pair : triangle_intersections) {
        int triangle_id = triangle_pair.first;
        const std::vector<int>& intersection_indices = triangle_pair.second;

        if (intersection_indices.size() == 2) {
            SegmentInfo seg_info;

            const PlaneIntersection& p1 = intersections[intersection_indices[0]];
            const PlaneIntersection& p2 = intersections[intersection_indices[1]];

            seg_info.segment.f_id = triangle_id;
            seg_info.segment.origin_segment_id = all_segments.size();
            seg_info.segment.bcs[0] = p1.barycentric;
            seg_info.segment.bcs[1] = p2.barycentric;
            seg_info.segment.fv_ids << F(triangle_id, 0), F(triangle_id, 1), F(triangle_id, 2);

            seg_info.intersection_idx0 = intersection_indices[0];
            seg_info.intersection_idx1 = intersection_indices[1];

            intersection_to_segment[intersection_indices[0]] = all_segments.size();
            intersection_to_segment[intersection_indices[1]] = all_segments.size();

            all_segments.push_back(seg_info);
        }
    }

    // Step 3: Build connected curves
    std::vector<bool> segment_used(all_segments.size(), false);

    struct candidate_segment
    {
        int seg_id;
        bool need_flip;
        int next_seg_local_id;
    };

    for (size_t start_seg = 0; start_seg < all_segments.size(); ++start_seg) {
        if (segment_used[start_seg]) continue;

        std::cout << "\n=== Starting new curve from segment " << start_seg << " ===" << std::endl;
        
        query_curve curve;

        // Build curve by following connected segments in order

        int current_seg = start_seg;
        std::queue<candidate_segment> seg_queue;
        seg_queue.push({current_seg, false, -1});

        std::cout << "Right traversal (next direction):" << std::endl;
        // check right loop(next)
        while (!seg_queue.empty()) {
            candidate_segment current_seg = seg_queue.front();
            seg_queue.pop();
            
            std::cout << "  Processing segment " << current_seg.seg_id 
                      << " (flip=" << current_seg.need_flip 
                      << ", next_local_id=" << current_seg.next_seg_local_id << ")" << std::endl;
            
            if (segment_used[current_seg.seg_id]) {
                std::cout << "    SKIP: segment already used" << std::endl;
                continue;
            }

            segment_used[current_seg.seg_id] = true;

            SegmentInfo& current_seg_info = all_segments[current_seg.seg_id];

            std::cout << "    Before flip: intersection_idx0=" << current_seg_info.intersection_idx0 
                      << ", intersection_idx1=" << current_seg_info.intersection_idx1 << std::endl;

            if (current_seg.need_flip) {
                std::swap(current_seg_info.segment.bcs[0], current_seg_info.segment.bcs[1]);
                std::swap(current_seg_info.intersection_idx0, current_seg_info.intersection_idx1);
                std::cout << "    FLIPPED segment" << std::endl;
            }

            std::cout << "    After flip: intersection_idx0=" << current_seg_info.intersection_idx0 
                      << ", intersection_idx1=" << current_seg_info.intersection_idx1 << std::endl;

            curve.segments.push_back(current_seg_info.segment);
            curve.next_segment_ids.push_back(current_seg.next_seg_local_id);
            
            std::cout << "    Added to curve: segments.size()=" << curve.segments.size() 
                      << ", next_segment_ids.size()=" << curve.next_segment_ids.size() << std::endl;

            int intersection_next =
                intersections[current_seg_info.intersection_idx1].connected_intersection_id;

            std::cout << "    Looking for next connection: intersection_next=" << intersection_next << std::endl;

            if (intersection_next != -1) {
                auto seg_it = intersection_to_segment.find(intersection_next);
                if (seg_it != intersection_to_segment.end()) {
                    int candidate_seg = seg_it->second;
                    std::cout << "    Found candidate next segment: " << candidate_seg << std::endl;
                    
                    // set current next
                    if (segment_used[candidate_seg]) {
                        std::cout << "    LOOP detected: candidate segment already used, setting next_id to 0" << std::endl;
                        curve.next_segment_ids.back() = 0; // first segment, because it is a loop
                    } else {
                        curve.next_segment_ids.back() = curve.next_segment_ids.size();
                        std::cout << "    Set current next_segment_id to " << curve.next_segment_ids.back() << std::endl;
                        
                        // check direction and push next segment
                        if (all_segments[candidate_seg].intersection_idx0 == intersection_next) {
                            std::cout << "    Direction check: candidate starts at intersection " << intersection_next << " -> no flip needed" << std::endl;
                            seg_queue.push({candidate_seg, false, -1});
                        } else {
                            std::cout << "    Direction check: candidate ends at intersection " << intersection_next << " -> flip needed" << std::endl;
                            seg_queue.push({candidate_seg, true, -1});
                        }
                    }
                } else {
                    std::cout << "    No segment found for intersection " << intersection_next << std::endl;
                }
            } else {
                std::cout << "    No next intersection (end of chain)" << std::endl;
            }
        }

        std::cout << "Left traversal (prev direction):" << std::endl;
        // check left loop(prev)
        {
            int first_prev_intersection =
                intersections[all_segments[start_seg].intersection_idx0].connected_intersection_id;
            std::cout << "  Initial prev intersection: " << first_prev_intersection << std::endl;
            
            auto first_prev_seg_it = intersection_to_segment.find(first_prev_intersection);
            if (first_prev_seg_it != intersection_to_segment.end()) {
                int first_prev_seg = first_prev_seg_it->second;
                std::cout << "  Found initial prev segment: " << first_prev_seg << std::endl;
                
                if (!segment_used[first_prev_seg]) {
                    std::cout << "  Initial prev segment not used, adding to queue" << std::endl;
                    if (all_segments[first_prev_seg].intersection_idx1 == first_prev_intersection) {
                        std::cout << "    Direction check: prev segment ends at intersection " << first_prev_intersection << " -> no flip needed" << std::endl;
                        seg_queue.push({first_prev_seg, false, 0});
                    } else {
                        std::cout << "    Direction check: prev segment starts at intersection " << first_prev_intersection << " -> flip needed" << std::endl;
                        seg_queue.push({first_prev_seg, true, 0});
                    }
                } else {
                    std::cout << "  Initial prev segment already used" << std::endl;
                }
            } else {
                std::cout << "  No segment found for initial prev intersection " << first_prev_intersection << std::endl;
            }
        }
        while (!seg_queue.empty()) {
            candidate_segment current_seg = seg_queue.front();
            seg_queue.pop();
            
            std::cout << "  Processing prev segment " << current_seg.seg_id 
                      << " (flip=" << current_seg.need_flip 
                      << ", next_local_id=" << current_seg.next_seg_local_id << ")" << std::endl;
            
            if (segment_used[current_seg.seg_id]) {
                std::cout << "    SKIP: prev segment already used" << std::endl;
                continue;
            }

            segment_used[current_seg.seg_id] = true;
            SegmentInfo& current_seg_info = all_segments[current_seg.seg_id];

            std::cout << "    Before flip: intersection_idx0=" << current_seg_info.intersection_idx0 
                      << ", intersection_idx1=" << current_seg_info.intersection_idx1 << std::endl;

            if (current_seg.need_flip) {
                std::swap(current_seg_info.segment.bcs[0], current_seg_info.segment.bcs[1]);
                std::swap(current_seg_info.intersection_idx0, current_seg_info.intersection_idx1);
                std::cout << "    FLIPPED prev segment" << std::endl;
            }

            std::cout << "    After flip: intersection_idx0=" << current_seg_info.intersection_idx0 
                      << ", intersection_idx1=" << current_seg_info.intersection_idx1 << std::endl;

            curve.segments.push_back(current_seg_info.segment);
            curve.next_segment_ids.push_back(current_seg.next_seg_local_id);
            
            std::cout << "    Added prev to curve: segments.size()=" << curve.segments.size() 
                      << ", next_segment_ids.size()=" << curve.next_segment_ids.size() << std::endl;

            int intersection_prev =
                intersections[current_seg_info.intersection_idx0].connected_intersection_id;

            std::cout << "    Looking for prev connection: intersection_prev=" << intersection_prev << std::endl;

            if (intersection_prev != -1) {
                auto seg_it = intersection_to_segment.find(intersection_prev);
                if (seg_it != intersection_to_segment.end()) {
                    int candidate_seg = seg_it->second;
                    std::cout << "    Found candidate prev segment: " << candidate_seg << std::endl;
                    
                    if (!segment_used[candidate_seg]) {
                        if (all_segments[candidate_seg].intersection_idx1 == intersection_prev) {
                            std::cout << "    Direction check: prev candidate ends at intersection " << intersection_prev << " -> no flip needed" << std::endl;
                            seg_queue.push({candidate_seg, false, int(curve.segments.size()) - 1});
                        } else {
                            std::cout << "    Direction check: prev candidate starts at intersection " << intersection_prev << " -> flip needed" << std::endl;
                            seg_queue.push({candidate_seg, true, int(curve.segments.size()) - 1});
                        }
                    } else {
                        std::cout << "    Candidate prev segment already used" << std::endl;
                    }
                } else {
                    std::cout << "    No segment found for prev intersection " << intersection_prev << std::endl;
                }
            } else {
                std::cout << "    No prev intersection (end of chain)" << std::endl;
            }
        }

        std::cout << "Completed curve: " << curve.segments.size() << " segments, next_ids: [";
        for (size_t i = 0; i < curve.next_segment_ids.size(); ++i) {
            std::cout << curve.next_segment_ids[i];
            if (i < curve.next_segment_ids.size() - 1) std::cout << ", ";
        }
        std::cout << "]" << std::endl;
        
        query_curves.push_back(curve);
    }


    std::cout << "Built " << query_curves.size() << " query curves from " << all_segments.size()
              << " segments" << std::endl;
    return query_curves;
}