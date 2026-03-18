namespace wmtk::components::image_simulation::tri {

std::vector<ImageSimulationMeshTri::ConnectedComponent>
ImageSimulationMeshTri::compute_connected_components()
    const
{
    const size_t n_faces = tri_capacity();
    std::vector<int> comp_id(n_faces, -1);
    std::vector<ConnectedComponent> components;

    // Pass 1: BFS to assign component ids and collect faces per component
    for (size_t fid = 0; fid < n_faces; ++fid) {
        if (!tuple_from_tri(fid).is_valid(*this)) {
            continue;
        }
        if (comp_id[fid] != -1) {
            continue;
        }

        const int64_t tag = m_face_attribute[fid].tags[0];
        const int comp_idx = (int)components.size();
        ConnectedComponent& comp = components.emplace_back();
        if (tag == -1) {
            log_and_throw_error(fmt::format("Face {} has invalid tag -1", fid));
        }
        comp.tag = tag;

        comp.faces.push_back(fid);
        comp_id[fid] = comp_idx;

        for (size_t i = 0; i < comp.faces.size(); ++i) { // BFS loop with comp.faces growing
            const size_t cur = comp.faces[i];
            const auto fvs = oriented_tri_vids(cur);
            const Vector2d& fp0 = m_vertex_attribute[fvs[0]].m_pos;
            const Vector2d& fp1 = m_vertex_attribute[fvs[1]].m_pos;
            const Vector2d& fp2 = m_vertex_attribute[fvs[2]].m_pos;
            comp.area += 0.5 * std::abs(
                                   (fp1[0] - fp0[0]) * (fp2[1] - fp0[1]) -
                                   (fp1[1] - fp0[1]) * (fp2[0] - fp0[0]));
            for (int j = 0; j < 3; ++j) {
                const Tuple edge_tup = tuple_from_edge(cur, j);
                const auto t_opp = edge_tup.switch_face(*this);
                if (!t_opp) {
                    continue;
                }
                const size_t nbr = t_opp->fid(*this);
                if (m_face_attribute[nbr].tags[0] == tag && comp_id[nbr] == -1) {
                    comp_id[nbr] = comp_idx;
                    comp.faces.push_back(nbr);
                }
            }
        }
    }

    // Pass 2: populate surrounding_comp_ids by scanning all boundary edges
    for (size_t fid = 0; fid < n_faces; ++fid) {
        if (!tuple_from_tri(fid).is_valid(*this)) {
            continue;
        }
        const size_t cidx = comp_id[fid];
        for (int j = 0; j < 3; ++j) {
            const Tuple edge_tup = tuple_from_edge(fid, j);
            const auto t_opp = edge_tup.switch_face(*this);
            if (!t_opp) {
                components[cidx].touches_boundary = true;
                continue;
            }
            const size_t nbr_cidx = comp_id[t_opp->fid(*this)];
            if (nbr_cidx != cidx) {
                components[cidx].surrounding_comp_ids.insert(nbr_cidx);
            }
        }
    }

    return components;
}

void ImageSimulationMeshTri::engulf_component(
    std::vector<ImageSimulationMeshTri::ConnectedComponent>& components,
    const size_t hole_comp_id,
    const size_t engulfing_comp_id)
{
    if (hole_comp_id >= components.size() || engulfing_comp_id >= components.size()) {
        log_and_throw_error(
            "Invalid component ids for engulfing: hole_comp_id = {}, engulfing_comp_id = {}, total "
            "components = {}",
            hole_comp_id,
            engulfing_comp_id,
            components.size());
    }
    auto& hole_comp = components[hole_comp_id];
    auto& engulfing_comp = components[engulfing_comp_id];

    for (const size_t fid : hole_comp.faces) {
        m_face_attribute[fid].tags[0] = engulfing_comp.tag;
    }
    engulfing_comp.area += hole_comp.area;
    engulfing_comp.faces.insert(
        engulfing_comp.faces.end(),
        hole_comp.faces.begin(),
        hole_comp.faces.end());
    engulfing_comp.surrounding_comp_ids.erase(hole_comp_id);
    hole_comp = ConnectedComponent();
}

void ImageSimulationMeshTri::engulf_components(
    std::vector<ImageSimulationMeshTri::ConnectedComponent>& components,
    const std::vector<size_t>& hole_comp_ids,
    const std::unordered_set<size_t>& engulfing_comp_ids)
{
    if (engulfing_comp_ids.empty()) {
        log_and_throw_error(
            "No engulfing components found for hole component(s) {}",
            fmt::join(hole_comp_ids, ","));
    } else if (engulfing_comp_ids.size() == 1) {
        for (const size_t comp_id : hole_comp_ids) {
            engulf_component(components, comp_id, *engulfing_comp_ids.begin());
        }
        return;
    } else {
        // Build face->component mapping from current components state
        std::vector<size_t> comp_id_map(tri_capacity(), std::numeric_limits<size_t>::max());
        for (size_t i = 0; i < components.size(); ++i) {
            for (const size_t fid : components[i].faces) {
                comp_id_map[fid] = i;
            }
        }

        // Build boundary edge centroids between hole and engulfing components
        std::vector<std::pair<Vector2d, size_t>> boundary_edge_centroids;
        for (const size_t comp_id : hole_comp_ids) {
            for (const size_t fid : components[comp_id].faces) {
                if (!tuple_from_tri(fid).is_valid(*this)) {
                    continue;
                }
                for (int j = 0; j < 3; ++j) {
                    const Tuple edge_tup = tuple_from_edge(fid, j);
                    const auto t_opp = edge_tup.switch_face(*this);
                    if (!t_opp) {
                        continue;
                    }
                    const size_t nbr_cidx = comp_id_map[t_opp->fid(*this)];
                    if (engulfing_comp_ids.count(
                            nbr_cidx)) { // neighbor is one of the engulfing components
                        const auto vs = get_edge_vids(edge_tup);
                        const Vector2d& p0 = m_vertex_attribute[vs[0]].m_pos;
                        const Vector2d& p1 = m_vertex_attribute[vs[1]].m_pos;
                        boundary_edge_centroids.emplace_back((p0 + p1) / 2.0, nbr_cidx);
                    }
                }
            }
        }

        // Helper: per-comp minimum squared distance from pos to any centroid of that comp.
        const auto comp_min_sq_dists = [&](const Vector2d& pos) {
            std::unordered_map<size_t, double> result;
            for (const auto& [centroid, cidx] : boundary_edge_centroids) {
                const double d = (pos - centroid).squaredNorm();
                auto [it, ins] = result.emplace(cidx, d);
                if (!ins) {
                    it->second = std::min(it->second, d);
                }
            }
            return result;
        };

        // Helper: find closest engulfing comp for a given vertex position.
        const auto compute_vertex_closest_comp = [&](const Vector2d& pos) -> size_t {
            const auto dists = comp_min_sq_dists(pos);
            size_t closest = std::numeric_limits<size_t>::max();
            double min_dist = std::numeric_limits<double>::max();
            for (const auto& [cidx, d] : dists) {
                if (d < min_dist) {
                    min_dist = d;
                    closest = cidx;
                }
            }
            return closest;
        };

        // Compute vertex_closest_comp for all vertices of hole faces
        const std::unordered_set<size_t> hole_comp_ids_set(
            hole_comp_ids.begin(),
            hole_comp_ids.end());
        std::unordered_map<size_t, size_t> vertex_closest_comp; // vid -> closest engulfing comp_id
        for (const size_t comp_id : hole_comp_ids) {
            for (const size_t fid : components[comp_id].faces) {
                const auto fvs = oriented_tri_vids(fid);
                for (const size_t vid : fvs) {
                    if (vertex_closest_comp.count(vid)) {
                        continue;
                    }
                    vertex_closest_comp[vid] =
                        compute_vertex_closest_comp(m_vertex_attribute[vid].m_pos);
                }
            }
        }

        // Find hole edges where the two endpoint vertices disagree on closest comp -> split.
        // Deduplicate by sorted vertex pair to avoid splitting same edge twice.
        std::set<std::pair<size_t, size_t>> edges_to_split_set;
        for (const size_t comp_id : hole_comp_ids) {
            for (const size_t fid : components[comp_id].faces) {
                if (!tuple_from_tri(fid).is_valid(*this)) {
                    continue;
                }
                for (int j = 0; j < 3; ++j) {
                    const Tuple edge_tup = tuple_from_edge(fid, j);
                    const auto vs = get_edge_vids(edge_tup);
                    const size_t va = vs[0], vb = vs[1];
                    if (vertex_closest_comp.count(va) && vertex_closest_comp.count(vb)) {
                        const size_t ca = vertex_closest_comp.at(va);
                        const size_t cb = vertex_closest_comp.at(vb);
                        // Skip if either endpoint is equidistant
                        if (ca != std::numeric_limits<size_t>::max() &&
                            cb != std::numeric_limits<size_t>::max() && ca != cb) {
                            edges_to_split_set.emplace(std::min(va, vb), std::max(va, vb));
                        }
                    } else {
                        log_and_throw_error(
                            fmt::format(
                                "Vertex {} or {} has no closest component; this should not happen",
                                va,
                                vb));
                    }
                }
            }
        }

        // Helper: signed distance between two engulfing comps at position p.
        // Negative = closer to comp_a centroids, positive = closer to comp_b centroids.
        const auto voronoi_sign =
            [&](const Vector2d& p, const size_t comp_a, const size_t comp_b) -> double {
            const auto dists = comp_min_sq_dists(p);
            const double da =
                dists.count(comp_a) ? dists.at(comp_a) : std::numeric_limits<double>::max();
            const double db =
                dists.count(comp_b) ? dists.at(comp_b) : std::numeric_limits<double>::max();
            return da - db;
        };

        // Split disagreement edges; split_edge_after binary-searches vmid onto the
        // equidistant surface between the two competing engulfing comps via m_voronoi_split_fn.
        for (const auto& [va, vb] : edges_to_split_set) {
            auto [edge_tup, _eid] = tuple_from_edge({{va, vb}});
            if (!edge_tup.is_valid(*this)) {
                continue;
            }
            const size_t comp_a = vertex_closest_comp.at(va);
            const size_t comp_b = vertex_closest_comp.at(vb);
            // Skip if either endpoint was relabeled equidistant by a previous iteration's
            // near-endpoint check — voronoi_sign with equidistant comp returns garbage.
            if (comp_a == std::numeric_limits<size_t>::max() ||
                comp_b == std::numeric_limits<size_t>::max()) {
                continue;
            }

            // If the Voronoi boundary is so close to va or vb that vmid would land on top of
            // it, skip the split — it would create a degenerate face.
            // Use linear interpolation: t = |sign_a| / (|sign_a| + |sign_b|) is the fraction
            // along the edge where the zero-crossing lies.
            // Also relabel the near-boundary vertex as equidistant so assignment is correct.
            {
                const double sign_a = voronoi_sign(m_vertex_attribute[va].m_pos, comp_a, comp_b);
                const double sign_b = voronoi_sign(m_vertex_attribute[vb].m_pos, comp_a, comp_b);

                if (sign_a * sign_b > 0.0) {
                    log_and_throw_error(
                        fmt::format(
                            "Endpoints {} and {} of edge to split have same sign {}, {}: this "
                            "should not happen",
                            va,
                            vb,
                            sign_a,
                            sign_b));
                }
                if (sign_a > 0.0 || sign_b < 0.0) {
                    log_and_throw_error(
                        fmt::format(
                            "Endpoint signs for edge to split are sign_a = {}, sign_b = {}: this "
                            "should not happen",
                            sign_a,
                            sign_b));
                }

                const double total = sign_b - sign_a;
                if (total < 1e-10) // both comps are very close to equidistant — skip split and mark
                                   // both endpoints as equidistant to avoid numerical issues with
                                   // voronoi_sign near the boundary
                {
                    vertex_closest_comp[va] = std::numeric_limits<size_t>::max();
                    vertex_closest_comp[vb] = std::numeric_limits<size_t>::max();
                    continue;
                }
                constexpr double kMinFrac =
                    1e-3; // manual tolerance to avoid degenerate splits; if the boundary is within
                          // 0.1% of the edge length from an endpoint, skip the split and mark that
                          // endpoint as equidistant.
                const double t = std::abs(sign_a) / total;
                if (t < kMinFrac) {
                    vertex_closest_comp[va] = std::numeric_limits<size_t>::max();
                    continue;
                }
                if (t > 1.0 - kMinFrac) {
                    vertex_closest_comp[vb] = std::numeric_limits<size_t>::max();
                    continue;
                }
            }

            m_voronoi_split_fn = [&, comp_a, comp_b](const Vector2d& p) -> double {
                return voronoi_sign(p, comp_a, comp_b);
            };
            std::vector<Tuple> new_tris;
            const bool split_success = split_edge(edge_tup, new_tris);
            m_voronoi_split_fn = nullptr;
            if (!split_success || new_tris.empty()) {
                if (!split_success) {
                    log_and_throw_error(
                        "engulf_components: split_edge({},{}) failed; assignment will use centroid "
                        "fallback",
                        va,
                        vb);
                }
            } else {
                const size_t vmid = new_tris[0].vid(*this);
                vertex_closest_comp[vmid] = std::numeric_limits<size_t>::max(); // equidistant

                // Extend comp_id_map and register genuinely new hole faces
                if (tri_capacity() > comp_id_map.size()) {
                    comp_id_map.resize(tri_capacity(), std::numeric_limits<size_t>::max());
                }
                for (const Tuple& nt : new_tris) {
                    const size_t nfid = nt.fid(*this);
                    if (comp_id_map[nfid] != std::numeric_limits<size_t>::max()) {
                        continue;
                    }
                    const int64_t ftag = m_face_attribute[nfid].tags[0];
                    for (const size_t hcid : hole_comp_ids) {
                        if (components[hcid].tag == ftag) {
                            comp_id_map[nfid] = hcid;
                            components[hcid].faces.push_back(nfid);
                            break;
                        }
                    }
                }
            }
        }

        // Assign all hole faces to engulfing comp.
        // Boundary (equidistant) vertices have vertex_closest_comp == max — skip them.
        // All definite vertices of a face must agree on the comp; if all are boundary, skip face.
        std::unordered_map<size_t, std::vector<size_t>> newly_assigned; // engulfing cidx -> fids
        for (const size_t comp_id : hole_comp_ids) {
            for (const size_t fid : components[comp_id].faces) {
                if (!tuple_from_tri(fid).is_valid(*this)) {
                    continue;
                }
                const auto fvs = oriented_tri_vids(fid);
                // Ensure all vertices have an entry (new faces may have new verts)
                for (const size_t vid : fvs) {
                    if (!vertex_closest_comp.count(vid)) {
                        vertex_closest_comp[vid] =
                            compute_vertex_closest_comp(m_vertex_attribute[vid].m_pos);
                    }
                }
                // Collect definite (non-boundary) vertex assignments
                size_t closest = std::numeric_limits<size_t>::max();
                bool conflict = false;
                for (const size_t vid : fvs) {
                    const size_t c = vertex_closest_comp.at(vid);
                    if (c == std::numeric_limits<size_t>::max()) { // equidistant vertex
                        continue;
                    }
                    if (closest == std::numeric_limits<size_t>::max()) {
                        closest = c;
                    } else if (closest != c) {
                        conflict = true;
                        break;
                    }
                }
                if (conflict || closest == std::numeric_limits<size_t>::max()) {
                    const Vector2d centroid =
                        (m_vertex_attribute[fvs[0]].m_pos + m_vertex_attribute[fvs[1]].m_pos +
                         m_vertex_attribute[fvs[2]].m_pos) /
                        3.0;
                    closest = compute_vertex_closest_comp(centroid);
                    if (conflict) {
                        logger().warn(
                            "Face {} has conflicting vertex assignments after split; using "
                            "centroid fallback for assignment",
                            fid);
                    }
                    if (closest == std::numeric_limits<size_t>::max()) {
                        logger().warn(
                            "Face {} has no definite vertex assignments after split; using "
                            "centroid fallback for assignment",
                            fid);
                    }
                }

                const auto& fp0 = m_vertex_attribute[fvs[0]].m_pos;
                const auto& fp1 = m_vertex_attribute[fvs[1]].m_pos;
                const auto& fp2 = m_vertex_attribute[fvs[2]].m_pos;
                m_face_attribute[fid].tags[0] = components[closest].tag;
                components[closest].area += 0.5 * std::abs(
                                                      (fp1[0] - fp0[0]) * (fp2[1] - fp0[1]) -
                                                      (fp1[1] - fp0[1]) * (fp2[0] - fp0[0]));
                components[closest].faces.push_back(fid);
                comp_id_map[fid] = closest;
                newly_assigned[closest].push_back(fid);
            }
        }

        // Update surrounding_comp_ids using newly assigned faces (not stale comp.faces)
        for (const auto& [cidx, fids] : newly_assigned) {
            for (const size_t fid : fids) {
                if (!tuple_from_tri(fid).is_valid(*this)) {
                    continue;
                }
                for (int j = 0; j < 3; ++j) {
                    const Tuple edge_tup = tuple_from_edge(fid, j);
                    const auto t_opp = edge_tup.switch_face(*this);
                    if (!t_opp) {
                        continue;
                    }
                    const size_t nbr_cidx = comp_id_map[t_opp->fid(*this)];
                    if (nbr_cidx == std::numeric_limits<size_t>::max()) {
                        continue;
                    }
                    if (nbr_cidx == cidx) {
                        continue;
                    }
                    if (hole_comp_ids_set.count(nbr_cidx)) { // will be erased
                        continue;
                    }
                    components[cidx].surrounding_comp_ids.insert(nbr_cidx);
                    components[nbr_cidx].surrounding_comp_ids.insert(cidx);
                }
            }
        }

        // --- DEBUG:
        {
            static int debug_call = 0;
            const std::string debug_path = fmt::format("/tmp/engulf_debug_{}.vtu", debug_call++);

            // Collect all valid vertices referenced by hole faces after splitting
            std::map<size_t, int> vid_to_row;
            std::vector<size_t> row_vids;
            for (const size_t comp_id : hole_comp_ids) {
                for (const size_t fid : components[comp_id].faces) {
                    if (!tuple_from_tri(fid).is_valid(*this)) {
                        continue;
                    }
                    for (const size_t v : oriented_tri_vids(fid)) {
                        if (!vid_to_row.count(v)) {
                            vid_to_row[v] = (int)row_vids.size();
                            row_vids.push_back(v);
                        }
                    }
                }
            }

            const int nv = (int)row_vids.size();
            Eigen::MatrixXd V(nv, 3);
            Eigen::VectorXd closest_label(nv), voronoi_sdf(nv);
            V.setZero();
            closest_label.setZero();
            voronoi_sdf.setZero();

            for (int i = 0; i < nv; ++i) {
                const size_t vid = row_vids[i];
                const Vector2d& p = m_vertex_attribute[vid].m_pos;
                V.row(i) = Vector3d(p[0], p[1], 0.0);

                // Find the two closest comps using the shared helper.
                const auto comp_min_sq = comp_min_sq_dists(p);
                size_t c1 = std::numeric_limits<size_t>::max();
                size_t c2 = std::numeric_limits<size_t>::max();
                double d1sq = std::numeric_limits<double>::max();
                double d2sq = std::numeric_limits<double>::max();
                for (const auto& [cidx, dsq] : comp_min_sq) {
                    if (dsq < d1sq) {
                        d2sq = d1sq;
                        c2 = c1;
                        d1sq = dsq;
                        c1 = cidx;
                    } else if (dsq < d2sq) {
                        d2sq = dsq;
                        c2 = cidx;
                    }
                }
                // voronoi_sign(p, c1, c2) = min_sq_dist_c1 - min_sq_dist_c2
                // This is exactly what is used in the splitting λ, so values here are
                // directly comparable to the binary-search target (zero crossing).
                voronoi_sdf[i] =
                    (c2 != std::numeric_limits<size_t>::max()) ? voronoi_sign(p, c1, c2) : 0.0;

                // Label: closest comp from vertex_closest_comp (-1 = equidistant)
                const size_t c = vertex_closest_comp.count(vid)
                                     ? vertex_closest_comp.at(vid)
                                     : std::numeric_limits<size_t>::max();
                closest_label[i] = (c == std::numeric_limits<size_t>::max()) ? -1.0 : (double)c;
            }

            // Build face connectivity
            std::vector<std::array<size_t, 3>> face_list;
            for (const size_t comp_id : hole_comp_ids) {
                for (const size_t fid : components[comp_id].faces) {
                    if (!tuple_from_tri(fid).is_valid(*this)) {
                        continue;
                    }
                    const auto fvs = oriented_tri_vids(fid);
                    face_list.push_back(
                        {(size_t)vid_to_row.at(fvs[0]),
                         (size_t)vid_to_row.at(fvs[1]),
                         (size_t)vid_to_row.at(fvs[2])});
                }
            }
            Eigen::MatrixXi F((int)face_list.size(), 3);
            Eigen::VectorXd face_area((int)face_list.size());
            for (int i = 0; i < (int)face_list.size(); ++i) {
                F(i, 0) = (int)face_list[i][0];
                F(i, 1) = (int)face_list[i][1];
                F(i, 2) = (int)face_list[i][2];
                const double ax = V(F(i, 0), 0), ay = V(F(i, 0), 1);
                const double bx = V(F(i, 1), 0), by = V(F(i, 1), 1);
                const double cx = V(F(i, 2), 0), cy = V(F(i, 2), 1);
                face_area[i] = 0.5 * std::abs((bx - ax) * (cy - ay) - (by - ay) * (cx - ax));
            }
            // Check for zero/near-zero area faces
            constexpr double kAreaTol = 1e-10;
            int zero_area_count = 0;
            for (int i = 0; i < (int)face_list.size(); ++i) {
                if (face_area[i] < kAreaTol) {
                    ++zero_area_count;
                    wmtk::log_and_throw_error(
                        "DEBUG engulf_components: face row {} (verts [{},{},{}]) "
                        "has near-zero area = {} | pos [{},{}] [{},{}] [{},{}]",
                        i,
                        F.row(i).transpose(),
                        face_area[i],
                        V.row(F(i, 0)).transpose(),
                        V.row(F(i, 1)).transpose(),
                        V.row(F(i, 2)).transpose());
                }
            }
            if (zero_area_count > 0) {
                wmtk::log_and_throw_error(
                    "DEBUG engulf_components: {} / {} hole faces have near-zero area",
                    zero_area_count,
                    face_list.size());
            }

            // Verify: equidistant-labeled vertices should have voronoi_sdf ≈ 0 for SOME
            // component pair — the pair used when splitting may differ from the globally
            // nearest two, so we check the minimum |voronoi_sign| over all pairs.
            for (int i = 0; i < nv; ++i) {
                if (closest_label[i] != -1.0) {
                    continue;
                }
                const auto comp_min_sq = comp_min_sq_dists(m_vertex_attribute[row_vids[i]].m_pos);
                if (comp_min_sq.size() < 2) {
                    continue;
                }
                // Find the nearest centroid distance (denominator for rel)
                double d1sq = std::numeric_limits<double>::max();
                for (const auto& [cidx, dsq] : comp_min_sq) {
                    d1sq = std::min(d1sq, dsq);
                }
                // Find the pair (ca, cb) that minimizes |voronoi_sign| = |d_ca - d_cb|
                double min_abs_sign = std::numeric_limits<double>::max();
                size_t best_ca = 0, best_cb = 0;
                const std::vector<std::pair<size_t, double>> comp_vec(
                    comp_min_sq.begin(),
                    comp_min_sq.end());
                for (size_t ai = 0; ai < comp_vec.size(); ++ai) {
                    for (size_t bi = ai + 1; bi < comp_vec.size(); ++bi) {
                        const double s = std::abs(comp_vec[ai].second - comp_vec[bi].second);
                        if (s < min_abs_sign) {
                            min_abs_sign = s;
                            best_ca = comp_vec[ai].first;
                            best_cb = comp_vec[bi].first;
                        }
                    }
                }
                constexpr double kMinFrac = 0.01;
                const double rel = (abs(d1sq) > 1e-8) ? min_abs_sign / d1sq : min_abs_sign;
                if (rel > kMinFrac) {
                    const size_t vid = row_vids[i];
                    wmtk::log_and_throw_error(
                        "DEBUG engulf_components: vertex {} labeled equidistant "
                        "but best pair voronoi_sign(ca={},cb={})={:.4f} rel={:.4f} | pos [{}, {}]",
                        vid,
                        best_ca,
                        best_cb,
                        min_abs_sign,
                        rel,
                        V(i, 0),
                        V(i, 1));
                }
            }
        }
        // --- END DEBUG ---

        for (const size_t comp_id : hole_comp_ids) {
            for (auto& c : components) {
                c.surrounding_comp_ids.erase(comp_id);
            }
            components[comp_id] = ConnectedComponent();
        }
    }
}

void ImageSimulationMeshTri::extract_hole_clusters(
    std::vector<ImageSimulationMeshTri::ConnectedComponent>& components,
    std::unordered_set<int64_t>& tags,
    std::vector<std::vector<size_t>>& hole_clusters,
    double threshold)
{
    // BFS-cluster all non-fill-tag components by adjacency.
    // Each cluster is a maximal group of connected non-fill-tag components.
    // A cluster is enclosed if every surrounding component outside it is in tags.
    std::vector<size_t> cluster_id(components.size(), -1);
    std::vector<ComponentCluster> component_clusters;
    for (size_t i = 0; i < components.size(); ++i) {
        if (components[i].faces.empty() || tags.count(components[i].tag) ||
            components[i].touches_boundary) {
            continue;
        }
        if (cluster_id[i] != -1) {
            continue;
        }
        const size_t cid = component_clusters.size();
        ComponentCluster& cluster = component_clusters.emplace_back();
        cluster.comp_ids.push_back(i);
        cluster_id[i] = cid;
        for (size_t qi = 0; qi < cluster.comp_ids.size();
             ++qi) { // BFS loop with cluster.comp_ids growing
            const size_t cur = cluster.comp_ids[qi];
            cluster.area += components[cur].area;
            for (const size_t sid : components[cur].surrounding_comp_ids) {
                if (components[sid].faces.empty()) {
                    continue;
                }
                if (tags.count(components[sid].tag)) {
                    continue;
                }
                if (components[sid].touches_boundary) {
                    cluster.enclosed = false; // neighbour escapes to mesh boundary
                    continue;
                }
                if (cluster_id[sid] == -1) {
                    cluster_id[sid] = cid;
                    cluster.comp_ids.push_back(sid);
                }
            }
        }
        // also mark not enclosed if any member touches a non-fill-tag component outside this
        // cluster
        for (const size_t ci : cluster.comp_ids) {
            for (const size_t sid : components[ci].surrounding_comp_ids) {
                if (!components[sid].faces.empty() && !tags.count(components[sid].tag) &&
                    cluster_id[sid] != cid) {
                    cluster.enclosed = false;
                }
            }
        }
    }

    // fill enclosed clusters whose total area is below threshold
    for (const ComponentCluster& cluster : component_clusters) {
        if (!cluster.enclosed) {
            continue;
        }
        if (cluster.area >= threshold) {
            continue;
        }
        hole_clusters.push_back({});
        for (const size_t ci : cluster.comp_ids) {
            hole_clusters.back().push_back(ci);
        }
    }
}

void ImageSimulationMeshTri::recompute_surface_info()
{
    for (const Tuple& e : get_edges()) {
        SmartTuple ee(*this, e);
        m_edge_attribute[ee.eid()].m_is_surface_fs = 0;
    }
    for (size_t vid = 0; vid < vert_capacity(); ++vid) {
        m_vertex_attribute[vid].m_is_on_surface = false;
    }
    for (const Tuple& e : get_edges()) {
        SmartTuple ee(*this, e);
        const auto t_opp = ee.switch_face();
        if (!t_opp) {
            continue;
        }
        bool has_diff_tag = false;
        for (size_t j = 0; j < m_tags_count; ++j) {
            if (m_face_attribute[ee.fid()].tags[j] != m_face_attribute[t_opp->fid()].tags[j]) {
                has_diff_tag = true;
                break;
            }
        }
        if (!has_diff_tag) {
            continue;
        }
        m_edge_attribute[ee.eid()].m_is_surface_fs = 1;
        const size_t v1 = ee.vid();
        const size_t v2 = ee.switch_vertex().vid();
        m_vertex_attribute[v1].m_is_on_surface = true;
        m_vertex_attribute[v2].m_is_on_surface = true;
    }
}

} // namespace wmtk::components::image_simulation::tri
