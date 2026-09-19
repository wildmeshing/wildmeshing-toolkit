#include "TaggedMesh.hpp"

#include "PythonFormat.hpp"

#include <wmtk/components/simwild/expression_parser/Parser.hpp>
#include <wmtk/utils/Logger.hpp>

#include <mshio/mshio.h>

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <unordered_map>

namespace wmtk::components::polyfem_ops {

namespace {

/// Python's str.strip() with no argument, restricted to ASCII whitespace: selection expressions
/// are ASCII. Used for the dedupe key in assign_selection_ids, so it must match Python's.
std::string strip(const std::string& s)
{
    const auto is_ws = [](unsigned char c) { return std::isspace(c) != 0; };
    size_t b = 0;
    size_t e = s.size();
    while (b < e && is_ws(static_cast<unsigned char>(s[b]))) ++b;
    while (e > b && is_ws(static_cast<unsigned char>(s[e - 1]))) --e;
    return s.substr(b, e - b);
}

/// The 3-of-4 (3D) or 2-of-3 (2D) sub-tuples of a cell, in `itertools.combinations` order --
/// lexicographic by position, which is what decides face_repr's vertex order.
std::vector<std::vector<int64_t>> combinations(const std::vector<int64_t>& nodes, size_t k)
{
    std::vector<std::vector<int64_t>> out;
    const size_t n = nodes.size();
    std::vector<size_t> idx(k);
    for (size_t i = 0; i < k; ++i) idx[i] = i;
    while (true) {
        std::vector<int64_t> combo;
        combo.reserve(k);
        for (size_t i = 0; i < k; ++i) combo.push_back(nodes[idx[i]]);
        out.push_back(std::move(combo));

        size_t i = k;
        while (i > 0 && idx[i - 1] == n - k + (i - 1)) --i;
        if (i == 0) break;
        ++idx[i - 1];
        for (size_t j = i; j < k; ++j) idx[j] = idx[j - 1] + 1;
    }
    return out;
}

std::vector<int64_t> sorted_key(const std::vector<int64_t>& v)
{
    std::vector<int64_t> key = v;
    std::sort(key.begin(), key.end());
    return key;
}

} // namespace

// ---------------------------------------------------------------------------
// Reading a .msh
// ---------------------------------------------------------------------------

GroupedMsh read_grouped(const std::string& msh_path)
{
    if (!std::filesystem::exists(msh_path)) {
        log_and_throw_error("File {} does not exist.", msh_path);
    }
    return read_grouped(mshio::load_msh(msh_path));
}

GroupedMsh read_grouped(const mshio::MshSpec& spec)
{
    GroupedMsh out;
    for (const auto& block : spec.nodes.entity_blocks) {
        for (size_t i = 0; i < block.num_nodes_in_block; ++i) {
            out.node_tags.push_back(static_cast<int64_t>(block.tags[i]));
            out.node_coords.push_back(
                {block.data[3 * i], block.data[3 * i + 1], block.data[3 * i + 2]});
        }
    }

    // Auto-detect the mesh dimension: 3D iff any volume physical group exists, else 2D.
    out.dim = 2;
    for (const auto& ph : spec.physical_groups) {
        if (ph.dim == 3) {
            out.dim = 3;
            break;
        }
    }
    const int elem_type = out.dim == 3 ? 4 : 2; // gmsh element types: 4 = tet, 2 = triangle
    const size_t npp = out.dim == 3 ? 4 : 3;

    // gmsh.model.getPhysicalGroups(dim) hands back the groups of that dimension ordered by tag;
    // mshio hands back file order, which gmsh writes sorted the same way. Sorting makes the cell
    // order -- and with it face_repr, and with it the OBJ -- independent of the writer.
    std::vector<const mshio::PhysicalGroup*> groups;
    for (const auto& ph : spec.physical_groups) {
        if (ph.dim == out.dim) groups.push_back(&ph);
    }
    std::sort(groups.begin(), groups.end(), [](const auto* a, const auto* b) {
        return a->tag < b->tag;
    });

    // entity tag -> the physical groups it belongs to, for the entities of dimension out.dim.
    std::map<int, std::vector<int>> entity_to_groups;
    if (out.dim == 3) {
        for (const auto& e : spec.entities.volumes) entity_to_groups[e.tag] = e.physical_group_tags;
    } else {
        for (const auto& e : spec.entities.surfaces) {
            entity_to_groups[e.tag] = e.physical_group_tags;
        }
    }

    for (const auto* ph : groups) {
        out.groups.emplace_back(ph->tag, ph->name);
        // gmsh.model.getEntitiesForPhysicalGroup(dim, tag), in ascending entity-tag order.
        for (const auto& [ent_tag, group_tags] : entity_to_groups) {
            if (std::find(group_tags.begin(), group_tags.end(), ph->tag) == group_tags.end()) {
                continue;
            }
            for (const auto& block : spec.elements.entity_blocks) {
                if (block.entity_dim != out.dim || block.entity_tag != ent_tag ||
                    block.element_type != elem_type) {
                    continue;
                }
                for (size_t j = 0; j < block.num_elements_in_block; ++j) {
                    const size_t off = j * (npp + 1);
                    GroupedMsh::Item item;
                    item.group_name = ph->name;
                    item.group_tag = ph->tag;
                    item.element_tag = static_cast<int64_t>(block.data[off]);
                    item.nodes.reserve(npp);
                    for (size_t k = 0; k < npp; ++k) {
                        item.nodes.push_back(static_cast<int64_t>(block.data[off + 1 + k]));
                    }
                    out.items.push_back(std::move(item));
                }
            }
        }
    }
    return out;
}

std::map<int64_t, int64_t> node_tag_to_index(const std::vector<int64_t>& tags)
{
    std::map<int64_t, int64_t> out;
    const int64_t n = static_cast<int64_t>(tags.size());
    const int64_t max_tag = *std::max_element(tags.begin(), tags.end());
    if (max_tag != n) {
        std::vector<int64_t> sorted_tags = tags;
        std::sort(sorted_tags.begin(), sorted_tags.end());
        for (size_t i = 0; i < sorted_tags.size(); ++i) {
            out[sorted_tags[i]] = static_cast<int64_t>(i);
        }
    } else {
        for (const int64_t t : tags) {
            out[t] = t - 1;
        }
    }
    return out;
}

// ---------------------------------------------------------------------------
// Tag expressions
// ---------------------------------------------------------------------------

std::set<std::string> CompiledExpression::atoms(const std::string& expr)
{
    std::set<std::string> out;
    std::string cur;
    for (const char c : expr) {
        if (std::isalnum(static_cast<unsigned char>(c)) || c == '_' || c == '-') {
            cur.push_back(c);
        } else {
            if (!cur.empty()) out.insert(cur);
            cur.clear();
        }
    }
    if (!cur.empty()) out.insert(cur);
    return out;
}

CompiledExpression::CompiledExpression(const std::string& expr)
{
    m_names = atoms(expr);
    m_names.erase("_");
    int64_t i = 0;
    for (const auto& n : m_names) {
        m_name_to_id.emplace(n, i++);
    }
    try {
        m_expr = simwild::expression_parser::parse(expr, m_name_to_id);
    } catch (const std::exception& e) {
        // The parser reports the position but not which selection it was parsing, and a scene
        // can carry a dozen of them; mesh_core.parse_expression prefixes the same way.
        log_and_throw_error("selection '{}': {}", expr, e.what());
    }
}

bool CompiledExpression::eval(const TagNames& tags) const
{
    simwild::CellTag ct;
    int64_t fresh = static_cast<int64_t>(m_name_to_id.size());
    for (const auto& t : tags) {
        const auto it = m_name_to_id.find(t);
        ct.insert(it != m_name_to_id.end() ? it->second : fresh++);
    }
    return m_expr->eval(ct);
}

// ---------------------------------------------------------------------------
// Selections
// ---------------------------------------------------------------------------

Selection normalize_selection(const nlohmann::json& spec)
{
    if (spec.is_string()) {
        return Selection{spec.get<std::string>(), std::nullopt, std::nullopt};
    }
    if (spec.is_object() && spec.contains("region")) {
        std::set<std::string> extra;
        for (const auto& item : spec.items()) {
            if (item.key() != "region" && item.key() != "filter" && item.key() != "id") {
                extra.insert(item.key());
            }
        }
        if (!extra.empty()) {
            log_and_throw_error(
                "selection {}: unknown key(s) {}",
                spec.dump(),
                python_list(extra));
        }
        if (!spec["region"].is_string() ||
            (spec.contains("filter") && !spec["filter"].is_null() &&
             !spec["filter"].is_string())) {
            log_and_throw_error("selection {}: region/filter must be strings", spec.dump());
        }
        Selection sel;
        sel.region = spec["region"].get<std::string>();
        if (spec.contains("filter") && !spec["filter"].is_null()) {
            sel.filter = spec["filter"].get<std::string>();
        }
        if (spec.contains("id") && !spec["id"].is_null()) {
            sel.id = spec["id"].get<int64_t>();
        }
        return sel;
    }
    log_and_throw_error(
        "a selection is a region expression string or {{'region': str, 'filter': str, 'id': "
        "int}}, got {} (pairs and 'a & b' conjunction selections are no longer supported)",
        spec.dump());
}

void assign_selection_ids(
    const nlohmann::json& selections,
    std::vector<Selection>& unique,
    std::vector<int64_t>& ids_per_input,
    bool require_ids)
{
    std::vector<Selection> specs;
    for (const auto& s : selections) {
        specs.push_back(normalize_selection(s));
    }
    if (require_ids) {
        for (const auto& s : specs) {
            if (!s.id.has_value()) {
                log_and_throw_error("every selection needs an explicit 'id' here");
            }
        }
    }

    // (stripped region, stripped filter or "no filter") -> id, in first-appearance order. The
    // Python dict is insertion-ordered; `order` here plays that role.
    using Key = std::pair<std::string, std::optional<std::string>>;
    std::vector<Key> order;
    std::map<Key, std::optional<int64_t>> by_key;

    const auto key_of = [](const Selection& s) {
        return Key{strip(s.region), s.filter ? std::optional<std::string>(strip(*s.filter)) : std::nullopt};
    };

    for (const auto& s : specs) {
        const Key key = key_of(s);
        const auto it = by_key.find(key);
        if (it == by_key.end()) {
            by_key.emplace(key, s.id);
            order.push_back(key);
        } else if (s.id.has_value()) {
            if (it->second.has_value() && *it->second != *s.id) {
                log_and_throw_error(
                    "selection region='{}' filter={} given conflicting ids {} and {}",
                    key.first,
                    key.second ? "'" + *key.second + "'" : "None",
                    *it->second,
                    *s.id);
            }
            it->second = s.id;
        }
    }

    std::set<int64_t> reserved;
    for (const auto& [k, v] : by_key) {
        if (v.has_value()) reserved.insert(*v);
    }
    int64_t next_id = 1;
    for (const auto& key : order) {
        auto& slot = by_key[key];
        if (!slot.has_value()) {
            while (reserved.count(next_id) != 0) ++next_id;
            slot = next_id;
            ++next_id;
        }
    }

    unique.clear();
    for (const auto& key : order) {
        unique.push_back(Selection{key.first, key.second, by_key[key]});
    }
    ids_per_input.clear();
    for (const auto& s : specs) {
        ids_per_input.push_back(*by_key[key_of(s)]);
    }
}

// ---------------------------------------------------------------------------
// Tagged-mesh loading
// ---------------------------------------------------------------------------

TaggedMesh::TaggedMesh(const std::string& msh_path)
{
    const GroupedMsh in = read_grouped(msh_path);

    // --- nodes -------------------------------------------------------------
    total_n_nodes = static_cast<int64_t>(in.node_tags.size());
    if (total_n_nodes == 0) {
        log_and_throw_error("No nodes found in {}", msh_path);
    }
    node_tag_to_idx = node_tag_to_index(in.node_tags);

    // --- dimension and coordinates ----------------------------------------
    mesh_dim = in.dim;
    coords = MatrixXd::Zero(total_n_nodes, mesh_dim);
    for (size_t i = 0; i < in.node_tags.size(); ++i) {
        const int64_t idx = node_tag_to_idx.at(in.node_tags[i]);
        for (int d = 0; d < mesh_dim; ++d) {
            coords(idx, d) = in.node_coords[i][d];
        }
    }

    // --- cells -------------------------------------------------------------
    // Every group of the mesh's own dimension is named, whether or not it carries a cell, as the
    // Python's `names` dict is; the traversal lists them in ascending tag order.
    for (const auto& [tag, name] : in.groups) {
        names[name] = tag;
    }

    // cell node-tag set -> cell index. WMTK writes one copy of a multi-tagged cell per tag; the
    // copies share a node set, so this map is what merges them (Python: `canonical`).
    std::map<std::vector<int64_t>, int64_t> canonical;
    for (const auto& item : in.items) {
        const std::vector<int64_t> vt = sorted_key(item.nodes);
        auto [it, inserted] = canonical.emplace(vt, static_cast<int64_t>(prim_nodes.size()));
        if (inserted) {
            std::vector<int64_t> nodes;
            nodes.reserve(item.nodes.size());
            for (const int64_t t : item.nodes) {
                nodes.push_back(node_tag_to_idx.at(t));
            }
            prim_nodes.push_back(std::move(nodes));
            prim_tags.emplace_back();
        }
        prim_tags[it->second].insert(item.group_name);
    }

    // --- face adjacency ----------------------------------------------------
    const size_t nppf = (mesh_dim == 3 ? 4 : 3) - 1;
    std::map<std::vector<int64_t>, int64_t> face_index;
    for (size_t p = 0; p < prim_nodes.size(); ++p) {
        for (const auto& fn : combinations(prim_nodes[p], nppf)) {
            const std::vector<int64_t> fk = sorted_key(fn);
            auto [it, inserted] = face_index.emplace(fk, static_cast<int64_t>(face_repr.size()));
            if (inserted) {
                face_repr.push_back(fn);
                face_to_prims.emplace_back();
            }
            face_to_prims[it->second].push_back(static_cast<int64_t>(p));
        }
    }
}

VectorXd TaggedMesh::centroid(int64_t prim) const
{
    // np.mean over the cell's coordinate rows: numpy sums sequentially for fewer than 8 terms
    // (pairwise summation only kicks in above that), so a plain accumulate matches it bit for bit.
    const auto& nodes = prim_nodes[prim];
    VectorXd sum = VectorXd::Zero(mesh_dim);
    for (const int64_t v : nodes) {
        sum += coords.row(v).transpose();
    }
    return sum / static_cast<double>(nodes.size());
}

std::vector<int64_t> select_region_nodes(
    const TaggedMesh& mesh,
    const std::vector<std::string>& exprs)
{
    std::set<int64_t> ids;
    for (const auto& expr : exprs) {
        const CompiledExpression pred(expr);
        bool hit = false;
        for (size_t p = 0; p < mesh.prim_tags.size(); ++p) {
            if (!pred.eval(mesh.prim_tags[p])) continue;
            hit = true;
            ids.insert(mesh.prim_nodes[p].begin(), mesh.prim_nodes[p].end());
        }
        if (!hit) {
            log_and_throw_error("region expression '{}' selects no cells", expr);
        }
    }
    return std::vector<int64_t>(ids.begin(), ids.end());
}

// ---------------------------------------------------------------------------
// Interface selection
// ---------------------------------------------------------------------------

namespace {

/// Mirrors `mesh_core._compile_selection`: parse first (so a syntax error is reported before
/// anything else), then reject '_', then reject names the mesh does not have -- the same order
/// the Python raises them in.
std::pair<std::shared_ptr<CompiledExpression>, std::shared_ptr<CompiledExpression>>
compile_selection(const Selection& sel, const TaggedMesh& mesh)
{
    std::array<std::shared_ptr<CompiledExpression>, 2> preds{};
    const std::array<std::pair<const char*, const std::string*>, 2> parts{
        {{"region", &sel.region}, {"filter", sel.filter ? &(*sel.filter) : nullptr}}};

    for (size_t i = 0; i < 2; ++i) {
        if (parts[i].second == nullptr) continue;
        const std::string& expr = *parts[i].second;
        auto compiled = std::make_shared<CompiledExpression>(expr);
        if (CompiledExpression::atoms(expr).count("_") != 0) {
            log_and_throw_error(
                "selection {}='{}': '_' has no meaning here -- every cell has a group name; "
                "write 'ambient' instead",
                parts[i].first,
                expr);
        }
        std::set<std::string> unknown;
        for (const auto& n : compiled->names()) {
            if (mesh.names.count(n) == 0) unknown.insert(n);
        }
        if (!unknown.empty()) {
            std::set<std::string> available;
            for (const auto& [n, t] : mesh.names) available.insert(n);
            log_and_throw_error(
                "selection {}='{}' references unknown tag(s) {}; available: {}",
                parts[i].first,
                expr,
                python_list(unknown),
                python_list(available));
        }
        preds[i] = compiled;
    }
    return {preds[0], preds[1]};
}

} // namespace

std::vector<BoundaryFaceRecord> select_boundary_faces(
    const TaggedMesh& mesh,
    const std::vector<Selection>& selections)
{
    struct Compiled
    {
        std::shared_ptr<CompiledExpression> region;
        std::shared_ptr<CompiledExpression> filter;
        int64_t id = 0;
    };
    std::vector<Compiled> compiled;
    for (const auto& sel : selections) {
        auto [pr, pf] = compile_selection(sel, mesh);
        compiled.push_back(Compiled{pr, pf, sel.id.value()});
    }

    // face -> inside cell -> ids, both levels in first-hit order (Python: nested defaultdicts,
    // which are insertion-ordered; that order is the record order and therefore the OBJ order).
    struct Hit
    {
        int64_t face = -1;
        std::vector<int64_t> inside_order;
        std::map<int64_t, std::set<int64_t>> ids_by_inside;
    };
    std::vector<Hit> hits;
    std::unordered_map<int64_t, size_t> hit_of_face;

    for (size_t fk = 0; fk < mesh.num_faces(); ++fk) {
        const auto& prims = mesh.face_to_prims[fk];
        if (prims.size() != 2) continue; // interior faces only
        const int64_t p = prims[0];
        const int64_t q = prims[1];
        const TagNames& tp = mesh.prim_tags[p];
        const TagNames& tq = mesh.prim_tags[q];
        for (const auto& c : compiled) {
            const std::array<std::tuple<int64_t, const TagNames*, const TagNames*>, 2> sides{
                {{p, &tp, &tq}, {q, &tq, &tp}}};
            for (const auto& [inside, t_in, t_out] : sides) {
                // pr(t_in) and not pr(t_out) and (pf is None or pf(t_out))
                if (!c.region->eval(*t_in)) continue;
                if (c.region->eval(*t_out)) continue;
                if (c.filter != nullptr && !c.filter->eval(*t_out)) continue;

                auto it = hit_of_face.find(static_cast<int64_t>(fk));
                if (it == hit_of_face.end()) {
                    it = hit_of_face.emplace(static_cast<int64_t>(fk), hits.size()).first;
                    hits.push_back(Hit{static_cast<int64_t>(fk), {}, {}});
                }
                Hit& hit = hits[it->second];
                if (hit.ids_by_inside.count(inside) == 0) {
                    hit.inside_order.push_back(inside);
                }
                hit.ids_by_inside[inside].insert(c.id);
            }
        }
    }

    std::vector<BoundaryFaceRecord> out;
    for (const auto& hit : hits) {
        const auto& prims = mesh.face_to_prims[hit.face];
        for (const int64_t a_prim : hit.inside_order) {
            const int64_t b_prim = prims[0] == a_prim ? prims[1] : prims[0];
            BoundaryFaceRecord rec;
            rec.face = mesh.face_repr[hit.face];
            rec.a_prim = a_prim;
            rec.b_prim = b_prim;
            const auto& ids = hit.ids_by_inside.at(a_prim);
            rec.ids.assign(ids.begin(), ids.end()); // std::set iterates sorted, like sorted()
            out.push_back(std::move(rec));
        }
    }
    return out;
}

} // namespace wmtk::components::polyfem_ops
