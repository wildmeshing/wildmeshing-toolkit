#include "VolumeMap.hpp"

#include "volume_complex.hpp"
#include "volume_geometry.hpp"

#include <wmtk/history/Replay.hpp>
#include <wmtk/utils/Logger.hpp>

#include <Eigen/Dense>

#include <algorithm>
#include <functional>
#include <set>
#include <stdexcept>
#include <unordered_map>

namespace wmtk::components::tracking {

using namespace volume;

namespace {

/// Why an operation's map could not be built.
struct Fail : std::runtime_error
{
    using std::runtime_error::runtime_error;
};

constexpr uint64_t kNone = history::kNoVertex;

/// A point on a face: the face (sorted corners) and barycentric coordinates in that order.
using FacePoint = std::pair<Tri, Eigen::Vector3d>;
/// A map of face points, one way or the other (`forward`).
using CapFn = std::function<FacePoint(const Tri&, const Eigen::Vector3d&, bool forward)>;
/// A translation of face points between two namings of the same faces.
using Rename = std::function<FacePoint(const Tri&, const Eigen::Vector3d&)>;

int index_in(const Tri& t, const Vid v)
{
    return int(std::find(t.begin(), t.end(), v) - t.begin());
}

int index_in(const Tet& t, const Vid v)
{
    return int(std::find(t.begin(), t.end(), v) - t.begin());
}

Faces minus(const Faces& a, const Faces& b)
{
    Faces out;
    std::set_difference(a.begin(), a.end(), b.begin(), b.end(), std::inserter(out, out.end()));
    return out;
}

Faces intersect(const Faces& a, const Faces& b)
{
    Faces out;
    std::set_intersection(a.begin(), a.end(), b.begin(), b.end(), std::inserter(out, out.end()));
    return out;
}

Tets with_vertex(const Tets& T, const Vid v)
{
    Tets out;
    for (const Tet& t : T) {
        if (contains(t, v)) out.push_back(t);
    }
    return out;
}

Tets merged(const Tets& a, const Tets& b)
{
    Tets out = a;
    std::set<Tet> k = keys(a);
    for (const Tet& t : b) {
        if (k.insert(sorted(t)).second) out.push_back(t);
    }
    return out;
}

} // namespace

/// The tracked faces among an operation's tets, before and after it: per face (sorted local
/// corners), its surface face and its corners in the order the surface history stores them.
struct VolumeMap::Faces
{
    struct Tracked
    {
        Tri face;
        uint64_t fid;
        Tri stored;
    };
    /// Sorted by face.
    std::vector<Tracked> before, after;

    static const Tracked* find(const std::vector<Tracked>& fs, const Tri& f)
    {
        const auto it =
            std::lower_bound(fs.begin(), fs.end(), f, [](const Tracked& a, const Tri& b) {
                return a.face < b;
            });
        return it != fs.end() && it->face == f ? &*it : nullptr;
    }
    static const Tracked* by_fid(const std::vector<Tracked>& fs, const uint64_t fid)
    {
        for (const Tracked& t : fs) {
            if (t.fid == fid) return &t;
        }
        return nullptr;
    }
};

/// One operation's map: patch pairs applied in turn, each to the points in its tets.
struct VolumeMap::OpMap
{
    struct Pair
    {
        virtual ~Pair() = default;
        virtual bool has(const Tet& t, bool forward) const = 0;
        /// Maps a point, its tet (any vertex order) and barycentric coordinates in that order.
        virtual void map(Tet& t, Eigen::Vector4d& b, bool forward) const = 0;
        std::string kind;
        bool real = true;
    };
    std::vector<std::unique_ptr<Pair>> pairs;
    /// A real vertex after the operation -> its name in the pairs' complex after (a collapse
    /// mapped the other way round).
    std::map<Vid, Vid> rename;

    /// Whether a pair mapped the point.
    bool apply(Tet& t, Eigen::Vector4d& b, const bool forward) const
    {
        bool mapped = false;
        if (!forward) {
            for (Vid& v : t) {
                if (const auto it = rename.find(v); it != rename.end()) v = it->second;
            }
            for (auto p = pairs.rbegin(); p != pairs.rend(); ++p) {
                if ((*p)->has(t, false)) {
                    (*p)->map(t, b, false);
                    mapped = true;
                }
            }
            return mapped;
        }
        for (const auto& p : pairs) {
            if (p->has(t, true)) {
                p->map(t, b, true);
                mapped = true;
            }
        }
        for (Vid& v : t) {
            for (const auto& [real, name] : rename) {
                if (v == name) v = real;
            }
        }
        return mapped;
    }
};

namespace {

using Pair = VolumeMap::OpMap::Pair;

/// The radial map between two star-shaped patches: a point leaves its patch along the ray from
/// the centre, crosses to the other patch's boundary by the boundary map -- the same face and
/// coordinates on a fixed face, the cap map on the others -- and comes back in along the ray
/// from the other centre, as far in as it was.
struct RadialPair : Pair
{
    std::shared_ptr<StarPatch> Sb, Sa;
    Faces fixed;
    CapFn cap;

    bool has(const Tet& t, const bool forward) const override
    {
        return (forward ? Sb : Sa)->index_of(t).has_value();
    }
    void map(Tet& t, Eigen::Vector4d& b, const bool forward) const override
    {
        const StarPatch& S1 = forward ? *Sb : *Sa;
        const StarPatch& S2 = forward ? *Sa : *Sb;
        const Tet& tt = S1.tets()[*S1.index_of(t)];
        Eigen::Vector3d X = Eigen::Vector3d::Zero();
        for (int k = 0; k < 4; ++k) X += b[index_in(t, tt[k])] * S1.positions()[tt[k]];
        const StarPatch::Exit ex = S1.exit(X);
        const Tri& f = S1.faces()[ex.face];
        Eigen::Vector3d q = Eigen::Vector3d::Zero();
        if (fixed.count(f)) {
            for (int k = 0; k < 3; ++k) q += ex.w[k] * S2.positions()[f[k]];
        } else {
            const auto [g, w] = cap(f, ex.w, forward);
            for (int k = 0; k < 3; ++k) q += w[k] * S2.positions()[g[k]];
        }
        const Eigen::Vector3d Y = S2.centre() + ex.t * (q - S2.centre());
        const StarPatch::Located loc = S2.locate(Y);
        t = S2.tets()[loc.tet];
        b = loc.b;
    }
};

/// An edge or face split at a point m of it, which changes nothing in space: done on barycentric
/// coordinates alone (a point's piece is the one replacing the corner of the split simplex it is
/// nearest the opposite of).
struct SplitPair : Pair
{
    std::map<Tet, Tet> removed, split; // by sorted vertices
    std::vector<Vid> common;
    Vid m = 0;

    bool has(const Tet& t, const bool forward) const override
    {
        return (forward ? removed : split).count(sorted(t)) > 0;
    }
    void map(Tet& t, Eigen::Vector4d& b, const bool forward) const override
    {
        std::map<Vid, double> lam;
        for (int k = 0; k < 4; ++k) lam[t[k]] = b[k];
        const auto in_common = [this](Vid v) {
            return std::find(common.begin(), common.end(), v) != common.end();
        };
        std::map<Vid, double> nb;
        Tet nt;
        if (forward) {
            Vid lo = common[0];
            for (const Vid v : common) {
                if (lam[v] < lam[lo]) lo = v;
            }
            nt = t;
            std::replace(nt.begin(), nt.end(), lo, m);
            for (const Vid v : t) {
                if (v != lo) nb[v] = lam[v] - (in_common(v) ? lam[lo] : 0);
            }
            nb[m] = double(common.size()) * lam[lo];
            nt = split.at(sorted(nt));
        } else {
            const double share = lam[m] / double(common.size());
            std::set<Vid> want;
            for (const Vid v : t) {
                if (v != m) want.insert(v);
            }
            for (const Vid v : common) want.insert(v);
            const auto it = std::find_if(removed.begin(), removed.end(), [&](const auto& kv) {
                return std::set<Vid>(kv.first.begin(), kv.first.end()) == want;
            });
            if (it == removed.end()) throw Fail("split: no tet to unsplit into");
            nt = it->second;
            for (const Vid v : nt) nb[v] = (lam.count(v) ? lam[v] : 0) + (in_common(v) ? share : 0);
        }
        for (int k = 0; k < 4; ++k) b[k] = nb[nt[k]];
        t = nt;
    }
};

/// The surface map's local maps of an operation, by face.
struct CellIndex
{
    std::vector<const SurfaceMap::Cell*> cells;
    std::map<uint64_t, std::vector<std::pair<size_t, size_t>>> index[2];

    explicit CellIndex(std::vector<const SurfaceMap::Cell*> cs)
        : cells(std::move(cs))
    {
        for (size_t c = 0; c < cells.size(); ++c) {
            for (size_t p = 0; p < cells[c]->before.size(); ++p) {
                index[0][cells[c]->before[p].fid].emplace_back(c, p);
            }
            for (size_t p = 0; p < cells[c]->after.size(); ++p) {
                index[1][cells[c]->after[p].fid].emplace_back(c, p);
            }
        }
    }
    std::set<uint64_t> fids(const int side) const
    {
        std::set<uint64_t> f;
        for (const auto& [fid, _] : index[side]) f.insert(fid);
        return f;
    }
    /// A point of face `fid` (coordinates in its stored corner order) through the cell holding
    /// it: the face it lands on, and its coordinates there.
    std::pair<uint64_t, Eigen::Vector3d>
    map(const uint64_t fid, const Eigen::Vector3d& bs, const bool forward) const
    {
        const int src = forward ? 0 : 1;
        const auto it = index[src].find(fid);
        if (it == index[src].end()) throw Fail("cap point on a face no cell holds");
        double best = -std::numeric_limits<double>::infinity();
        size_t bc = 0;
        Eigen::Vector2d q = Eigen::Vector2d::Zero();
        for (const auto& [c, p] : it->second) {
            const SurfaceMap::Face& piece = forward ? cells[c]->before[p] : cells[c]->after[p];
            Eigen::Matrix3d S;
            for (int k = 0; k < 3; ++k) S.col(k) = piece.sub[k];
            const Eigen::Vector3d lam = S.partialPivLu().solve(bs);
            if (lam.minCoeff() > best) {
                best = lam.minCoeff();
                bc = c;
                q = lam[0] * piece.uv[0] + lam[1] * piece.uv[1] + lam[2] * piece.uv[2];
            }
        }
        const auto& dst = forward ? cells[bc]->after : cells[bc]->before;
        best = -std::numeric_limits<double>::infinity();
        std::pair<uint64_t, Eigen::Vector3d> out{0, Eigen::Vector3d::Zero()};
        for (const SurfaceMap::Face& g : dst) {
            const Eigen::Vector2d a = g.uv[0], e1 = g.uv[1] - a, e2 = g.uv[2] - a;
            Eigen::Matrix2d M;
            M << e1, e2;
            const Eigen::Vector2d l12 = M.partialPivLu().solve(q - a);
            Eigen::Vector3d l(1 - l12.sum(), l12[0], l12[1]);
            if (!(l.minCoeff() > best)) continue;
            best = l.minCoeff();
            l = l.cwiseMax(0.);
            l /= l.sum();
            out = {g.fid, l[0] * g.sub[0] + l[1] * g.sub[1] + l[2] * g.sub[2]};
        }
        return out;
    }
};

/// Builds one operation's map.
class Builder
{
public:
    Builder(
        const history::VolumeEdit& e,
        const VolumeMap::Faces& faces,
        std::vector<const SurfaceMap::Cell*> cells,
        VolumeMapStats& stats)
        : m_e(e)
        , m_faces(faces)
        , m_cells(std::move(cells))
        , m_stats(stats)
        , m_n(Vid(e.positions.size()))
    {
        m_K = e.tets;
        std::vector<uint32_t> ra;
        m_A = e.tets_after(&ra);
        for (size_t k = 0; k < m_K.size(); ++k) m_region[sorted(m_K[k])] = e.regions[k];
        for (size_t k = 0; k < m_A.size(); ++k) m_region[sorted(m_A[k])] = ra[k];
        for (const auto& t : faces.before) m_surf_b.insert(t.face);
        for (const auto& t : faces.after) m_surf_a.insert(t.face);
        m_cell_index = std::make_shared<CellIndex>(m_cells);
        m_Pb = e.positions;
        m_Pa = e.positions_after();
        for (const Tet& t : m_K) {
            const int o = orient(m_Pb[t[0]], m_Pb[t[1]], m_Pb[t[2]], m_Pb[t[3]]);
            if (o != 0) {
                m_sign = o;
                break;
            }
        }
    }

    std::unique_ptr<VolumeMap::OpMap> build()
    {
        m_op = std::make_unique<VolumeMap::OpMap>();
        try {
            switch (m_e.op) {
            case history::VolumeOp::collapse: build_collapse(); break;
            case history::VolumeOp::smooth: build_move(); break;
            default: build_swap(); break;
            }
        } catch (const Fail& f) {
            // Where the surface cannot be kept (the boundary of an open sheet, a weld), the volume
            // still can: one pair over the whole operation, every face of it kept.
            m_op = std::make_unique<VolumeMap::OpMap>();
            if (!volume_only()) throw;
            ++m_stats.surface_not_kept;
            return std::move(m_op);
        }
        if (!keeps_surface()) ++m_stats.surface_not_kept;
        return std::move(m_op);
    }

private:
    // ------------------------------------------------------------------ positions, regions
    /// P with room for vertex v.
    static void make_room(Positions& P, const Vid v)
    {
        if (P.size() <= v) P.resize(size_t(v) + 1, Eigen::Vector3d::Zero());
    }
    std::optional<uint32_t> region_of(const Tets& T) const
    {
        for (const Tet& t : T) {
            const auto it = m_region.find(sorted(t));
            if (it != m_region.end()) return it->second;
        }
        return std::nullopt;
    }

    // ------------------------------------------------------------------ embeddings
    /// A star-shaped embedding of T: its real geometry P if that is one, else a fallback.
    std::shared_ptr<StarPatch> embed(const Tets& T, const Positions& P, bool force, bool& real)
    {
        if (!force) {
            if (auto S = StarPatch::make(T, P)) {
                bool thin = false;
                if (const auto c = centre_of(*S, &thin)) {
                    S->set_centre(*c);
                    if (thin && S->boundary_apex()) {
                        // a kernel too thin for the LP: the spherical embedding, if its centre is
                        // farther from the face planes, relative to the patch
                        auto S2 = spherical_patch(T, P);
                        if (S2 && S2->margin() > S->margin()) {
                            real = false;
                            return S2;
                        }
                    }
                    real = true;
                    return std::make_shared<StarPatch>(std::move(*S));
                }
            }
        }
        real = false;
        if (auto S = spherical_patch(T, P)) return S;
        for (const auto& Q : {bowl(T, P), convex(T, P)}) {
            if (!Q) continue;
            if (auto S = StarPatch::make(T, *Q)) {
                if (const auto c = centre_of(*S)) {
                    S->set_centre(*c);
                    return std::make_shared<StarPatch>(std::move(*S));
                }
            }
        }
        throw Fail("no star-shaped embedding");
    }
    std::shared_ptr<StarPatch> spherical_patch(const Tets& T, const Positions& P)
    {
        const auto Q = spherical(T, P);
        if (!Q) return nullptr;
        auto S = StarPatch::make(T, *Q);
        if (!S) return nullptr;
        const auto c = centre_of(*S);
        if (!c) return nullptr;
        S->set_centre(*c);
        return std::make_shared<StarPatch>(std::move(*S));
    }
    bool real_ok(const Tets& Tb, const Tets& Ta, const Positions& Pb, const Positions& Pa) const
    {
        for (const auto& [T, P] : {std::make_pair(&Tb, &Pb), std::make_pair(&Ta, &Pa)}) {
            const auto S = StarPatch::make(*T, *P);
            if (!S || !centre_of(*S)) return false;
        }
        return true;
    }
    void add(
        const std::string& kind,
        const Tets& Tb,
        const Tets& Ta,
        const Faces& fixed,
        const Positions& Pb,
        const Positions& Pa,
        CapFn cap,
        const bool force = false)
    {
        auto p = std::make_unique<RadialPair>();
        p->kind = kind;
        p->fixed = fixed;
        p->cap = std::move(cap);
        bool rb = false, ra = false;
        p->Sb = embed(Tb, Pb, force, rb);
        p->Sa = embed(Ta, Pa, force, ra);
        if (keys(Tb) == keys(Ta) && !(rb && ra)) {
            // One triangulation before and after (a move): one embedding serves both, a real
            // one if either side is star-shaped; outside the cap's cone a point then follows
            // its tet's affine map, whatever the embedding.
            if (!rb && ra) p->Sb = p->Sa;
            p->Sa = p->Sb;
            p->real = rb || ra;
        } else {
            if (rb && ra) {
                // a common centre makes the map the identity on the cone over the fixed faces
                if (const auto s = kernel({p->Sb.get(), p->Sa.get()})) {
                    p->Sb->set_centre(*s);
                    p->Sa->set_centre(*s);
                }
            }
            p->real = rb && ra;
        }
        ++(p->real ? m_stats.real_pairs : m_stats.fallback_pairs);
        m_op->pairs.push_back(std::move(p));
    }

    // ------------------------------------------------------------------ cap maps
    /// The surface map's local map on the cap, the faces given in the pairs' naming, which
    /// `to_real_*` / `from_real_*` translate to and from the operation's faces.
    CapFn cell_cap(
        Rename to_real_b = nullptr,
        Rename from_real_b = nullptr,
        Rename to_real_a = nullptr,
        Rename from_real_a = nullptr) const
    {
        const auto cells = m_cell_index;
        const VolumeMap::Faces* F = &m_faces;
        const bool barycentric = m_cells.empty() && m_e.op == history::VolumeOp::smooth;
        return [=](const Tri& face, const Eigen::Vector3d& w, const bool forward) -> FacePoint {
            if (barycentric) return {face, w}; // a move the surface map keeps barycentric
            const Rename& to_r = forward ? to_real_b : to_real_a;
            const Rename& from_r = forward ? from_real_a : from_real_b;
            const auto& src = forward ? F->before : F->after;
            const auto& dst = forward ? F->after : F->before;
            const auto [g, wg] = to_r ? to_r(face, w) : FacePoint{face, w};
            const auto* it = VolumeMap::Faces::find(src, g);
            if (!it) throw Fail("cap point on a face that is not tracked");
            Eigen::Vector3d bs;
            for (int k = 0; k < 3; ++k) bs[k] = wg[index_in(g, it->stored[k])];
            const auto [fid2, b2] = cells->map(it->fid, bs, forward);
            const auto* jt = VolumeMap::Faces::by_fid(dst, fid2);
            if (!jt) throw Fail("cap point mapped onto a face not among the tets");
            const Tri g2 = jt->face;
            const Tri& stored2 = jt->stored;
            Eigen::Vector3d w2;
            for (int k = 0; k < 3; ++k) w2[index_in(g2, stored2[k])] = b2[k];
            return from_r ? from_r(g2, w2) : FacePoint{g2, w2};
        };
    }
    static CapFn no_cap()
    {
        return [](const Tri&, const Eigen::Vector3d&, bool) -> FacePoint {
            throw Fail("cap point on a pair with no cap");
        };
    }
    /// The tracked faces the surface map's cells hold, before and after.
    Faces cap_faces(const bool after) const
    {
        const std::set<uint64_t> fids = m_cell_index->fids(after ? 1 : 0);
        Faces out;
        for (const auto& tr : after ? m_faces.after : m_faces.before) {
            if (fids.count(tr.fid)) out.insert(tr.face);
        }
        return out;
    }

    // ------------------------------------------------------------------ virtual vertices
    /// Places each of `W` in `tets` (the decomposition's tets after its collapse) where its tets
    /// are as far from flat as an LP can make them, starting where it is.
    void lp_place(const std::vector<Vid>& W, const Tets& tets, Positions& pos) const
    {
        for (int sweep = 0; sweep < 2; ++sweep) {
            for (const Vid w : W) {
                const Tets ts = with_vertex(tets, w);
                if (ts.empty()) continue;
                std::vector<Eigen::Vector3d> N;
                std::vector<double> b;
                double ell = 0;
                for (const Tet& t : ts) {
                    const auto det_at = [&](const Eigen::Vector3d& x) {
                        Eigen::Vector3d q[4];
                        for (int k = 0; k < 4; ++k) q[k] = t[k] == w ? x : pos[t[k]];
                        return m_sign * (q[1] - q[0]).dot((q[2] - q[0]).cross(q[3] - q[0]));
                    };
                    const double h = det_at(Eigen::Vector3d::Zero());
                    Eigen::Vector3d g;
                    for (int k = 0; k < 3; ++k) g[k] = det_at(Eigen::Vector3d::Unit(k)) - h;
                    const double l = g.norm() + 1e-300;
                    N.push_back(-g / l);
                    b.push_back(h / l);
                    for (const Vid v : t) {
                        if (v != w) ell = std::max(ell, (pos[v] - pos[w]).norm());
                    }
                }
                const auto r = chebyshev(N, b, pos[w], ell, ell);
                if (r && r->second > 0) pos[w] = r->first;
            }
        }
    }
    /// Positions for a decomposition's states: every virtual vertex at its chord's midpoint
    /// once split (an exact subdivision), then where lp_place() puts it after the collapse.
    std::pair<Positions, Positions>
    virtual_states(const Decomposition& d, const Positions& pos, const Vid j) const
    {
        Positions p1 = pos;
        std::vector<Vid> W;
        for (const auto& [w, x] : d.virtuals) {
            make_room(p1, w);
            p1[w] = (pos[j] + pos[x]) / 2;
            W.push_back(w);
        }
        Positions p2 = p1;
        if (!W.empty()) lp_place(W, d.collapsed, p2);
        return {p1, p2};
    }
    /// The positions of a decomposition step, before and after.
    std::pair<const Positions*, const Positions*> state_positions(
        const std::string& kind,
        const Positions& real_b,
        const Positions& real_a,
        const Positions& p1,
        const Positions& p2) const
    {
        if (kind == "split") return {&p1, &p1};
        if (kind.rfind("refined", 0) == 0) return {&p1, &p2};
        if (kind == "unsplit") return {&p2, &p2};
        return {&real_b, &real_a};
    }
    /// Every region, to add_decomposition().
    static constexpr uint32_t kAll = uint32_t(-1);
    /// Adds a decomposition's pairs of region r (the first time each), regions followed through
    /// the steps; r = kAll: all of them.
    void add_decomposition(
        const Decomposition& d,
        const uint32_t r,
        std::vector<char>& added,
        const Positions& real_b,
        const Positions& real_a,
        const Positions& p1,
        const Positions& p2,
        const CapFn& cap,
        const std::string& prefix = "")
    {
        std::map<Tet, uint32_t> region = m_region;
        for (size_t k = 0; k < d.pairs.size(); ++k) {
            const PairSpec& p = d.pairs[k];
            std::optional<uint32_t> rp;
            for (const Tet& t : p.before) {
                const auto it = region.find(sorted(t));
                if (it != region.end()) {
                    rp = it->second;
                    break;
                }
            }
            if (!rp && r != kAll) throw Fail("decomposition: a step in no known region");
            if (rp) {
                for (const Tet& t : p.after) region.emplace(sorted(t), *rp);
            }
            if ((r != kAll && *rp != r) || added[k]) continue;
            added[k] = 1;
            const auto [Pb, Pa] = state_positions(p.kind, real_b, real_a, p1, p2);
            const bool has_cap = !p.cap_before.empty() || !p.cap_after.empty();
            add(prefix + p.kind, p.before, p.after, p.fixed, *Pb, *Pa, has_cap ? cap : no_cap());
        }
    }

    /// Drops the pairs added so far: the operation goes through a decomposition as a whole.
    void drop_pairs()
    {
        for (const auto& p : m_op->pairs) {
            if (dynamic_cast<const RadialPair*>(p.get())) {
                --(p->real ? m_stats.real_pairs : m_stats.fallback_pairs);
            }
        }
        m_op->pairs.clear();
    }
    /// Whether every tracked face the surface map's cells map before the operation is on the
    /// boundary of a patch a pair maps (it, or a piece of it a split made): else the map does not
    /// keep the surface there.
    bool keeps_surface() const
    {
        for (const Tri& f : cap_faces(false)) {
            bool found = false;
            for (const auto& p : m_op->pairs) {
                const auto* rp = dynamic_cast<const RadialPair*>(p.get());
                if (!rp) continue;
                for (const Tri& g : rp->Sb->faces()) {
                    int common = 0, virtual_corners = 0;
                    for (const Vid v : g) {
                        common += contains(f, v);
                        virtual_corners += v >= m_n;
                    }
                    found = found || common == 3 || (common == 2 && virtual_corners == 1);
                }
            }
            if (!found) return false;
        }
        return true;
    }
    /// One pair over all the tets the operation changes, every face of the patch kept: a
    /// bijection of the volume that leaves the tracked surface to the radial map. The removed
    /// vertex's whole star for a collapse (its boundary, the link, stays), the swap's tets, the
    /// moved vertex's star.
    bool volume_only()
    {
        Tets B, A;
        if (m_e.op == history::VolumeOp::collapse) {
            const Vid i = m_e.op_vertices[0], j = m_e.op_vertices[1];
            B = with_vertex(m_K, i);
            const auto c = collapse(B, i, j);
            if (!c) return false;
            A = *c;
        } else if (m_e.op == history::VolumeOp::smooth) {
            B = A = with_vertex(m_K, m_e.op_vertices[0]);
        } else {
            const std::set<Tet> kb = keys(m_K), ka = keys(m_A);
            for (const Tet& t : m_K) {
                if (!ka.count(sorted(t))) B.push_back(t);
            }
            for (const Tet& t : m_A) {
                if (!kb.count(sorted(t))) A.push_back(t);
            }
        }
        const auto p = make_pair("volume only", B, A, {}, {});
        if (!p) return false;
        try {
            add("volume only", B, A, p->fixed, m_Pb, m_Pa, no_cap());
        } catch (const Fail&) {
            return false;
        }
        return true;
    }

    // ------------------------------------------------------------------ collapse
    void build_collapse()
    {
        if (m_e.op_vertices.size() != 2) throw Fail("collapse: not two vertices");
        const Vid i = m_e.op_vertices[0], j = m_e.op_vertices[1];
        const auto direct = collapse(m_K, i, j);
        if (!direct || keys(*direct) != keys(m_A)) throw Fail("collapse: the tets do not match it");
        const volume::Faces sa = surf_after(m_surf_b, i, j);
        const volume::Faces capB = cap_faces(false), capA = cap_faces(true);
        const bool j_on = std::any_of(m_surf_b.begin(), m_surf_b.end(), [j](const Tri& f) {
            return contains(f, j);
        });
        if (!j_on) {
            build_collapse_off(i, j, sa, capB, capA);
            return;
        }
        volume::Faces U, F;
        for (const Tri& f : m_surf_b) {
            if (contains(f, i)) U.insert(f);
        }
        for (const Tri& f : sa) {
            if (!m_surf_b.count(f)) F.insert(f);
        }
        const bool star_cells = capB == U && capA == F;
        const Positions &Pb = m_Pb, &Pa = m_Pa;
        const CapFn cell = cell_cap();
        std::optional<Decomposition> dec;
        std::vector<char> added;
        Positions p1, p2;
        const std::vector<Tets> i_sides = sides(m_K, i, m_surf_b);
        // Each side in a region of its own, as between simwild's tags: a side the decomposition
        // maps takes its region's steps. Else (one region all round, as in tetwild) a side that
        // needs it sends the whole collapse through it.
        std::set<uint32_t> side_regions;
        for (const Tets& M : i_sides) {
            if (const auto r = region_of(M)) side_regions.insert(*r);
        }
        const bool distinct = side_regions.size() == i_sides.size();
        for (const Tets& M : i_sides) {
            const auto r = region_of(M);
            if (!r) throw Fail("collapse: a side in no region");
            const std::set<Tet> km = keys(M);
            std::vector<Tets> Ns;
            for (const Tets& N : sides(m_K, j, m_surf_b)) {
                const std::set<Tet> kn = keys(N);
                if (std::any_of(km.begin(), km.end(), [&](const Tet& t) { return kn.count(t); })) {
                    Ns.push_back(N);
                }
            }
            const char cls = side_class(M, i, j);
            const bool vanishing =
                cls == 'b' && !Ns.empty() && std::all_of(km.begin(), km.end(), [&](const Tet& t) {
                    return keys(Ns[0]).count(t) > 0;
                });
            struct Candidate
            {
                std::string kind;
                Tets before;
                bool real_only;
            };
            std::vector<Candidate> candidates;
            if (star_cells) {
                if (cls == 'a') candidates.push_back({"small", M, true});
                if (vanishing) candidates.push_back({"vanishing side", Ns[0], true});
            }
            if (!Ns.empty()) candidates.push_back({"single", merged(M, Ns[0]), true});
            if (star_cells) {
                if (cls == 'a') candidates.push_back({"small", M, false});
                if (vanishing) candidates.push_back({"vanishing side", Ns[0], false});
            } else if (!Ns.empty()) {
                candidates.push_back({"single", merged(M, Ns[0]), false});
            }
            bool done = false;
            for (const Candidate& c : candidates) {
                std::optional<Tets> Na;
                if (c.kind == "vanishing side") {
                    Tets a;
                    for (const Tet& t : c.before) {
                        if (!contains(t, i)) a.push_back(t);
                    }
                    Na = a;
                } else {
                    Na = collapse(c.before, i, j);
                }
                if (!Na) continue;
                if (!make_pair(c.kind, c.before, *Na, m_surf_b, sa)) continue;
                const volume::Faces Bb = *boundary(c.before), Ba = *boundary(*Na);
                const volume::Faces cb = intersect(Bb, capB), ca = intersect(Ba, capA);
                if (minus(Bb, cb) != minus(Ba, ca)) continue;
                if (c.real_only && !real_ok(c.before, *Na, Pb, Pa)) continue;
                try {
                    add(c.kind, c.before, *Na, minus(Bb, cb), Pb, Pa, cell);
                } catch (const Fail&) {
                    continue;
                }
                done = true;
                break;
            }
            if (done) continue;
            if (!star_cells) throw Fail("collapse: a side no pair over both stars maps");
            if (!dec) {
                std::string why;
                dec = decompose_collapse(m_K, i, j, m_surf_b, m_n, &why);
                if (!dec) throw Fail("collapse decomposition: " + why);
                std::tie(p1, p2) = virtual_states(*dec, Pb, j);
                added.assign(dec->pairs.size(), 0);
                ++m_stats.decompositions;
            }
            if (!distinct) {
                drop_pairs();
                add_decomposition(*dec, kAll, added, Pb, Pa, p1, p2, cell);
                return;
            }
            add_decomposition(*dec, *r, added, Pb, Pa, p1, p2, cell);
        }
    }

    void build_collapse_off(
        const Vid i,
        const Vid j,
        const volume::Faces& sa,
        const volume::Faces& capB,
        const volume::Faces& capA)
    {
        const Positions &Pb = m_Pb, &Pa = m_Pa;
        struct Built
        {
            Tets before, after;
            volume::Faces fixed;
        };
        std::vector<Built> built;
        for (const Tets& M : sides(m_K, i, m_surf_b)) {
            const std::set<Tet> km = keys(M);
            Tets Nb = M;
            for (const Tets& N : sides(m_K, j, m_surf_b)) {
                const std::set<Tet> kn = keys(N);
                if (std::any_of(km.begin(), km.end(), [&](const Tet& t) { return kn.count(t); })) {
                    Nb = merged(M, N);
                    break;
                }
            }
            const auto Na = collapse(Nb, i, j);
            bool ok = Na && make_pair("single", Nb, *Na, m_surf_b, sa);
            if (ok) {
                const volume::Faces Bb = *boundary(Nb), Ba = *boundary(*Na);
                const volume::Faces cb = intersect(Bb, capB), ca = intersect(Ba, capA);
                ok = minus(Bb, cb) == minus(Ba, ca) && real_ok(Nb, *Na, Pb, Pa);
                if (ok) built.push_back({Nb, *Na, minus(Bb, cb)});
            }
            if (!ok) {
                build_reverse(i, j);
                return;
            }
        }
        const CapFn cell = cell_cap();
        for (const Built& b : built) add("single", b.before, b.after, b.fixed, Pb, Pa, cell);
    }

    /// j off the tracked surface: as complexes, collapsing i into j is collapsing j into i with
    /// the survivor named j. So: an interior collapse of j into i in j's region (no tracked face
    /// changes), then a move of i to where j was (the surface map's cells).
    void build_reverse(const Vid i, const Vid j)
    {
        const Positions &Pb = m_Pb, &Pa = m_Pa;
        const Tets Tj = with_vertex(m_K, j);
        Tets Ta;
        for (Tet t : Tj) {
            if (contains(t, i)) continue;
            std::replace(t.begin(), t.end(), j, i);
            Ta.push_back(t);
        }
        const auto p = make_pair("interior j->i", Tj, Ta, m_surf_b, m_surf_b);
        if (!p) throw Fail("collapse (reverse): the interior collapse is not a pair");
        Positions mid = Pb;
        mid[i] = Pa[j];
        add("interior j->i", Tj, Ta, p->fixed, Pb, mid, no_cap());
        const auto K2 = collapse(m_K, j, i);
        if (!K2) throw Fail("collapse (reverse): the collapse makes a tet twice");
        Positions after = Pa;
        after[i] = Pa[j];
        m_op->rename[j] = i;
        const auto rename_to = [](const Vid from, const Vid to) {
            return [from, to](const Tri& f, const Eigen::Vector3d& w) -> FacePoint {
                Tri g = f;
                std::replace(g.begin(), g.end(), from, to);
                const Tri gs = sorted(g);
                Eigen::Vector3d ws;
                for (int k = 0; k < 3; ++k) ws[index_in(gs, g[k])] = w[k];
                return {gs, ws};
            };
        };
        const CapFn cap = cell_cap(nullptr, nullptr, rename_to(i, j), rename_to(j, i));
        for (const Tets& M : sides(*K2, i, m_surf_b)) {
            volume::Faces caps;
            const volume::Faces Bm = *boundary(M);
            for (const Tri& f : Bm) {
                if (contains(f, i) && m_surf_b.count(f)) caps.insert(f);
            }
            add("move i", M, M, minus(Bm, caps), mid, after, cap);
        }
    }

    // ------------------------------------------------------------------ smoothing
    void build_move()
    {
        if (m_e.op_vertices.empty()) throw Fail("move: no vertex");
        const Vid v = m_e.op_vertices[0];
        const CapFn cap = cell_cap();
        for (const Tets& M : sides(with_vertex(m_K, v), v, m_surf_b)) {
            volume::Faces caps;
            const volume::Faces Bm = *boundary(M);
            for (const Tri& f : Bm) {
                if (contains(f, v) && m_surf_b.count(f)) caps.insert(f);
            }
            if (caps.empty()) continue;
            add("move", M, M, minus(Bm, caps), m_Pb, m_Pa, cap);
        }
    }

    // ------------------------------------------------------------------ swaps
    /// Region by region: the region's tets the swap removes onto the ones it adds, one pair;
    /// else the swap as an exact split of the shared edge (face) at a point m, then m collapsed
    /// into the apex whose fan the swap makes -- the collapse decomposed as above, its cap map
    /// the cell after the split's inverse.
    void build_swap()
    {
        const std::set<Tet> kb = keys(m_K), ka = keys(m_A);
        Tets removed, added;
        for (const Tet& t : m_K) {
            if (!ka.count(sorted(t))) removed.push_back(t);
        }
        for (const Tet& t : m_A) {
            if (!kb.count(sorted(t))) added.push_back(t);
        }
        // The sides of the tracked surface the swap changes: its tets before (after) joined
        // across faces that are not tracked, each side before matched with the side after that
        // keeps the same untracked faces.
        const std::vector<Tets> before_sides = side_groups(removed, m_surf_b);
        const std::vector<Tets> after_sides = side_groups(added, m_surf_a);
        std::vector<std::pair<Tets, Tets>> sides;
        bool matched = before_sides.size() == after_sides.size();
        for (const Tets& gb : before_sides) {
            const auto Bb = boundary(gb);
            bool found = false;
            for (const Tets& ga : after_sides) {
                const auto Ba = boundary(ga);
                if (Bb && Ba && minus(*Bb, m_surf_b) == minus(*Ba, m_surf_a)) {
                    sides.emplace_back(gb, ga);
                    found = true;
                    break;
                }
            }
            matched = matched && found;
        }
        std::set<uint32_t> side_regions;
        for (const auto& [gb, ga] : sides) side_regions.insert(m_region.at(sorted(gb[0])));
        const bool distinct = matched && side_regions.size() == sides.size();
        const Positions &Pb = m_Pb, &Pa = m_Pa;
        std::vector<size_t> undone;
        for (size_t k = 0; matched && k < sides.size(); ++k) {
            const auto& [Tb, Ta] = sides[k];
            if (const auto p = make_pair("swap side", Tb, Ta, m_surf_b, m_surf_a)) {
                if (real_ok(Tb, Ta, Pb, Pa)) {
                    try {
                        add("swap side", Tb, Ta, p->fixed, Pb, Pa, cell_cap());
                        continue;
                    } catch (const Fail&) {
                    }
                }
            }
            undone.push_back(k);
        }
        if (matched && undone.empty()) return;
        Decomposition dec;
        Positions pos, p1, p2;
        CapFn dec_cap;
        std::shared_ptr<SplitPair> split_all;
        swap_decomposition(removed, Pb, dec, pos, p1, p2, dec_cap, split_all);
        std::vector<char> dec_added(dec.pairs.size(), 0);
        ++m_stats.decompositions;
        if (!distinct) {
            // one region on both sides (tetwild), or sides that do not match: the whole swap
            drop_pairs();
            split_all->kind = "swap split";
            m_op->pairs.push_back(std::make_unique<SplitPair>(*split_all));
            add_decomposition(dec, kAll, dec_added, pos, Pa, p1, p2, dec_cap, "swap ");
            return;
        }
        for (const size_t k : undone) {
            const Tets& Tb = sides[k].first;
            const uint32_t r = m_region.at(sorted(Tb[0]));
            auto sp = std::make_unique<SplitPair>();
            sp->kind = "swap split";
            sp->common = split_all->common;
            sp->m = split_all->m;
            for (const Tet& t : Tb) {
                const Tet key = sorted(t);
                sp->removed[key] = split_all->removed.at(key);
                for (const auto& [sk, st] : split_all->split) {
                    std::set<Vid> a(st.begin(), st.end());
                    a.erase(sp->m);
                    std::set<Vid> b(t.begin(), t.end());
                    if (std::includes(b.begin(), b.end(), a.begin(), a.end())) sp->split[sk] = st;
                }
            }
            m_op->pairs.push_back(std::move(sp));
            add_decomposition(dec, r, dec_added, pos, Pa, p1, p2, dec_cap, "swap ");
        }
    }

    void swap_decomposition(
        const Tets& removed,
        const Positions& Pb,
        Decomposition& dec,
        Positions& pos,
        Positions& p1,
        Positions& p2,
        CapFn& cap,
        std::shared_ptr<SplitPair>& split_all)
    {
        std::set<Vid> common(removed[0].begin(), removed[0].end());
        for (const Tet& t : removed) {
            std::set<Vid> next;
            for (const Vid v : t) {
                if (common.count(v)) next.insert(v);
            }
            common = next;
        }
        if (common.size() != 2 && common.size() != 3) {
            throw Fail("swap: the removed tets do not share an edge or a face");
        }
        const Vid m = m_n;
        pos = Pb;
        make_room(pos, m);
        pos[m] = Eigen::Vector3d::Zero();
        for (const Vid v : common) pos[m] += Pb[v] / double(common.size());
        split_all = std::make_shared<SplitPair>();
        split_all->common.assign(common.begin(), common.end());
        split_all->m = m;
        Tets K1;
        const std::set<Tet> kr = keys(removed);
        for (const Tet& t : m_K) {
            if (!kr.count(sorted(t))) K1.push_back(t);
        }
        for (const Tet& t : removed) {
            split_all->removed[sorted(t)] = t;
            for (const Vid v : common) {
                Tet s = t;
                std::replace(s.begin(), s.end(), v, m);
                split_all->split[sorted(s)] = s;
                K1.push_back(s);
                m_region[sorted(s)] = m_region.at(sorted(t));
            }
        }
        volume::Faces surf1;
        std::map<Tri, Tri> sub_of;
        for (const Tri& f : m_surf_b) {
            const bool has_all =
                std::all_of(common.begin(), common.end(), [&](Vid v) { return contains(f, v); });
            if (!has_all) {
                surf1.insert(f);
                continue;
            }
            for (const Vid v : common) {
                Tri g = f;
                std::replace(g.begin(), g.end(), v, m);
                g = sorted(g);
                surf1.insert(g);
                sub_of[g] = f;
            }
        }
        const std::set<Tet> target = keys(m_A);
        std::set<Vid> ring;
        for (const Tet& t : removed) {
            for (const Vid v : t) {
                if (!common.count(v)) ring.insert(v);
            }
        }
        std::vector<Vid> order;
        for (const Vid c : ring) {
            const bool on = std::any_of(surf1.begin(), surf1.end(), [&](const Tri& f) {
                return contains(f, m) && contains(f, c);
            });
            if (on) order.push_back(c);
        }
        for (const Vid c : ring) {
            if (std::find(order.begin(), order.end(), c) == order.end()) order.push_back(c);
        }
        std::optional<Vid> apex;
        for (const Vid c : order) {
            const auto K2 = collapse(K1, m, c);
            if (K2 && keys(*K2) == target) {
                apex = c;
                break;
            }
        }
        if (!apex) throw Fail("swap: no split and collapse makes it");
        const Tets Kmc = merged(with_vertex(K1, m), with_vertex(K1, *apex));
        std::string why;
        auto d = decompose_collapse(Kmc, m, *apex, surf1, m + 1, &why);
        if (!d) throw Fail("swap decomposition: " + why);
        dec = std::move(*d);
        std::tie(p1, p2) = virtual_states(dec, pos, *apex);
        const std::vector<Vid> cv(common.begin(), common.end());
        // a piece of a split face (holding m) <-> the face it halves
        const Rename to_real_b = [=](const Tri& f, const Eigen::Vector3d& w) -> FacePoint {
            if (!contains(f, m)) return {f, w};
            const Tri g = sub_of.at(f);
            Eigen::Vector3d out = Eigen::Vector3d::Zero();
            int n_in = 0;
            for (const Vid x : cv) n_in += contains(g, x);
            for (int k = 0; k < 3; ++k) {
                if (f[k] == m) {
                    for (const Vid u : cv) {
                        if (contains(g, u)) out[index_in(g, u)] += w[k] / n_in;
                    }
                } else {
                    out[index_in(g, f[k])] += w[k];
                }
            }
            return {g, out};
        };
        const Rename from_real_b = [=](const Tri& g, const Eigen::Vector3d& w) -> FacePoint {
            bool split = false;
            for (const auto& [piece, parent] : sub_of) split = split || parent == g;
            if (!split) return {g, w};
            std::map<Vid, double> lam;
            for (int k = 0; k < 3; ++k) lam[g[k]] = w[k];
            std::vector<Vid> cin;
            for (const Vid u : cv) {
                if (contains(g, u)) cin.push_back(u);
            }
            Vid lo = cin[0];
            for (const Vid u : cin) {
                if (lam[u] < lam[lo]) lo = u;
            }
            Tri f = g;
            std::replace(f.begin(), f.end(), lo, m);
            f = sorted(f);
            std::map<Vid, double> nb;
            for (const Vid v : g) {
                if (v == lo) continue;
                const bool in = std::find(cin.begin(), cin.end(), v) != cin.end();
                nb[v] = lam[v] - (in ? lam[lo] : 0);
            }
            nb[m] = double(cin.size()) * lam[lo];
            return {f, Eigen::Vector3d(nb[f[0]], nb[f[1]], nb[f[2]])};
        };
        cap = cell_cap(to_real_b, from_real_b);
    }

    const history::VolumeEdit& m_e;
    const VolumeMap::Faces& m_faces;
    std::vector<const SurfaceMap::Cell*> m_cells;
    std::shared_ptr<CellIndex> m_cell_index;
    VolumeMapStats& m_stats;
    Vid m_n;
    Tets m_K, m_A;
    std::map<Tet, uint32_t> m_region;
    volume::Faces m_surf_b, m_surf_a;
    Positions m_Pb, m_Pa;
    int m_sign = 1;
    std::unique_ptr<VolumeMap::OpMap> m_op;
};

/// The volume history's surface ids are the recorder's; the saved surface history may number
/// faces differently only before its first recorded edit, which the offset skips.
SurfaceMapOptions volume_options()
{
    SurfaceMapOptions o;
    o.collapse_patch = CollapsePatch::removed_star;
    return o;
}

} // namespace

VolumeMap::VolumeMap(const history::SurfaceHistory& surface, history::VolumeHistory volume)
    : m_volume(std::move(volume))
    , m_surface(surface, volume_options())
{
    history::Replay r(surface);
    size_t applied = 0;
    const auto advance = [&](const size_t to) {
        while (applied < to && applied < surface.edits.size()) r.apply(surface.edits[applied++]);
    };
    using Tracked = Faces::Tracked;
    const auto lookup =
        [&r](const Tets& tets, const std::vector<uint64_t>& sid, std::vector<Tracked>& out) {
            std::map<uint64_t, Vid> local;
            for (Vid v = 0; v < Vid(sid.size()); ++v) {
                if (sid[v] != kNone) local[sid[v]] = v;
            }
            std::set<Tri> seen;
            for (const Tet& t : tets) {
                for (const Tri& f : faces_of(t)) {
                    if (!seen.insert(f).second) continue;
                    if (sid[f[0]] == kNone || sid[f[1]] == kNone || sid[f[2]] == kNone) continue;
                    const uint64_t a = sid[f[0]];
                    if (a >= r.vertex_alive().size()) continue;
                    std::array<uint64_t, 3> want = {{sid[f[0]], sid[f[1]], sid[f[2]]}};
                    std::sort(want.begin(), want.end());
                    for (const uint64_t fid : r.faces_of(a)) {
                        std::array<uint64_t, 3> c = r.faces()[fid];
                        std::array<uint64_t, 3> cs = c;
                        std::sort(cs.begin(), cs.end());
                        if (cs != want) continue;
                        out.push_back(
                            {f, fid, Tri{{local.at(c[0]), local.at(c[1]), local.at(c[2])}}});
                        break;
                    }
                }
            }
        };
    for (const auto& e : m_volume.edits) {
        const size_t first = e.first_edit + m_volume.surface_edit_offset;
        auto f = std::make_unique<Faces>();
        advance(first);
        lookup(e.tets, e.surface_before, f->before);
        advance(first + e.n_edits);
        lookup(e.tets_after(), e.surface_after, f->after);
        for (auto* fs : {&f->before, &f->after}) {
            std::sort(fs->begin(), fs->end(), [](const Tracked& a, const Tracked& b) {
                return a.face < b.face;
            });
            fs->shrink_to_fit();
        }
        m_faces.push_back(std::move(f));
    }
    m_built.assign(m_volume.edits.size(), 0);
}

VolumeMap::~VolumeMap() = default;
VolumeMap::VolumeMap(VolumeMap&&) noexcept = default;

std::optional<uint32_t> VolumeMap::region_of(std::vector<int64_t> tags) const
{
    std::sort(tags.begin(), tags.end());
    tags.erase(std::unique(tags.begin(), tags.end()), tags.end());
    const auto& R = m_volume.regions;
    const auto it = std::find(R.begin(), R.end(), tags);
    if (it == R.end()) return std::nullopt;
    return uint32_t(it - R.begin());
}

std::optional<uint32_t> VolumeMap::region_of_groups(const std::set<std::string>& names) const
{
    if (names == std::set<std::string>{"ambient"}) return region_of({});
    std::vector<int64_t> tags;
    for (const std::string& n : names) {
        const auto it = std::find_if(
            m_volume.tag_names.begin(),
            m_volume.tag_names.end(),
            [&n](const auto& kv) { return kv.second == n; });
        if (it == m_volume.tag_names.end()) return std::nullopt;
        tags.push_back(it->first);
    }
    return region_of(tags);
}

std::unique_ptr<VolumeMap::OpMap> VolumeMap::op(const size_t k) const
{
    if (m_built[k] == 2) return nullptr;
    const auto& e = m_volume.edits[k];
    std::vector<const SurfaceMap::Cell*> cells;
    for (uint64_t d = 0; d < e.n_edits; ++d) {
        for (const auto* c : m_surface.cells_of(e.first_edit + m_volume.surface_edit_offset + d)) {
            cells.push_back(c);
        }
    }
    // the stats count each operation once, however often it is built
    VolumeMapStats scratch;
    VolumeMapStats& stats = m_built[k] ? scratch : m_stats;
    try {
        auto om = Builder(e, *m_faces[k], std::move(cells), stats).build();
        if (!m_built[k]) ++m_stats.built;
        m_built[k] = 1;
        return om;
    } catch (const Fail& f) {
        ++m_stats.failed;
        ++m_stats.failures[f.what()];
        m_built[k] = 2;
        return nullptr;
    }
}

void VolumeMap::build_all() const
{
    for (size_t k = 0; k < m_built.size(); ++k) op(k);
}

std::vector<VolumePoint> VolumeMap::to_output(std::vector<VolumePoint> points) const
{
    return map(std::move(points), true);
}

std::vector<VolumePoint> VolumeMap::to_input(std::vector<VolumePoint> points) const
{
    return map(std::move(points), false);
}

std::vector<VolumePoint> VolumeMap::map(std::vector<VolumePoint> pts, const bool forward) const
{
    if (pts.empty()) return pts;
    // a uniform grid over the points, to find those in an operation's tets
    Eigen::Vector3d lo = pts[0].position, hi = lo;
    for (const auto& p : pts) {
        lo = lo.cwiseMin(p.position);
        hi = hi.cwiseMax(p.position);
    }
    for (const auto& e : m_volume.edits) {
        for (const auto& x : e.positions) {
            lo = lo.cwiseMin(x);
            hi = hi.cwiseMax(x);
        }
    }
    const double h = std::max((hi - lo).norm() / 256, 1e-300);
    struct KeyHash
    {
        size_t operator()(const std::array<int64_t, 3>& k) const
        {
            return size_t(k[0] * 73856093) ^ size_t(k[1] * 19349663) ^ size_t(k[2] * 83492791);
        }
    };
    const auto key = [&](const Eigen::Vector3d& x) {
        return std::array<int64_t, 3>{
            {int64_t(std::floor((x[0] - lo[0]) / h)),
             int64_t(std::floor((x[1] - lo[1]) / h)),
             int64_t(std::floor((x[2] - lo[2]) / h))}};
    };
    std::unordered_map<std::array<int64_t, 3>, std::vector<size_t>, KeyHash> grid;
    for (size_t n = 0; n < pts.size(); ++n) grid[key(pts[n].position)].push_back(n);
    const auto move_point = [&](const size_t n, const Eigen::Vector3d& to) {
        auto& cell = grid[key(pts[n].position)];
        cell.erase(std::find(cell.begin(), cell.end(), n));
        pts[n].position = to;
        grid[key(to)].push_back(n);
    };

    const size_t n_edits = m_volume.edits.size();
    for (size_t step = 0; step < n_edits; ++step) {
        const size_t k = forward ? step : n_edits - 1 - step;
        const auto& e = m_volume.edits[k];
        std::vector<uint32_t> regions_after;
        const Tets tets_after = e.tets_after(&regions_after);
        const Positions P_after = e.positions_after();
        const Tets& T1 = forward ? e.tets : tets_after;
        const std::vector<uint32_t>& R1 = forward ? e.regions : regions_after;
        const Positions& X1 = forward ? e.positions : P_after;
        const Tets& T2 = forward ? tets_after : e.tets;
        const std::vector<uint32_t>& R2 = forward ? regions_after : e.regions;
        const Positions& X2 = forward ? P_after : e.positions;
        Eigen::Vector3d blo = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
        Eigen::Vector3d bhi = -blo;
        for (const Tet& t : T1) {
            for (const Vid v : t) {
                blo = blo.cwiseMin(X1[v]);
                bhi = bhi.cwiseMax(X1[v]);
            }
        }
        const auto k0 = key(blo), k1 = key(bhi);
        std::vector<size_t> cand;
        for (int64_t a = k0[0]; a <= k1[0]; ++a) {
            for (int64_t b = k0[1]; b <= k1[1]; ++b) {
                for (int64_t c = k0[2]; c <= k1[2]; ++c) {
                    const auto it = grid.find({{a, b, c}});
                    if (it == grid.end()) continue;
                    for (const size_t n : it->second) {
                        const auto& x = pts[n].position;
                        if ((x.array() >= blo.array()).all() && (x.array() <= bhi.array()).all()) {
                            cand.push_back(n);
                        }
                    }
                }
            }
        }
        if (cand.empty()) continue;
        std::map<Tet, uint32_t> region_after;
        for (size_t t = 0; t < T2.size(); ++t) region_after[sorted(T2[t])] = R2[t];
        std::unique_ptr<OpMap> om;
        bool tried = false;
        for (const size_t n : cand) {
            // the tet of the point's region holding it
            const bool any = pts[n].region == VolumePoint::kAnyRegion;
            const auto loc = locate_in(T1, X1, pts[n].position, [&](size_t t) {
                return any || R1[t] == pts[n].region;
            });
            if (!loc) continue;
            const size_t bt = loc->tet;
            const double best = loc->min_coordinate;
            const Eigen::Vector4d bb = loc->b;
            if (!(best >= -1e-9)) continue; // not in the operation's tets
            if (any) pts[n].region = R1[bt];
            if (!tried) {
                om = op(k);
                tried = true;
            }
            if (!om) {
                ++m_stats.through_failed;
                continue;
            }
            Tet t = T1[bt];
            Eigen::Vector4d b = bb.cwiseMax(0.);
            b /= b.sum();
            const Tet t0 = t;
            bool mapped = false;
            try {
                mapped = om->apply(t, b, forward);
            } catch (const Fail&) {
                ++m_stats.through_failed;
                continue;
            }
            const auto it = region_after.find(sorted(t));
            if (it == region_after.end()) {
                ++m_stats.through_failed;
                continue;
            }
            if (it->second != pts[n].region) ++m_stats.region_changed;
            // in a tet no pair maps and whose corners stay: the point stays, exactly
            if (!mapped && t == t0 &&
                std::all_of(t.begin(), t.end(), [&](Vid v) { return X1[v] == X2[v]; })) {
                continue;
            }
            Eigen::Vector3d y = Eigen::Vector3d::Zero();
            for (int c = 0; c < 4; ++c) y += b[c] * X2[t[c]];
            move_point(n, y);
        }
    }
    return pts;
}

} // namespace wmtk::components::tracking
