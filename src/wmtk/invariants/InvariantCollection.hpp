#pragma once
#include <map>
#include <memory>
#include <vector>
#include "Invariant.hpp"


namespace wmtk {
namespace simplex {
class Simplex;
}

namespace invariants {

class InvariantCollection : public Invariant
{
public:
    InvariantCollection(const Mesh& m);
    InvariantCollection(const InvariantCollection&);
    InvariantCollection(InvariantCollection&&);
    InvariantCollection& operator=(const InvariantCollection&);
    InvariantCollection& operator=(InvariantCollection&&);
    ~InvariantCollection();
    bool before(const simplex::Simplex& t) const override;
    bool after(
        const std::vector<Tuple>& top_dimension_tuples_before,
        const std::vector<Tuple>& top_dimension_tuples_after) const override;


    bool directly_modified_after(
        const std::vector<simplex::Simplex>& simplices_before,
        const std::vector<simplex::Simplex>& simplices_after) const final override;

    // optimization for evaluating connected subgraphs of invariants that share the same mesh
    // In this case we can cache the cofaces computed once rather than re-evaluate them
    bool directly_modified_after_cached(
        const std::vector<simplex::Simplex>& simplices_before,
        const std::vector<simplex::Simplex>& simplices_after,
        std::vector<Tuple>& cofaces_before,
        std::vector<Tuple>& cofaces_after) const;

    bool is_collection() const final override;
    // pass by value so this can be internally moved
    void add(std::shared_ptr<Invariant> invariant);

    const std::shared_ptr<Invariant>& get(int64_t index) const;
    int64_t size() const;
    bool empty() const;
    const std::vector<std::shared_ptr<Invariant>>& invariants() const;

    [[noreturn]] std::map<Mesh const*, std::vector<std::shared_ptr<Invariant>>>
    get_map_mesh_to_invariants();

private:
    std::vector<std::shared_ptr<Invariant>> m_invariants;
    std::vector<std::shared_ptr<Invariant>> m_same_mesh_invariants;
    bool m_use_same_mesh_caching = false;
};

} // namespace invariants

class Mesh;
} // namespace wmtk
