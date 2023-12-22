#pragma once

#include <wmtk/Accessor.hpp>
#include <wmtk/Tuple.hpp>
#include <wmtk/invariants/InvariantCollection.hpp>


namespace wmtk {
class Mesh;

namespace operations {


// namespace utils {
// class MultiMeshEdgeSplitFunctor;
// class MultiMeshEdgeCollapseFunctor;

// } // namespace utils

class Operation
{
public:
    // friend class utils::MultiMeshEdgeSplitFunctor;
    // friend class utils::MultiMeshEdgeCollapseFunctor;

    Operation(Mesh& mesh);
    virtual ~Operation();

    // main entry point of the operator by the scheduler
    std::vector<Simplex> operator()(const Simplex& simplex);

    // add lambda
    virtual std::vector<double> priority(const Simplex&) const { return {0}; }

    virtual PrimitiveType primitive_type() const = 0;

    const Mesh& mesh() const { return m_mesh; }
    Mesh& mesh() { return m_mesh; }

    void add_invariant(std::shared_ptr<Invariant> invariant) { m_invariants.add(invariant); }

protected:
    /**
     * @brief returns an empty vector in case of failure
     */
    virtual std::vector<Simplex> execute(const Simplex& simplex) = 0;

    /**
     * Returns all simplices that will be potentially affected by the operation
     */
    virtual std::vector<Simplex> unmodified_primitives(const Simplex& simplex) const = 0;

    // does invariant pre-checks
    virtual bool before(const Simplex& simplex) const;
    // does invariant pre-checks
    virtual bool after(const std::vector<Simplex>& unmods, const std::vector<Simplex>& mods) const;

    /// @brief utility for subclasses
    /// @param cells
    void update_cell_hashes(const std::vector<Tuple>& cells);

    /// @brief utility for subclasses
    /// @param tuple
    Tuple resurrect_tuple(const Tuple& tuple) const;

    /// @brief utility for subclasses
    Accessor<long> hash_accessor();
    /// @brief utility for subclasses
    ConstAccessor<long> hash_accessor() const;


private:
    Mesh& m_mesh;
    InvariantCollection m_invariants;
};

} // namespace operations
} // namespace wmtk
