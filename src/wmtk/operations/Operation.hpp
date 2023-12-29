#pragma once

#include "AttributeTransferStrategyBase.hpp"
#include "NewAttributeStrategy.hpp"

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

    virtual std::vector<double> priority(const Simplex& simplex) const
    {
        return m_priority == nullptr ? std::vector<double>({0}) : m_priority(simplex);
    }

    virtual PrimitiveType primitive_type() const = 0;

    const Mesh& mesh() const { return m_mesh; }
    Mesh& mesh() { return m_mesh; }

    void add_invariant(std::shared_ptr<Invariant> invariant) { m_invariants.add(invariant); }

    void set_priority(const std::function<std::vector<double>(const Simplex&)>& func)
    {
        m_priority = func;
    }


    // TODO :make this name more descriptive

    std::shared_ptr<operations::NewAttributeStrategy> get_strategy(
        const attribute::MeshAttributeHandleVariant& attribute);

    void set_strategy(
        const attribute::MeshAttributeHandleVariant& attribute,
        const std::shared_ptr<operations::NewAttributeStrategy>& other);

    /**
     * @brief returns an empty vector in case of failure
     */
    virtual std::vector<Simplex> execute(const Simplex& simplex) = 0;

    std::shared_ptr<operations::AttributeTransferStrategyBase> get_transfer_strategy(
        const attribute::MeshAttributeHandleVariant& attribute);


    void add_transfer_strategy(
        const std::shared_ptr<operations::AttributeTransferStrategyBase>& other);

    void set_transfer_strategy(
        const attribute::MeshAttributeHandleVariant& attribute,
        const std::shared_ptr<operations::AttributeTransferStrategyBase>& other);

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


    void apply_attribute_transfer(const std::vector<Simplex>& direct_mods);


private:
    Mesh& m_mesh;

    std::function<std::vector<double>(const Simplex&)> m_priority = nullptr;

protected:
    InvariantCollection m_invariants;

    std::vector<std::shared_ptr<operations::NewAttributeStrategy>> m_new_attr_strategies;
    std::vector<std::shared_ptr<operations::AttributeTransferStrategyBase>>
        m_attr_transfer_strategies;
};

} // namespace operations
} // namespace wmtk
