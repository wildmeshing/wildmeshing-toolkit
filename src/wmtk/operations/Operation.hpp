#pragma once

#include "attribute_new/NewAttributeStrategy.hpp"
#include "attribute_update/AttributeTransferStrategyBase.hpp"
#include "attribute_update/TopologicalTransferStrategy.hpp"

#include <wmtk/Tuple.hpp>
#include <wmtk/attribute/Accessor.hpp>
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
    virtual std::vector<simplex::Simplex> operator()(const simplex::Simplex& simplex);

    virtual double priority(const simplex::Simplex& simplex) const
    {
        return m_priority == nullptr ? 0 : m_priority(simplex);
    }

    virtual bool use_random_priority() const { return m_use_random_priority; }
    virtual bool& use_random_priority() { return m_use_random_priority; }

    virtual PrimitiveType primitive_type() const = 0;

    const Mesh& mesh() const { return m_mesh; }
    Mesh& mesh() { return m_mesh; }

    void add_invariant(std::shared_ptr<Invariant> invariant) { m_invariants.add(invariant); }

    void set_priority(const std::function<double(const simplex::Simplex&)>& func)
    {
        m_priority = func;
    }

    std::shared_ptr<const operations::AttributeTransferStrategyBase> get_transfer_strategy(
        const attribute::MeshAttributeHandle& attribute);


    void add_transfer_strategy(
        const std::shared_ptr<const operations::AttributeTransferStrategyBase>& other);

    void add_topology_transfer_strategy(
        const std::shared_ptr<const operations::TopologicalTransferStrategy>& other);

    void set_transfer_strategy(
        const attribute::MeshAttributeHandle& attribute,
        const std::shared_ptr<const operations::AttributeTransferStrategyBase>& other);

    void clear_attribute_transfer_strategies();

    virtual void reserve_enough_simplices();

protected:
    /**
     * @brief returns an empty vector in case of failure
     */
    virtual std::vector<simplex::Simplex> execute(const simplex::Simplex& simplex) = 0;

    /**
     * Returns all simplices that will be potentially affected by the operation
     */
    virtual std::vector<simplex::Simplex> unmodified_primitives(
        const simplex::Simplex& simplex) const = 0;

    // does invariant pre-checks
    virtual bool before(const simplex::Simplex& simplex) const;
    // does invariant pre-checks
    virtual bool after(
        const std::vector<simplex::Simplex>& unmods,
        const std::vector<simplex::Simplex>& mods) const;


    void apply_attribute_transfer(const std::vector<simplex::Simplex>& direct_mods);


private:
    Mesh& m_mesh;
    bool m_use_random_priority = false;


protected:
    std::function<double(const simplex::Simplex&)> m_priority = nullptr;

    invariants::InvariantCollection m_invariants;

    std::vector<std::shared_ptr<const operations::AttributeTransferStrategyBase>>
        m_attr_transfer_strategies;

    std::vector<std::shared_ptr<const operations::TopologicalTransferStrategy>>
        m_topology_transfer_strategies;
};

} // namespace operations
} // namespace wmtk
