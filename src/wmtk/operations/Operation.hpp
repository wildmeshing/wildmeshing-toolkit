#pragma once

#include "attribute_new/NewAttributeStrategy.hpp"
#include "attribute_update/AttributeTransferStrategyBase.hpp"

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
    std::vector<simplex::Simplex> operator()(const simplex::Simplex& simplex);

    virtual std::vector<double> priority(const simplex::Simplex& simplex) const
    {
        return m_priority == nullptr ? std::vector<double>({0}) : m_priority(simplex);
    }

    bool use_random_priority() const { return m_use_random_priority; }
    bool& use_random_priority() { return m_use_random_priority; }

    virtual PrimitiveType primitive_type() const = 0;

    const Mesh& mesh() const { return m_mesh; }
    Mesh& mesh() { return m_mesh; }

    void add_invariant(std::shared_ptr<Invariant> invariant) { m_invariants.add(invariant); }

    void set_priority(const std::function<std::vector<double>(const simplex::Simplex&)>& func)
    {
        m_priority = func;
    }


    std::shared_ptr<operations::NewAttributeStrategy> get_new_attribute_strategy(
        const attribute::MeshAttributeHandleVariant& attribute);

    void set_new_attribute_strategy(
        const attribute::MeshAttributeHandleVariant& attribute,
        const std::shared_ptr<operations::NewAttributeStrategy>& other);


    std::shared_ptr<operations::AttributeTransferStrategyBase> get_transfer_strategy(
        const attribute::MeshAttributeHandleVariant& attribute);


    void add_transfer_strategy(
        const std::shared_ptr<operations::AttributeTransferStrategyBase>& other);

    void set_transfer_strategy(
        const attribute::MeshAttributeHandleVariant& attribute,
        const std::shared_ptr<operations::AttributeTransferStrategyBase>& other);

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

    /// @brief utility for subclasses
    /// @param cells
    void update_cell_hashes(const std::vector<Tuple>& cells);

    /// @brief utility for subclasses
    /// @param tuple
    Tuple resurrect_tuple(const Tuple& tuple) const;

    /// @brief utility for subclasses
    Accessor<int64_t> hash_accessor();
    /// @brief utility for subclasses
    ConstAccessor<int64_t> hash_accessor() const;


    void apply_attribute_transfer(const std::vector<simplex::Simplex>& direct_mods);


private:
    Mesh& m_mesh;
    bool m_use_random_priority = false;

    std::function<std::vector<double>(const simplex::Simplex&)> m_priority = nullptr;

protected:
    InvariantCollection m_invariants;

    std::vector<std::shared_ptr<operations::NewAttributeStrategy>> m_new_attr_strategies;
    std::vector<std::shared_ptr<operations::AttributeTransferStrategyBase>>
        m_attr_transfer_strategies;
};

} // namespace operations
} // namespace wmtk
