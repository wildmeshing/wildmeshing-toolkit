#pragma once
#include <optional>
#include <wmtk/invariants/InvariantCollection.hpp>
#include <wmtk/operations/TupleOperation.hpp>

#include "Operation.hpp"

namespace wmtk::operations {
class AttributesUpdateBase;

template <>
struct OperationSettings<AttributesUpdateBase> : public OperationSettingsBase
{
    OperationSettings<AttributesUpdateBase>(Mesh& m)
        : m_mesh(m)
    {}

    Mesh& m_mesh;

    void create_invariants();
};

class AttributesUpdateBase : public TupleOperation
{
public:
    AttributesUpdateBase(
        Mesh& m,
        const Simplex& t,
        const OperationSettings<AttributesUpdateBase>& settings);

    std::string name() const override;

    static PrimitiveType primitive_type() { return PrimitiveType::Vertex; }

    const Tuple& return_tuple() const;
    std::vector<Simplex> modified_primitives() const override;
    std::vector<Simplex> unmodified_primitives() const override;

protected:
    bool execute() override;

    Mesh& mesh() { return m_mesh; }
    Mesh& base_mesh() const override { return m_mesh; }
    const Mesh& mesh() const { return m_mesh; }
    Accessor<long>& hash_accessor() override;

protected:
    Tuple m_output_tuple;
    const OperationSettings<AttributesUpdateBase>& m_settings;
    Mesh& m_mesh;
    Accessor<long> m_hash_accessor;
};

} // namespace wmtk::operations
