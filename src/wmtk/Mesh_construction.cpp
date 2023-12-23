
#include <numeric>
#include "Mesh.hpp"

#include <wmtk/SimplicialComplex.hpp>
#include <wmtk/operations/CollapseNewAttributeStrategy.hpp>
#include <wmtk/operations/SplitNewAttributeStrategy.hpp>
#include <wmtk/utils/Logger.hpp>

#include "Primitive.hpp"

namespace wmtk {

Mesh::Mesh(Mesh&& other) = default;
Mesh::Mesh(const Mesh& other) = default;
Mesh& Mesh::operator=(const Mesh& other) = default;
Mesh& Mesh::operator=(Mesh&& other) = default;
Mesh::Mesh(const long& dimension)
    : Mesh(dimension, dimension, get_primitive_type_from_id(dimension))
{}

Mesh::Mesh(const long& dimension, const long& max_primitive_type_id, PrimitiveType hash_type)
    : m_attribute_manager(max_primitive_type_id + 1)
    , m_cell_hash_handle(register_attribute_nomesh<long>("hash", hash_type, 1))
{
    m_flag_handles.reserve(max_primitive_type_id + 1);
    for (long j = 0; j <= max_primitive_type_id; ++j) {
        m_flag_handles.emplace_back(
            register_attribute_nomesh<char>("flags", get_primitive_type_from_id(j), 1));
    }
}
void Mesh::fix_op_handles()
{
    for (auto& ptr : m_split_strategies) {
        ptr->update_handle_mesh(*this);
    }
    for (auto& ptr : m_collapse_strategies) {
        ptr->update_handle_mesh(*this);
    }
}

Mesh::~Mesh() = default;
} // namespace wmtk
