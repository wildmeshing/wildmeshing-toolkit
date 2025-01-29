#pragma once
#include <cstdint>
#include <wmtk/PrimitiveType.hpp>
#include <wmtk/Tuple.hpp>

namespace wmtk::autogen::utils {
// external access to each mesh type's local id offsets from tuple code
// mostl'y for unit testing
int8_t local_id_table_offset(PrimitiveType mesh_type, const Tuple& t);

} // namespace wmtk::autogen::utils
