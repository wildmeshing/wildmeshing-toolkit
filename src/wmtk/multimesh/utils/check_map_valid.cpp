#include <wmtk/Mesh.hpp>
#include <wmtk/multimesh/utils/tuple_map_attribute_io.hpp>
#include <wmtk/simplex/top_dimension_cofaces.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/TupleInspector.hpp>
#include "MapValidator.hpp"
namespace wmtk::multimesh::utils {
bool check_maps_valid(const Mesh& m) {
    
    MapValidator validator(m);
    return validator.check_all();
}
bool check_child_maps_valid(const Mesh& m) {
    MapValidator validator(m);
    return validator.check_child_map_attributes_valid();
}
bool check_parent_map_valid(const Mesh& m) {
    MapValidator validator(m);
    return validator.check_child_map_attributes_valid();
}
}
