#include "attribute_missing_error.hpp"
#include "named_error_text.hpp"
namespace wmtk::components::multimesh::utils::detail {

std::string attribute_missing_error::make_message(const AttributeDescription& ad)
{
    return "Could not find attribute " + make_named_error_string(ad);
}
} // namespace wmtk::components::multimesh::utils::detail
