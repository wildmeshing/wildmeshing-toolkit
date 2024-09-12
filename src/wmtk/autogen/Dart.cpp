#include "Dart.hpp"
#include <fmt/format.h>
#include <ostream>
namespace wmtk::autogen {

    Dart::operator std::string() const {
        return fmt::format("Dart[{}:{}]", global_id(),local_orientation());
    }
}
std::ostream& operator<<(std::ostream& out, wmtk::autogen::Dart const& dart) {
    out << std::string(dart);
    return out;
}
