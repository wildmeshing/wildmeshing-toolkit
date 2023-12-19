#pragma once

#include "OperationSettingsBase.hpp"

namespace wmtk {
class Mesh;
class InvariantCollection;

namespace operations {

/**
 * Operation settings are implemented as specializations of this template. Any specialization must
 * derive from OperationSettingsBase
 */
template <typename T>
class OperationSettings : public OperationSettingsBase
{
};
} // namespace operations
} // namespace wmtk
