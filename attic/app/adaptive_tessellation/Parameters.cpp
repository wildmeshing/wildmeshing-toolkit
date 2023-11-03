#include "Parameters.h"

namespace adaptive_tessellation {
void Parameters::log(
    const nlohmann::json& js,
    bool flush) // flush should force file output immediately, but will be slow for
                // per-operation things
    const
{
    ATlogger->info(js.dump());


    if (flush) {
        ATlogger->flush();
    }
}
} // namespace adaptive_tessellation