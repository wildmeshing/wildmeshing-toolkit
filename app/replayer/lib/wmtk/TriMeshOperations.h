
#include <wmtk/utils/TriMeshOperationLogger.h>
auto TriMeshOperation::operator()(TriMesh& m, const Tuple& t) -> ExecuteReturnData
{
    ExecuteReturnData retdata;
    retdata.success = false;

    if (before(m, t)) {
        m.start_protected_connectivity();
        {
#if defined(USE_OPERATION_LOGGER)
            std::shared_ptr<TriMeshOperationRecorder> recorder;

            // If the operation logger exists then log
            if (m.p_operation_logger) {
                auto& wp_op_rec = m.p_operation_recorder.local();
                recorder = m.p_operation_logger->start_ptr(m, name(), t);
                wp_op_rec = recorder;
            }
#endif
            retdata = execute(m, t);

            if (retdata.success) {
#if defined(USE_OPERATION_LOGGER)
                if (recorder != nullptr) {
                    recorder->set_output_tuple(retdata.tuple);
                }
#endif
                m.start_protected_attributes();

                if (!(after(m, retdata) && invariants(m, retdata))) {
                    spdlog::info("Failed after in {}", name());
                    retdata.success = false;
#if defined(USE_OPERATION_LOGGER)
                    if (recorder != nullptr) {
                        recorder->cancel();
                    }
#endif

                    m.rollback_protected_connectivity();
                    m.rollback_protected_attributes();
                }
            } else {
                spdlog::info("Failed execute in {}", name());
            }
        }
        m.release_protected_connectivity();
        m.release_protected_attributes();
    } else {
        spdlog::info("Failed before in {}", name());
    }


    return retdata;
}

#if defined(USE_OPERATION_LOGGER)
std::weak_ptr<OperationRecorder> TriMeshOperation::recorder(TriMesh& m) const
{
    return m.p_operation_recorder.local();
}
#endif
