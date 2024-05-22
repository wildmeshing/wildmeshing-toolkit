#include "Record_Operations.hpp"
#ifdef WMTK_RECORD_OPERATIONS

long succ_operations_count = 0;
std::string OperationLogPath = generatePathNameWithCurrentTime();
std::string OperationLogPrefix = "/operation_log_";

std::string generatePathNameWithCurrentTime()
{
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);

    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "operation_log_%Y-%m-%d_%H-%M-%S");

    if (!std::filesystem::exists(ss.str())) {
        std::cout << "Path does not exist, creating: " << ss.str() << std::endl;

        try {
            if (std::filesystem::create_directories(ss.str())) {
                std::cout << "Path created successfully." << std::endl;
            } else {
                std::cout << "Failed to create path." << std::endl;
            }
        } catch (const std::filesystem::filesystem_error& e) {
            std::cerr << "Error: " << e.what() << std::endl;
        }
    } else {
        std::cout << "Path already exists." << std::endl;
    }

    return ss.str();
}
#endif
