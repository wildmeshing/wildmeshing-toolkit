#include "Record_Operations.hpp"
#ifdef WMTK_RECORD_OPERATIONS

long succ_operations_count = 0;
std::string OperationLogPath = "";
std::string OperationLogPrefix = "/operation_log_";

// Batch logging variables
std::vector<nlohmann::json> operation_batch;
int current_batch_number = 0;

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

std::string generatePathNameWithModelName(const std::string& model_name)
{
    std::stringstream ss;
    ss << "operation_log_" << model_name;

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

void initializeBatchLogging()
{
    operation_batch.clear();
    operation_batch.reserve(OPERATIONS_PER_BATCH);
    current_batch_number = 0;
}

void addOperationToBatch(const nlohmann::json& operation_log)
{
    operation_batch.push_back(operation_log);
    
    // Check if batch is full
    if (operation_batch.size() >= OPERATIONS_PER_BATCH) {
        flushCurrentBatch();
    }
}

void flushCurrentBatch()
{
    if (operation_batch.empty()) return;
    
    std::string filename = OperationLogPath + OperationLogPrefix + "batch_" + 
                          std::to_string(current_batch_number) + ".json";
    
    std::ofstream batch_file(filename);
    if (batch_file.is_open()) {
        nlohmann::json batch_json;
        batch_json["batch_number"] = current_batch_number;
        batch_json["operation_count"] = operation_batch.size();
        batch_json["operations"] = operation_batch;
        
        batch_file << batch_json.dump(4);
        batch_file.close();
        
        std::cout << "Flushed batch " << current_batch_number << " with " 
                  << operation_batch.size() << " operations to " << filename << std::endl;
    } else {
        std::cerr << "Unable to open batch file " << filename << " for writing" << std::endl;
    }
    
    // Clear batch and increment batch number
    operation_batch.clear();
    current_batch_number++;
}

void finalizeBatchLogging()
{
    // Flush any remaining operations
    flushCurrentBatch();
    
    std::cout << "Finalized batch logging. Total batches written: " << current_batch_number << std::endl;
}
#endif
