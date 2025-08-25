#pragma once
#ifdef WMTK_RECORD_OPERATIONS
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <nlohmann/json.hpp>

extern long succ_operations_count;
extern std::string OperationLogPath;
extern std::string OperationLogPrefix;

// Batch logging system - collect operations in batches to reduce file count
constexpr int OPERATIONS_PER_BATCH = 1000;
extern std::vector<nlohmann::json> operation_batch;
extern int current_batch_number;

std::string generatePathNameWithCurrentTime();
std::string generatePathNameWithModelName(const std::string& model_name);

// Batch logging functions
void addOperationToBatch(const nlohmann::json& operation_log);
void flushCurrentBatch();
void initializeBatchLogging();
void finalizeBatchLogging(); // Call at program end to flush remaining operations
#endif
