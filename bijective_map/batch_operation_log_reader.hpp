#pragma once

#include <nlohmann/json.hpp>
#include <filesystem>
#include <vector>
#include <string>
#include <fstream>
#include <iostream>

using json = nlohmann::json;
namespace fs = std::filesystem;

/**
 * BatchOperationLogReader - Efficiently reads operation logs from batch files
 * 
 * This class handles both legacy single-file format and new batch format:
 * - Legacy: operation_log_0.json, operation_log_1.json, ...
 * - Batch: operation_log_batch_0.json, operation_log_batch_1.json, ...
 * 
 * Features:
 * - Automatic format detection
 * - Caching to avoid re-reading files
 * - Sequential access with get_operation(index)
 * - Total operation count discovery
 */
class BatchOperationLogReader {
public:
    /**
     * Constructor
     * @param logs_dir Path to directory containing operation log files
     */
    explicit BatchOperationLogReader(const fs::path& logs_dir);

    /**
     * Get total number of operations available
     * @return Total operation count across all files
     */
    size_t get_total_operations() const;

    /**
     * Get operation by sequential index
     * @param index Sequential operation index (0-based)
     * @return JSON object containing operation data, or empty JSON if not found
     */
    json get_operation(size_t index);

    /**
     * Check if using batch format (vs legacy single-file format)
     * @return true if batch format detected, false for legacy format
     */
    bool is_batch_format() const { return batch_format; }

    /**
     * Get number of batch files (only relevant for batch format)
     * @return Number of batch files found
     */
    size_t get_batch_count() const { return batch_files.size(); }

private:
    fs::path logs_directory;
    bool batch_format;
    
    // For batch format
    std::vector<fs::path> batch_files;
    mutable std::vector<json> cached_batches;
    mutable std::vector<bool> batch_loaded;
    
    // For legacy format  
    size_t legacy_max_index;
    
    // Common
    mutable size_t total_operations;
    mutable bool total_operations_computed;

    void discover_files();
    void load_batch(size_t batch_index) const;
    size_t compute_total_operations() const;
};