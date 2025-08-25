#include "batch_operation_log_reader.hpp"
#include <algorithm>
#include <regex>

BatchOperationLogReader::BatchOperationLogReader(const fs::path& logs_dir)
    : logs_directory(logs_dir), batch_format(false), legacy_max_index(0),
      total_operations(0), total_operations_computed(false) {
    discover_files();
}

void BatchOperationLogReader::discover_files() {
    if (!fs::exists(logs_directory) || !fs::is_directory(logs_directory)) {
        std::cerr << "Warning: Operation logs directory not found: " << logs_directory << std::endl;
        return;
    }

    std::vector<fs::path> batch_candidates;
    std::vector<fs::path> legacy_candidates;
    
    // Scan directory for both formats
    std::regex batch_pattern(R"(operation_log_batch_(\d+)\.json)");
    std::regex legacy_pattern(R"(operation_log_(\d+)\.json)");
    
    for (const auto& entry : fs::directory_iterator(logs_directory)) {
        if (!entry.is_regular_file()) continue;
        
        std::string filename = entry.path().filename().string();
        std::smatch match;
        
        if (std::regex_match(filename, match, batch_pattern)) {
            batch_candidates.push_back(entry.path());
        } else if (std::regex_match(filename, match, legacy_pattern)) {
            legacy_candidates.push_back(entry.path());
        }
    }
    
    // Prioritize batch format if both exist
    if (!batch_candidates.empty()) {
        batch_format = true;
        batch_files = std::move(batch_candidates);
        
        // Sort batch files by batch number
        std::sort(batch_files.begin(), batch_files.end(), [](const fs::path& a, const fs::path& b) {
            std::regex pattern(R"(operation_log_batch_(\d+)\.json)");
            std::smatch match_a, match_b;
            std::string name_a = a.filename().string();
            std::string name_b = b.filename().string();
            
            if (std::regex_match(name_a, match_a, pattern) && 
                std::regex_match(name_b, match_b, pattern)) {
                return std::stoi(match_a[1].str()) < std::stoi(match_b[1].str());
            }
            return name_a < name_b;
        });
        
        cached_batches.resize(batch_files.size());
        batch_loaded.resize(batch_files.size(), false);
        
        std::cout << "Detected batch format with " << batch_files.size() << " batch files" << std::endl;
        
    } else if (!legacy_candidates.empty()) {
        batch_format = false;
        
        // Find max index for legacy format
        legacy_max_index = 0;
        std::regex pattern(R"(operation_log_(\d+)\.json)");
        for (const auto& file : legacy_candidates) {
            std::smatch match;
            std::string filename = file.filename().string();
            if (std::regex_match(filename, match, pattern)) {
                size_t index = std::stoull(match[1].str());
                legacy_max_index = std::max(legacy_max_index, index);
            }
        }
        
        std::cout << "Detected legacy format with max index: " << legacy_max_index << std::endl;
        
    } else {
        std::cerr << "Warning: No operation log files found in " << logs_directory << std::endl;
    }
}

size_t BatchOperationLogReader::get_total_operations() const {
    if (!total_operations_computed) {
        total_operations = compute_total_operations();
        total_operations_computed = true;
    }
    return total_operations;
}

size_t BatchOperationLogReader::compute_total_operations() const {
    if (batch_format) {
        size_t total = 0;
        for (size_t i = 0; i < batch_files.size(); ++i) {
            load_batch(i);
            if (cached_batches[i].contains("operation_count")) {
                total += cached_batches[i]["operation_count"].get<size_t>();
            }
        }
        return total;
    } else {
        // For legacy format, we need to count existing files
        size_t count = 0;
        for (size_t i = 0; i <= legacy_max_index; ++i) {
            fs::path file_path = logs_directory / ("operation_log_" + std::to_string(i) + ".json");
            if (fs::exists(file_path)) {
                count++;
            }
        }
        return count;
    }
}

json BatchOperationLogReader::get_operation(size_t index) {
    if (batch_format) {
        // Find which batch contains this operation
        size_t current_offset = 0;
        for (size_t batch_idx = 0; batch_idx < batch_files.size(); ++batch_idx) {
            load_batch(batch_idx);
            
            const auto& batch = cached_batches[batch_idx];
            if (!batch.contains("operations") || !batch.contains("operation_count")) {
                continue;
            }
            
            size_t batch_size = batch["operation_count"].get<size_t>();
            if (index >= current_offset && index < current_offset + batch_size) {
                // Found the right batch
                size_t local_index = index - current_offset;
                const auto& operations = batch["operations"];
                if (local_index < operations.size()) {
                    return operations[local_index];
                }
            }
            current_offset += batch_size;
        }
        return json{}; // Not found
        
    } else {
        // Legacy format - direct file access
        fs::path file_path = logs_directory / ("operation_log_" + std::to_string(index) + ".json");
        if (!fs::exists(file_path)) {
            return json{};
        }
        
        std::ifstream file(file_path);
        if (!file.is_open()) {
            std::cerr << "Failed to open file: " << file_path << std::endl;
            return json{};
        }
        
        json operation_log;
        try {
            file >> operation_log;
        } catch (const json::exception& e) {
            std::cerr << "Failed to parse JSON from file: " << file_path << " - " << e.what() << std::endl;
            return json{};
        }
        
        return operation_log;
    }
}

void BatchOperationLogReader::load_batch(size_t batch_index) const {
    if (batch_index >= batch_files.size() || batch_loaded[batch_index]) {
        return;
    }
    
    const auto& file_path = batch_files[batch_index];
    std::ifstream file(file_path);
    if (!file.is_open()) {
        std::cerr << "Failed to open batch file: " << file_path << std::endl;
        return;
    }
    
    try {
        file >> cached_batches[batch_index];
        batch_loaded[batch_index] = true;
        std::cout << "Loaded batch " << batch_index << " from " << file_path.filename() << std::endl;
    } catch (const json::exception& e) {
        std::cerr << "Failed to parse batch JSON from file: " << file_path << " - " << e.what() << std::endl;
    }
}