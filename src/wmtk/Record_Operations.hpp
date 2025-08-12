#pragma once
#ifdef WMTK_RECORD_OPERATIONS
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

extern long succ_operations_count;
extern std::string OperationLogPath;
extern std::string OperationLogPrefix;

std::string generatePathNameWithCurrentTime();
std::string generatePathNameWithModelName(const std::string& model_name);
#endif
