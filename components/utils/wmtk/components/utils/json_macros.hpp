#pragma once
#include <nlohmann/json_fwd.hpp>


// place these in the class for which serialization is desired
#define WMTK_NLOHMANN_JSON_FRIEND_DECLARATION(Type)\
    friend void to_json(nlohmann::json& nlohmann_json_j, const Type& nlohmann_json_t);\
    friend void from_json(const nlohmann::json& nlohmann_json_j, Type& nlohmann_json_t);

// place these in the class for which serialization is desired
#define WMTK_NLOHMANN_JSON_DECLARATION(Type)\
    void to_json(nlohmann::json& nlohmann_json_j, const Type& nlohmann_json_t);\
    void from_json(const nlohmann::json& nlohmann_json_j, Type& nlohmann_json_t);


// place this to define the prototype of the to_json function
#define WMTK_NLOHMANN_JSON_FRIEND_TO_JSON_PROTOTYPE(Type)\
    void to_json(nlohmann::json& nlohmann_json_j, const Type& nlohmann_json_t)
// place this to define the prototype of the from_json function
#define WMTK_NLOHMANN_JSON_FRIEND_FROM_JSON_PROTOTYPE(Type)\
    void from_json(const nlohmann::json& nlohmann_json_j, Type& nlohmann_json_t)

#define WMTK_NLOHMANN_ASSIGN_TYPE_TO_JSON(...)  \
    { NLOHMANN_JSON_EXPAND(NLOHMANN_JSON_PASTE(NLOHMANN_JSON_TO, __VA_ARGS__)) }

#define WMTK_NLOHMANN_ASSIGN_TYPE_FROM_JSON(...)  \
    { NLOHMANN_JSON_EXPAND(NLOHMANN_JSON_PASTE(NLOHMANN_JSON_FROM, __VA_ARGS__)) }



