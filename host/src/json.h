// Just enough JSON for the web UI: flat objects in, text out.
#pragma once

#include <map>
#include <string>

struct JsonValue {
    enum Kind { Null, Boolean, Number, String } kind = Null;
    bool boolean = false;
    double number = 0;
    std::string string;
};
using JsonObject = std::map<std::string, JsonValue>;

// Parses one object whose values are strings, numbers, booleans or null.
// Throws std::runtime_error on anything else.
JsonObject parse_json_object(const std::string &text);

std::string json_quote(const std::string &text);  // "text", escaped
std::string json_number(double value);            // finite; null otherwise
