// Minimal JSON reader, enough for PlatformIO board manifests.
#pragma once
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace esp32sim {

class Json {
public:
    enum class Type { Null, Bool, Number, String, Array, Object };

    static bool parse(const std::string& text, Json& out, std::string* error = nullptr);

    Type type() const { return type_; }
    bool is_object() const { return type_ == Type::Object; }
    bool is_array() const { return type_ == Type::Array; }

    // Lookups return a shared null value when missing / wrong type.
    const Json& operator[](const std::string& key) const;
    const Json& operator[](size_t index) const;
    size_t size() const;

    std::string str(const std::string& fallback = "") const;
    double num(double fallback = 0) const;
    bool boolean(bool fallback = false) const;
    const std::map<std::string, Json>& object() const { return obj_; }
    const std::vector<Json>& array() const { return arr_; }

private:
    friend class JsonParser;
    Type type_ = Type::Null;
    bool b_ = false;
    double n_ = 0;
    std::string s_;
    std::vector<Json> arr_;
    std::map<std::string, Json> obj_;
};

std::string json_escape(const std::string& s);

}  // namespace esp32sim
