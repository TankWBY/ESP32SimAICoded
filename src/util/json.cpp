#include "util/json.hpp"

#include <cstdio>
#include <cstdlib>

namespace esp32sim {

class JsonParser {
public:
    explicit JsonParser(const std::string& t) : t_(t) {}

    bool parse(Json& out, std::string* err) {
        bool ok = value(out, 0);
        ws();
        if (ok && p_ != t_.size()) ok = fail("trailing characters");
        if (!ok && err) *err = err_ + " at offset " + std::to_string(p_);
        return ok;
    }

private:
    bool fail(const char* m) { if (err_.empty()) err_ = m; return false; }
    void ws() { while (p_ < t_.size() && (t_[p_] == ' ' || t_[p_] == '\n' || t_[p_] == '\r' || t_[p_] == '\t')) ++p_; }
    bool lit(const char* s) {
        size_t n = 0;
        while (s[n]) ++n;
        if (t_.compare(p_, n, s) != 0) return false;
        p_ += n;
        return true;
    }

    bool string(std::string& out) {
        if (t_[p_] != '"') return fail("expected string");
        ++p_;
        while (p_ < t_.size() && t_[p_] != '"') {
            char c = t_[p_++];
            if (c != '\\') { out += c; continue; }
            if (p_ >= t_.size()) return fail("bad escape");
            char e = t_[p_++];
            switch (e) {
            case 'n': out += '\n'; break;
            case 't': out += '\t'; break;
            case 'r': out += '\r'; break;
            case 'b': out += '\b'; break;
            case 'f': out += '\f'; break;
            case 'u': {
                if (p_ + 4 > t_.size()) return fail("bad \\u escape");
                unsigned cp = unsigned(std::strtoul(t_.substr(p_, 4).c_str(), nullptr, 16));
                p_ += 4;
                if (cp < 0x80) out += char(cp);
                else if (cp < 0x800) { out += char(0xC0 | (cp >> 6)); out += char(0x80 | (cp & 0x3F)); }
                else { out += char(0xE0 | (cp >> 12)); out += char(0x80 | ((cp >> 6) & 0x3F)); out += char(0x80 | (cp & 0x3F)); }
                break;
            }
            default: out += e;
            }
        }
        if (p_ >= t_.size()) return fail("unterminated string");
        ++p_;
        return true;
    }

    bool value(Json& v, int depth) {
        if (depth > 64) return fail("too deep");
        ws();
        if (p_ >= t_.size()) return fail("unexpected end");
        char c = t_[p_];
        if (c == '{') {
            v.type_ = Json::Type::Object;
            ++p_;
            ws();
            if (p_ < t_.size() && t_[p_] == '}') { ++p_; return true; }
            for (;;) {
                ws();
                std::string key;
                if (p_ >= t_.size() || !string(key)) return fail("expected key");
                ws();
                if (p_ >= t_.size() || t_[p_] != ':') return fail("expected ':'");
                ++p_;
                if (!value(v.obj_[key], depth + 1)) return false;
                ws();
                if (p_ < t_.size() && t_[p_] == ',') { ++p_; continue; }
                if (p_ < t_.size() && t_[p_] == '}') { ++p_; return true; }
                return fail("expected ',' or '}'");
            }
        }
        if (c == '[') {
            v.type_ = Json::Type::Array;
            ++p_;
            ws();
            if (p_ < t_.size() && t_[p_] == ']') { ++p_; return true; }
            for (;;) {
                v.arr_.emplace_back();
                if (!value(v.arr_.back(), depth + 1)) return false;
                ws();
                if (p_ < t_.size() && t_[p_] == ',') { ++p_; continue; }
                if (p_ < t_.size() && t_[p_] == ']') { ++p_; return true; }
                return fail("expected ',' or ']'");
            }
        }
        if (c == '"') { v.type_ = Json::Type::String; return string(v.s_); }
        if (lit("true")) { v.type_ = Json::Type::Bool; v.b_ = true; return true; }
        if (lit("false")) { v.type_ = Json::Type::Bool; v.b_ = false; return true; }
        if (lit("null")) { v.type_ = Json::Type::Null; return true; }
        const char* start = t_.c_str() + p_;
        char* end = nullptr;
        double d = std::strtod(start, &end);
        if (end == start) return fail("unexpected character");
        p_ += size_t(end - start);
        v.type_ = Json::Type::Number;
        v.n_ = d;
        return true;
    }

    const std::string& t_;
    size_t p_ = 0;
    std::string err_;
};

namespace {
const Json& null_json() {
    static const Json n;
    return n;
}
}  // namespace

bool Json::parse(const std::string& text, Json& out, std::string* error) {
    out = Json();
    return JsonParser(text).parse(out, error);
}

const Json& Json::operator[](const std::string& key) const {
    if (type_ != Type::Object) return null_json();
    auto it = obj_.find(key);
    return it == obj_.end() ? null_json() : it->second;
}

const Json& Json::operator[](size_t index) const {
    if (type_ != Type::Array || index >= arr_.size()) return null_json();
    return arr_[index];
}

size_t Json::size() const {
    return type_ == Type::Array ? arr_.size() : type_ == Type::Object ? obj_.size() : 0;
}

std::string Json::str(const std::string& fallback) const {
    if (type_ == Type::String) return s_;
    if (type_ == Type::Number) {
        char buf[32];
        std::snprintf(buf, sizeof buf, "%.0f", n_);
        return buf;
    }
    return fallback;
}

double Json::num(double fallback) const {
    if (type_ == Type::Number) return n_;
    if (type_ == Type::String) {
        char* end = nullptr;
        double d = std::strtod(s_.c_str(), &end);
        if (end != s_.c_str()) return d;
    }
    return fallback;
}

bool Json::boolean(bool fallback) const { return type_ == Type::Bool ? b_ : fallback; }

std::string json_escape(const std::string& s) {
    std::string o;
    for (char c : s) {
        if (c == '"' || c == '\\') { o += '\\'; o += c; }
        else if (c == '\n') o += "\\n";
        else if (static_cast<unsigned char>(c) < 0x20) { char b[8]; std::snprintf(b, sizeof b, "\\u%04x", c); o += b; }
        else o += c;
    }
    return o;
}

}  // namespace esp32sim
